#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <algorithm>
#include <vector>
#include <map>
#include <chrono>

#include "acados_solver_racing_control_hyperplane.h"
#include "dbscan.hpp"
#include "hyperplane_util.hpp"
#include "nmpc_visualizer.hpp"
#include "astar_planner.hpp"

#define N_PARAM 25 
#define N_HORIZON 60
#define DT 0.05

class NmpcTrackerNode : public rclcpp::Node {
public:
    NmpcTrackerNode() : Node("nmpc_node") {
        // --- 参数声明 ---
        this->declare_parameter("nmpc_config.ref_velocity", 5.0);
        this->declare_parameter("nmpc_config.control_loop_ms", 50);
        
        // 感知与避障参数
        this->declare_parameter("perception.dbscan_eps", 1.2);
        this->declare_parameter("perception.dbscan_min_pts", 3);
        this->declare_parameter("perception.fov_half_angle_deg", 120.0);
        this->declare_parameter("obstacle_avoidance.max_obstacles", 5);
        this->declare_parameter("obstacle_avoidance.base_margin", 1.0);
        this->declare_parameter("obstacle_avoidance.speed_gain", 0.15);
        this->declare_parameter("obstacle_avoidance.smoothing_alpha", 0.3);
        
        // 机器人限制
        this->declare_parameter("robot_limits.max_linear_velocity", 6.0);
        this->declare_parameter("robot_limits.min_linear_velocity", 0.0);
        this->declare_parameter("robot_limits.max_angular_velocity", 2.5);
        this->declare_parameter("robot_limits.min_angular_velocity", -2.5);

        // [新增] 是否启用 A* 混合规划开关
        this->declare_parameter("enable_astar", true); 

        // --- 初始化 Acados ---
        capsule_ = racing_control_hyperplane_acados_create_capsule();
        racing_control_hyperplane_acados_create(capsule_);

        // --- 初始化模块 ---
        double eps = this->get_parameter("perception.dbscan_eps").as_double();
        int min_pts = this->get_parameter("perception.dbscan_min_pts").as_int();
        cluster_worker_ = std::make_unique<DBSCAN>(eps, min_pts);
        
        visualizer_ = std::make_unique<NmpcVisualizer>();

        AStarPlanner::Config astar_cfg;
        astar_cfg.resolution = 0.2; 
        astar_cfg.margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
        astar_cfg.grid_padding = 15;
        astar_planner_ = std::make_unique<AStarPlanner>(astar_cfg);

        setup_ros_interfaces();
        RCLCPP_INFO(this->get_logger(), "NMPC Tracker Initialized (A* Enabled: %s)", 
            this->get_parameter("enable_astar").as_bool() ? "True" : "False");
    }

    ~NmpcTrackerNode() {
        racing_control_hyperplane_acados_free(capsule_);
        racing_control_hyperplane_acados_free_capsule(capsule_);
    }

private:
    // --- 辅助数学函数 ---
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double unwrap_yaw(double target_yaw, double current_yaw) {
        double diff = target_yaw - current_yaw;
        while (diff > M_PI) { target_yaw -= 2.0 * M_PI; diff -= 2.0 * M_PI; }
        while (diff < -M_PI) { target_yaw += 2.0 * M_PI; diff += 2.0 * M_PI; }
        return target_yaw;
    }

    void solve_cycle() {
        if (!odom_ok_ || !path_ok_ || full_path_.poses.empty()) return;

        // 获取实时参数
        double ref_vel = this->get_parameter("nmpc_config.ref_velocity").as_double();
        double base_margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
        double speed_gain = this->get_parameter("obstacle_avoidance.speed_gain").as_double();
        double alpha = this->get_parameter("obstacle_avoidance.smoothing_alpha").as_double();
        int max_obs = this->get_parameter("obstacle_avoidance.max_obstacles").as_int();
        bool enable_astar = this->get_parameter("enable_astar").as_bool(); // 获取开关状态

        auto start_time = std::chrono::steady_clock::now();
        
        // Acados 接口
        auto conf = racing_control_hyperplane_acados_get_nlp_config(capsule_);
        auto dims = racing_control_hyperplane_acados_get_nlp_dims(capsule_);
        auto in = racing_control_hyperplane_acados_get_nlp_in(capsule_);
        auto out = racing_control_hyperplane_acados_get_nlp_out(capsule_);

        // Warm Start Shift
        std::vector<std::pair<double, double>> pred_traj_xy(N_HORIZON + 1);
        for (int i = 0; i < N_HORIZON; i++) {
            double xt[5], ut[2];
            ocp_nlp_out_get(conf, dims, out, i + 1, "x", xt); 
            ocp_nlp_out_get(conf, dims, out, i, "u", ut);
            ocp_nlp_out_set(conf, dims, out, in, i, "x", xt);
            ocp_nlp_out_set(conf, dims, out, in, i, "u", ut);
            pred_traj_xy[i] = {xt[0], xt[1]};
        }
        pred_traj_xy[N_HORIZON] = pred_traj_xy[N_HORIZON-1]; 

        double dynamic_margin = base_margin + std::abs(cur_x_[3]) * speed_gain; 

        // 感知处理
        std::vector<Point> all_obstacle_points;       
        std::vector<ObstacleParam> active_obstacles;  
        {
            std::lock_guard<std::mutex> lock(cluster_mutex_);
            for (auto const& [id, pts] : current_clusters_) {
                all_obstacle_points.insert(all_obstacle_points.end(), pts.begin(), pts.end());
                ObstacleParam p = HyperplaneUtil::fit_obstacle(pts, cur_x_[0], cur_x_[1], dynamic_margin);
                if (is_in_fov(p.ox, p.oy)) active_obstacles.push_back(p);
            }
        }

        // --- 路径决策逻辑 ---
        int closest_idx = get_closest_path_index(cur_x_[0], cur_x_[1]);
        int lookahead = 100;   
        int goal_offset = 120; 

        std::vector<std::pair<double, double>> ref_segment;
        for(int i = 0; i < lookahead; ++i) {
            int idx = std::min(closest_idx + i, (int)full_path_.poses.size()-1);
            ref_segment.push_back({full_path_.poses[idx].pose.position.x, full_path_.poses[idx].pose.position.y});
        }

        std::vector<std::pair<double, double>> target_path;
        bool using_astar = false;

        // 仅当启用开关时才运行检测和规划逻辑
        if (enable_astar) {
            bool is_blocked_now = check_path_blocked(ref_segment, all_obstacle_points, dynamic_margin * 1.2);
            if (is_blocked_now) astar_keep_active_counter_ = ASTAR_KEEP_CYCLES;

            if (astar_keep_active_counter_ > 0) {
                int goal_idx = std::min(closest_idx + goal_offset, (int)full_path_.poses.size() - 1);
                double goal_x = full_path_.poses[goal_idx].pose.position.x;
                double goal_y = full_path_.poses[goal_idx].pose.position.y;

                auto raw_astar_pts = astar_planner_->plan(cur_x_[0], cur_x_[1], goal_x, goal_y, all_obstacle_points);

                if (!raw_astar_pts.empty()) {
                    using_astar = true;
                    astar_keep_active_counter_--;
                    
                    // 1. 平滑
                    auto smoothed_pts = AStarPlanner::smooth_path(raw_astar_pts);
                    // 2. 重采样
                    double step_size = std::max(0.1, ref_vel * DT); 
                    auto resampled = AStarPlanner::resample_path(smoothed_pts, step_size);

                    int count = std::min((int)resampled.size(), N_HORIZON + 1);
                    for(int i=0; i<count; ++i) target_path.push_back({resampled[i].x, resampled[i].y});
                    while((int)target_path.size() <= N_HORIZON) target_path.push_back(target_path.back());
                } else {
                    target_path = ref_segment; 
                }
            } 
        } else {
            // 如果禁用 A*，强制复位状态机
            astar_keep_active_counter_ = 0;
        }
        
        // 默认回退到全局路径
        if (target_path.empty()) target_path = ref_segment;
        
        // 补齐长度
        while((int)target_path.size() <= N_HORIZON) {
            if (full_path_.poses.empty()) target_path.push_back({cur_x_[0], cur_x_[1]});
            else target_path.push_back({full_path_.poses.back().pose.position.x, full_path_.poses.back().pose.position.y}); 
        }

        std::vector<NmpcVisualizer::VizObs> all_constraint_viz;

        // 配置 NMPC 参考
        double last_yaw_ref = cur_x_[2]; 
        
        for (int i = 0; i <= N_HORIZON; i++) {
            double tx = target_path[i].first;
            double ty = target_path[i].second;

            double current_ref_vel = using_astar ? ref_vel * 0.6 : ref_vel;
            
            // 智能 Yaw 计算
            double ref_yaw = 0.0;
            int look_ahead_idx = std::min(i + 3, (int)target_path.size() - 1);
            if (look_ahead_idx > i) {
                double dx = target_path[look_ahead_idx].first - tx;
                double dy = target_path[look_ahead_idx].second - ty;
                ref_yaw = std::atan2(dy, dx);
            } else if (i > 0) {
                ref_yaw = last_yaw_ref; 
            } else {
                ref_yaw = cur_x_[2];
            }

            // 解缠
            ref_yaw = unwrap_yaw(ref_yaw, last_yaw_ref);
            last_yaw_ref = ref_yaw;

            double yref[7] = {tx, ty, current_ref_vel, ref_yaw, 0, 0, 0}; 
            ocp_nlp_cost_model_set(conf, dims, in, i, "yref", yref);

            // 障碍物约束 (The Hyperplane Trick)
            std::vector<ObstacleParam> step_obs;
            {
                std::lock_guard<std::mutex> lock(cluster_mutex_);
                for (auto const& [id, pts] : current_clusters_) {
                    ObstacleParam p = HyperplaneUtil::fit_obstacle(pts, pred_traj_xy[i].first, pred_traj_xy[i].second, dynamic_margin);
                    if (is_in_fov(p.ox, p.oy) && (int)step_obs.size() < max_obs) {
                        std::string key = std::to_string(id) + "_" + std::to_string(i);
                        if (prev_normal_map_.count(key)) {
                            p.nx = (1.0 - alpha) * prev_normal_map_[key].first + alpha * p.nx;
                            p.ny = (1.0 - alpha) * prev_normal_map_[key].second + alpha * p.ny;
                            double norm = std::hypot(p.nx, p.ny);
                            if(norm > 1e-3) { p.nx /= norm; p.ny /= norm; }
                        }
                        prev_normal_map_[key] = {p.nx, p.ny};
                        step_obs.push_back(p);
                        if (i % 5 == 0) { 
                            NmpcVisualizer::VizObs vo; vo.id = i * 100 + id; vo.param = p; vo.is_active = true;
                            vo.alpha = 1.0 - (double)i / N_HORIZON; all_constraint_viz.push_back(vo);
                        }
                    }
                }
            }
            double p_array[N_PARAM];
            HyperplaneUtil::pack_params(p_array, step_obs, max_obs);
            racing_control_hyperplane_acados_update_params(capsule_, i, p_array, N_PARAM);
        }

        // 求解
        ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "lbx", cur_x_);
        ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "ubx", cur_x_);

        int status = racing_control_hyperplane_acados_solve(capsule_);
        
        auto end_time = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        std_msgs::msg::Float32 time_msg; time_msg.data = elapsed_ms;
        pub_solve_time_->publish(time_msg);

        if (status != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Solve Failed: %d", status);
            publish_emergency_brake();
        } else {
            publish_command(conf, dims, out);
            render_visualization(conf, dims, out, all_constraint_viz, target_path, using_astar);
        }

        if (using_astar) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "A* Active (Hysteresis: %d)", astar_keep_active_counter_);
        }
    }

    // --- 辅助函数 ---
    bool check_path_blocked(const std::vector<std::pair<double, double>>& path, const std::vector<Point>& all_points, double check_margin) {
        if (path.empty() || all_points.empty()) return false;
        double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
        for(auto& p : path) {
            if(p.first < min_x) min_x = p.first; if(p.first > max_x) max_x = p.first;
            if(p.second < min_y) min_y = p.second; if(p.second > max_y) max_y = p.second;
        }
        min_x -= check_margin; max_x += check_margin; min_y -= check_margin; max_y += check_margin;
        double margin_sq = check_margin * check_margin;
        for (const auto& obs_pt : all_points) {
            if (obs_pt.x < min_x || obs_pt.x > max_x || obs_pt.y < min_y || obs_pt.y > max_y) continue;
            for (const auto& path_pt : path) {
                double dx = path_pt.first - obs_pt.x; double dy = path_pt.second - obs_pt.y;
                if (dx*dx + dy*dy < margin_sq) return true; 
            }
        }
        return false;
    }

    int get_closest_path_index(double x, double y) {
        int idx = 0; double min_dist = 1e9;
        for(size_t i=0; i<full_path_.poses.size(); ++i) {
            double d = std::hypot(full_path_.poses[i].pose.position.x - x, full_path_.poses[i].pose.position.y - y);
            if(d < min_dist) { min_dist = d; idx = i; }
        }
        return idx;
    }

    void publish_command(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out) {
        double u0[2]; ocp_nlp_out_get(conf, dims, out, 0, "u", u0);
        geometry_msgs::msg::Twist cmd;
        double max_v = this->get_parameter("robot_limits.max_linear_velocity").as_double();
        double min_v = this->get_parameter("robot_limits.min_linear_velocity").as_double();
        double max_w = this->get_parameter("robot_limits.max_angular_velocity").as_double();
        double min_w = this->get_parameter("robot_limits.min_angular_velocity").as_double();
        cmd.linear.x = std::clamp(cur_x_[3] + u0[0] * DT, min_v, max_v);
        cmd.angular.z = std::clamp(cur_x_[4] + u0[1] * DT, min_w, max_w);
        pub_cmd_->publish(cmd);
    }
    void publish_emergency_brake() { geometry_msgs::msg::Twist cmd; pub_cmd_->publish(cmd); }
    bool is_in_fov(double ox, double oy) {
        double fov = this->get_parameter("perception.fov_half_angle_deg").as_double() * M_PI / 180.0;
        double angle = std::atan2(oy - cur_x_[1], ox - cur_x_[0]) - cur_x_[2];
        while(angle > M_PI) angle -= 2*M_PI; while(angle < -M_PI) angle += 2*M_PI;
        return std::abs(angle) <= fov;
    }
    void render_visualization(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out, 
                              const std::vector<NmpcVisualizer::VizObs>& constraint_viz, 
                              const std::vector<std::pair<double, double>>& target_path, bool using_astar) 
    {
        std::vector<std::vector<double>> pred_traj;
        for (int i = 0; i <= N_HORIZON; i++) {
            double x[5]; ocp_nlp_out_get(conf, dims, out, i, "x", x);
            pred_traj.push_back({x[0], x[1]});
        }
        pub_viz_->publish(visualizer_->create_viz_packet(this->get_clock()->now(), pred_traj, constraint_viz, target_path, using_astar));
    }
    void setup_ros_interfaces() {
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                cur_x_[0] = msg->pose.pose.position.x; cur_x_[1] = msg->pose.pose.position.y;
                cur_x_[2] = tf2::getYaw(msg->pose.pose.orientation);
                cur_x_[3] = msg->twist.twist.linear.x; cur_x_[4] = msg->twist.twist.angular.z;
                odom_ok_ = true;
            });
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("/scan_cloud", 10, 
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::vector<Point> pts;
                sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y");
                for (; it_x != it_x.end(); ++it_x, ++it_y) pts.push_back({*it_x, *it_y});
                auto labels = cluster_worker_->cluster(pts);
                std::lock_guard<std::mutex> lock(cluster_mutex_);
                current_clusters_.clear();
                for (size_t i = 0; i < labels.size(); i++) if (labels[i] > 0) current_clusters_[labels[i]].push_back(pts[i]);
            });
        sub_path_ = create_subscription<nav_msgs::msg::Path>("/ref_path", 10, 
            [this](const nav_msgs::msg::Path::SharedPtr msg) { full_path_ = *msg; path_ok_ = true; });
        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_viz_ = create_publisher<visualization_msgs::msg::MarkerArray>("/nmpc_viz", 10);
        pub_solve_time_ = create_publisher<std_msgs::msg::Float32>("/nmpc/solve_time", 10);
        int ms = this->get_parameter("nmpc_config.control_loop_ms").as_int();
        timer_ = create_wall_timer(std::chrono::milliseconds(ms), std::bind(&NmpcTrackerNode::solve_cycle, this));
    }
    racing_control_hyperplane_solver_capsule* capsule_;
    std::unique_ptr<NmpcVisualizer> visualizer_;
    std::unique_ptr<DBSCAN> cluster_worker_;
    std::unique_ptr<AStarPlanner> astar_planner_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_solve_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex cluster_mutex_;
    std::map<int, std::vector<Point>> current_clusters_;
    std::map<std::string, std::pair<double, double>> prev_normal_map_;
    double cur_x_[5]; 
    nav_msgs::msg::Path full_path_;
    bool odom_ok_ = false, path_ok_ = false;
    int astar_keep_active_counter_ = 0; 
    const int ASTAR_KEEP_CYCLES = 40; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NmpcTrackerNode>());
    rclcpp::shutdown();
    return 0;
}