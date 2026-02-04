#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "acados_solver_racing_control_hyperplane.h"
#include "dbscan.hpp"
#include "hyperplane_util.hpp"
#include "nmpc_visualizer.hpp"

#define N_PARAM 25 
#define N_HORIZON 60
#define DT 0.05
#define REF_VEL 2.0  

class NmpcTrackerNode : public rclcpp::Node {
public:
    NmpcTrackerNode() : Node("nmpc_node") {
        capsule_ = racing_control_hyperplane_acados_create_capsule();
        racing_control_hyperplane_acados_create(capsule_);

        visualizer_ = std::make_unique<NmpcVisualizer>();
        //这里根据实际场景微调
        cluster_worker_ = std::make_unique<DBSCAN>(1.2, 3); 

        setup_ros_interfaces();
        RCLCPP_INFO(this->get_logger(), "NMPC Tracker Node (Python Logic Synced) Started.");
    }

private:
    void setup_ros_interfaces() {
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                cur_x_[0] = msg->pose.pose.position.x;
                cur_x_[1] = msg->pose.pose.position.y;
                cur_x_[2] = tf2::getYaw(msg->pose.pose.orientation);
                cur_x_[3] = msg->twist.twist.linear.x;
                cur_x_[4] = msg->twist.twist.angular.z;
                odom_ok_ = true;
            });

        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("/scan_cloud", 10, 
            std::bind(&NmpcTrackerNode::cloud_callback, this, std::placeholders::_1));

        sub_path_ = create_subscription<nav_msgs::msg::Path>("/ref_path", 10, 
            [this](const nav_msgs::msg::Path::SharedPtr msg) { 
                full_path_ = *msg; path_ok_ = true; 
            });

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_viz_ = create_publisher<visualization_msgs::msg::MarkerArray>("/nmpc_viz", 10);

        // 50ms 周期，与 DT=0.05 对应
        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&NmpcTrackerNode::solve_cycle, this));
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::vector<Point> pts;
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y");
        for (; it_x != it_x.end(); ++it_x, ++it_y) pts.push_back({*it_x, *it_y});

        auto labels = cluster_worker_->cluster(pts);
        
        std::lock_guard<std::mutex> lock(cluster_mutex_);
        current_clusters_.clear();
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i] > 0) current_clusters_[labels[i]].push_back(pts[i]);
        }
    }

    void solve_cycle() {
        if (!odom_ok_ || !path_ok_ || full_path_.poses.empty()) return;

        auto conf = racing_control_hyperplane_acados_get_nlp_config(capsule_);
        auto dims = racing_control_hyperplane_acados_get_nlp_dims(capsule_);
        auto in = racing_control_hyperplane_acados_get_nlp_in(capsule_);
        auto out = racing_control_hyperplane_acados_get_nlp_out(capsule_);

        // 热启动：平移解序列 (Warm Start)
        for (int i = 0; i < N_HORIZON; i++) {
            double xt[5], ut[2];
            ocp_nlp_out_get(conf, dims, out, i + 1, "x", xt);
            ocp_nlp_out_get(conf, dims, out, i, "u", ut);
            ocp_nlp_out_set(conf, dims, out, in, i, "x", xt);
            ocp_nlp_out_set(conf, dims, out, in, i, "u", ut);
        }

        // 障碍物预处理：聚类 -> 排序 (参考 Python step 3)
        struct ScoredObs { ObstacleParam param; double dist; };
        std::vector<ScoredObs> sorted_obs;
        
        {
            std::lock_guard<std::mutex> lock(cluster_mutex_);
            for (auto const& [id, pts] : current_clusters_) {
                // 拟合障碍物 (初始使用当前位置评估距离)
                auto obs = HyperplaneUtil::fit_obstacle(pts, cur_x_[0], cur_x_[1]);
                double d = std::hypot(obs.ox - cur_x_[0], obs.oy - cur_x_[1]) - obs.r;
                sorted_obs.push_back({obs, d});
            }
        }
        // 按距离由近到远排序
        std::sort(sorted_obs.begin(), sorted_obs.end(), [](const ScoredObs& a, const ScoredObs& b){
            return a.dist < b.dist;
        });

        // 设置初始约束 (Current State)
        ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "lbx", cur_x_);
        ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "ubx", cur_x_);

        int start_idx = find_closest_index(cur_x_[0], cur_x_[1]);
        std::vector<std::pair<int, ObstacleParam>> viz_obs_data;
        std::vector<std::pair<double, double>> viz_ref_data;

        // 预测时域循环
        for (int i = 0; i <= N_HORIZON; i++) {
            // 设置参考轨迹 (参考速度 5.0)
            int ref_idx = std::min(start_idx + i, (int)full_path_.poses.size() - 1);
            double yref[7] = {
                full_path_.poses[ref_idx].pose.position.x, 
                full_path_.poses[ref_idx].pose.position.y, 
                REF_VEL, 0, 0, 0, 0
            }; 
            ocp_nlp_cost_model_set(conf, dims, in, i, "yref", yref);
            viz_ref_data.push_back({yref[0], yref[1]});

            // 动态更新超平面 (基于热启动预测的状态)
            double xi[5]; 
            ocp_nlp_out_get(conf, dims, out, i, "x", xi);

            std::vector<ObstacleParam> stage_obs;
            for (int k = 0; k < 5; ++k) { // 取最近的 5 个
                if (k < (int)sorted_obs.size()) {
                    // 重新根据预测位置 xi 拟合更精确的法向量 (参考 Python compute_hyperplane)
                    auto refined_obs = HyperplaneUtil::fit_obstacle(
                        current_clusters_[k], xi[0], xi[1] // 注意：此处简化了索引对应关系
                    );
                    stage_obs.push_back(sorted_obs[k].param); 
                    if (i % 20 == 0) viz_obs_data.push_back({i * 10 + k, sorted_obs[k].param});
                }
            }

            double p_array[N_PARAM];
            HyperplaneUtil::pack_params(p_array, stage_obs);
            racing_control_hyperplane_acados_update_params(capsule_, i, p_array, N_PARAM);
        }

        int status = racing_control_hyperplane_acados_solve(capsule_);
        if (status == 0) {
            publish_command(conf, dims, out);
            
            // 可视化预测轨迹
            std::vector<std::vector<double>> pred;
            for (int i = 0; i <= N_HORIZON; i++) {
                double xp[5]; ocp_nlp_out_get(conf, dims, out, i, "x", xp);
                pred.push_back({xp[0], xp[1]});
            }
            pub_viz_->publish(visualizer_->create_viz_packet(this->get_clock()->now(), pred, viz_obs_data, viz_ref_data));
        } else {
            RCLCPP_WARN(this->get_logger(), "Acados Solve Failed with status %d", status);
        }
    }

    void publish_command(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out) {
        double u0[2]; 
        ocp_nlp_out_get(conf, dims, out, 0, "u", u0);
        geometry_msgs::msg::Twist cmd;
        // 运动学平滑：在当前速度基础上增加控制增量
        cmd.linear.x = std::clamp(cur_x_[3] + u0[0] * DT, 0.0, 5.0);
        cmd.angular.z = std::clamp(cur_x_[4] + u0[1] * DT, -2.0, 2.0);
        pub_cmd_->publish(cmd);
    }

    int find_closest_index(double x, double y) {
        int closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < (int)full_path_.poses.size(); ++i) {
            double dx = full_path_.poses[i].pose.position.x - x;
            double dy = full_path_.poses[i].pose.position.y - y;
            double dist = dx*dx + dy*dy;
            if (dist < min_dist) { min_dist = dist; closest_idx = i; }
        }
        return closest_idx;
    }

    racing_control_hyperplane_solver_capsule* capsule_;
    std::unique_ptr<NmpcVisualizer> visualizer_;
    std::unique_ptr<DBSCAN> cluster_worker_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex cluster_mutex_;
    std::map<int, std::vector<Point>> current_clusters_;
    double cur_x_[5];
    nav_msgs::msg::Path full_path_;
    bool odom_ok_ = false, path_ok_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NmpcTrackerNode>());
    rclcpp::shutdown();
    return 0;
}