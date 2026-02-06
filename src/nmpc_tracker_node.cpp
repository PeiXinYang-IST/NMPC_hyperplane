#include "nmpc_tracker_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

// 定义常量
#define N_PARAM 25
#define N_HORIZON 60
#define DT 0.05

NmpcTrackerNode::NmpcTrackerNode() : Node("nmpc_node") {
    // --- 参数声明 ---
    this->declare_parameter("nmpc_config.ref_velocity", 5.0);
    this->declare_parameter("nmpc_config.control_loop_ms", 50);
    
    // 道路半宽 (例如 5.0 米)
    this->declare_parameter("track.road_half_width", 5.0); 
    // 是否开启虚拟墙模式
    this->declare_parameter("track.use_virtual_walls", true);

    this->declare_parameter("perception.dbscan_eps", 1.2);
    this->declare_parameter("perception.dbscan_min_pts", 3);
    this->declare_parameter("perception.fov_half_angle_deg", 120.0);
    
    // 障碍物槽位建议至少设为 2 (仅墙) 或 5 (墙 + 3个动态障碍物)
    this->declare_parameter("obstacle_avoidance.max_obstacles", 5);
    this->declare_parameter("obstacle_avoidance.base_margin", 0.8); // 离墙的安全距离
    this->declare_parameter("obstacle_avoidance.smoothing_alpha", 0.15); 
    
    this->declare_parameter("robot_limits.max_linear_velocity", 6.0);
    this->declare_parameter("robot_limits.min_linear_velocity", 0.0);
    this->declare_parameter("robot_limits.max_angular_velocity", 2.5);
    this->declare_parameter("robot_limits.min_angular_velocity", -2.5);

    // --- 初始化 Acados ---
    capsule_ = racing_control_hyperplane_acados_create_capsule();
    racing_control_hyperplane_acados_create(capsule_);

    double eps = this->get_parameter("perception.dbscan_eps").as_double();
    int min_pts = this->get_parameter("perception.dbscan_min_pts").as_int();
    cluster_worker_ = std::make_unique<DBSCAN>(eps, min_pts);
    
    visualizer_ = std::make_unique<NmpcVisualizer>();

    AStarPlanner::Config astar_cfg; 
    // 此处 A* 配置略，本模式下主要依赖虚拟墙引导
    astar_planner_ = std::make_unique<AStarPlanner>(astar_cfg);

    setup_ros_interfaces();
    RCLCPP_INFO(this->get_logger(), "NMPC Tracker Initialized (Virtual Road Boundary Mode).");
}

NmpcTrackerNode::~NmpcTrackerNode() {
    racing_control_hyperplane_acados_free(capsule_);
    racing_control_hyperplane_acados_free_capsule(capsule_);
}

void NmpcTrackerNode::solve_cycle() {
    if (!odom_ok_ || !path_ok_ || full_path_.poses.empty()) return;

    // 获取实时参数
    double target_ref_vel = this->get_parameter("nmpc_config.ref_velocity").as_double();
    double road_half_width = this->get_parameter("track.road_half_width").as_double();
    bool use_virtual_walls = this->get_parameter("track.use_virtual_walls").as_bool();
    double base_margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
    double alpha = this->get_parameter("obstacle_avoidance.smoothing_alpha").as_double();
    int max_obs = this->get_parameter("obstacle_avoidance.max_obstacles").as_int();

    auto start_time = std::chrono::steady_clock::now();
    
    auto conf = racing_control_hyperplane_acados_get_nlp_config(capsule_);
    auto dims = racing_control_hyperplane_acados_get_nlp_dims(capsule_);
    auto in = racing_control_hyperplane_acados_get_nlp_in(capsule_);
    auto out = racing_control_hyperplane_acados_get_nlp_out(capsule_);

    // 1. Warm Start Shift
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

    // 2. 简单的参考路径提取 (Lookahead)
    int closest_idx = get_closest_path_index(cur_x_[0], cur_x_[1]);
    std::vector<std::pair<double, double>> target_path;
    
    // 直接提取路径点用于 Horizon
    for(int i=0; i <= N_HORIZON; ++i) {
        int idx = std::min(closest_idx + i, (int)full_path_.poses.size()-1);
        target_path.push_back({full_path_.poses[idx].pose.position.x, full_path_.poses[idx].pose.position.y});
    }

    std::vector<NmpcVisualizer::VizObs> all_constraint_viz;
    
    double last_yaw_ref = cur_x_[2]; 
    
    for (int i = 0; i <= N_HORIZON; i++) {
        // --- A. 设置参考轨迹 (Cost Reference) ---
        double tx = target_path[i].first;
        double ty = target_path[i].second;
        
        // 计算当前路径点的切线方向 (Reference Yaw)
        double ref_yaw = 0.0;
        int look_ahead_idx = std::min(i + 1, (int)target_path.size() - 1); // 看稍微近一点计算更准的法向
        if (look_ahead_idx > i) {
            double dx = target_path[look_ahead_idx].first - tx;
            double dy = target_path[look_ahead_idx].second - ty;
            ref_yaw = std::atan2(dy, dx);
        } else {
            ref_yaw = last_yaw_ref; 
        }
        // 平滑 Yaw 角，防止 +/- PI 跳变
        double diff = ref_yaw - last_yaw_ref;
        while(diff > M_PI) diff -= 2*M_PI; while(diff < -M_PI) diff += 2*M_PI;
        ref_yaw = last_yaw_ref + diff;
        last_yaw_ref = ref_yaw;

        double yref[7] = {tx, ty, target_ref_vel, ref_yaw, 0, 0, 0}; 
        ocp_nlp_cost_model_set(conf, dims, in, i, "yref", yref);

        std::vector<ObstacleParam> step_obs;

        if (use_virtual_walls) {
            
            double cos_yaw = std::cos(ref_yaw);
            double sin_yaw = std::sin(ref_yaw);

            ObstacleParam left_wall;
            left_wall.ox = tx - sin_yaw * road_half_width;
            left_wall.oy = ty + cos_yaw * road_half_width;
            left_wall.r  = base_margin; // 墙的厚度/安全边距
            left_wall.nx = sin_yaw; 
            left_wall.ny = -cos_yaw; 
            step_obs.push_back(left_wall);

            ObstacleParam right_wall;
            right_wall.ox = tx + sin_yaw * road_half_width;
            right_wall.oy = ty - cos_yaw * road_half_width;
            right_wall.r  = base_margin;
            right_wall.nx = -sin_yaw;
            right_wall.ny = cos_yaw;
            step_obs.push_back(right_wall);

            if (i % 5 == 0) {
                NmpcVisualizer::VizObs vl; vl.id = 10000 + i; vl.param = left_wall; vl.alpha = 0.5; vl.is_active = true;
                NmpcVisualizer::VizObs vr; vr.id = 40000 + i; vr.param = right_wall; vr.alpha = 0.5; vr.is_active = true;
                all_constraint_viz.push_back(vl);
                all_constraint_viz.push_back(vr);
            }
        }

        if ((int)step_obs.size() < max_obs) {
             std::lock_guard<std::mutex> lock(cluster_mutex_);
             for (auto const& [id, pts] : current_clusters_) {
                 if ((int)step_obs.size() >= max_obs) break;

                 ObstacleParam p = HyperplaneUtil::fit_obstacle(pts, pred_traj_xy[i].first, pred_traj_xy[i].second, base_margin);
                 
                 // 简单的距离过滤，太远的不考虑
                 double d_rob = std::hypot(p.ox - cur_x_[0], p.oy - cur_x_[1]);
                 if (d_rob < 12.0 && is_in_fov(p.ox, p.oy)) {
                     std::string key = std::to_string(id) + "_" + std::to_string(i);
                     apply_normal_smoothing(p, key, alpha, 0.087); 
                     step_obs.push_back(p);

                     if (i % 5 == 0) {
                         NmpcVisualizer::VizObs vo; vo.id = i * 100 + id; vo.param = p; vo.is_active = true; vo.alpha = 0.9;
                         all_constraint_viz.push_back(vo);
                     }
                 }
             }
        }

        // 填充参数到 Acados
        double p_array[N_PARAM];
        HyperplaneUtil::pack_params(p_array, step_obs, max_obs);
        racing_control_hyperplane_acados_update_params(capsule_, i, p_array, N_PARAM);
    }

    if (is_first_run_) {
        for (int i = 0; i <= N_HORIZON; i++) {
            ocp_nlp_out_set(conf, dims, out, in, i, "x", cur_x_);
            double u0[2] = {0.0, 0.0};
            ocp_nlp_out_set(conf, dims, out, in, i, "u", u0);
        }
        is_first_run_ = false;
    }

    ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "lbx", cur_x_);
    ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "ubx", cur_x_);

    int status = racing_control_hyperplane_acados_solve(capsule_);
    
    auto end_time = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    std_msgs::msg::Float32 time_msg; time_msg.data = elapsed_ms;
    pub_solve_time_->publish(time_msg);

    if (status != 0) {
        publish_emergency_brake();
        RCLCPP_WARN(this->get_logger(), "Acados Solve Failed: %d", status);
    } else {
        publish_command(conf, dims, out);
        render_visualization(conf, dims, out, all_constraint_viz, target_path);
    }
}

void NmpcTrackerNode::apply_normal_smoothing(ObstacleParam& p, const std::string& key, double alpha, double max_rot_rad) {
    if (prev_normal_map_.find(key) == prev_normal_map_.end()) {
        prev_normal_map_[key] = {p.nx, p.ny};
        return;
    }
    double prev_nx = prev_normal_map_[key].first;
    double prev_ny = prev_normal_map_[key].second;
    double smooth_nx = (1.0 - alpha) * prev_nx + alpha * p.nx;
    double smooth_ny = (1.0 - alpha) * prev_ny + alpha * p.ny;
    double norm = std::hypot(smooth_nx, smooth_ny);
    if (norm > 1e-3) { smooth_nx /= norm; smooth_ny /= norm; }
    
    // 角度限制逻辑
    double dot = prev_nx * smooth_nx + prev_ny * smooth_ny;
    dot = std::max(-1.0, std::min(1.0, dot));
    double angle_diff = std::acos(dot);
    if (angle_diff > max_rot_rad) {
        double t = max_rot_rad / angle_diff;
        smooth_nx = (1.0 - t) * prev_nx + t * smooth_nx;
        smooth_ny = (1.0 - t) * prev_ny + t * smooth_ny;
        norm = std::hypot(smooth_nx, smooth_ny);
        if (norm > 1e-3) { smooth_nx /= norm; smooth_ny /= norm; }
    }
    p.nx = smooth_nx; p.ny = smooth_ny;
    prev_normal_map_[key] = {p.nx, p.ny};
}

void NmpcTrackerNode::clean_old_normals(const std::vector<int>& active_ids) {
   // 清理逻辑
}

double NmpcTrackerNode::unwrap_yaw(double target_yaw, double current_yaw) {
    double diff = target_yaw - current_yaw;
    while (diff > M_PI) { target_yaw -= 2.0 * M_PI; diff -= 2.0 * M_PI; }
    while (diff < -M_PI) { target_yaw += 2.0 * M_PI; diff += 2.0 * M_PI; }
    return target_yaw;
}

int NmpcTrackerNode::get_closest_path_index(double x, double y) {
    int idx = 0; double min_dist = 1e9;
    for(size_t i=0; i<full_path_.poses.size(); ++i) {
        double d = std::hypot(full_path_.poses[i].pose.position.x - x, full_path_.poses[i].pose.position.y - y);
        if(d < min_dist) { min_dist = d; idx = i; }
    }
    return idx;
}

void NmpcTrackerNode::publish_command(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out) {
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

void NmpcTrackerNode::publish_emergency_brake() { 
    geometry_msgs::msg::Twist cmd; pub_cmd_->publish(cmd); 
}

bool NmpcTrackerNode::is_in_fov(double ox, double oy) {
    double fov = this->get_parameter("perception.fov_half_angle_deg").as_double() * M_PI / 180.0;
    double angle = std::atan2(oy - cur_x_[1], ox - cur_x_[0]) - cur_x_[2];
    while(angle > M_PI) angle -= 2*M_PI; while(angle < -M_PI) angle += 2*M_PI;
    return std::abs(angle) <= fov;
}

void NmpcTrackerNode::render_visualization(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out, 
                          const std::vector<NmpcVisualizer::VizObs>& constraint_viz, 
                          const std::vector<std::pair<double, double>>& target_path) 
{
    std::vector<std::vector<double>> pred_traj;
    for (int i = 0; i <= N_HORIZON; i++) {
        double x[5]; ocp_nlp_out_get(conf, dims, out, i, "x", x);
        pred_traj.push_back({x[0], x[1]});
    }
    double fov_deg = this->get_parameter("perception.fov_half_angle_deg").as_double();
    std::vector<double> robot_state = {cur_x_[0], cur_x_[1], cur_x_[2]};
    pub_viz_->publish(visualizer_->create_viz_packet(
        this->get_clock()->now(), pred_traj, constraint_viz, target_path, false, robot_state, fov_deg * M_PI / 180.0));
}

void NmpcTrackerNode::setup_ros_interfaces() {
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