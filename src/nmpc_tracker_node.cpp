#include "nmpc_tracker_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono> 

#define N_PARAM 25    
#define N_HORIZON 60  
#define DT 0.05       

NmpcTrackerNode::NmpcTrackerNode() : Node("nmpc_node") {
    this->declare_parameter("nmpc_config.ref_velocity", 5.0);
    this->declare_parameter("nmpc_config.control_loop_ms", 50);
    
    this->declare_parameter("track.road_half_width", 5.0); 
    this->declare_parameter("track.use_virtual_walls", true);

    this->declare_parameter("perception.dbscan_eps", 1.2);
    this->declare_parameter("perception.dbscan_min_pts", 3);
    this->declare_parameter("perception.fov_half_angle_deg", 120.0);
    
    this->declare_parameter("obstacle_avoidance.max_obstacles", 5);
    this->declare_parameter("obstacle_avoidance.base_margin", 0.8); 
    this->declare_parameter("obstacle_avoidance.smoothing_alpha", 0.15); 
    this->declare_parameter("obstacle_avoidance.history_bias_weight", 0.2); 
    
    this->declare_parameter("robot_limits.max_linear_velocity", 6.0);
    this->declare_parameter("robot_limits.min_linear_velocity", 0.0);
    this->declare_parameter("robot_limits.max_angular_velocity", 2.5);
    this->declare_parameter("robot_limits.min_angular_velocity", -2.5);

    capsule_ = racing_control_hyperplane_acados_create_capsule();
    racing_control_hyperplane_acados_create(capsule_);

    double eps = this->get_parameter("perception.dbscan_eps").as_double();
    int min_pts = this->get_parameter("perception.dbscan_min_pts").as_int();
    cluster_worker_ = std::make_unique<DBSCAN>(eps, min_pts);
    
    visualizer_ = std::make_unique<NmpcVisualizer>();

    AStarPlanner::Config astar_cfg;
    astar_cfg.resolution = 0.3; 
    astar_cfg.margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
    astar_cfg.history_bias_weight = this->get_parameter("obstacle_avoidance.history_bias_weight").as_double();
    astar_planner_ = std::make_unique<AStarPlanner>(astar_cfg);

    setup_ros_interfaces();
    RCLCPP_INFO(this->get_logger(), "NMPC Tracker Initialized: Full Vis & Always Margin Mode.");
}

NmpcTrackerNode::~NmpcTrackerNode() {
    racing_control_hyperplane_acados_free(capsule_);
    racing_control_hyperplane_acados_free_capsule(capsule_);
}

void NmpcTrackerNode::solve_cycle() {
    // [计时] 总耗时开始
    auto start_total = std::chrono::high_resolution_clock::now();
    double t_astar = 0.0;
    double t_obs_prep = 0.0;
    double t_solve = 0.0;

    if (!odom_ok_ || !path_ok_ || full_path_.poses.empty()) return;

    // 1. 获取参数
    double target_ref_vel = this->get_parameter("nmpc_config.ref_velocity").as_double();
    double road_half_width = this->get_parameter("track.road_half_width").as_double();
    bool use_virtual_walls = this->get_parameter("track.use_virtual_walls").as_bool();
    double base_margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
    int max_obs = this->get_parameter("obstacle_avoidance.max_obstacles").as_int();

    auto conf = racing_control_hyperplane_acados_get_nlp_config(capsule_);
    auto dims = racing_control_hyperplane_acados_get_nlp_dims(capsule_);
    auto in = racing_control_hyperplane_acados_get_nlp_in(capsule_);
    auto out = racing_control_hyperplane_acados_get_nlp_out(capsule_);

    // 2. Warm Start Shift
    for (int i = 0; i < N_HORIZON; i++) {
        double xt[5], ut[2];
        ocp_nlp_out_get(conf, dims, out, i + 1, "x", xt); 
        ocp_nlp_out_get(conf, dims, out, i, "u", ut);
        ocp_nlp_out_set(conf, dims, out, in, i, "x", xt);
        ocp_nlp_out_set(conf, dims, out, in, i, "u", ut);
    }

    // 提前获取障碍物点云
    std::vector<Point> all_obs_pts = get_all_obstacle_points(); 

    int closest_idx = get_closest_path_index(cur_x_[0], cur_x_[1]);
    double lookahead_dist = std::max(5.0, target_ref_vel * N_HORIZON * DT * 1.2); 
    
    int target_idx = closest_idx;
    double dist_acc = 0;
    while(target_idx < (int)full_path_.poses.size() - 1 && dist_acc < lookahead_dist) {
        double d = std::hypot(
            full_path_.poses[target_idx+1].pose.position.x - full_path_.poses[target_idx].pose.position.x,
            full_path_.poses[target_idx+1].pose.position.y - full_path_.poses[target_idx].pose.position.y);
        dist_acc += d;
        target_idx++;
    }

    // Smart Goal Adjustment: 目标点碰撞检测与前向延申
    if (!all_obs_pts.empty()) {
        double margin_sq = base_margin * base_margin; // 碰撞检测阈值
        int search_limit_idx = std::min((int)full_path_.poses.size() - 1, target_idx + 600); // 限制搜索范围

        // Lambda: 检查单个点是否安全
        auto check_point_safe = [&](double x, double y) {
            for (const auto& opt : all_obs_pts) {
                double dx = x - opt.x;
                double dy = y - opt.y;
                if (dx*dx + dy*dy < margin_sq) return false; // 碰撞
            }
            return true; // 安全
        };

        while (target_idx < search_limit_idx) {
            double tx = full_path_.poses[target_idx].pose.position.x;
            double ty = full_path_.poses[target_idx].pose.position.y;
            
            if (check_point_safe(tx, ty)) {
                break; // 找到安全点了
            }
            
            target_idx += 5; // 步进搜索
        }
    }

    // 此时的 goal_x/y 要么是原本的预瞄点，要么是避开障碍后的前方点
    double goal_x = full_path_.poses[target_idx].pose.position.x;
    double goal_y = full_path_.poses[target_idx].pose.position.y;

    auto start_astar = std::chrono::high_resolution_clock::now();

    std::vector<Point> guide_path;     
    std::vector<Point> raw_astar_path;
    bool astar_success = false;
    bool reused_old = false; 

    // 尝试复用
    if (!last_guide_path_.empty()) {
        std::vector<Point> candidate = prune_path_by_distance(last_guide_path_, cur_x_[0], cur_x_[1], lookahead_dist);
        // 复用检查
        if (candidate.size() > 5 && !check_path_collision(candidate, base_margin)) {
            Point end_pt = candidate.back();
            // 如果旧路径终点和新目标点足够近，就复用
            if (std::hypot(end_pt.x - goal_x, end_pt.y - goal_y) < 3.0) {
                raw_astar_path = candidate;
                reused_old = true;
                last_guide_path_ = candidate;
            }
        }
    }

    if (!reused_old) {
        raw_astar_path = astar_planner_->plan(
            cur_x_[0], cur_x_[1], goal_x, goal_y, all_obs_pts, last_guide_path_
        );
        if (!raw_astar_path.empty()) {
            last_guide_path_ = raw_astar_path;
        }
    }

    // 后处理
    if (!raw_astar_path.empty()) {
        std::vector<Point> smoothed;
        if (reused_old) smoothed = raw_astar_path;
        else smoothed = AStarPlanner::smooth_path(raw_astar_path, 0.5, 0.35);

        double step_size = std::max(target_ref_vel * DT, 0.1); 
        guide_path = AStarPlanner::resample_path(smoothed, step_size);
        astar_success = true;
    } else {
        // Fallback: 如果 A* 依然失败（例如被完全包围），回退到参考路径
        for(int i=0; i <= N_HORIZON; ++i) {
            int idx = std::min(closest_idx + i, (int)full_path_.poses.size()-1);
            guide_path.push_back({
                full_path_.poses[idx].pose.position.x,
                full_path_.poses[idx].pose.position.y
            });
        }
    }

    // [计时结束] A*
    auto end_astar = std::chrono::high_resolution_clock::now();
    t_astar = std::chrono::duration<double, std::milli>(end_astar - start_astar).count();

    auto start_prep = std::chrono::high_resolution_clock::now();

    std::vector<NmpcVisualizer::VizObs> all_constraint_viz;
    std::vector<std::pair<double, double>> target_path_viz; 
    std::vector<std::pair<double, double>> astar_guide_viz; 

    if (!guide_path.empty()) {
        for (const auto& p : guide_path) {
            astar_guide_viz.push_back({p.x, p.y});
        }
    }

    double last_yaw_ref = cur_x_[2]; 

    for (int i = 0; i <= N_HORIZON; i++) {
        // Cost Ref
        int global_idx = std::min(closest_idx + i, (int)full_path_.poses.size() - 1);
        double ref_tx = full_path_.poses[global_idx].pose.position.x;
        double ref_ty = full_path_.poses[global_idx].pose.position.y;
        target_path_viz.push_back({ref_tx, ref_ty});

        double ref_yaw = 0.0;
        int global_next = std::min(global_idx + 1, (int)full_path_.poses.size() - 1);
        if (global_next > global_idx) {
            double dx = full_path_.poses[global_next].pose.position.x - ref_tx;
            double dy = full_path_.poses[global_next].pose.position.y - ref_ty;
            ref_yaw = std::atan2(dy, dx);
        } else { ref_yaw = last_yaw_ref; }

        double diff = ref_yaw - last_yaw_ref;
        while(diff > M_PI) diff -= 2*M_PI; while(diff < -M_PI) diff += 2*M_PI;
        ref_yaw = last_yaw_ref + diff;
        last_yaw_ref = ref_yaw;

        double yref[7] = {ref_tx, ref_ty, target_ref_vel, ref_yaw, 0, 0, 0}; 
        ocp_nlp_cost_model_set(conf, dims, in, i, "yref", yref);

        // Constraints
        std::vector<ObstacleParam> step_obs;
        int guide_idx = std::min(i, (int)guide_path.size() - 1);
        Point guide_seed = guide_path[guide_idx];

        if (use_virtual_walls) {
            double cy = std::cos(ref_yaw); double sy = std::sin(ref_yaw);
            ObstacleParam left_wall;
            left_wall.ox = ref_tx - sy * road_half_width; left_wall.oy = ref_ty + cy * road_half_width;
            left_wall.r  = base_margin; left_wall.nx = sy; left_wall.ny = -cy; 
            step_obs.push_back(left_wall);

            ObstacleParam right_wall;
            right_wall.ox = ref_tx + sy * road_half_width; right_wall.oy = ref_ty - cy * road_half_width;
            right_wall.r  = base_margin; right_wall.nx = -sy; right_wall.ny = cy;
            step_obs.push_back(right_wall);

            if (i % 2 == 0) {
                NmpcVisualizer::VizObs vl; vl.id = 10000 + i; vl.param = left_wall; vl.alpha = 0.2; vl.is_active = true;
                NmpcVisualizer::VizObs vr; vr.id = 40000 + i; vr.param = right_wall; vr.alpha = 0.2; vr.is_active = true;
                all_constraint_viz.push_back(vl); all_constraint_viz.push_back(vr);
            }
        }

        if (!all_obs_pts.empty()) {
            ObstacleParam p = HyperplaneUtil::fit_obstacle(all_obs_pts, guide_seed.x, guide_seed.y, base_margin);
            // 距离检测，只对附近的障碍物生效
            if (std::hypot(p.ox - guide_seed.x, p.oy - guide_seed.y) < 6.0) {
                 step_obs.push_back(p);
                 // 始终可视化
                 if (i % 5 == 0) { 
                     NmpcVisualizer::VizObs vo; 
                     vo.id = 50000 + i; vo.param = p; vo.is_active = true; vo.alpha = 0.6; 
                     all_constraint_viz.push_back(vo);
                 }
            }
        }

        double p_array[N_PARAM];
        HyperplaneUtil::pack_params(p_array, step_obs, max_obs);
        racing_control_hyperplane_acados_update_params(capsule_, i, p_array, N_PARAM);
    }
    
    // [计时结束] 构建问题
    auto end_prep = std::chrono::high_resolution_clock::now();
    t_obs_prep = std::chrono::duration<double, std::milli>(end_prep - start_prep).count();

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

    auto start_solve = std::chrono::high_resolution_clock::now();
    int status = racing_control_hyperplane_acados_solve(capsule_);
    auto end_solve = std::chrono::high_resolution_clock::now();
    t_solve = std::chrono::duration<double, std::milli>(end_solve - start_solve).count();
    
    auto end_total = std::chrono::high_resolution_clock::now();
    double t_total = std::chrono::duration<double, std::milli>(end_total - start_total).count();

    std_msgs::msg::Float32 time_msg; time_msg.data = t_total;
    pub_solve_time_->publish(time_msg);

    // 状态显示逻辑：因为强制运行 A*，所以只要 astar_success 就是 Plan 或 Reuse
    std::string astar_str = astar_success ? (reused_old ? "Reuse" : "Plan") : "Fail";
    
    char log_buf[256];
    if (status == 0) {
        snprintf(log_buf, sizeof(log_buf), 
            "TIME(ms)[Tot:%.1f A*:%.1f Obs:%.1f QP:%.1f] ST:[OK] V:%.2f W:%.2f A*:%s",
            t_total, t_astar, t_obs_prep, t_solve, 
            cur_x_[3], cur_x_[4], astar_str.c_str());
        RCLCPP_INFO(this->get_logger(), "%s", log_buf);
    } else {
        snprintf(log_buf, sizeof(log_buf), 
            "TIME(ms)[Tot:%.1f A*:%.1f Obs:%.1f QP:%.1f] ST:[FAIL %d] V:%.2f W:%.2f A*:%s",
            t_total, t_astar, t_obs_prep, t_solve, status, 
            cur_x_[3], cur_x_[4], astar_str.c_str());
        RCLCPP_WARN(this->get_logger(), "%s", log_buf);
        publish_emergency_brake();
    }

    if (status == 0) {
        publish_command(conf, dims, out);
        // [修改] 传入 goal_x 和 goal_y 进行可视化
        render_visualization(conf, dims, out, all_constraint_viz, target_path_viz, astar_guide_viz, goal_x, goal_y);
    }
}

std::vector<Point> NmpcTrackerNode::get_all_obstacle_points() {
    std::lock_guard<std::mutex> lock(cluster_mutex_);
    std::vector<Point> all_pts;
    for (auto const& [id, pts] : current_clusters_) {
        all_pts.insert(all_pts.end(), pts.begin(), pts.end());
    }
    return all_pts;
}

std::vector<Point> NmpcTrackerNode::prune_path_by_distance(const std::vector<Point>& path, double curr_x, double curr_y, double lookahead_dist) {
    if (path.empty()) return {};
    int closest_idx = 0; double min_dist = 1e9;
    for (size_t i = 0; i < path.size(); ++i) {
        double d = std::hypot(path[i].x - curr_x, path[i].y - curr_y);
        if (d < min_dist) { min_dist = d; closest_idx = i; }
    }
    if (min_dist > 2.0) return {};
    std::vector<Point> pruned; double dist_accum = 0;
    for (size_t i = closest_idx; i < path.size(); ++i) {
        pruned.push_back(path[i]);
        if (i > closest_idx) dist_accum += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
        if (dist_accum > lookahead_dist * 1.5) break; 
    }
    return pruned;
}

bool NmpcTrackerNode::check_path_collision(const std::vector<Point>& path, double margin) {
    if (path.empty()) return true;
    std::lock_guard<std::mutex> lock(cluster_mutex_);
    if (current_clusters_.empty()) return false;
    double margin_sq = margin * margin;
    for (size_t i = 0; i < path.size(); i += 3) {
        for (auto const& [id, pts] : current_clusters_) {
            for (const auto& opt : pts) {
                double dx = path[i].x - opt.x; double dy = path[i].y - opt.y;
                if (dx*dx + dy*dy < margin_sq) return true;
            }
        }
    }
    return false;
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
    p.nx = smooth_nx; p.ny = smooth_ny;
    prev_normal_map_[key] = {p.nx, p.ny};
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
    
    cmd.linear.x = std::clamp(cur_x_[3] + u0[0] * DT, min_v, max_v);
    cmd.angular.z = std::clamp(cur_x_[4] + u0[1] * DT, -2.5, 2.5);
    pub_cmd_->publish(cmd);
}

void NmpcTrackerNode::publish_emergency_brake() { 
    geometry_msgs::msg::Twist cmd; pub_cmd_->publish(cmd); 
}

bool NmpcTrackerNode::is_in_fov(double ox, double oy) {
    return true; 
}

void NmpcTrackerNode::render_visualization(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out, 
                          const std::vector<NmpcVisualizer::VizObs>& constraint_viz, 
                          const std::vector<std::pair<double, double>>& target_path_viz,
                          const std::vector<std::pair<double, double>>& astar_guide_viz,
                          double goal_x, double goal_y) 
{
    std::vector<std::vector<double>> pred_traj;
    for (int i = 0; i <= N_HORIZON; i++) {
        double x[5]; ocp_nlp_out_get(conf, dims, out, i, "x", x);
        pred_traj.push_back({x[0], x[1]});
    }
    double fov_deg = this->get_parameter("perception.fov_half_angle_deg").as_double();
    std::vector<double> robot_state = {cur_x_[0], cur_x_[1], cur_x_[2]};
    
    auto markers = visualizer_->create_viz_packet(
        this->get_clock()->now(), pred_traj, constraint_viz, target_path_viz, false, robot_state, fov_deg * M_PI / 180.0);

    visualization_msgs::msg::Marker goal_mk;
    goal_mk.header.frame_id = "map";
    goal_mk.header.stamp = this->get_clock()->now();
    goal_mk.ns = "nmpc_goal";
    goal_mk.id = 8888;
    goal_mk.type = visualization_msgs::msg::Marker::SPHERE;
    goal_mk.action = visualization_msgs::msg::Marker::ADD;
    goal_mk.pose.position.x = goal_x;
    goal_mk.pose.position.y = goal_y;
    goal_mk.pose.position.z = 0.2;
    goal_mk.pose.orientation.w = 1.0;
    goal_mk.scale.x = 0.4; 
    goal_mk.scale.y = 0.4;
    goal_mk.scale.z = 0.4;
    goal_mk.color.r = 1.0; // Magenta
    goal_mk.color.g = 0.0;
    goal_mk.color.b = 1.0;
    goal_mk.color.a = 0.9;
    markers.markers.push_back(goal_mk);

    if (!astar_guide_viz.empty()) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->get_clock()->now();
        m.ns = "astar_path_debug";
        m.id = 9999;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 0.15;
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.8; 
        for (const auto& p : astar_guide_viz) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first; pt.y = p.second; pt.z = 0.1;
            m.points.push_back(pt);
        }
        markers.markers.push_back(m);
    }
    pub_viz_->publish(markers);
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