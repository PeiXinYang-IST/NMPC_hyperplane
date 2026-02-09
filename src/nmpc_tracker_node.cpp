#include "nmpc_tracker_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono> 
#include <cmath>
#include <algorithm> // std::clamp, std::min, std::max

#define N_PARAM 25    
#define N_HORIZON 60  
#define DT 0.05       

NmpcTrackerNode::NmpcTrackerNode() : Node("nmpc_node") {
    // ==========================================
    // 1. 参数声明与加载
    // ==========================================
    
    // NMPC 基础配置
    this->declare_parameter("nmpc_config.ref_velocity", 5.0);
    this->declare_parameter("nmpc_config.control_loop_ms", 50);
    this->declare_parameter("nmpc_config.enable_warm_start", true); 
    
    // 动态步长阈值
    this->declare_parameter("nmpc_config.curvature_threshold", 1.5); 
    
    // 曲率权重参数
    this->declare_parameter("nmpc_config.global_curvature_weight", 1.0);
    this->declare_parameter("nmpc_config.local_curvature_weight", 1.5);
    this->declare_parameter("nmpc_config.local_curvature_smoothing", 0.3);

    // 赛道与感知配置
    this->declare_parameter("track.road_half_width", 5.0); 
    this->declare_parameter("track.use_virtual_walls", true);
    this->declare_parameter("perception.dbscan_eps", 1.2);
    this->declare_parameter("perception.dbscan_min_pts", 3);
    this->declare_parameter("perception.fov_half_angle_deg", 120.0);
    
    // 避障配置
    this->declare_parameter("obstacle_avoidance.max_obstacles", 5);
    this->declare_parameter("obstacle_avoidance.base_margin", 0.8); 
    this->declare_parameter("obstacle_avoidance.smoothing_alpha", 0.15); 
    this->declare_parameter("obstacle_avoidance.history_bias_weight", 0.2); 
    this->declare_parameter("obstacle_avoidance.astar_reference_cost_weight", 1.0); 
    
    // 安全策略配置
    this->declare_parameter("safety.max_cte_ratio", 0.66);     
    this->declare_parameter("safety.recovery_velocity", 1.0);  

    // 机器人物理限制
    this->declare_parameter("robot_limits.max_linear_velocity", 6.0);
    this->declare_parameter("robot_limits.min_linear_velocity", 0.0);
    this->declare_parameter("robot_limits.max_angular_velocity", 2.5);
    this->declare_parameter("robot_limits.min_angular_velocity", -2.5);

    // ESO 配置
    this->declare_parameter("eso.enable", true);        
    this->declare_parameter("eso.omega_linear", 10.0);  
    this->declare_parameter("eso.omega_angular", 5.0); 
    this->declare_parameter("eso.b0_linear", 1.0);      
    this->declare_parameter("eso.b0_angular", 1.0);     

    // [SFC 配置]
    this->declare_parameter("sfc.robot_radius", 0.5);
    this->declare_parameter("sfc.search_radius", 6.0);
    this->declare_parameter("sfc.longitudinal_length", 4.0);

    // ==========================================
    // 2. 模块初始化
    // ==========================================

    // Acados Solver
    capsule_ = racing_control_hyperplane_acados_create_capsule();
    racing_control_hyperplane_acados_create(capsule_);

    // DBSCAN
    double eps = this->get_parameter("perception.dbscan_eps").as_double();
    int min_pts = this->get_parameter("perception.dbscan_min_pts").as_int();
    cluster_worker_ = std::make_unique<DBSCAN>(eps, min_pts);
    
    // Visualizer
    visualizer_ = std::make_unique<NmpcVisualizer>();

    // A* Planner
    double base_margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
    AStarPlanner::Config astar_cfg;
    astar_cfg.resolution = 0.3; 
    astar_cfg.margin = base_margin * 1.2; 
    astar_cfg.history_bias_weight = this->get_parameter("obstacle_avoidance.history_bias_weight").as_double();
    astar_cfg.reference_cost_weight = this->get_parameter("obstacle_avoidance.astar_reference_cost_weight").as_double();
    astar_planner_ = std::make_unique<AStarPlanner>(astar_cfg);

    // SFC Generator
    SFC_Config sfc_cfg;
    sfc_cfg.robot_radius = this->get_parameter("sfc.robot_radius").as_double();
    sfc_cfg.search_radius = this->get_parameter("sfc.search_radius").as_double();
    sfc_cfg.longitudinal_length = this->get_parameter("sfc.longitudinal_length").as_double();
    sfc_gen_ = std::make_unique<SFCGenerator>(sfc_cfg);

    // ESO
    ESO::Config lin_cfg; lin_cfg.dt = DT; 
    lin_cfg.omega_o = this->get_parameter("eso.omega_linear").as_double();
    lin_cfg.b0 = this->get_parameter("eso.b0_linear").as_double();
    lin_cfg.max_dist = 10.0; 
    linear_eso_ = std::make_unique<ESO>(lin_cfg);

    ESO::Config ang_cfg; ang_cfg.dt = DT;
    ang_cfg.omega_o = this->get_parameter("eso.omega_angular").as_double();
    ang_cfg.b0 = this->get_parameter("eso.b0_angular").as_double();
    ang_cfg.max_dist = 10.0; 
    angular_eso_ = std::make_unique<ESO>(ang_cfg);

    setup_ros_interfaces();
    
    bool enable_eso = this->get_parameter("eso.enable").as_bool();
    RCLCPP_INFO(this->get_logger(), "NMPC Tracker Ready (SFC + Virtual Walls). [ESO: %s]", enable_eso ? "ON" : "OFF");
}

NmpcTrackerNode::~NmpcTrackerNode() {
    if (capsule_) {
        racing_control_hyperplane_acados_free(capsule_);
        racing_control_hyperplane_acados_free_capsule(capsule_);
    }
}

void NmpcTrackerNode::solve_cycle() {
    auto start_total = std::chrono::high_resolution_clock::now();
    
    if (!odom_ok_ || !path_ok_ || full_path_.poses.empty()) return;

    // 获取当前状态
    double gx = cur_x_[0]; double gy = cur_x_[1]; double gyaw = cur_x_[2];
    double gv = cur_x_[3]; double gw = cur_x_[4];

    // 获取配置参数
    bool enable_eso = this->get_parameter("eso.enable").as_bool();
    bool enable_warm_start = this->get_parameter("nmpc_config.enable_warm_start").as_bool();
    double target_ref_vel = this->get_parameter("nmpc_config.ref_velocity").as_double();
    double base_margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
    double search_margin = base_margin * 1.2; 
    double road_half_width = this->get_parameter("track.road_half_width").as_double();
    double safe_ratio = this->get_parameter("safety.max_cte_ratio").as_double();
    double recovery_vel = this->get_parameter("safety.recovery_velocity").as_double();
    double fov_deg = this->get_parameter("perception.fov_half_angle_deg").as_double();
    double fov_rad = fov_deg * M_PI / 180.0;
    bool use_virtual_walls = this->get_parameter("track.use_virtual_walls").as_bool();

    // =========================================================
    // 1. ESO 观测更新
    // =========================================================
    if (enable_eso) {
        if (std::abs(gv) < 0.02 && std::abs(last_cmd_acc_) < 0.01) linear_eso_->reset();
        else linear_eso_->update(last_cmd_acc_, gv);

        if (std::abs(gw) < 0.02 && std::abs(last_cmd_w_acc_) < 0.01) angular_eso_->reset();
        else angular_eso_->update(last_cmd_w_acc_, gw);
    } else {
        linear_eso_->reset(); angular_eso_->reset();
    }
    double dist_acc_lin = linear_eso_->get_disturbance(); 
    double dist_acc_ang = angular_eso_->get_disturbance();

    // =========================================================
    // 2. 全局路径规划 & 安全检查
    // =========================================================
    int closest_idx = get_closest_path_index(gx, gy);
    
    // CTE 检查
    double path_x = full_path_.poses[closest_idx].pose.position.x;
    double path_y = full_path_.poses[closest_idx].pose.position.y;
    double current_cte = std::hypot(gx - path_x, gy - path_y);
    double cte_limit = road_half_width * safe_ratio; 

    bool is_emergency = false;
    if (current_cte > cte_limit) {
        is_emergency = true;
        target_ref_vel = recovery_vel; 
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "!!! LARGE CTE DETECTED (%.2fm) !!! Emergency Slowdown", current_cte);
    }

    // 寻找前瞻目标点
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

    // 获取障碍物
    std::vector<Point> all_obs_pts = get_all_obstacle_points(); 

    // 前向拓展与碰撞检查
    int max_search_idx = std::min((int)full_path_.poses.size() - 1, target_idx + 300);
    double check_margin_sq = search_margin * search_margin; 

    while(target_idx < max_search_idx) {
        double tx = full_path_.poses[target_idx].pose.position.x;
        double ty = full_path_.poses[target_idx].pose.position.y;
        bool collision = false;
        for(const auto& op : all_obs_pts) {
            double dx = tx - op.x; double dy = ty - op.y;
            if(dx*dx + dy*dy < check_margin_sq) { collision = true; break; }
        }
        if(collision) target_idx++; else break;
    }
    double goal_x = full_path_.poses[target_idx].pose.position.x;
    double goal_y = full_path_.poses[target_idx].pose.position.y;

    // =========================================================
    // 3. A* 引导路径规划
    // =========================================================
    auto start_astar = std::chrono::high_resolution_clock::now();
    std::vector<Point> guide_path;     
    std::vector<Point> raw_astar_path;
    bool astar_success = false;
    bool reused_old = false; 

    // 提取全局参考段
    std::vector<Point> ref_path_segment;
    int ref_scan_end = std::min((int)full_path_.poses.size(), closest_idx + 400); 
    int ref_scan_start = std::max(0, closest_idx - 20); 
    for(int i = ref_scan_start; i < ref_scan_end; ++i) {
        ref_path_segment.push_back({full_path_.poses[i].pose.position.x, full_path_.poses[i].pose.position.y});
    }

    // 复用旧路径逻辑
    if (!last_guide_path_.empty()) {
        std::vector<Point> candidate = prune_path_by_distance(last_guide_path_, gx, gy, lookahead_dist);
        if (candidate.size() > 5 && !check_path_collision(candidate, search_margin)) {
            Point end_pt = candidate.back();
            if (std::hypot(end_pt.x - goal_x, end_pt.y - goal_y) < 3.0) {
                raw_astar_path = candidate; 
                reused_old = true; 
                last_guide_path_ = candidate;
            }
        }
    }
    // 重规划
    if (!reused_old) {
        raw_astar_path = astar_planner_->plan(gx, gy, goal_x, goal_y, all_obs_pts, last_guide_path_, ref_path_segment);
        if (!raw_astar_path.empty()) last_guide_path_ = raw_astar_path;
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
        // Fallback
        for(int i=0; i <= N_HORIZON; ++i) {
            int idx = std::min(closest_idx + i, (int)full_path_.poses.size()-1);
            guide_path.push_back({full_path_.poses[idx].pose.position.x, full_path_.poses[idx].pose.position.y});
        }
    }
    auto end_astar = std::chrono::high_resolution_clock::now();
    double t_astar = std::chrono::duration<double, std::milli>(end_astar - start_astar).count();

    // =================================================================================
    // [新增模块] SFC 生成 (包含虚拟墙)
    // =================================================================================
    std::vector<SFC_Constraint> sfc_corridor;
    if (astar_success) {
        // 1. 转换参考路径
        std::vector<Eigen::Vector2d> eigen_path;
        eigen_path.reserve(guide_path.size());
        for(const auto& p : guide_path) eigen_path.emplace_back(p.x, p.y);

        // 2. 准备障碍物点云 (包含感知点 + 虚拟墙)
        std::vector<Eigen::Vector2d> eigen_obs;
        eigen_obs.reserve(all_obs_pts.size() + 200); // 预留空间

        // 2.1 添加感知到的真实障碍物
        for(const auto& p : all_obs_pts) eigen_obs.emplace_back(p.x, p.y);

        // 2.2 [关键修改] 添加虚拟墙作为障碍物
        if (use_virtual_walls) {
            // 沿 global path 采样生成虚拟墙点
            // 只生成当前视野/规划范围内相关的部分
            int vw_scan_start = std::max(0, closest_idx - 10);
            int vw_scan_end = std::min((int)full_path_.poses.size() - 1, closest_idx + N_HORIZON + 50);
            
            // 步长取 2 减少计算量，只要点够密挡住 box 扩张即可
            for(int k = vw_scan_start; k < vw_scan_end; k += 2) {
                double wx = full_path_.poses[k].pose.position.x;
                double wy = full_path_.poses[k].pose.position.y;
                
                // 计算该点的切线 Yaw
                double wyaw = 0.0;
                if (k < (int)full_path_.poses.size() - 1) {
                    double ddx = full_path_.poses[k+1].pose.position.x - wx;
                    double ddy = full_path_.poses[k+1].pose.position.y - wy;
                    wyaw = std::atan2(ddy, ddx);
                } else if (k > 0) {
                     double ddx = wx - full_path_.poses[k-1].pose.position.x;
                     double ddy = wy - full_path_.poses[k-1].pose.position.y;
                     wyaw = std::atan2(ddy, ddx);
                }
                
                double wnx = -std::sin(wyaw);
                double wny = std::cos(wyaw);
                
                // 左墙点
                eigen_obs.emplace_back(wx + wnx * road_half_width, wy + wny * road_half_width);
                // 右墙点
                eigen_obs.emplace_back(wx - wnx * road_half_width, wy - wny * road_half_width);
            }
        }

        // 3. 生成走廊
        sfc_corridor = sfc_gen_->generate_corridor(eigen_path, eigen_obs);
    }
    // =================================================================================

    // =========================================================
    // 4. 动态步长与曲率计算 (可视化用)
    // =========================================================
    double curvature_threshold = this->get_parameter("nmpc_config.curvature_threshold").as_double();
    double global_curve_weight = this->get_parameter("nmpc_config.global_curvature_weight").as_double();
    double local_curve_weight = this->get_parameter("nmpc_config.local_curvature_weight").as_double();
    double smoothing_alpha = this->get_parameter("nmpc_config.local_curvature_smoothing").as_double();
    
    double raw_path_res = 0.1; 
    double stride_dist = 0.5; 
    int stride_step = std::max(1, (int)(stride_dist / raw_path_res)); 

    double predicted_path_len = target_ref_vel * N_HORIZON * DT; 
    double check_dist = std::max(15.0, predicted_path_len * 1.2);
    int max_check_steps = static_cast<int>(check_dist / raw_path_res);
    int end_scan_idx = std::min((int)full_path_.poses.size() - 1 - stride_step, closest_idx + max_check_steps);
    
    // 全局曲率
    double global_curve_sum = 0.0;
    std::vector<geometry_msgs::msg::Point> curve_viz_pts;
    std::vector<std_msgs::msg::ColorRGBA> curve_viz_cols;

    if (end_scan_idx > closest_idx) {
        for (int k = closest_idx; k < end_scan_idx; k += stride_step) {
            double dx1 = full_path_.poses[k + stride_step].pose.position.x - full_path_.poses[k].pose.position.x;
            double dy1 = full_path_.poses[k + stride_step].pose.position.y - full_path_.poses[k].pose.position.y;
            double yaw1 = std::atan2(dy1, dx1);

            geometry_msgs::msg::Point pt;
            pt.x = full_path_.poses[k].pose.position.x; pt.y = full_path_.poses[k].pose.position.y; pt.z = 0.2; 
            curve_viz_pts.push_back(pt);

            double diff = 0.0;
            if (k > closest_idx) {
                double dx0 = full_path_.poses[k].pose.position.x - full_path_.poses[k - stride_step].pose.position.x;
                double dy0 = full_path_.poses[k].pose.position.y - full_path_.poses[k - stride_step].pose.position.y;
                double yaw0 = std::atan2(dy0, dx0);
                diff = std::abs(unwrap_yaw(yaw1, yaw0) - yaw0);
                global_curve_sum += diff;
            }

            std_msgs::msg::ColorRGBA col; col.a = 1.0;
            double ratio = std::clamp(diff / 0.3, 0.0, 1.0);
            col.r = ratio; col.g = 1.0 - ratio; col.b = 0.0;
            curve_viz_cols.push_back(col);
        }
    }
    global_curve_sum *= global_curve_weight;

    // 局部曲率
    double raw_local_curve_sum = 0.0;
    if (guide_path.size() > 2) {
        int check_limit_local = std::min((int)guide_path.size() - 1, 60); 
        for (int k = 0; k < check_limit_local - 1; k++) {
            double dx1 = guide_path[k+1].x - guide_path[k].x; double dy1 = guide_path[k+1].y - guide_path[k].y;
            double yaw1 = std::atan2(dy1, dx1);
            if (k > 0) {
                double dx0 = guide_path[k].x - guide_path[k-1].x; double dy0 = guide_path[k].y - guide_path[k-1].y;
                double yaw0 = std::atan2(dy0, dx0);
                raw_local_curve_sum += std::abs(unwrap_yaw(yaw1, yaw0) - yaw0);
            }
        }
    }
    last_filtered_local_curve_ = smoothing_alpha * raw_local_curve_sum + (1.0 - smoothing_alpha) * last_filtered_local_curve_;
    double weighted_local_curve = last_filtered_local_curve_ * local_curve_weight;

    double total_curve = std::max(global_curve_sum, weighted_local_curve);
    double curve_ratio = std::clamp(total_curve / curvature_threshold, 0.0, 1.0);
    double dynamic_step_dist = 0.2 - curve_ratio * (0.2 - 0.1); 

    // =========================================================
    // 5. 构建 NMPC 问题 (保持原有 Hyperplane 逻辑)
    // =========================================================
    double cos_theta = std::cos(gyaw);
    double sin_theta = std::sin(gyaw);
    
    auto transform_to_local = [&](double x, double y) -> std::pair<double, double> {
        double dx = x - gx; double dy = y - gy;
        return {dx * cos_theta + dy * sin_theta, -dx * sin_theta + dy * cos_theta};
    };

    auto conf = racing_control_hyperplane_acados_get_nlp_config(capsule_);
    auto dims = racing_control_hyperplane_acados_get_nlp_dims(capsule_);
    auto in = racing_control_hyperplane_acados_get_nlp_in(capsule_);
    auto out = racing_control_hyperplane_acados_get_nlp_out(capsule_);

    double x0_local[5] = {0.0, 0.0, 0.0, gv, gw};
    ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "lbx", x0_local);
    ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "ubx", x0_local);

    std::vector<Point> local_obs_pts;
    local_obs_pts.reserve(all_obs_pts.size());
    for(const auto& p : all_obs_pts) {
        auto lp = transform_to_local(p.x, p.y);
        local_obs_pts.push_back({lp.first, lp.second});
    }

    double last_yaw_ref_global = gyaw; 
    int max_obs = this->get_parameter("obstacle_avoidance.max_obstacles").as_int();

    std::vector<NmpcVisualizer::VizObs> all_constraint_viz;
    std::vector<std::pair<double, double>> target_path_viz; 
    double current_ref_progress = 0.0;

    for (int i = 0; i <= N_HORIZON; i++) {
        int idx_offset = static_cast<int>(current_ref_progress / raw_path_res);
        int global_idx = std::min(closest_idx + idx_offset, (int)full_path_.poses.size() - 1);
        current_ref_progress += dynamic_step_dist;

        double ref_tx_g = full_path_.poses[global_idx].pose.position.x;
        double ref_ty_g = full_path_.poses[global_idx].pose.position.y;
        auto ref_local = transform_to_local(ref_tx_g, ref_ty_g);
        
        double global_next_yaw = 0.0;
        int global_next_tangent_idx = std::min(global_idx + 1, (int)full_path_.poses.size() - 1);
        if (global_next_tangent_idx > global_idx) {
            double dx = full_path_.poses[global_next_tangent_idx].pose.position.x - ref_tx_g;
            double dy = full_path_.poses[global_next_tangent_idx].pose.position.y - ref_ty_g;
            global_next_yaw = std::atan2(dy, dx);
        } else { global_next_yaw = last_yaw_ref_global; }

        double smooth_yaw_global = unwrap_yaw(global_next_yaw, last_yaw_ref_global);
        last_yaw_ref_global = smooth_yaw_global;
        double yaw_ref_local = smooth_yaw_global - gyaw; 

        double yref[7] = {ref_local.first, ref_local.second, target_ref_vel, yaw_ref_local, 0, 0, 0};
        ocp_nlp_cost_model_set(conf, dims, in, i, "yref", yref);
        target_path_viz.push_back({ref_tx_g, ref_ty_g}); 

        // 障碍物超平面 (Hyperplanes)
        int guide_idx = std::min(i, (int)guide_path.size() - 1);
        Point guide_pt_g = guide_path[guide_idx];
        auto guide_pt_l = transform_to_local(guide_pt_g.x, guide_pt_g.y);
        
        std::vector<ObstacleParam> step_obs;
        
        // 此处的 Virtual Walls 是为 Hyperplane 方法用的局部墙
        if (use_virtual_walls) {
            double cy = std::cos(yaw_ref_local); double sy = std::sin(yaw_ref_local);
            ObstacleParam left_wall, right_wall;
            left_wall.ox = ref_local.first - sy * road_half_width; 
            left_wall.oy = ref_local.second + cy * road_half_width;
            left_wall.r  = base_margin; left_wall.nx = sy; left_wall.ny = -cy; 
            step_obs.push_back(left_wall);
            
            right_wall.ox = ref_local.first + sy * road_half_width; 
            right_wall.oy = ref_local.second - cy * road_half_width;
            right_wall.r  = base_margin; right_wall.nx = -sy; right_wall.ny = cy;
            step_obs.push_back(right_wall);

            if (i % 2 == 0) {
                auto obs_local_to_global = [&](const ObstacleParam& p_local) -> ObstacleParam {
                    ObstacleParam p_global = p_local;
                    p_global.nx = p_local.nx * cos_theta - p_local.ny * sin_theta;
                    p_global.ny = p_local.nx * sin_theta + p_local.ny * cos_theta;
                    p_global.ox = p_local.ox * cos_theta - p_local.oy * sin_theta + gx;
                    p_global.oy = p_local.ox * sin_theta + p_local.oy * cos_theta + gy;
                    return p_global;
                };
                NmpcVisualizer::VizObs vl; vl.id = 10000 + i; vl.param = obs_local_to_global(left_wall); vl.alpha = 0.2; vl.is_active = true;
                all_constraint_viz.push_back(vl);
                NmpcVisualizer::VizObs vr; vr.id = 40000 + i; vr.param = obs_local_to_global(right_wall); vr.alpha = 0.2; vr.is_active = true;
                all_constraint_viz.push_back(vr);
            }
        }

        if (!local_obs_pts.empty()) {
            ObstacleParam p = HyperplaneUtil::fit_obstacle(local_obs_pts, guide_pt_l.first, guide_pt_l.second, base_margin);
            bool is_close = std::hypot(p.ox - guide_pt_l.first, p.oy - guide_pt_l.second) < 6.0;
            double obs_angle = std::atan2(p.oy, p.ox);
            bool in_fov = std::abs(obs_angle) <= fov_rad;
            bool not_behind = p.ox > -1.0;

            if (is_close && in_fov && not_behind) {
                 step_obs.push_back(p);
                 if (i % 5 == 0) {
                    NmpcVisualizer::VizObs vo; 
                    ObstacleParam p_global = p;
                    p_global.nx = p.nx * cos_theta - p.ny * sin_theta;
                    p_global.ny = p.nx * sin_theta + p.ny * cos_theta;
                    p_global.ox = p.ox * cos_theta - p.oy * sin_theta + gx;
                    p_global.oy = p.ox * sin_theta + p.oy * cos_theta + gy;
                    vo.id = 50000 + i; vo.param = p_global; vo.is_active = true; vo.alpha = 0.6; 
                    all_constraint_viz.push_back(vo);
                 }
            }
        }
        
        double p_array[N_PARAM];
        HyperplaneUtil::pack_params(p_array, step_obs, max_obs);
        racing_control_hyperplane_acados_update_params(capsule_, i, p_array, N_PARAM);
    }

    // =========================================================
    // 6. 热启动 & 求解
    // =========================================================
    if (enable_warm_start) {
        for (int i = 0; i < N_HORIZON; i++) {
            double xt[5], ut[2];
            ocp_nlp_out_get(conf, dims, out, i + 1, "x", xt); 
            ocp_nlp_out_get(conf, dims, out, i, "u", ut);
            ocp_nlp_out_set(conf, dims, out, in, i, "x", xt);
            ocp_nlp_out_set(conf, dims, out, in, i, "u", ut);
        }
        double xt_last[5], ut_last[2];
        ocp_nlp_out_get(conf, dims, out, N_HORIZON, "x", xt_last); 
        ocp_nlp_out_get(conf, dims, out, N_HORIZON-1, "u", ut_last);
        ocp_nlp_out_set(conf, dims, out, in, N_HORIZON, "x", xt_last);
    } else {
        for (int i = 0; i <= N_HORIZON; i++) {
            ocp_nlp_out_set(conf, dims, out, in, i, "x", x0_local);
            if(i < N_HORIZON) { 
                double u0[2] = {0.0, 0.0}; 
                ocp_nlp_out_set(conf, dims, out, in, i, "u", u0); 
            }
        }
    }

    int status = racing_control_hyperplane_acados_solve(capsule_);
    auto end_solve = std::chrono::high_resolution_clock::now();
    double t_total = std::chrono::duration<double, std::milli>(end_solve - start_total).count();
    
    std_msgs::msg::Float32 time_msg; time_msg.data = t_total;
    pub_solve_time_->publish(time_msg);

    // =========================================================
    // 7. 输出与可视化 (包含 SFC)
    // =========================================================
    std::string astar_str = astar_success ? (reused_old ? "Reuse" : "Plan") : "Fail";
    char log_buf[256];
    
    if (status == 0) {
        snprintf(log_buf, sizeof(log_buf), 
            "TIME[Tot:%.1f] A*:%s ESO[Lin:%.2f] Curve[G:%.1f|L:%.1f] Step[%.3f] V:%.2f %s",
            t_total, astar_str.c_str(), dist_acc_lin, 
            global_curve_sum, weighted_local_curve, dynamic_step_dist, gv,
            is_emergency ? "[RECOVERY]" : "");
            
        if (is_emergency) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", log_buf);
        else RCLCPP_INFO(this->get_logger(), "%s", log_buf);

        double u0[2]; 
        ocp_nlp_out_get(conf, dims, out, 0, "u", u0);
        
        double b0_lin = this->get_parameter("eso.b0_linear").as_double();
        double b0_ang = this->get_parameter("eso.b0_angular").as_double();
        double u_acc_actual = u0[0] - dist_acc_lin / b0_lin;
        double u_ang_actual = u0[1] - dist_acc_ang / b0_ang;

        last_cmd_acc_ = u_acc_actual;
        last_cmd_w_acc_ = u_ang_actual;

        publish_command(conf, dims, out, dist_acc_lin, dist_acc_ang);
        
        std::vector<std::vector<double>> global_pred_traj;
        for (int i = 0; i <= N_HORIZON; i++) {
            double x[5]; ocp_nlp_out_get(conf, dims, out, i, "x", x);
            double gx_pred = x[0] * cos_theta - x[1] * sin_theta + gx;
            double gy_pred = x[0] * sin_theta + x[1] * cos_theta + gy;
            global_pred_traj.push_back({gx_pred, gy_pred});
        }
        
        std::vector<std::pair<double, double>> astar_guide_viz; 
        for (const auto& p : guide_path) astar_guide_viz.push_back({p.x, p.y});
        
        // 传递 sfc_corridor 给可视化
        render_visualization(
            global_pred_traj, 
            all_constraint_viz, 
            target_path_viz, 
            astar_guide_viz, 
            goal_x, goal_y, 
            curve_viz_pts, 
            curve_viz_cols, 
            sfc_corridor // [新增]
        );

    } else {
        snprintf(log_buf, sizeof(log_buf), "QP FAIL: %d", status);
        RCLCPP_WARN(this->get_logger(), "%s", log_buf);
        publish_emergency_brake();
    }
}

// =========================================================
// 辅助函数
// =========================================================

void NmpcTrackerNode::publish_command(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out, 
                                      double dist_lin, double dist_ang) 
{
    double u0[2]; 
    ocp_nlp_out_get(conf, dims, out, 0, "u", u0);
    
    geometry_msgs::msg::Twist cmd;
    double max_v = this->get_parameter("robot_limits.max_linear_velocity").as_double();
    double min_v = this->get_parameter("robot_limits.min_linear_velocity").as_double();
    
    double b0_lin = this->get_parameter("eso.b0_linear").as_double();
    double b0_ang = this->get_parameter("eso.b0_angular").as_double();

    double u_acc_comp = u0[0] - dist_lin / b0_lin;
    double u_ang_comp = u0[1] - dist_ang / b0_ang;

    double v_cmd = cur_x_[3] + u_acc_comp * DT;
    double w_cmd = cur_x_[4] + u_ang_comp * DT;
    
    cmd.linear.x = std::clamp(v_cmd, min_v, max_v);
    cmd.angular.z = std::clamp(w_cmd, -2.5, 2.5);

    pub_cmd_->publish(cmd);
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

void NmpcTrackerNode::publish_emergency_brake() { 
    geometry_msgs::msg::Twist cmd; 
    cmd.linear.x = 0.0; cmd.angular.z = 0.0;
    pub_cmd_->publish(cmd); 
}

bool NmpcTrackerNode::is_in_fov(double ox, double oy) { return true; }

void NmpcTrackerNode::render_visualization(
    const std::vector<std::vector<double>>& pred_traj, 
    const std::vector<NmpcVisualizer::VizObs>& constraint_viz, 
    const std::vector<std::pair<double, double>>& target_path_viz,
    const std::vector<std::pair<double, double>>& astar_guide_viz,
    double goal_x, double goal_y,
    const std::vector<geometry_msgs::msg::Point>& curve_pts,
    const std::vector<std_msgs::msg::ColorRGBA>& curve_cols,
    const std::vector<SFC_Constraint>& sfc_corridor) // [新增]
{
    double fov_deg = this->get_parameter("perception.fov_half_angle_deg").as_double();
    std::vector<double> robot_state = {cur_x_[0], cur_x_[1], cur_x_[2]};
    
    // 传递 sfc_corridor 给 Visualizer
    auto markers = visualizer_->create_viz_packet(
        this->get_clock()->now(), pred_traj, constraint_viz, target_path_viz, false, 
        robot_state, fov_deg * M_PI / 180.0,
        curve_pts, curve_cols,
        sfc_corridor // [新增]
    );

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
    goal_mk.scale.x = 0.4; goal_mk.scale.y = 0.4; goal_mk.scale.z = 0.4;
    goal_mk.color.r = 1.0; goal_mk.color.g = 0.0; goal_mk.color.b = 1.0; goal_mk.color.a = 0.9;
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
            cur_x_[0] = msg->pose.pose.position.x; 
            cur_x_[1] = msg->pose.pose.position.y;
            cur_x_[2] = tf2::getYaw(msg->pose.pose.orientation);
            cur_x_[3] = msg->twist.twist.linear.x; 
            cur_x_[4] = msg->twist.twist.angular.z;
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
            for (size_t i = 0; i < labels.size(); i++) 
                if (labels[i] > 0) current_clusters_[labels[i]].push_back(pts[i]);
        });
        
    sub_path_ = create_subscription<nav_msgs::msg::Path>("/ref_path", 10, 
        [this](const nav_msgs::msg::Path::SharedPtr msg) { 
            full_path_ = *msg; 
            path_ok_ = true; 
        });
        
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_viz_ = create_publisher<visualization_msgs::msg::MarkerArray>("/nmpc_viz", 10);
    pub_solve_time_ = create_publisher<std_msgs::msg::Float32>("/nmpc/solve_time", 10);
    
    int ms = this->get_parameter("nmpc_config.control_loop_ms").as_int();
    timer_ = create_wall_timer(std::chrono::milliseconds(ms), std::bind(&NmpcTrackerNode::solve_cycle, this));
}