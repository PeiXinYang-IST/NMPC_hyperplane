#include "nmpc_tracker_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <random>

#define N_PARAM 0
#define N_HORIZON 30
#define DT 0.1
#define VELOCITY_NOISE_STD 0.0  
#define VELOCITY_X_BIAS 0.0  

NmpcTrackerNode::NmpcTrackerNode() : Node("nmpc_node"), vel_noise_generator_(std::random_device{}()), vel_noise_dist_(0.0, VELOCITY_NOISE_STD) {
    // ==========================================
    // 1. 参数声明
    // ==========================================
    // [新增] 性能模式开关：开启后关闭所有可视化和非必要日志，极致降低延迟
    this->declare_parameter("system.performance_mode", false);

    this->declare_parameter("nmpc_config.ref_velocity", 5.0);
    this->declare_parameter("nmpc_config.control_loop_ms", 50);
    this->declare_parameter("nmpc_config.enable_warm_start", true); 
    this->declare_parameter("nmpc_config.curvature_threshold", 1.5); 
    this->declare_parameter("nmpc_config.global_curvature_weight", 1.0);
    this->declare_parameter("track.road_half_width", 5.0); 
    this->declare_parameter("track.use_virtual_walls", true);
    this->declare_parameter("perception.dbscan_eps", 1.2);
    this->declare_parameter("perception.dbscan_min_pts", 3);
    this->declare_parameter("perception.fov_half_angle_deg", 120.0);
    this->declare_parameter("obstacle_avoidance.base_margin", 0.8); 
    this->declare_parameter("safety.max_cte_ratio", 0.66);     
    this->declare_parameter("safety.recovery_velocity", 1.0);  
    this->declare_parameter("robot_limits.max_linear_velocity", 6.0);
    this->declare_parameter("robot_limits.min_linear_velocity", 0.0);
    this->declare_parameter("robot_limits.max_angular_velocity", 2.5);
    this->declare_parameter("robot_limits.min_angular_velocity", -2.5);
    this->declare_parameter("eso.enable", true);        
    this->declare_parameter("eso.omega_linear", 10.0);  
    this->declare_parameter("eso.omega_angular", 5.0); 
    this->declare_parameter("eso.b0_linear", 1.0);      
    this->declare_parameter("eso.b0_angular", 1.0);     
    this->declare_parameter("sfc.enable", true);
    this->declare_parameter("sfc.robot_radius", 0.5);
    this->declare_parameter("sfc.search_radius", 6.0);
    this->declare_parameter("sfc.longitudinal_length", 4.0);

    // Lattice 参数
    this->declare_parameter("lattice.path_resolution", 0.2);
    this->declare_parameter("lattice.lookahead_dist", 20.0); 
    this->declare_parameter("lattice.num_samples", 7);       
    this->declare_parameter("lattice.sample_width", 0.5);    
    this->declare_parameter("lattice.max_width", 3.0);       
    this->declare_parameter("lattice.collision_radius", 0.8);
    this->declare_parameter("lattice.weight_collision", 1000.0);
    this->declare_parameter("lattice.weight_offset", 1.0);
    this->declare_parameter("lattice.weight_consistency", 2.0);

    // Dummy A* params
    this->declare_parameter("astar.resolution", 0.3);
    this->declare_parameter("astar.grid_padding", 15);
    this->declare_parameter("astar.heuristic_weight", 1.1);
    this->declare_parameter("astar.reference_cost_weight", 2.0);
    this->declare_parameter("astar.turning_weight", 2.0);
    this->declare_parameter("astar.history_bias_weight", 0.5);
    this->declare_parameter("astar.collision_penalty_weight", 50.0);
    this->declare_parameter("astar.smooth_data_weight", 0.45);
    this->declare_parameter("astar.smooth_smooth_weight", 0.40);
    this->declare_parameter("astar.smooth_curvature_weight", 0.40);

    // ==========================================
    // 2. 模块初始化
    // ==========================================
    capsule_ = racing_control_hyperplane_acados_create_capsule();
    racing_control_hyperplane_acados_create(capsule_);

    double eps = this->get_parameter("perception.dbscan_eps").as_double();
    int min_pts = this->get_parameter("perception.dbscan_min_pts").as_int();
    cluster_worker_ = std::make_unique<DBSCAN>(eps, min_pts);
    
    visualizer_ = std::make_unique<NmpcVisualizer>();
    AStarPlanner::Config astar_cfg; 
    astar_planner_ = std::make_unique<AStarPlanner>(astar_cfg);
    lattice_planner_ = std::make_unique<LatticePlanner>();

    SFC_Config sfc_cfg;
    sfc_cfg.robot_radius = this->get_parameter("sfc.robot_radius").as_double();
    sfc_cfg.search_radius = this->get_parameter("sfc.search_radius").as_double();
    sfc_cfg.longitudinal_length = this->get_parameter("sfc.longitudinal_length").as_double();
    sfc_gen_ = std::make_unique<SFCGenerator>(sfc_cfg);

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
    RCLCPP_INFO(this->get_logger(), "NMPC Tracker Ready. [Ref Path Visualization: Pillars]");
}

NmpcTrackerNode::~NmpcTrackerNode() {
    if (capsule_) {
        racing_control_hyperplane_acados_free(capsule_);
        racing_control_hyperplane_acados_free_capsule(capsule_);
    }
}

void NmpcTrackerNode::solve_cycle() {
    auto start_total = std::chrono::high_resolution_clock::now();
    
    // 0. 基础状态检查
    if (!odom_ok_ || !path_ok_ || full_path_.poses.empty()) return;

    // 获取当前机器人状态
    double gx = cur_x_[0]; double gy = cur_x_[1]; double gyaw = cur_x_[2];
    double gv = cur_x_[3]; double gw = cur_x_[4];

    // ==========================================
    // 1. 参数读取 (实时更新以支持动态调参)
    // ==========================================
    bool perf_mode = this->get_parameter("system.performance_mode").as_bool(); // [Performance Check]

    bool enable_eso = this->get_parameter("eso.enable").as_bool();
    bool enable_warm_start = this->get_parameter("nmpc_config.enable_warm_start").as_bool();
    double target_ref_vel = this->get_parameter("nmpc_config.ref_velocity").as_double();
    
    // 赛道与安全参数
    double road_half_width = this->get_parameter("track.road_half_width").as_double();
    double safe_ratio = this->get_parameter("safety.max_cte_ratio").as_double();
    double recovery_vel = this->get_parameter("safety.recovery_velocity").as_double();
    bool use_virtual_walls = this->get_parameter("track.use_virtual_walls").as_bool();
    bool enable_sfc = this->get_parameter("sfc.enable").as_bool();

    // 避障参数 (计算膨胀后的 Margin)
    double base_margin = this->get_parameter("obstacle_avoidance.base_margin").as_double();
    double search_margin = base_margin * 1.0; 

    // =========================================================
    // 2. ESO (扩张状态观测器) 更新
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
    // 3. 全局路径定位 & 安全与终点检查
    // =========================================================
    int closest_idx = get_closest_path_index(gx, gy);
    
    double path_x = full_path_.poses[closest_idx].pose.position.x;
    double path_y = full_path_.poses[closest_idx].pose.position.y;
    double current_cte = std::hypot(gx - path_x, gy - path_y);
    
    bool is_emergency = false;
    if (current_cte > road_half_width * safe_ratio) {
        is_emergency = true;
        target_ref_vel = recovery_vel; 
    }

    // 获取障碍物点云
    std::vector<Point> all_obs_pts = get_all_obstacle_points(); 
    
    // [Performance] 仅在非性能模式下计算 Goal Point，因为这纯粹为了可视化
    double goal_x = 0.0, goal_y = 0.0;
    if (!perf_mode) {
        int target_idx = closest_idx;
        double lookahead_dist_viz = std::max(5.0, target_ref_vel * N_HORIZON * DT * 1.2); 
        double dist_acc = 0;
        while(target_idx < (int)full_path_.poses.size() - 1 && dist_acc < lookahead_dist_viz) {
            double d = std::hypot(
                full_path_.poses[target_idx+1].pose.position.x - full_path_.poses[target_idx].pose.position.x,
                full_path_.poses[target_idx+1].pose.position.y - full_path_.poses[target_idx].pose.position.y);
            dist_acc += d; 
            target_idx++;
        }
        goal_x = full_path_.poses[target_idx].pose.position.x;
        goal_y = full_path_.poses[target_idx].pose.position.y;
    }

    // =========================================================
    // 4. Lattice Planner 局部规划
    // =========================================================
    auto start_plan = std::chrono::high_resolution_clock::now();
    
    std::vector<Point> guide_path; 
    CandidatePath best_lattice_path;
    bool plan_success = false;
    LatticePlanner::Config l_cfg;

    {
        l_cfg.path_resolution = this->get_parameter("lattice.path_resolution").as_double();
        l_cfg.lookahead_dist = this->get_parameter("lattice.lookahead_dist").as_double();
        l_cfg.num_samples = this->get_parameter("lattice.num_samples").as_int();
        l_cfg.sample_width = this->get_parameter("lattice.sample_width").as_double();
        l_cfg.max_width = this->get_parameter("lattice.max_width").as_double();
        l_cfg.collision_radius = search_margin; 
        
        l_cfg.w_collision = this->get_parameter("lattice.weight_collision").as_double();
        l_cfg.w_offset = this->get_parameter("lattice.weight_offset").as_double();
        l_cfg.w_consistency = this->get_parameter("lattice.weight_consistency").as_double();
        
        lattice_planner_->update_config(l_cfg);

        auto lattice_global_path = convert_global_path_to_lattice(full_path_);
        auto lattice_obs = convert_obs_to_lattice(all_obs_pts);
        LatticePlanner::RobotState robot_state = {gx, gy, gyaw, gv};

        best_lattice_path = lattice_planner_->plan(robot_state, lattice_global_path, lattice_obs);

        if (!best_lattice_path.points.empty()) {
            for(const auto& p : best_lattice_path.points) {
                guide_path.push_back({p.x, p.y});
            }
            plan_success = true;
        } else {
            for(int i=0; i <= N_HORIZON; ++i) {
                int idx = std::min(closest_idx + i, (int)full_path_.poses.size()-1);
                guide_path.push_back({full_path_.poses[idx].pose.position.x, full_path_.poses[idx].pose.position.y});
            }
        }
    }
    
    auto end_plan = std::chrono::high_resolution_clock::now();
    double t_plan = std::chrono::duration<double, std::milli>(end_plan - start_plan).count();

    // =========================================================
    // 5. SFC (安全飞行走廊) 生成
    // =========================================================
    std::vector<SFC_Constraint> sfc_corridor;
    if (enable_sfc && !guide_path.empty()) {
        std::vector<Eigen::Vector2d> eigen_path;
        eigen_path.reserve(guide_path.size());
        for(const auto& p : guide_path) eigen_path.emplace_back(p.x, p.y);

        std::vector<Eigen::Vector2d> eigen_obs;
        eigen_obs.reserve(all_obs_pts.size() + 200); 
        for(const auto& p : all_obs_pts) eigen_obs.emplace_back(p.x, p.y);

        if (use_virtual_walls) {
            int vw_scan_start = std::max(0, closest_idx - 10);
            int vw_scan_end = std::min((int)full_path_.poses.size() - 1, closest_idx + N_HORIZON + 50);
            for(int k = vw_scan_start; k < vw_scan_end; k += 2) {
                double wx = full_path_.poses[k].pose.position.x;
                double wy = full_path_.poses[k].pose.position.y;
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
                eigen_obs.emplace_back(wx + wnx * road_half_width, wy + wny * road_half_width);
                eigen_obs.emplace_back(wx - wnx * road_half_width, wy - wny * road_half_width);
            }
        }
        sfc_corridor = sfc_gen_->generate_corridor(eigen_path, eigen_obs);
    }

    // =========================================================
    // 6. 曲率计算与动态步长 (Adaptive Step Size) - 使用 Lattice Guide Path
    // =========================================================
    double curvature_threshold = this->get_parameter("nmpc_config.curvature_threshold").as_double();
    double global_curve_weight = this->get_parameter("nmpc_config.global_curvature_weight").as_double();

    // 6.1 Lattice路径曲率 (前瞻未来路段)
    double global_curve_sum = 0.0;
    std::vector<geometry_msgs::msg::Point> curve_viz_pts;
    std::vector<std_msgs::msg::ColorRGBA> curve_viz_cols;

    // 使用 guide_path 的分辨率 (lattice.path_resolution = 0.2)
    double lattice_path_res = this->get_parameter("lattice.path_resolution").as_double();
    double stride_dist = 0.5;
    int stride_step = std::max(1, (int)(stride_dist / lattice_path_res));

    // 找到 guide_path 上距离机器人最近的点索引
    int guide_closest_idx = 0;
    if (!guide_path.empty()) {
        double min_dist = 1e9;
        for (size_t i = 0; i < guide_path.size(); ++i) {
            double d = std::hypot(guide_path[i].x - gx, guide_path[i].y - gy);
            if (d < min_dist) {
                min_dist = d;
                guide_closest_idx = i;
            }
        }
    }

    double predicted_path_len = target_ref_vel * N_HORIZON * DT;
    double check_dist = std::max(15.0, predicted_path_len * 1.2);
    int max_check_steps = static_cast<int>(check_dist / lattice_path_res);

    // 使用 guide_path 而不是 full_path_
    if (!guide_path.empty()) {
        int end_scan_idx = std::min((int)guide_path.size() - 1 - stride_step, guide_closest_idx + max_check_steps);

        if (end_scan_idx > guide_closest_idx) {
            for (int k = guide_closest_idx; k < end_scan_idx; k += stride_step) {
                // [Performance] 可视化点填充只在非性能模式下进行
                if (!perf_mode) {
                    geometry_msgs::msg::Point pt;
                    pt.x = guide_path[k].x; pt.y = guide_path[k].y; pt.z = 0.2;
                    curve_viz_pts.push_back(pt);
                }

                double diff = 0.0;
                // 核心逻辑计算必须保留
                if (k > guide_closest_idx) {
                    double dx1 = guide_path[k + stride_step].x - guide_path[k].x;
                    double dy1 = guide_path[k + stride_step].y - guide_path[k].y;
                    double yaw1 = std::atan2(dy1, dx1);

                    double dx0 = guide_path[k].x - guide_path[k - stride_step].x;
                    double dy0 = guide_path[k].y - guide_path[k - stride_step].y;
                    double yaw0 = std::atan2(dy0, dx0);
                    diff = std::abs(unwrap_yaw(yaw1, yaw0) - yaw0);
                    global_curve_sum += diff;
                }

                // [Performance] 颜色计算
                if (!perf_mode) {
                    std_msgs::msg::ColorRGBA col; col.a = 1.0;
                    double ratio = std::clamp(diff / 0.3, 0.0, 1.0);
                    col.r = ratio; col.g = 1.0 - ratio; col.b = 0.0;
                    curve_viz_cols.push_back(col);
                }
            }
        }
    }
    global_curve_sum *= global_curve_weight;

    // 6.2 动态步长计算 (仅使用全局曲率)
    double total_curve = global_curve_sum;
    double curve_ratio = std::clamp(total_curve / curvature_threshold, 0.0, 1.0);
    double max_step = 0.35;
    double min_step = 0.2; 
    double dynamic_step_dist = max_step - curve_ratio * (max_step - min_step); 
    if (dynamic_step_dist < 0.1) dynamic_step_dist = 0.1; 
    
    // =========================================================
    // 7. NMPC 构建 & 求解 (带插值 Interpolation)
    // =========================================================
    double cos_theta = std::cos(gyaw);
    double sin_theta = std::sin(gyaw);
    
    auto transform_to_local = [&](double x, double y) -> std::pair<double, double> {
        double dx = x - gx; double dy = y - gy;
        return {dx * cos_theta + dy * sin_theta, -dx * sin_theta + dy * cos_theta};
    };

    std::vector<double> guide_path_cum_dists;
    if (!guide_path.empty()) {
        guide_path_cum_dists.reserve(guide_path.size());
        guide_path_cum_dists.push_back(0.0);
        for (size_t i = 0; i < guide_path.size() - 1; ++i) {
            double d = std::hypot(guide_path[i+1].x - guide_path[i].x, guide_path[i+1].y - guide_path[i].y);
            guide_path_cum_dists.push_back(guide_path_cum_dists.back() + d);
        }
    }

    auto conf = racing_control_hyperplane_acados_get_nlp_config(capsule_);
    auto dims = racing_control_hyperplane_acados_get_nlp_dims(capsule_);
    auto in = racing_control_hyperplane_acados_get_nlp_in(capsule_);
    auto out = racing_control_hyperplane_acados_get_nlp_out(capsule_);

    double x0_local[5] = {0.0, 0.0, 0.0, gv, gw};
    ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "lbx", x0_local);
    ocp_nlp_constraints_model_set(conf, dims, in, out, 0, "ubx", x0_local);

    double last_yaw_ref_global = gyaw; 
    std::vector<std::pair<double, double>> target_path_viz; 
    double current_ref_progress = 0.0;
    size_t last_search_idx = 0; 

    for (int i = 0; i <= N_HORIZON; i++) {
        double ref_tx_g, ref_ty_g, ref_yaw_g;

        if (guide_path.empty()) {
            ref_tx_g = gx; ref_ty_g = gy; ref_yaw_g = gyaw;
        } else {
            size_t idx = last_search_idx;
            while (idx < guide_path_cum_dists.size() - 1 && guide_path_cum_dists[idx+1] < current_ref_progress) {
                idx++;
            }
            last_search_idx = idx; 

            if (idx >= guide_path.size() - 1) {
                ref_tx_g = guide_path.back().x;
                ref_ty_g = guide_path.back().y;
                if (guide_path.size() > 1) {
                    double dx = guide_path.back().x - guide_path[guide_path.size()-2].x;
                    double dy = guide_path.back().y - guide_path[guide_path.size()-2].y;
                    ref_yaw_g = std::atan2(dy, dx);
                } else {
                    ref_yaw_g = last_yaw_ref_global;
                }
            } else {
                double seg_len = guide_path_cum_dists[idx+1] - guide_path_cum_dists[idx];
                double ratio = 0.0;
                if (seg_len > 1e-4) {
                    ratio = (current_ref_progress - guide_path_cum_dists[idx]) / seg_len;
                }
                const auto& p0 = guide_path[idx];
                const auto& p1 = guide_path[idx+1];
                ref_tx_g = p0.x + ratio * (p1.x - p0.x);
                ref_ty_g = p0.y + ratio * (p1.y - p0.y);
                
                double yaw0;
                if (idx < guide_path.size() - 1) {
                    yaw0 = std::atan2(guide_path[idx+1].y - guide_path[idx].y, guide_path[idx+1].x - guide_path[idx].x);
                } else {
                    yaw0 = last_yaw_ref_global;
                }
                ref_yaw_g = yaw0; 
            }
        }
        
        current_ref_progress += dynamic_step_dist;

        auto ref_local = transform_to_local(ref_tx_g, ref_ty_g);
        double smooth_yaw_global = unwrap_yaw(ref_yaw_g, last_yaw_ref_global);
        last_yaw_ref_global = smooth_yaw_global;
        double yaw_ref_local = smooth_yaw_global - gyaw; 

        double yref[7] = {ref_local.first, ref_local.second, target_ref_vel, yaw_ref_local, 0, 0, 0};
        ocp_nlp_cost_model_set(conf, dims, in, i, "yref", yref);
        
        // [Performance] 仅在非性能模式下保存柱子可视化数据
        if (!perf_mode) {
            target_path_viz.push_back({ref_tx_g, ref_ty_g}); 
        }

        std::vector<ObstacleParam> step_obs; 
        if (!sfc_corridor.empty()) {
            int sfc_idx = std::min((int)last_search_idx, (int)sfc_corridor.size() - 1);
            const auto& box = sfc_corridor[sfc_idx];
            for(int row = 0; row < 4; ++row) {
                ObstacleParam wall;
                double Ax_w = box.A(row, 0);
                double Ay_w = box.A(row, 1);
                double b_w  = box.b(row);
                double Ax_l = Ax_w * cos_theta + Ay_w * sin_theta;
                double Ay_l = Ax_w * (-sin_theta) + Ay_w * cos_theta;
                double b_l = b_w - (Ax_w * gx + Ay_w * gy);
                wall.ox = 0.0; wall.oy = 0.0; wall.r  = -b_l;      
                wall.nx = -Ax_l; wall.ny = -Ay_l;   
                step_obs.push_back(wall); 
            }
        } 
        
        int n_obs_solver = 4; 
        while(step_obs.size() < (size_t)n_obs_solver) {
             ObstacleParam dummy;
             dummy.ox=0; dummy.oy=0; dummy.r=-1e9; dummy.nx=0; dummy.ny=0;
             step_obs.push_back(dummy);
        }
        double p_array[N_PARAM];
        HyperplaneUtil::pack_params(p_array, step_obs, n_obs_solver);
        racing_control_hyperplane_acados_update_params(capsule_, i, p_array, N_PARAM);
    }

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
    auto solver = racing_control_hyperplane_acados_get_nlp_solver(capsule_);
    int qp_iter_sum = 0;
    ocp_nlp_get(solver, "qp_iter", &qp_iter_sum);

    auto end_solve = std::chrono::high_resolution_clock::now();
    double t_total = std::chrono::duration<double, std::milli>(end_solve - start_total).count();
    
    // [Performance] 时间发布也可以视为非必要，如果为了debug保留也可以，此处暂且保留，因为数据量很小
    if (!perf_mode) {
        std_msgs::msg::Float32 time_msg; time_msg.data = t_total;
        pub_solve_time_->publish(time_msg);
    }

    // =========================================================
    // 8. 输出与可视化
    // =========================================================
    char log_buf[512]; 
    std::string plan_status = plan_success ? "Lattice" : "Fallback";
    
    if (status == 0) {
        // [Performance] 关闭繁重的字符串格式化和 INFO 日志

        // 获取求解出的控制量（加速度和角速度）
        double u0[2];
        ocp_nlp_out_get(conf, dims, out, 0, "u", u0);
        double u_acc_raw = u0[0];   // 加速度（原始）
        double u_w = u0[1];         // 角速度（直接控制）

        // ESO 补偿后的加速度
        double b0_lin = this->get_parameter("eso.b0_linear").as_double();
        double u_acc_comp = u_acc_raw - dist_acc_lin / b0_lin;

        // 仅在非性能模式下输出详细日志
        if (!perf_mode) {
            snprintf(log_buf, sizeof(log_buf),
                "TIME[Tot:%.1f Plan:%.1f] Stat:%s Iter:%d V:%.2f W:%.2f "
                "Acc_Raw:%.3f Acc_Comp:%.3f ESO:%.3f CTE:%.2f %s",
                t_total,
                t_plan,
                plan_status.c_str(),
                qp_iter_sum,
                gv,
                gw,
                u_acc_raw,      // 补偿前
                u_acc_comp,     // 补偿后
                dist_acc_lin,   // ESO估计的扰动
                current_cte,
                is_emergency ? "[RECOVERY]" : "");
            RCLCPP_INFO(this->get_logger(), "%s", log_buf);
        }
        

        publish_command(conf, dims, out, dist_acc_lin, dist_acc_ang);
        
        // [Performance] 如果开启性能模式，直接跳过所有可视化计算
        if (!perf_mode) {
            std::vector<std::vector<double>> global_pred_traj;
            // 预测轨迹的三角函数计算也很耗时
            for (int i = 0; i <= N_HORIZON; i++) {
                double x[5]; ocp_nlp_out_get(conf, dims, out, i, "x", x);
                double gx_pred = x[0] * cos_theta - x[1] * sin_theta + gx;
                double gy_pred = x[0] * sin_theta + x[1] * cos_theta + gy;
                global_pred_traj.push_back({gx_pred, gy_pred});
            }
            
            std::vector<std::pair<double, double>> empty_astar_viz; 
            std::vector<NmpcVisualizer::VizObs> empty_cons_viz; 

            render_visualization(
                global_pred_traj, 
                empty_cons_viz, 
                target_path_viz, 
                empty_astar_viz, 
                goal_x, goal_y, 
                curve_viz_pts, 
                curve_viz_cols, 
                sfc_corridor,
                lattice_planner_->get_last_candidates(),
                best_lattice_path 
            );
        }

    } else {
        // ERROR 日志始终保留
        snprintf(log_buf, sizeof(log_buf), "QP FAIL: %d", status);
        RCLCPP_WARN(this->get_logger(), "%s", log_buf);
        publish_emergency_brake();
    }
}

// 辅助函数
std::vector<Point2D> NmpcTrackerNode::convert_global_path_to_lattice(const nav_msgs::msg::Path& path) {
    std::vector<Point2D> out;
    if (path.poses.empty()) return out;
    out.reserve(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
        Point2D p;
        p.x = path.poses[i].pose.position.x;
        p.y = path.poses[i].pose.position.y;
        if (i < path.poses.size() - 1) {
            double dx = path.poses[i+1].pose.position.x - p.x;
            double dy = path.poses[i+1].pose.position.y - p.y;
            p.yaw = std::atan2(dy, dx);
        } else if (i > 0) {
            p.yaw = out.back().yaw;
        } else {
            p.yaw = 0.0;
        }
        out.push_back(p);
    }
    return out;
}

std::vector<Point2D> NmpcTrackerNode::convert_obs_to_lattice(const std::vector<Point>& obs) {
    std::vector<Point2D> out;
    out.reserve(obs.size());
    for(const auto& p : obs) out.push_back({p.x, p.y, 0.0});
    return out;
}

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

    // 控制量: [ax, w] - 加速度 + 角速度（直接控制）
    double u_acc_comp = u0[0] - dist_lin / b0_lin;
    double w_cmd_raw = u0[1];
    // 角速度ESO补偿
    double w_cmd = w_cmd_raw - dist_ang / b0_ang;

    // 计算角加速度（用于ESO更新）
    double w_acc = (w_cmd - last_w_cmd_) / DT;

    // 线速度通过积分加速度得到
    double v_cmd = cur_x_[3] + u_acc_comp * DT;

    cmd.linear.x = std::clamp(v_cmd, min_v, max_v);
    cmd.angular.z = std::clamp(w_cmd, -2.5, 2.5);

    last_cmd_acc_ = u_acc_comp;
    last_cmd_w_acc_ = w_acc;
    last_w_cmd_ = w_cmd;

    pub_cmd_->publish(cmd);

    // 发布 ESO 诊断数据 (用于外部分析)
    // 数据格式: [u_acc_raw, u_acc_comp, dist_lin, v_cmd, current_velocity]
    std_msgs::msg::Float32MultiArray eso_diag;
    eso_diag.data.push_back(u0[0]);       // 原始加速度
    eso_diag.data.push_back(u_acc_comp);  // 补偿后加速度
    eso_diag.data.push_back(dist_lin);     // ESO估计扰动
    eso_diag.data.push_back(v_cmd);        // 命令速度
    eso_diag.data.push_back(cur_x_[3]);     // 当前实际速度
    pub_eso_diag_->publish(eso_diag);
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
    const std::vector<SFC_Constraint>& sfc_corridor,
    const std::vector<CandidatePath>& lattice_candidates,
    const CandidatePath& best_lattice_path) 
{
    double fov_deg = this->get_parameter("perception.fov_half_angle_deg").as_double();
    std::vector<double> robot_state = {cur_x_[0], cur_x_[1], cur_x_[2]};
    
    // 1. 调用 visualizer，传入空的 target_path 以禁用默认可视化 (flat disks)
    std::vector<std::pair<double, double>> empty_target_viz;
    auto markers = visualizer_->create_viz_packet(
        this->get_clock()->now(), pred_traj, constraint_viz, empty_target_viz, false, 
        robot_state, fov_deg * M_PI / 180.0,
        curve_pts, curve_cols,
        sfc_corridor 
    );

    rclcpp::Time now = this->get_clock()->now();

    // 2. [关键修改] 手动绘制 Target Path 为柱子 (Pillars)
    int id_offset = 20000;
    for (size_t i = 0; i < target_path_viz.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now;
        m.ns = "ref_pillars";
        m.id = id_offset + i;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        
        m.pose.position.x = target_path_viz[i].first;
        m.pose.position.y = target_path_viz[i].second;
        double height = 1.2; 
        m.pose.position.z = height / 2.0; 
        
        m.scale.x = 0.15; // 柱子直径
        m.scale.y = 0.15;
        m.scale.z = height; 

        // 蓝色/紫色系
        m.color.r = 0.2; m.color.g = 0.2; m.color.b = 1.0; m.color.a = 0.8; 
        
        markers.markers.push_back(m);
    }

    // 绘制所有候选路径 (青色半透明线)
    int id_counter = 40000;
    for (const auto& candidate : lattice_candidates) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map"; 
        m.header.stamp = now;
        m.ns = "lattice_candidates"; 
        m.id = id_counter++; 
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.02; 
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.3;
        
        if (candidate.collision) {
             m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.2;
        }

        for (const auto& pt : candidate.points) {
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = 0.05;
            m.points.push_back(p);
        }
        markers.markers.push_back(m);
    }

    // 绘制最佳路径 (绿色粗线)
    if (!best_lattice_path.points.empty()) {
        visualization_msgs::msg::Marker m_best;
        m_best.header.frame_id = "map";
        m_best.header.stamp = now;
        m_best.ns = "lattice_best";
        m_best.id = 50000;
        m_best.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m_best.action = visualization_msgs::msg::Marker::ADD;
        m_best.pose.orientation.w = 1.0;
        m_best.scale.x = 0.15; 
        m_best.color.r = 0.0; m_best.color.g = 1.0; m_best.color.b = 0.0; m_best.color.a = 1.0;

        for (const auto& pt : best_lattice_path.points) {
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; 
            p.z = 0.10; 
            m_best.points.push_back(p);
        }
        markers.markers.push_back(m_best);
    }

    visualization_msgs::msg::Marker goal_mk;
    goal_mk.header.frame_id = "map"; 
    goal_mk.header.stamp = now;
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

    pub_viz_->publish(markers);
}

void NmpcTrackerNode::setup_ros_interfaces() {
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            cur_x_[0] = msg->pose.pose.position.x;
            cur_x_[1] = msg->pose.pose.position.y;
            cur_x_[2] = tf2::getYaw(msg->pose.pose.orientation);
            // std::cout << "yaw:" << cur_x_[2] << std::endl;
            // 添加高斯噪声模拟速度观测误差 (0.5m/s 标准差)
            double vel_noise = vel_noise_dist_(vel_noise_generator_);
            if(cur_x_[3]>VELOCITY_X_BIAS)
            cur_x_[3] = msg->twist.twist.linear.x + vel_noise + VELOCITY_X_BIAS;
            else
            cur_x_[3] = msg->twist.twist.linear.x + vel_noise;
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
    pub_eso_diag_ = create_publisher<std_msgs::msg::Float32MultiArray>("/nmpc/eso_diag", 10);
    
    int ms = this->get_parameter("nmpc_config.control_loop_ms").as_int();
    timer_ = create_wall_timer(std::chrono::milliseconds(ms), std::bind(&NmpcTrackerNode::solve_cycle, this));
}