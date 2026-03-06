#ifndef NMPC_TRACKER_NODE_HPP
#define NMPC_TRACKER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <string>
#include <random>
#include <Eigen/Dense>

extern "C" {
    #include "acados_solver_racing_control_hyperplane.h"
}

#include "dbscan.hpp"
#include "hyperplane_util.hpp"
#include "nmpc_visualizer.hpp"
#include "astar_planner.hpp"
#include "eso.hpp"
#include "sfc_generator.hpp"
#include "lattice_planner.hpp" 

class NmpcTrackerNode : public rclcpp::Node {
public:
    NmpcTrackerNode();
    ~NmpcTrackerNode();

private:
    void solve_cycle();

    // 辅助函数
    double unwrap_yaw(double target_yaw, double current_yaw);
    int get_closest_path_index(double x, double y);
    bool is_in_fov(double ox, double oy);
    std::vector<Point> get_all_obstacle_points();
    bool check_path_collision(const std::vector<Point>& path, double margin);
    std::vector<Point> prune_path_by_distance(const std::vector<Point>& path, double curr_x, double curr_y, double lookahead_dist);
    void apply_normal_smoothing(ObstacleParam& p, const std::string& key, double alpha, double max_rot_rad);
    void setup_ros_interfaces();
    void publish_command(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out, double dist_lin, double dist_ang);
    void publish_emergency_brake();

    std::vector<Point2D> convert_global_path_to_lattice(const nav_msgs::msg::Path& path);
    std::vector<Point2D> convert_obs_to_lattice(const std::vector<Point>& obs);

    // 可视化函数，包含 Lattice 的 Candidates 和 Best Path
    void render_visualization(const std::vector<std::vector<double>>& pred_traj, 
                              const std::vector<NmpcVisualizer::VizObs>& constraint_viz, 
                              const std::vector<std::pair<double, double>>& target_path_viz,
                              const std::vector<std::pair<double, double>>& astar_guide_viz,
                              double goal_x, double goal_y,
                              const std::vector<geometry_msgs::msg::Point>& curve_pts,
                              const std::vector<std_msgs::msg::ColorRGBA>& curve_cols,
                              const std::vector<SFC_Constraint>& sfc_corridor,
                              const std::vector<CandidatePath>& lattice_candidates,
                              const CandidatePath& best_lattice_path);

    racing_control_hyperplane_solver_capsule* capsule_;
    std::unique_ptr<NmpcVisualizer> visualizer_;
    std::unique_ptr<DBSCAN> cluster_worker_;
    std::unique_ptr<AStarPlanner> astar_planner_; // 保留指针但暂不使用
    std::unique_ptr<SFCGenerator> sfc_gen_;
    std::unique_ptr<ESO> linear_eso_;
    std::unique_ptr<ESO> angular_eso_;
    
    std::unique_ptr<LatticePlanner> lattice_planner_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_solve_time_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_eso_diag_;  // ESO诊断数据
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex cluster_mutex_;
    std::map<int, std::vector<Point>> current_clusters_;
    std::map<std::string, std::pair<double, double>> prev_normal_map_;
    std::vector<Point> last_guide_path_;
    double last_cmd_acc_ = 0.0;
    double last_cmd_w_acc_ = 0.0;
    double last_w_cmd_ = 0.0;  // 上一时刻角速度命令（用于计算角加速度）
    double last_v_cmd_ = 0.0;  // 上一时刻线速度命令（用于计算线加速度） 
    double cur_x_[3];  // 状态量: [x, y, theta]
    double cur_velocity_ = 0.0;  // 当前线速度 (从 odom 获取)
    double cur_angular_velocity_ = 0.0;  // 当前角速度 (从 odom 获取)
    nav_msgs::msg::Path full_path_;
    bool odom_ok_ = false;
    bool path_ok_ = false;
    bool is_first_run_ = true;
    double last_filtered_local_curve_ = 0.0;

    // 速度观测噪声随机数生成器
    std::mt19937 vel_noise_generator_;
    std::normal_distribution<double> vel_noise_dist_;
};
#endif