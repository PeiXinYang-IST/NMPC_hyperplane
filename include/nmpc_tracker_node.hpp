#ifndef NMPC_TRACKER_NODE_HPP
#define NMPC_TRACKER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <string>

extern "C" {
    #include "acados_solver_racing_control_hyperplane.h"
}

#include "dbscan.hpp"
#include "hyperplane_util.hpp"
#include "nmpc_visualizer.hpp"
#include "astar_planner.hpp"

class NmpcTrackerNode : public rclcpp::Node {
public:
    NmpcTrackerNode();
    ~NmpcTrackerNode();

private:
    void solve_cycle();
    double unwrap_yaw(double target_yaw, double current_yaw);
    int get_closest_path_index(double x, double y);
    bool is_in_fov(double ox, double oy);

    // 依然保留平滑函数，用于处理额外的动态障碍物（如果有的话）
    void apply_normal_smoothing(ObstacleParam& p, const std::string& key, double alpha, double max_rot_rad);
    void clean_old_normals(const std::vector<int>& active_ids);

    void setup_ros_interfaces();
    void publish_command(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out);
    void publish_emergency_brake();
    void render_visualization(ocp_nlp_config* conf, ocp_nlp_dims* dims, ocp_nlp_out* out, 
                              const std::vector<NmpcVisualizer::VizObs>& constraint_viz, 
                              const std::vector<std::pair<double, double>>& target_path);

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
    bool odom_ok_ = false;
    bool path_ok_ = false;
    bool is_first_run_ = true;
};

#endif