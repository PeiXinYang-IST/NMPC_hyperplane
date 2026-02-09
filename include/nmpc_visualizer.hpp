#ifndef NMPC_VISUALIZER_HPP
#define NMPC_VISUALIZER_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "hyperplane_util.hpp"

class NmpcVisualizer {
public:
    struct VizObs {
        int id;
        ObstacleParam param; 
        bool is_active;
        double alpha = 0.8; 
    };

    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

    // [修改] 参数列表增加了 robot_state (x, y, yaw) 和 fov_info
    MarkerArray create_viz_packet(
        const rclcpp::Time& stamp,
        const std::vector<std::vector<double>>& pred_traj,
        const std::vector<VizObs>& constraint_tunnel, 
        const std::vector<std::pair<double, double>>& target_path_viz,
        bool is_astar_active,
        const std::vector<double>& robot_state, // [x, y, yaw]
        double fov_half_rad,                    // 半视场角 (弧度)
        double fov_dist = 10.0                  // 视场距离 (米)
    ) 
    {
        MarkerArray array;
        Marker del; del.action = Marker::DELETEALL;
        del.header.frame_id = "map";
        array.markers.push_back(del);

        array.markers.push_back(make_fov_marker(999, robot_state, fov_half_rad, fov_dist, stamp));

        for (size_t i = 0; i < pred_traj.size(); ++i) 
            array.markers.push_back(make_marker(i + 1000, pred_traj[i][0], pred_traj[i][1], 0.1, 1.0, 0.0, 0.0, 0.8, stamp));
        
        for (size_t i = 0; i < target_path_viz.size(); ++i) {
            float r=0, g=0, b=1.0, radius=0.2;
            if (is_astar_active) { r=0.0; g=1.0; b=1.0; radius=0.06; } // 青色
            array.markers.push_back(make_marker(i + 2000, target_path_viz[i].first, target_path_viz[i].second, radius, r, g, b, 0.6, stamp));
        }

        for (const auto& viz : constraint_tunnel) {
            array.markers.push_back(make_marker(viz.id + 10000, viz.param.ox, viz.param.oy, viz.param.r, 1.0, 0.6, 0.0, viz.alpha * 0.4, stamp));
            array.markers.push_back(make_line(viz, stamp, viz.id + 20000));
        }
        
        return array;
    }

private:
    Marker make_fov_marker(int id, const std::vector<double>& state, double half_fov, double dist, rclcpp::Time t) {
        Marker m;
        m.header.frame_id = "map"; m.header.stamp = t;
        m.id = id; m.type = Marker::LINE_STRIP; // 使用线段连接
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.05; // 线宽
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8; // 黄色

        double x = state[0];
        double y = state[1];
        double yaw = state[2];

        geometry_msgs::msg::Point p_center, p_left, p_right;
        p_center.x = x; p_center.y = y; p_center.z = 0.1;

        p_left.x = x + dist * cos(yaw + half_fov);
        p_left.y = y + dist * sin(yaw + half_fov);
        p_left.z = 0.1;

        p_right.x = x + dist * cos(yaw - half_fov);
        p_right.y = y + dist * sin(yaw - half_fov);
        p_right.z = 0.1;

        m.points.push_back(p_center);
        m.points.push_back(p_left);
        m.points.push_back(p_right);
        m.points.push_back(p_center);

        return m;
    }

    Marker make_marker(int id, double x, double y, double r, float rd, float gn, float bl, float al, rclcpp::Time t) {
        Marker m;
        m.header.frame_id = "map"; m.header.stamp = t;
        m.id = id; m.type = Marker::CYLINDER;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.05;
        m.scale.x = m.scale.y = r * 2.0; m.scale.z = 0.02; 
        m.color.r = rd; m.color.g = gn; m.color.b = bl; m.color.a = al;
        m.pose.orientation.w = 1.0;
        return m;
    }
    
    Marker make_line(const VizObs& viz, rclcpp::Time t, int id) {
        Marker m;
        m.header.frame_id = "map"; m.header.stamp = t;
        m.id = id; m.type = Marker::LINE_LIST;
        m.scale.x = 0.03; 
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = viz.alpha;

        double px = viz.param.ox + viz.param.nx * viz.param.r;
        double py = viz.param.oy + viz.param.ny * viz.param.r;
        double tx = -viz.param.ny; double ty = viz.param.nx;

        geometry_msgs::msg::Point p1, p2;
        double len = 1.5; 
        p1.x = px - tx * len; p1.y = py - ty * len; p1.z = 0.05;
        p2.x = px + tx * len; p2.y = py + ty * len; p2.z = 0.05;
        m.points.push_back(p1); m.points.push_back(p2);
        m.pose.orientation.w = 1.0;
        return m;
    }
};
#endif