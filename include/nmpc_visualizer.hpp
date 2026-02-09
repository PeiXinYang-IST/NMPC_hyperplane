#ifndef NMPC_VISUALIZER_HPP
#define NMPC_VISUALIZER_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "hyperplane_util.hpp"
#include "sfc_generator.hpp" 

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

    MarkerArray create_viz_packet(
        const rclcpp::Time& stamp,
        const std::vector<std::vector<double>>& pred_traj,
        const std::vector<VizObs>& constraint_tunnel, 
        const std::vector<std::pair<double, double>>& target_path_viz,
        bool is_astar_active,
        const std::vector<double>& robot_state,
        double fov_half_rad,
        const std::vector<geometry_msgs::msg::Point>& curvature_pts,
        const std::vector<std_msgs::msg::ColorRGBA>& curvature_colors,
        // [新增] 接收 SFC 结果进行可视化
        const std::vector<SFC_Constraint>& sfc_corridor,
        double fov_dist = 10.0
    ) 
    {
        MarkerArray array;
        Marker del; del.action = Marker::DELETEALL;
        del.header.frame_id = "map";
        array.markers.push_back(del);

        // 1. 原有的可视化保持不变
        array.markers.push_back(make_fov_marker(999, robot_state, fov_half_rad, fov_dist, stamp));

        if (!curvature_pts.empty()) {
            array.markers.push_back(make_curvature_debug_marker(888, curvature_pts, curvature_colors, stamp));
        }

        for (size_t i = 0; i < pred_traj.size(); ++i) 
            array.markers.push_back(make_marker(i + 1000, pred_traj[i][0], pred_traj[i][1], 0.1, 1.0, 0.0, 0.0, 0.8, stamp));
        
        for (size_t i = 0; i < target_path_viz.size(); ++i) {
            float r=0, g=0, b=1.0, radius=0.2;
            if (is_astar_active) { r=0.0; g=1.0; b=1.0; radius=0.06; } 
            array.markers.push_back(make_marker(i + 2000, target_path_viz[i].first, target_path_viz[i].second, radius, r, g, b, 0.6, stamp));
        }

        for (const auto& viz : constraint_tunnel) {
            array.markers.push_back(make_marker(viz.id + 10000, viz.param.ox, viz.param.oy, viz.param.r, 1.0, 0.6, 0.0, viz.alpha * 0.4, stamp));
            array.markers.push_back(make_line(viz, stamp, viz.id + 20000));
        }

        // ====================================================
        // [新增] SFC 走廊可视化 (绘制矩形框)
        // ====================================================
        for (size_t i = 0; i < sfc_corridor.size(); ++i) {
            // 降采样显示，避免太密集
            if (i % 2 != 0) continue;

            const auto& box = sfc_corridor[i];
            Marker m;
            m.header.frame_id = "map"; m.header.stamp = stamp;
            m.id = 30000 + i;
            m.type = Marker::LINE_STRIP;
            m.action = Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.05; // 线条粗细

            // 颜色：青色 (Cyan)，半透明，区分于原有的黄色/红色约束
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.5;

            // 绘制矩形 0->1->2->3->0
            for(int k=0; k<4; ++k) {
                geometry_msgs::msg::Point p;
                p.x = box.vertices(k, 0);
                p.y = box.vertices(k, 1);
                p.z = 0.0;
                m.points.push_back(p);
            }
            // 闭合
            geometry_msgs::msg::Point p_start;
            p_start.x = box.vertices(0, 0);
            p_start.y = box.vertices(0, 1);
            p_start.z = 0.0;
            m.points.push_back(p_start);

            array.markers.push_back(m);
        }
        
        return array;
    }

private:
    // ... (保留 make_curvature_debug_marker, make_fov_marker, make_marker, make_line 等原有辅助函数，不做修改) ...
    Marker make_curvature_debug_marker(int id, const std::vector<geometry_msgs::msg::Point>& pts, const std::vector<std_msgs::msg::ColorRGBA>& cols, rclcpp::Time t) {
        Marker m; m.header.frame_id = "map"; m.header.stamp = t; m.id = id; m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
        m.pose.orientation.w = 1.0; m.scale.x = 0.15; m.points = pts; m.colors = cols; m.color.a = 1.0;
        return m;
    }

    Marker make_fov_marker(int id, const std::vector<double>& state, double half_fov, double dist, rclcpp::Time t) {
        Marker m; m.header.frame_id = "map"; m.header.stamp = t; m.id = id; m.type = Marker::LINE_STRIP; 
        m.pose.orientation.w = 1.0; m.scale.x = 0.05; 
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8; 
        double x = state[0]; double y = state[1]; double yaw = state[2];
        geometry_msgs::msg::Point p0, p1, p2;
        p0.x = x; p0.y = y; p0.z = 0.1;
        p1.x = x + dist * cos(yaw + half_fov); p1.y = y + dist * sin(yaw + half_fov); p1.z = 0.1;
        p2.x = x + dist * cos(yaw - half_fov); p2.y = y + dist * sin(yaw - half_fov); p2.z = 0.1;
        m.points = {p0, p1, p2, p0};
        return m;
    }

    Marker make_marker(int id, double x, double y, double r, float rd, float gn, float bl, float al, rclcpp::Time t) {
        Marker m; m.header.frame_id = "map"; m.header.stamp = t; m.id = id; m.type = Marker::CYLINDER;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.05;
        m.scale.x = m.scale.y = r * 2.0; m.scale.z = 0.02; 
        m.color.r = rd; m.color.g = gn; m.color.b = bl; m.color.a = al; m.pose.orientation.w = 1.0;
        return m;
    }
    
    Marker make_line(const VizObs& viz, rclcpp::Time t, int id) {
        Marker m; m.header.frame_id = "map"; m.header.stamp = t; m.id = id; m.type = Marker::LINE_LIST;
        m.scale.x = 0.03; 
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = viz.alpha;
        double px = viz.param.ox + viz.param.nx * viz.param.r; double py = viz.param.oy + viz.param.ny * viz.param.r;
        double tx = -viz.param.ny; double ty = viz.param.nx;
        geometry_msgs::msg::Point p1, p2; double len = 1.5; 
        p1.x = px - tx * len; p1.y = py - ty * len; p1.z = 0.05;
        p2.x = px + tx * len; p2.y = py + ty * len; p2.z = 0.05;
        m.points.push_back(p1); m.points.push_back(p2); m.pose.orientation.w = 1.0;
        return m;
    }
};
#endif