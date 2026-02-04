#ifndef NMPC_VISUALIZER_HPP
#define NMPC_VISUALIZER_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include "hyperplane_util.hpp"

class NmpcVisualizer {
public:
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

    MarkerArray create_viz_packet(
        const rclcpp::Time& stamp,
        const std::vector<std::vector<double>>& pred_traj,
        const std::vector<std::pair<int, ObstacleParam>>& obs_viz_data,
        const std::vector<std::pair<double, double>>& ref_points) 
    {
        MarkerArray array;
        Marker del; del.action = Marker::DELETEALL;
        array.markers.push_back(del);

        // 预测轨迹
        for (size_t i = 0; i < pred_traj.size(); ++i) {
            array.markers.push_back(make_pillar(i + 200, pred_traj[i][0], pred_traj[i][1], 0.08, 0.3, 0, 1, 0, 1.0, stamp));
        }

        // 参考路径点
        for (size_t i = 0; i < ref_points.size(); ++i) {
            array.markers.push_back(make_pillar(i + 100, ref_points[i].first, ref_points[i].second, 0.05, 0.2, 0, 0, 1, 0.8, stamp));
        }

        // 拟合的超平面
        for (auto const& [id_base, obs] : obs_viz_data) {
            array.markers.push_back(make_pillar(id_base, obs.ox, obs.oy, obs.r, 0.1, 1, 1, 0, 0.2, stamp));
            array.markers.push_back(make_line(id_base + 1000, obs, stamp));
        }
        return array;
    }

private:
    Marker make_pillar(int id, double x, double y, double r, double h, float red, float green, float blue, float alpha, rclcpp::Time t) {
        Marker m;
        m.header.frame_id = "map"; m.header.stamp = t;
        m.id = id; m.type = Marker::CYLINDER;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = h/2.0;
        m.scale.x = m.scale.y = r * 2.0; m.scale.z = h;
        m.color.r = red; m.color.g = green; m.color.b = blue; m.color.a = alpha;
        return m;
    }

    Marker make_line(int id, const ObstacleParam& obs, rclcpp::Time t) {
        Marker m;
        m.header.frame_id = "map"; m.header.stamp = t;
        m.id = id; m.type = Marker::LINE_LIST;
        m.scale.x = 0.03; m.color.r = 1.0; m.color.a = 0.6;
        double px = obs.ox + obs.nx * obs.r, py = obs.oy + obs.ny * obs.r;
        double tx = -obs.ny, ty = obs.nx;
        geometry_msgs::msg::Point p1, p2;
        p1.x = px - tx * 1.5; p1.y = py - ty * 1.5;
        p2.x = px + tx * 1.5; p2.y = py + ty * 1.5;
        m.points.push_back(p1); m.points.push_back(p2);
        return m;
    }
};
#endif