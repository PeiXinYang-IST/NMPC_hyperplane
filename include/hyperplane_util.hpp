#ifndef HYPERPLANE_UTIL_HPP
#define HYPERPLANE_UTIL_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include "dbscan.hpp"

struct ObstacleParam { 
    double ox, oy, r, nx, ny; 
};

class HyperplaneUtil {
public:
    // 使用“最近点”策略拟合障碍物，支持长墙和任意形状
    static ObstacleParam fit_obstacle(const std::vector<Point>& cluster_pts, double veh_x, double veh_y, double margin) {
        if (cluster_pts.empty()) return {-100.0, -100.0, 0.0, 1.0, 0.0};

        double min_dist_sq = 1e9;
        double closest_x = 0;
        double closest_y = 0;

        // 寻找聚类中距离机器人(或预测点)最近的点
        for (const auto& p : cluster_pts) {
            double dx = veh_x - p.x;
            double dy = veh_y - p.y;
            double d2 = dx*dx + dy*dy;
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                closest_x = p.x;
                closest_y = p.y;
            }
        }

        // 法向量指向机器人
        double dx = veh_x - closest_x;
        double dy = veh_y - closest_y;
        double norm = std::hypot(dx, dy);

        double nx = (norm > 1e-3) ? (dx / norm) : 1.0;
        double ny = (norm > 1e-3) ? (dy / norm) : 0.0;

        // r = margin (将障碍物视为点，把膨胀半径加在约束上)
        return {closest_x, closest_y, margin, nx, ny};
    }

    static void pack_params(double* p_array, const std::vector<ObstacleParam>& obs_list, int max_obs = 5) {
        for (int k = 0; k < max_obs; k++) {
            if (k < (int)obs_list.size()) {
                p_array[k * 5 + 0] = obs_list[k].ox;
                p_array[k * 5 + 1] = obs_list[k].oy;
                p_array[k * 5 + 2] = obs_list[k].r;
                p_array[k * 5 + 3] = obs_list[k].nx;
                p_array[k * 5 + 4] = obs_list[k].ny;
            } else {
                p_array[k * 5 + 0] = -100.0; // 远端无效化
                p_array[k * 5 + 1] = 0.0;
                p_array[k * 5 + 2] = 0.1;
                p_array[k * 5 + 3] = 1.0;
                p_array[k * 5 + 4] = 0.0;
            }
        }
    }
};
#endif