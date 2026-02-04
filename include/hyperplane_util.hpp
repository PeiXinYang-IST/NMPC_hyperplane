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
    static ObstacleParam fit_obstacle(const std::vector<Point>& cluster_pts, double veh_x, double veh_y) {
        if (cluster_pts.empty()) return {-100.0, -100.0, 0.0, 1.0, 0.0};

        double sx = 0, sy = 0;
        for (auto& p : cluster_pts) { sx += p.x; sy += p.y; }
        double cx = sx / cluster_pts.size();
        double cy = sy / cluster_pts.size();

        double max_r_sq = 0;
        for (auto& p : cluster_pts) {
            double d = std::pow(p.x - cx, 2) + std::pow(p.y - cy, 2);
            if (d > max_r_sq) max_r_sq = d;
        }

        double dx = veh_x - cx;
        double dy = veh_y - cy;
        double norm = std::hypot(dx, dy);

        // 增加 0.2m 安全裕度
        return {cx, cy, std::sqrt(max_r_sq) + 0.3, (norm > 0.01 ? dx/norm : 1.0), (norm > 0.01 ? dy/norm : 0.0)};
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
                p_array[k * 5 + 0] = -100.0; // 放置在远端不生效
                p_array[k * 5 + 1] = 0.0;
                p_array[k * 5 + 2] = 0.1;
                p_array[k * 5 + 3] = 1.0;
                p_array[k * 5 + 4] = 0.0;
            }
        }
    }
};
#endif