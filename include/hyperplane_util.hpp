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
    // [保持原有的 fit_obstacle 不变...]
    static ObstacleParam fit_obstacle(const std::vector<Point>& cluster_pts, double veh_x, double veh_y, double margin) {
        // ... (原代码保持不变) ...
        return {0,0,0,0,0}; // 仅示意，实际保留原文件内容
    }

    // [新增函数] 获取距离参考点 (ref_x, ref_y) 最近的 max_n 个障碍物点
    static std::vector<ObstacleParam> get_closest_obstacles(
        const std::vector<Point>& pts, 
        double ref_x, double ref_y, 
        double margin, 
        int max_n) 
    {
        if (pts.empty() || max_n <= 0) return {};

        // 1. 计算所有点到参考点的距离平方
        std::vector<std::pair<int, double>> dist_idx;
        dist_idx.reserve(pts.size());
        for(size_t i = 0; i < pts.size(); ++i) {
            double dx = pts[i].x - ref_x;
            double dy = pts[i].y - ref_y;
            dist_idx.push_back({(int)i, dx*dx + dy*dy});
        }

        // 2. 局部排序，取出最近的 max_n 个
        size_t keep_count = std::min((size_t)max_n, dist_idx.size());
        std::partial_sort(dist_idx.begin(), dist_idx.begin() + keep_count, dist_idx.end(),
            [](const std::pair<int, double>& a, const std::pair<int, double>& b){
                return a.second < b.second;
            });

        // 3. 生成超平面参数
        std::vector<ObstacleParam> result;
        result.reserve(keep_count);
        for(size_t k = 0; k < keep_count; ++k) {
            const auto& p = pts[dist_idx[k].first];
            
            // 法向量：从障碍物点指向参考点 (ref_x, ref_y)，即推开车辆的方向
            double dx = ref_x - p.x;
            double dy = ref_y - p.y;
            double norm = std::hypot(dx, dy);
            double nx = 1.0, ny = 0.0;
            if (norm > 1e-4) { nx = dx / norm; ny = dy / norm; }

            result.push_back({p.x, p.y, margin, nx, ny});
        }
        return result;
    }

    // [保持原有的 pack_params 不变...]
    static void pack_params(double* p_array, const std::vector<ObstacleParam>& obs_list, int max_obs = 5) {
        // ... (原代码保持不变) ...
        for (int k = 0; k < max_obs; k++) {
            if (k < (int)obs_list.size()) {
                p_array[k * 5 + 0] = obs_list[k].ox;
                p_array[k * 5 + 1] = obs_list[k].oy;
                p_array[k * 5 + 2] = obs_list[k].r;
                p_array[k * 5 + 3] = obs_list[k].nx;
                p_array[k * 5 + 4] = obs_list[k].ny;
            } else {
                // 填充无效参数
                p_array[k * 5 + 0] = -100.0; 
                p_array[k * 5 + 1] = 0.0;
                p_array[k * 5 + 2] = 0.1;
                p_array[k * 5 + 3] = 1.0;
                p_array[k * 5 + 4] = 0.0;
            }
        }
    }
};
#endif