#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <memory>
#include <iostream>
#include "dbscan.hpp" // 需要 Point 定义

struct GridNode {
    int x, y;
    double g, h;
    GridNode* parent = nullptr;
    bool closed = false;
    
    // 优先队列需要 operator> (最小堆)
    bool operator>(const GridNode& other) const { 
        return (g + h) > (other.g + other.h); 
    } 
};

class AStarPlanner {
public:
    struct Config {
        double resolution = 0.2;        // 栅格分辨率
        double margin = 0.5;            // 障碍物膨胀半径
        int grid_padding = 20;
        
        // --- 核心权重参数 ---
        double heuristic_weight = 1.1;      // 启发式权重 (>=1.0 加速搜索)
        
        // 1. 全局路径吸附权重：越大越贴近全局路径
        double reference_cost_weight = 4.0; 
        
        // 2. 转向惩罚权重：越大路径越平直 (抵抗变向)
        double turning_weight = 2.0;        

        // 3. 历史路径迟滞：防止路径在帧与帧之间跳变
        double history_bias_weight = 1.0;   

        // [后端平滑参数] Fem-Smoother
        double smooth_w_data = 0.45;
        double smooth_w_smooth = 0.40;
        double smooth_w_curvature = 0.40; 
    };

    AStarPlanner(Config config) : cfg_(config) {}

    // ========================================================================
    // 后端平滑器 (Fem-Smoother) - 保持不变，这是让路径变圆滑的关键
    // ========================================================================
    static std::vector<Point> smooth_path(const std::vector<Point>& raw_path, 
                                          double w_data = 0.45, 
                                          double w_smooth = 0.35, 
                                          double w_curve = 0.35,
                                          double tolerance = 0.001) 
    {
        if (raw_path.size() < 3) return raw_path;
        std::vector<Point> new_path = raw_path;
        int n = new_path.size();
        
        double path_len = 0;
        for(size_t i=1; i<raw_path.size(); ++i) 
            path_len += std::hypot(raw_path[i].x - raw_path[i-1].x, raw_path[i].y - raw_path[i-1].y);
        int max_iter = std::min(500, std::max(100, (int)(path_len * 20))); 
        
        double change = tolerance;
        int iter = 0;
        while (change >= tolerance && iter++ < max_iter) {
            change = 0.0;
            for (int i = 1; i < n - 1; i++) {
                double x_orig = new_path[i].x; double y_orig = new_path[i].y;

                double grad_data_x = 2.0 * (new_path[i].x - raw_path[i].x);
                double grad_data_y = 2.0 * (new_path[i].y - raw_path[i].y);

                double grad_smooth_x = 2.0 * (2.0 * new_path[i].x - new_path[i-1].x - new_path[i+1].x);
                double grad_smooth_y = 2.0 * (2.0 * new_path[i].y - new_path[i-1].y - new_path[i+1].y);

                double grad_curve_x = 0.0, grad_curve_y = 0.0;
                if (i >= 2 && i < n - 2) {
                    grad_curve_x = 2.0 * (new_path[i-2].x - 4.0*new_path[i-1].x + 6.0*new_path[i].x - 4.0*new_path[i+1].x + new_path[i+2].x);
                    grad_curve_y = 2.0 * (new_path[i-2].y - 4.0*new_path[i-1].y + 6.0*new_path[i].y - 4.0*new_path[i+1].y + new_path[i+2].y);
                } else {
                     grad_curve_x = grad_smooth_x; grad_curve_y = grad_smooth_y;
                }

                double alpha = 0.05; 
                double dx = alpha * (w_data * grad_data_x + w_smooth * grad_smooth_x + w_curve * grad_curve_x);
                double dy = alpha * (w_data * grad_data_y + w_smooth * grad_smooth_y + w_curve * grad_curve_y);

                new_path[i].x -= dx; new_path[i].y -= dy;
                change += std::abs(dx) + std::abs(dy);
            }
        }
        return new_path;
    }

    // 重采样函数 - 保持不变
    static std::vector<Point> resample_path(const std::vector<Point>& raw_path, double step_size) {
        if (raw_path.size() < 2) return raw_path;
        std::vector<Point> resampled; resampled.push_back(raw_path.front());
        double accumulated_dist = 0.0;
        for (size_t i = 0; i < raw_path.size() - 1; ++i) {
            double dx = raw_path[i+1].x - raw_path[i].x; double dy = raw_path[i+1].y - raw_path[i].y;
            double seg_len = std::hypot(dx, dy);
            if (seg_len < 1e-4) continue;
            double current_seg_dist = 0.0; double needed = step_size - accumulated_dist;
            while (current_seg_dist + needed <= seg_len) {
                current_seg_dist += needed;
                double t = current_seg_dist / seg_len;
                resampled.push_back({raw_path[i].x + dx * t, raw_path[i].y + dy * t});
                accumulated_dist = 0; needed = step_size;   
            }
            accumulated_dist += (seg_len - current_seg_dist);
        }
        if (std::hypot(resampled.back().x - raw_path.back().x, resampled.back().y - raw_path.back().y) > step_size * 0.1) 
             resampled.push_back(raw_path.back());
        return resampled;
    }

    // ========================================================================
    // Plan 入口
    // ========================================================================
    std::vector<Point> plan(double start_x, double start_y, 
                            double goal_x, double goal_y, 
                            const std::vector<Point>& all_points,
                            const std::vector<Point>& history_path = {},
                            const std::vector<Point>& global_ref_path = {}) 
    {
        int min_gx, max_gx, min_gy, max_gy;
        calc_grid_bounds(start_x, start_y, goal_x, goal_y, all_points, min_gx, max_gx, min_gy, max_gy);
        
        int width = max_gx - min_gx + 1;
        int height = max_gy - min_gy + 1;
        // 限制最大搜索范围
        if (width * height > 400000) return {}; 

        // 障碍物栅格化
        std::vector<int8_t> occupancy_grid(width * height, 0);
        int margin_cells = std::ceil(cfg_.margin / cfg_.resolution);
        int margin_sq = margin_cells * margin_cells;

        for (const auto& pt : all_points) {
            int cx, cy;
            world_to_grid(pt.x, pt.y, cx, cy, min_gx, min_gy);
            // 简单裁剪
            if (cx < -margin_cells || cx >= width + margin_cells || cy < -margin_cells || cy >= height + margin_cells) continue;
            // 膨胀
            for (int i = -margin_cells; i <= margin_cells; ++i) {
                for (int j = -margin_cells; j <= margin_cells; ++j) {
                    if (i*i + j*j <= margin_sq) {
                        int nx = cx + i; int ny = cy + j;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            occupancy_grid[ny * width + nx] = 1;
                        }
                    }
                }
            }
        }

        // 预处理参考点 (世界坐标 -> 栅格坐标)
        std::vector<std::pair<int, int>> history_grid_pts;
        if (!history_path.empty()) {
            for(const auto& p : history_path) {
                int hx, hy; world_to_grid(p.x, p.y, hx, hy, min_gx, min_gy);
                if(hx >= 0 && hx < width && hy >= 0 && hy < height) history_grid_pts.push_back({hx, hy});
            }
        }

        // 预处理全局参考路径 (大幅提升 Cost 计算速度)
        // 只保留当前窗口内的参考点
        std::vector<std::pair<int, int>> ref_grid_pts;
        if (!global_ref_path.empty()) {
            int pad = 5;
            for (const auto& p : global_ref_path) {
                int rx, ry; world_to_grid(p.x, p.y, rx, ry, min_gx, min_gy);
                if (rx >= -pad && rx < width + pad && ry >= -pad && ry < height + pad) {
                    ref_grid_pts.push_back({rx, ry});
                }
            }
        }

        int start_gx, start_gy, goal_gx, goal_gy;
        world_to_grid(start_x, start_y, start_gx, start_gy, min_gx, min_gy);
        world_to_grid(goal_x, goal_y, goal_gx, goal_gy, min_gx, min_gy);

        if (!is_valid(start_gx, start_gy, width, height, occupancy_grid)) return {};

        return run_astar(start_gx, start_gy, goal_gx, goal_gy, width, height, occupancy_grid, min_gx, min_gy, history_grid_pts, ref_grid_pts);
    }

private:
    Config cfg_;

    std::vector<Point> run_astar(int sx, int sy, int gx, int gy, int w, int h, 
                                 const std::vector<int8_t>& grid, int min_gx, int min_gy,
                                 const std::vector<std::pair<int, int>>& history_pts,
                                 const std::vector<std::pair<int, int>>& ref_pts) 
    {
        // 节点池
        std::vector<GridNode*> node_registry(w * h, nullptr);
        
        auto cmp = [](GridNode* a, GridNode* b){ return (a->g + a->h) > (b->g + b->h); };
        std::priority_queue<GridNode*, std::vector<GridNode*>, decltype(cmp)> open_set(cmp);

        // 起点
        GridNode* start = new GridNode{sx, sy, 0, heuristic(sx, sy, gx, gy) * cfg_.heuristic_weight, nullptr, false};
        node_registry[sy * w + sx] = start;
        open_set.push(start);

        GridNode* final_node = nullptr;

        // 8 邻域
        const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
        const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        // 移动基础代价
        const double move_cost_base[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

        int iter = 0;
        int max_iters = 60000;

        while(!open_set.empty() && iter++ < max_iters) {
            GridNode* curr = open_set.top(); open_set.pop();
            
            if (curr->closed) continue;
            curr->closed = true;

            if(std::abs(curr->x - gx) <= 1 && std::abs(curr->y - gy) <= 1) {
                final_node = curr; break;
            }

            for(int i=0; i<8; ++i) {
                int nx = curr->x + dx[i]; int ny = curr->y + dy[i];
                if(nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                if(grid[ny*w+nx] == 1) continue; 

                // === COST 计算核心部分 ===
                double step_cost = move_cost_base[i];

                // 1. [核心] 全局参考路径吸附代价 (Global Reference Cost)
                // 目标：让 A* 生成的路径尽量在全局参考线附近，形成“势能槽”
                if (!ref_pts.empty()) {
                    double min_ref_dist_sq = 1000.0;
                    // 局部搜索优化：只遍历最近的几个点
                    // 假设 ref_pts 已经是局部裁剪过的，遍历开销可控 (~50-100 pts)
                    for (const auto& rp : ref_pts) {
                        double d2 = (double)((nx - rp.first)*(nx - rp.first) + (ny - rp.second)*(ny - rp.second));
                        if (d2 < min_ref_dist_sq) min_ref_dist_sq = d2;
                        if (d2 < 1.0) break; // 已经够近了
                    }
                    // 代价函数：距离越远，惩罚越重 (线性惩罚)
                    step_cost += cfg_.reference_cost_weight * std::sqrt(min_ref_dist_sq);
                }

                // 2. [核心] 转向惩罚 (Turning Penalty)
                // 目标：减少不必要的弯道，惩罚急转弯
                if (curr->parent != nullptr) {
                    // 上一步的向量
                    int prev_dx = curr->x - curr->parent->x;
                    int prev_dy = curr->y - curr->parent->y;
                    
                    // 当前步的向量 (dx[i], dy[i])
                    
                    // 向量点积: a.b = |a||b|cos(theta)
                    // 如果方向相同，点积最大；方向相反，点积为负
                    // 我们希望方向改变越小越好
                    
                    // 简化计算：只判断是否发生方向改变
                    bool direction_changed = (prev_dx != dx[i] || prev_dy != dy[i]);
                    
                    if (direction_changed) {
                        double turn_penalty = cfg_.turning_weight;
                        
                        // 进阶：检测是否是“急转弯” (90度或掉头)
                        // 直线: dot > 0, 90度: dot = 0, 掉头: dot < 0
                        int dot = prev_dx * dx[i] + prev_dy * dy[i];
                        
                        if (dot <= 0) {
                            // 90度或更急的转弯，施加重罚
                            turn_penalty *= 3.0; 
                        }
                        
                        step_cost += turn_penalty;
                    }
                }

                // 3. 历史路径迟滞 (History Bias)
                if (!history_pts.empty()) {
                    double min_hist_sq = 100.0;
                    for(const auto& hp : history_pts) {
                        double d2 = (nx - hp.first)*(nx - hp.first) + (ny - hp.second)*(ny - hp.second);
                        if(d2 < min_hist_sq) min_hist_sq = d2;
                    }
                    if(min_hist_sq < 2.0) step_cost -= 0.1; // 奖励走旧路
                }

                double new_g = curr->g + step_cost;
                int idx = ny * w + nx;
                GridNode* neighbor = node_registry[idx];

                if(neighbor == nullptr) {
                    neighbor = new GridNode{nx, ny, new_g, heuristic(nx, ny, gx, gy) * cfg_.heuristic_weight, curr, false};
                    node_registry[idx] = neighbor;
                    open_set.push(neighbor);
                } else if (!neighbor->closed) {
                    if(new_g < neighbor->g) {
                        neighbor->g = new_g;
                        neighbor->parent = curr;
                        open_set.push(neighbor);
                    }
                }
            }
        }

        // 回溯
        std::vector<Point> path;
        if(final_node) {
            while(final_node) {
                double wx, wy;
                grid_to_world(final_node->x, final_node->y, wx, wy, min_gx, min_gy);
                path.push_back({wx, wy});
                final_node = final_node->parent;
            }
            std::reverse(path.begin(), path.end());
        }

        // 释放
        for(auto* node : node_registry) { if(node) delete node; }

        return path;
    }

    void calc_grid_bounds(double sx, double sy, double gx, double gy, const std::vector<Point>& pts, int& min_x, int& max_x, int& min_y, int& max_y) {
        double min_wx = std::min(sx, gx), max_wx = std::max(sx, gx);
        double min_wy = std::min(sy, gy), max_wy = std::max(sy, gy);
        double scan_margin = 10.0; 
        for (const auto& p : pts) {
            if (p.x > min_wx - scan_margin && p.x < max_wx + scan_margin &&
                p.y > min_wy - scan_margin && p.y < max_wy + scan_margin) {
                min_wx = std::min(min_wx, p.x - cfg_.margin); max_wx = std::max(max_wx, p.x + cfg_.margin);
                min_wy = std::min(min_wy, p.y - cfg_.margin); max_wy = std::max(max_wy, p.y + cfg_.margin);
            }
        }
        min_x = std::floor(min_wx / cfg_.resolution) - cfg_.grid_padding;
        max_x = std::ceil(max_wx / cfg_.resolution) + cfg_.grid_padding;
        min_y = std::floor(min_wy / cfg_.resolution) - cfg_.grid_padding;
        max_y = std::ceil(max_wy / cfg_.resolution) + cfg_.grid_padding;
    }

    void world_to_grid(double wx, double wy, int& gx, int& gy, int min_gx, int min_gy) {
        gx = std::round(wx / cfg_.resolution) - min_gx; 
        gy = std::round(wy / cfg_.resolution) - min_gy;
    }
    void grid_to_world(int gx, int gy, double& wx, double& wy, int min_gx, int min_gy) {
        wx = (gx + min_gx) * cfg_.resolution; 
        wy = (gy + min_gy) * cfg_.resolution;
    }
    bool is_valid(int x, int y, int w, int h, const std::vector<int8_t>& grid) {
        return x >= 0 && x < w && y >= 0 && y < h && grid[y * w + x] == 0;
    }
    double heuristic(int x1, int y1, int x2, int y2) { return std::hypot(x1 - x2, y1 - y2); }
};

#endif