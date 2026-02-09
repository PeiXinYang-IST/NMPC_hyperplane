#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <memory>
#include "dbscan.hpp" 
#include "hyperplane_util.hpp" 

struct GridNode {
    int x, y;
    double g_cost, h_cost;
    GridNode* parent = nullptr;
    double f_cost() const { return g_cost + h_cost; }
    bool operator>(const GridNode& other) const { return f_cost() > other.f_cost(); } 
};

class AStarPlanner {
public:
    struct Config {
        double resolution = 0.2; 
        double margin = 1.0;     
        int grid_padding = 15;
        // 历史路径吸引权重
        double history_bias_weight = 0.5; 
        // [新增] 参考路径吸引权重 (建议 0.5 - 2.0)
        double reference_cost_weight = 1.0; 
    };

    AStarPlanner(Config config) : cfg_(config) {}

    static std::vector<Point> smooth_path(const std::vector<Point>& path, double weight_data=0.5, double weight_smooth=0.3, double tolerance=0.00001) {
         if (path.size() < 3) return path;
        std::vector<Point> new_path = path;
        double change = tolerance;
        int max_iter = 1000; int iter = 0;
        while (change >= tolerance && iter++ < max_iter) {
            change = 0.0;
            for (size_t i = 1; i < path.size() - 1; i++) {
                double aux_x = new_path[i].x; double aux_y = new_path[i].y;
                new_path[i].x += weight_data * (path[i].x - new_path[i].x) + weight_smooth * (new_path[i-1].x + new_path[i+1].x - 2.0 * new_path[i].x);
                new_path[i].y += weight_data * (path[i].y - new_path[i].y) + weight_smooth * (new_path[i-1].y + new_path[i+1].y - 2.0 * new_path[i].y);
                change += std::abs(aux_x - new_path[i].x) + std::abs(aux_y - new_path[i].y);
            }
        }
        return new_path;
    }

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
        return resampled;
    }

    // [修改] 增加 global_ref_path 输入
    std::vector<Point> plan(double start_x, double start_y, 
                            double goal_x, double goal_y, 
                            const std::vector<Point>& all_points,
                            const std::vector<Point>& history_path = {},
                            const std::vector<Point>& global_ref_path = {}) // 新增参数
    {
        int min_gx, max_gx, min_gy, max_gy;
        calc_grid_bounds(start_x, start_y, goal_x, goal_y, all_points, min_gx, max_gx, min_gy, max_gy);
        
        int width = max_gx - min_gx + 1;
        int height = max_gy - min_gy + 1;
        if (width * height > 1000000) return {}; 

        std::vector<int8_t> occupancy_grid(width * height, 0);
        int margin_cells = std::ceil(cfg_.margin / cfg_.resolution);
        int margin_sq = margin_cells * margin_cells;

        // 1. 栅格化障碍物
        for (const auto& pt : all_points) {
            int cx, cy;
            world_to_grid(pt.x, pt.y, cx, cy, min_gx, min_gy);
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

        // 2. 预处理历史路径 (World -> Grid)
        std::vector<std::pair<int, int>> history_grid_pts;
        if (!history_path.empty()) {
            for(const auto& p : history_path) {
                int hx, hy;
                world_to_grid(p.x, p.y, hx, hy, min_gx, min_gy);
                if(hx >= 0 && hx < width && hy >= 0 && hy < height) {
                    history_grid_pts.push_back({hx, hy});
                }
            }
        }

        // 3. [新增] 预处理参考路径 (World -> Grid)
        // 只提取位于当前 Grid 范围内的参考点，减少后续搜索计算量
        std::vector<std::pair<int, int>> ref_grid_pts;
        if (!global_ref_path.empty()) {
            // 简单的裁剪逻辑：只取 bounding box 内的点
            // 为了防止边界效应，稍微扩大一点筛选范围
            int pad = 5; 
            for (const auto& p : global_ref_path) {
                int rx, ry;
                world_to_grid(p.x, p.y, rx, ry, min_gx, min_gy);
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
                                 const std::vector<std::pair<int, int>>& ref_pts) // 新增参数
    {
        struct Node { int x, y; double g, h; Node* parent; };
        auto cmp = [](Node* a, Node* b){ return (a->g + a->h) > (b->g + b->h); };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_set(cmp);
        std::unordered_map<int, Node*> all_nodes; 

        Node* start = new Node{sx, sy, 0, heuristic(sx, sy, gx, gy), nullptr};
        all_nodes[sy*w+sx] = start;
        open_set.push(start);

        Node* final_node = nullptr;
        const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
        const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        const double move_cost_base[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

        int iter = 0;
        while(!open_set.empty() && iter++ < 40000) { // 稍微增加迭代上限
            Node* curr = open_set.top(); open_set.pop();

            if(std::abs(curr->x - gx) <= 1 && std::abs(curr->y - gy) <= 1) {
                final_node = curr; break;
            }

            for(int i=0; i<8; ++i) {
                int nx = curr->x + dx[i]; int ny = curr->y + dy[i];
                if(nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                if(grid[ny*w+nx] == 1) continue; 

                // --- Cost 计算区域 ---
                double step_cost = move_cost_base[i];
                
                // 1. 历史路径吸引 (保持原样)
                if (!history_pts.empty()) {
                    double min_dist_sq = 1e9;
                    for(const auto& hp : history_pts) {
                        double d2 = (double)((nx - hp.first)*(nx - hp.first) + (ny - hp.second)*(ny - hp.second));
                        if(d2 < min_dist_sq) min_dist_sq = d2;
                        if(min_dist_sq < 2.0) break; 
                    }
                    step_cost += cfg_.history_bias_weight * std::sqrt(min_dist_sq);
                }

                // 2. [新增] 参考路径吸引 (Reference Path Bias)
                // 计算当前节点 (nx, ny) 距离最近参考点的距离
                if (!ref_pts.empty()) {
                    double min_ref_dist_sq = 1e9;
                    // 这里遍历所有的局部 ref_pts
                    // 性能优化：如果 ref_pts 点数太多 (比如 >100)，可能影响性能
                    // 但通常局部窗口内的 ref path 点数在 20-50 左右，遍历是可以接受的
                    for (const auto& rp : ref_pts) {
                        double d2 = (double)((nx - rp.first)*(nx - rp.first) + (ny - rp.second)*(ny - rp.second));
                        if (d2 < min_ref_dist_sq) min_ref_dist_sq = d2;
                        if (min_ref_dist_sq < 1.0) break; // 已经在路径上了，无需继续
                    }
                    // 增加惩罚：距离越远，cost 越高
                    step_cost += cfg_.reference_cost_weight * std::sqrt(min_ref_dist_sq);
                }
                // --------------------

                double new_g = curr->g + step_cost;
                int idx = ny*w+nx;
                
                if(all_nodes.find(idx) == all_nodes.end()) {
                    Node* next = new Node{nx, ny, new_g, heuristic(nx, ny, gx, gy), curr};
                    all_nodes[idx] = next;
                    open_set.push(next);
                } else {
                    if(new_g < all_nodes[idx]->g) {
                        all_nodes[idx]->g = new_g;
                        all_nodes[idx]->parent = curr;
                        open_set.push(all_nodes[idx]);
                    }
                }
            }
        }

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
        for(auto& pair : all_nodes) delete pair.second;
        return path;
    }

    void calc_grid_bounds(double sx, double sy, double gx, double gy, const std::vector<Point>& pts, int& min_x, int& max_x, int& min_y, int& max_y) {
        double min_wx = std::min(sx, gx), max_wx = std::max(sx, gx);
        double min_wy = std::min(sy, gy), max_wy = std::max(sy, gy);
        for (const auto& p : pts) {
            min_wx = std::min(min_wx, p.x - cfg_.margin); max_wx = std::max(max_wx, p.x + cfg_.margin);
            min_wy = std::min(min_wy, p.y - cfg_.margin); max_wy = std::max(max_wy, p.y + cfg_.margin);
        }
        min_x = std::floor(min_wx / cfg_.resolution) - cfg_.grid_padding;
        max_x = std::ceil(max_wx / cfg_.resolution) + cfg_.grid_padding;
        min_y = std::floor(min_wy / cfg_.resolution) - cfg_.grid_padding;
        max_y = std::ceil(max_wy / cfg_.resolution) + cfg_.grid_padding;
    }

    void world_to_grid(double wx, double wy, int& gx, int& gy, int min_gx, int min_gy) {
        gx = std::round(wx / cfg_.resolution) - min_gx; gy = std::round(wy / cfg_.resolution) - min_gy;
    }
    void grid_to_world(int gx, int gy, double& wx, double& wy, int min_gx, int min_gy) {
        wx = (gx + min_gx) * cfg_.resolution; wy = (gy + min_gy) * cfg_.resolution;
    }
    bool is_valid(int x, int y, int w, int h, const std::vector<int8_t>& grid) {
        return x >= 0 && x < w && y >= 0 && y < h && grid[y * w + x] == 0;
    }
    double heuristic(int x1, int y1, int x2, int y2) { return std::hypot(x1 - x2, y1 - y2); }
};
#endif