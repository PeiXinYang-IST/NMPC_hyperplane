#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <memory>
#include <iostream>
#include "dbscan.hpp" 

// 紧凑的节点结构体
struct GridNode {
    int x, y;
    double g, f;
    GridNode* parent = nullptr;
    int visited_id = 0; 
    bool is_closed = false;

    GridNode() : x(0), y(0), g(0), f(0), parent(nullptr), visited_id(0), is_closed(false) {}
    
    void reset(int id, int _x, int _y) {
        visited_id = id;
        x = _x; y = _y;
        g = 1e9; f = 1e9;
        parent = nullptr;
        is_closed = false;
    }
};

struct NodeComparator {
    bool operator()(const GridNode* a, const GridNode* b) const { return a->f > b->f; }
};

class AStarPlanner {
public:
    struct Config {
        double resolution = 0.3;
        double margin = 0.5;         // 膨胀半径 (软边界)
        int grid_padding = 10;
        
        // --- 权重参数 ---
        double heuristic_weight = 1.2;      
        double reference_cost_weight = 2.0; 
        double turning_weight = 2.0;        
        double history_bias_weight = 0.5;   

        // [NEW] 碰撞惩罚权重 (软障碍代价)
        // 这个值要很大，比如 50.0 ~ 100.0，保证除非万不得已，否则不走这里
        double collision_penalty_weight = 100.0; 

        // 平滑参数
        double smooth_w_data = 0.45;
        double smooth_w_smooth = 0.40;
        double smooth_w_curvature = 0.40; 
    };

    AStarPlanner(Config config) : cfg_(config) {
        max_width_ = 800;
        max_height_ = 800;
        node_pool_.resize(max_width_ * max_height_);
        for(int y=0; y<max_height_; ++y) {
            for(int x=0; x<max_width_; ++x) {
                node_pool_[y * max_width_ + x].x = x;
                node_pool_[y * max_width_ + x].y = y;
            }
        }
    }

    void update_config(const Config& cfg) { cfg_ = cfg; }
    Config get_config() const { return cfg_; }

    // smooth_path 保持不变...
    static std::vector<Point> smooth_path(const std::vector<Point>& raw_path, double w_data, double w_smooth, double w_curve, double tolerance = 0.001) {
        if (raw_path.size() < 3) return raw_path;
        std::vector<Point> new_path = raw_path;
        // ... (此处省略 FEM-Smoother 代码，保持原样) ...
        // 建议：在 smooth_path 里也可以加一个简单的逻辑：如果原始点在障碍物内，w_data 设得更大
         // 为了篇幅，这里暂时复用你之前的 smooth_path 完整代码
        int n = new_path.size();
        double path_len = 0;
        for(size_t i=1; i<raw_path.size(); ++i) path_len += std::hypot(raw_path[i].x - raw_path[i-1].x, raw_path[i].y - raw_path[i-1].y);
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
                } else { grad_curve_x = grad_smooth_x; grad_curve_y = grad_smooth_y; }
                double alpha = 0.05; 
                double dx = alpha * (w_data * grad_data_x + w_smooth * grad_smooth_x + w_curve * grad_curve_x);
                double dy = alpha * (w_data * grad_data_y + w_smooth * grad_smooth_y + w_curve * grad_curve_y);
                new_path[i].x -= dx; new_path[i].y -= dy;
                change += std::abs(dx) + std::abs(dy);
            }
        }
        return new_path;
    }

    static std::vector<Point> resample_path(const std::vector<Point>& raw_path, double step_size) {
        // ... (保持 resample_path 原样) ...
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

    std::vector<Point> plan(double start_x, double start_y, 
                            double goal_x, double goal_y, 
                            const std::vector<Point>& all_points,
                            const std::vector<Point>& history_path = {},
                            const std::vector<Point>& global_ref_path = {}) 
    {
        run_id_++; 

        int min_gx, max_gx, min_gy, max_gy;
        calc_grid_bounds(start_x, start_y, goal_x, goal_y, all_points, min_gx, max_gx, min_gy, max_gy);
        
        int width = max_gx - min_gx + 1;
        int height = max_gy - min_gy + 1;
        if (width <= 0 || height <= 0 || width > max_width_ || height > max_height_) return {};

        // 2. 障碍物处理 [关键修改]
        // 0: Free
        // 1: Margin (Soft Obstacle) -> High Cost
        // 2: Lethal (Hard Obstacle) -> Blocked
        std::vector<int8_t> occupancy_grid(width * height, 0);
        int margin_cells = std::ceil(cfg_.margin / cfg_.resolution);
        int margin_sq = margin_cells * margin_cells;
        
        // 定义致命半径（比如小于分辨率，或者 0.15m），只有碰到障碍物中心才算彻底堵死
        // 这里简单处理：点云所在格子本身是 Lethal (2)，周围是 Margin (1)
        
        for (const auto& pt : all_points) {
            int cx, cy;
            world_to_grid(pt.x, pt.y, cx, cy, min_gx, min_gy);
            if (cx < -margin_cells || cx >= width + margin_cells || cy < -margin_cells || cy >= height + margin_cells) continue;

            for (int i = -margin_cells; i <= margin_cells; ++i) {
                for (int j = -margin_cells; j <= margin_cells; ++j) {
                    int dist_sq = i*i + j*j;
                    if (dist_sq <= margin_sq) {
                        int nx = cx + i; int ny = cy + j;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int idx = ny * width + nx;
                            // 如果是中心点（或非常近），标记为 2 (Lethal)
                            // 这里定义：距离 <= 1个格子算 Lethal
                            if (dist_sq <= 1) { 
                                occupancy_grid[idx] = 2; 
                            } 
                            // 否则，如果还没被标记为 2，标记为 1 (Margin)
                            else if (occupancy_grid[idx] != 2) {
                                occupancy_grid[idx] = 1;
                            }
                        }
                    }
                }
            }
        }

        // ... (历史点和参考点转换保持不变) ...
        std::vector<std::pair<int, int>> history_grid_pts;
        if (!history_path.empty()) {
            for(const auto& p : history_path) {
                int hx, hy; world_to_grid(p.x, p.y, hx, hy, min_gx, min_gy);
                if(hx >= 0 && hx < width && hy >= 0 && hy < height) history_grid_pts.push_back({hx, hy});
            }
        }
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

        // [关键] 起点在 Lethal 里？如果 A* 无法起步，把起点强制设为 Soft
        if (is_lethal(start_gx, start_gy, width, height, occupancy_grid)) {
             int idx = start_gy * width + start_gx;
             occupancy_grid[idx] = 1; // 降级为 Margin，允许起步
        }
        // [关键] 终点在 Lethal 里？把终点设为 Soft
        if (is_lethal(goal_gx, goal_gy, width, height, occupancy_grid)) {
             int idx = goal_gy * width + goal_gx;
             occupancy_grid[idx] = 1; 
        }

        return run_astar(start_gx, start_gy, goal_gx, goal_gy, width, height, occupancy_grid, min_gx, min_gy, history_grid_pts, ref_grid_pts);
    }

private:
    Config cfg_;
    std::vector<GridNode> node_pool_;
    int max_width_, max_height_;
    int run_id_ = 0;

    inline double get_heuristic(int x1, int y1, int x2, int y2) {
        double dx = std::abs(x1 - x2);
        double dy = std::abs(y1 - y2);
        return (dx + dy) + (1.41421 - 2.0) * std::min(dx, dy); 
    }

    inline GridNode* get_node(int x, int y) {
        return &node_pool_[y * max_width_ + x];
    }
    
    // 检查是否彻底不可通行
    bool is_lethal(int x, int y, int w, int h, const std::vector<int8_t>& grid) {
        return x >= 0 && x < w && y >= 0 && y < h && grid[y * w + x] == 2;
    }

    std::vector<Point> run_astar(int sx, int sy, int gx, int gy, int w, int h, 
                                 const std::vector<int8_t>& grid, int min_gx, int min_gy,
                                 const std::vector<std::pair<int, int>>& history_pts,
                                 const std::vector<std::pair<int, int>>& ref_pts) 
    {
        std::priority_queue<GridNode*, std::vector<GridNode*>, NodeComparator> open_set;
        GridNode* start_node = get_node(sx, sy);
        if (start_node->visited_id != run_id_) start_node->reset(run_id_, sx, sy);
        
        start_node->g = 0;
        start_node->f = get_heuristic(sx, sy, gx, gy) * cfg_.heuristic_weight;
        open_set.push(start_node);

        GridNode* final_node = nullptr;
        const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
        const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        const double move_cost_base[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

        int iter = 0;
        int max_iters = 30000;

        while(!open_set.empty() && iter++ < max_iters) {
            GridNode* curr = open_set.top(); open_set.pop();
            if (curr->is_closed) continue;
            curr->is_closed = true;

            if(std::abs(curr->x - gx) <= 1 && std::abs(curr->y - gy) <= 1) {
                final_node = curr; break;
            }

            for(int i=0; i<8; ++i) {
                int nx = curr->x + dx[i]; int ny = curr->y + dy[i];
                if(nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                
                // [关键修改] 检查栅格状态
                int8_t status = grid[ny*w+nx];
                if(status == 2) continue; // Lethal (物理撞击), 绝对不走
                
                // 获取邻居节点
                GridNode* neighbor = get_node(nx, ny);
                if (neighbor->visited_id != run_id_) neighbor->reset(run_id_, nx, ny);
                if (neighbor->is_closed) continue;

                // --- Cost 计算 ---
                double step_cost = move_cost_base[i];

                // [关键修改] 如果在 Margin (软障碍) 里，增加巨额惩罚
                if (status == 1) {
                    step_cost += cfg_.collision_penalty_weight;
                }

                // ... (参考路径、转向、历史偏置等其他 cost 保持不变) ...
                if (!ref_pts.empty()) {
                    double min_ref_dist_sq = 1000.0;
                    for (const auto& rp : ref_pts) {
                        double d2 = (double)((nx - rp.first)*(nx - rp.first) + (ny - rp.second)*(ny - rp.second));
                        if (d2 < min_ref_dist_sq) min_ref_dist_sq = d2;
                        if (d2 < 1.0) break; 
                    }
                    step_cost += cfg_.reference_cost_weight * std::sqrt(min_ref_dist_sq);
                }

                if (curr->parent != nullptr) {
                    int prev_dx = curr->x - curr->parent->x;
                    int prev_dy = curr->y - curr->parent->y;
                    if (prev_dx != dx[i] || prev_dy != dy[i]) {
                        double turn_penalty = cfg_.turning_weight;
                        int dot = prev_dx * dx[i] + prev_dy * dy[i];
                        if (dot <= 0) turn_penalty *= 3.0; 
                        step_cost += turn_penalty;
                    }
                }

                if (!history_pts.empty()) {
                   double min_hist_sq = 100.0;
                    for(const auto& hp : history_pts) {
                        double d2 = (nx - hp.first)*(nx - hp.first) + (ny - hp.second)*(ny - hp.second);
                        if(d2 < min_hist_sq) min_hist_sq = d2;
                    }
                    if(min_hist_sq < 2.0) step_cost -= 0.1 * cfg_.history_bias_weight; 
                }

                double new_g = curr->g + step_cost;
                if(new_g < neighbor->g) {
                    neighbor->g = new_g;
                    neighbor->f = new_g + get_heuristic(nx, ny, gx, gy) * cfg_.heuristic_weight;
                    neighbor->parent = curr;
                    open_set.push(neighbor);
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
        return path;
    }

    void calc_grid_bounds(double sx, double sy, double gx, double gy, const std::vector<Point>& pts, int& min_x, int& max_x, int& min_y, int& max_y) {
        // ... (保持原样) ...
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

        int w = max_x - min_x + 1;
        int h = max_y - min_y + 1;
        if (w > max_width_) { int reduce = (w - max_width_ + 1) / 2; min_x += reduce; max_x -= reduce; }
        if (h > max_height_) { int reduce = (h - max_height_ + 1) / 2; min_y += reduce; max_y -= reduce; }
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
        return x >= 0 && x < w && y >= 0 && y < h;
    }
};

#endif