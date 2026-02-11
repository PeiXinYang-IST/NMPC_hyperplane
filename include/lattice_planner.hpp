#ifndef LATTICE_PLANNER_HPP
#define LATTICE_PLANNER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <cstring> 

// 基础点结构
struct Point2D { double x, y; double yaw; };

// 候选路径结构
struct CandidatePath {
    std::vector<Point2D> points; 
    double cost;
    double lateral_offset;       
    bool collision;
    double curvature;            
};

class LatticePlanner {
public:
    struct Config {
        // [用户指定 CFG 参数]
        double path_resolution = 0.2; // 路径点分辨率
        double lookahead_dist = 20.0; // 前视距离
        int num_samples = 7;          // 采样条数 (如果设为500，需配合 sample_width)
        double sample_width = 0.5;    // 采样间隔
        double max_width = 3.0;       // 最大横向偏移限制
        
        // 碰撞检测半径 (包含膨胀)
        double collision_radius = 0.8; 

        // 权重
        double w_collision = 1000.0;
        double w_offset = 1.0;        // 偏离中心线代价
        double w_consistency = 2.0;   // 历史一致性代价
    };

    struct RobotState { double x, y, yaw, v; };

    LatticePlanner() {
        // 初始化局部网格内存 
        // 假设局部地图覆盖 40x40 米范围 (足够覆盖 lookahead_dist=20)
        // 分辨率 0.1m => 400x400 网格
        grid_width_ = 400; 
        grid_height_ = 400;
        grid_res_ = 0.1; 
        collision_grid_.resize(grid_width_ * grid_height_);
    }

    void update_config(const Config& cfg) { cfg_ = cfg; }
    
    double get_last_best_offset() const { return last_best_offset_; }
    std::vector<CandidatePath> get_last_candidates() const { return last_candidates_; }

    /**
     * @brief 核心规划函数 (带网格加速)
     */
    CandidatePath plan(const RobotState& robot, 
                       const std::vector<Point2D>& global_path, 
                       const std::vector<Point2D>& obstacles) 
    {
        last_candidates_.clear();
        
        // [Step 0] 构建局部碰撞网格 (加速核心: O(M) -> O(1))
        build_local_grid(robot, obstacles);

        // [Step 1] 找到最近点
        int start_idx = find_closest_index(robot, global_path);
        
        // [Step 2] 生成 Offset (严格遵循 Config)
        std::vector<double> offsets;
        offsets.reserve(cfg_.num_samples);
        offsets.push_back(0.0); // 优先中心线
        
        int half_samples = (cfg_.num_samples - 1) / 2;
        
        for (int i = 1; i <= half_samples; ++i) {
            double d = i * cfg_.sample_width;
            
            // 严格执行 max_width 截断
            if (d <= cfg_.max_width) {
                offsets.push_back(d);
                offsets.push_back(-d);
            } else {
                // 如果超出最大宽度，停止扩展
                break;
            }
        }
        
        // 启发式：加入上一帧的最佳偏移 (如果不在当前列表里)
        if (std::abs(last_best_offset_) > 0.01) {
            bool exists = false;
            for(double o : offsets) if(std::abs(o - last_best_offset_) < 0.01) exists = true;
            if(!exists && std::abs(last_best_offset_) <= cfg_.max_width) {
                offsets.push_back(last_best_offset_);
            }
        }

        // [Step 3] 生成并评估路径
        CandidatePath best_path;
        best_path.cost = std::numeric_limits<double>::infinity();
        bool found_valid = false;

        for (double offset : offsets) {
            CandidatePath candidate;
            candidate.lateral_offset = offset;
            candidate.collision = false;
            
            // 路径生成
            generate_offset_path(global_path, start_idx, offset, candidate.points);
            
            if (candidate.points.empty()) continue;

            // 碰撞检测 (极速查表)
            if (check_collision_grid(candidate.points)) {
                candidate.collision = true;
                candidate.cost = std::numeric_limits<double>::infinity();
            } else {
                // 评分
                candidate.cost = cfg_.w_offset * std::abs(offset) + 
                                 cfg_.w_consistency * std::abs(offset - last_best_offset_);
                candidate.curvature = 0.0; 
            }
            
            last_candidates_.push_back(candidate);

            // 优选
            if (!candidate.collision && candidate.cost < best_path.cost) {
                best_path = candidate;
                found_valid = true;
            }
        }

        // [Step 4] 返回结果
        if (found_valid) {
            last_best_offset_ = best_path.lateral_offset;
            return best_path;
        } else {
            // 全都碰撞，重置 offset，返回第一条(中心线)作为保底
            last_best_offset_ = 0.0;
            if(!last_candidates_.empty()) return last_candidates_[0];
            return CandidatePath();
        }
    }

private:
    Config cfg_;
    double last_best_offset_ = 0.0;
    std::vector<CandidatePath> last_candidates_;
    
    // --- 局部网格数据 ---
    std::vector<uint8_t> collision_grid_;
    int grid_width_, grid_height_;
    double grid_res_;
    double grid_origin_x_, grid_origin_y_; // 网格左下角 (随车移动)

    // 构建局部网格
    void build_local_grid(const RobotState& robot, const std::vector<Point2D>& obstacles) {
        // 1. 清空网格 (memset 比 vector assign 快)
        std::fill(collision_grid_.begin(), collision_grid_.end(), 0);
        
        // 2. 更新网格原点 (以车为中心)
        double half_w_m = (grid_width_ * grid_res_) / 2.0;
        double half_h_m = (grid_height_ * grid_res_) / 2.0;
        grid_origin_x_ = robot.x - half_w_m;
        grid_origin_y_ = robot.y - half_h_m;

        // 3. 计算膨胀半径对应的栅格数
        int margin_cells = std::ceil(cfg_.collision_radius / grid_res_);
        int margin_sq = margin_cells * margin_cells;

        // 4. 填充障碍物 (Rasterization)
        for (const auto& obs : obstacles) {
            // 转到 Grid 坐标
            int gx = (int)((obs.x - grid_origin_x_) / grid_res_);
            int gy = (int)((obs.y - grid_origin_y_) / grid_res_);
            
            // 简单的圆形膨胀
            for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
                for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
                    if (dx*dx + dy*dy > margin_sq) continue; 

                    int nx = gx + dx;
                    int ny = gy + dy;
                    // 边界检查
                    if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_) {
                        collision_grid_[ny * grid_width_ + nx] = 1; // 标记占用
                    }
                }
            }
        }
    }

    // 查表检测
    bool check_collision_grid(const std::vector<Point2D>& pts) {
        // 根据分辨率决定跳点步长，避免过度检测
        int step = 1;
        if (cfg_.path_resolution < grid_res_) step = 2; 

        for (size_t i = 0; i < pts.size(); i += step) {
            const auto& p = pts[i];
            int gx = (int)((p.x - grid_origin_x_) / grid_res_);
            int gy = (int)((p.y - grid_origin_y_) / grid_res_);
            
            if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_) {
                if (collision_grid_[gy * grid_width_ + gx] == 1) {
                    return true;
                }
            }
        }
        return false;
    }

    // 路径生成 (保持不变)
    void generate_offset_path(const std::vector<Point2D>& ref_path, int start_idx, double offset, std::vector<Point2D>& out_points) {
        double current_dist = 0.0;
        out_points.clear();
        if (start_idx >= (int)ref_path.size()) return;
        
        for (int i = start_idx; i < (int)ref_path.size() - 1; ++i) {
            const auto& p_curr = ref_path[i];
            const auto& p_next = ref_path[i+1];
            double nx = -std::sin(p_curr.yaw);
            double ny =  std::cos(p_curr.yaw);
            Point2D pt_shifted;
            pt_shifted.x = p_curr.x + nx * offset;
            pt_shifted.y = p_curr.y + ny * offset;
            pt_shifted.yaw = p_curr.yaw;
            
            if (out_points.empty()) {
                out_points.push_back(pt_shifted);
            } else {
                double dist = std::hypot(pt_shifted.x - out_points.back().x, pt_shifted.y - out_points.back().y);
                if (dist >= cfg_.path_resolution) out_points.push_back(pt_shifted);
            }
            double seg_len = std::hypot(p_next.x - p_curr.x, p_next.y - p_curr.y);
            current_dist += seg_len;
            if (current_dist > cfg_.lookahead_dist) break;
        }
    }

    int find_closest_index(const RobotState& robot, const std::vector<Point2D>& path) {
        double min_dist = 1e9; int idx = 0;
        for (int i = 0; i < (int)path.size(); ++i) {
            double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
            if (d < min_dist) { min_dist = d; idx = i; }
        }
        return idx;
    }
};

#endif