#ifndef LATTICE_PLANNER_HPP
#define LATTICE_PLANNER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <cstring> 
#include <array>

struct Point2D { double x, y; double yaw; };

struct CandidatePath {
    std::vector<Point2D> points; 
    std::vector<double> v_profile; 
    double cost;
    double lateral_offset;       
    bool collision;
    double curvature;            
};

class LatticePlanner {
public:
    struct Config {
        double path_resolution = 0.2; 
        double lookahead_dist = 20.0; 
        int num_samples = 7;          
        double sample_width = 0.5;    
        double max_width = 3.0;       
        double collision_radius = 0.8; 
        double w_collision = 1000.0;
        double w_offset = 2.0;        
        double w_consistency = 0.5;   
    };

    struct RobotState { double x, y, yaw, v; };

    LatticePlanner() {
        grid_width_ = 400; grid_height_ = 400; grid_res_ = 0.1; 
        collision_grid_.resize(grid_width_ * grid_height_);
    }

    void update_config(const Config& cfg) { cfg_ = cfg; }
    
    double get_last_best_offset() const { return last_best_offset_; }
    std::vector<CandidatePath> get_last_candidates() const { return last_candidates_; }

    CandidatePath plan(const RobotState& robot, 
                       const std::vector<Point2D>& global_path, 
                       const std::vector<Point2D>& obstacles) 
    {
        last_candidates_.clear();
        build_local_grid(robot, obstacles);
        
        // 衰减
        if (std::abs(last_best_offset_) > 0.05) { last_best_offset_ *= 0.95; } 
        else { last_best_offset_ = 0.0; }

        int start_idx = find_closest_index(robot, global_path);
        
        // 采样逻辑
        std::vector<double> offsets;
        offsets.reserve(cfg_.num_samples + 5);
        offsets.push_back(0.0); 
        
        int half_samples = (cfg_.num_samples - 1) / 2;
        for (int i = 1; i <= half_samples; ++i) {
            double d = i * cfg_.sample_width;
            if (d <= cfg_.max_width) {
                offsets.push_back(d); offsets.push_back(-d);
            }
        }
        
        // 历史点回注
        if (std::abs(last_best_offset_) > 0.01) {
            bool exists = false;
            for(double o : offsets) if(std::abs(o - last_best_offset_) < 0.01) exists = true;
            if(!exists && std::abs(last_best_offset_) <= cfg_.max_width) offsets.push_back(last_best_offset_);
        }

        CandidatePath best_path;
        best_path.cost = std::numeric_limits<double>::infinity();
        bool found_valid = false;

        for (double offset : offsets) {
            CandidatePath candidate;
            candidate.lateral_offset = offset;
            candidate.collision = false;
            
            generate_offset_path(robot, global_path, start_idx, offset, candidate.points);
            
            if (candidate.points.empty()) continue;

            if (check_collision_grid(candidate.points)) {
                candidate.collision = true;
                candidate.cost = std::numeric_limits<double>::infinity();
            } else {
                candidate.cost = cfg_.w_offset * (offset * offset) + // 使用平方代价
                                 cfg_.w_consistency * std::abs(offset - last_best_offset_);
                candidate.curvature = 0.0; 
            }
            last_candidates_.push_back(candidate);

            if (!candidate.collision && candidate.cost < best_path.cost) {
                best_path = candidate;
                found_valid = true;
            }
        }

        if (found_valid) {
            last_best_offset_ = best_path.lateral_offset;
            return best_path;
        } else {
            last_best_offset_ = 0.0;
            if(!last_candidates_.empty()) return last_candidates_[0];
            return CandidatePath();
        }
    }

private:
    Config cfg_;
    double last_best_offset_ = 0.0;
    std::vector<CandidatePath> last_candidates_;
    std::vector<uint8_t> collision_grid_;
    int grid_width_, grid_height_;
    double grid_res_, grid_origin_x_, grid_origin_y_;

    void build_local_grid(const RobotState& robot, const std::vector<Point2D>& obstacles) {
        std::fill(collision_grid_.begin(), collision_grid_.end(), 0);
        double half_w_m = (grid_width_ * grid_res_) / 2.0;
        double half_h_m = (grid_height_ * grid_res_) / 2.0;
        grid_origin_x_ = robot.x - half_w_m;
        grid_origin_y_ = robot.y - half_h_m;
        int margin_cells = std::ceil(cfg_.collision_radius / grid_res_);
        int margin_sq = margin_cells * margin_cells;
        for (const auto& obs : obstacles) {
            int gx = (int)((obs.x - grid_origin_x_) / grid_res_);
            int gy = (int)((obs.y - grid_origin_y_) / grid_res_);
            for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
                for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
                    if (dx*dx + dy*dy > margin_sq) continue; 
                    int nx = gx + dx; int ny = gy + dy;
                    if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_) {
                        collision_grid_[ny * grid_width_ + nx] = 1; 
                    }
                }
            }
        }
    }

    bool check_collision_grid(const std::vector<Point2D>& pts) {
        int step = (cfg_.path_resolution < grid_res_) ? 2 : 1; 
        for (size_t i = 0; i < pts.size(); i += step) {
            const auto& p = pts[i];
            int gx = (int)((p.x - grid_origin_x_) / grid_res_);
            int gy = (int)((p.y - grid_origin_y_) / grid_res_);
            if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_) {
                if (collision_grid_[gy * grid_width_ + gx] == 1) return true;
            }
        }
        return false;
    }

    void generate_offset_path(const RobotState& robot, 
                              const std::vector<Point2D>& ref_path, 
                              int start_idx, 
                              double target_offset, 
                              std::vector<Point2D>& out_points) 
    {
        double current_dist = 0.0;
        out_points.clear();
        if (start_idx >= (int)ref_path.size()) return;

        // 1. 计算初始横向偏差
        const auto& p_start = ref_path[start_idx];
        double start_nx = -std::sin(p_start.yaw);
        double start_ny =  std::cos(p_start.yaw);
        double dx = robot.x - p_start.x;
        double dy = robot.y - p_start.y;
        double current_lat_offset = dx * start_nx + dy * start_ny;

        // 2. [关键修改] 动态收敛距离
        // 目标：让扇形更开。策略：缩短收敛所需的纵向距离。
        // 系数 3.0 表示: 横向偏移 1m, 纵向需要 3m (约18度角)。减小此值角度变大。
        double required_len = std::abs(target_offset - current_lat_offset) * 3.0;
        
        // 限制收敛距离：最短 5m (防抖)，最长为 lookahead 的一半 (约10m，强制快速收敛)
        double convergence_dist = std::clamp(required_len, 5.0, cfg_.lookahead_dist * 0.5);

        for (int i = start_idx; i < (int)ref_path.size() - 1; ++i) {
            const auto& p_curr = ref_path[i];
            const auto& p_next = ref_path[i+1];
            
            double nx = -std::sin(p_curr.yaw);
            double ny =  std::cos(p_curr.yaw);

            double ratio = current_dist / convergence_dist;
            if (ratio > 1.0) ratio = 1.0;
            
            // Smootherstep (Quintic) 插值，让起步和结束更平滑
            double blend = ratio * ratio * ratio * (ratio * (ratio * 6 - 15) + 10);

            double interp_offset = current_lat_offset + (target_offset - current_lat_offset) * blend;

            Point2D pt_shifted;
            pt_shifted.x = p_curr.x + nx * interp_offset;
            pt_shifted.y = p_curr.y + ny * interp_offset;
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