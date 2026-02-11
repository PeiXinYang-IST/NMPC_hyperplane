#ifndef LATTICE_PLANNER_HPP
#define LATTICE_PLANNER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>

// 定义基础点结构
struct Point2D {
    double x, y;
    double yaw; 
};

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
        // [用户指定的可调参数]
        double path_resolution = 0.2; // 固定的分辨率
        double lookahead_dist = 20.0; // 前视距离
        int num_samples = 7;          // 采样多少条路径
        double sample_width = 0.5;    // 采样间隔
        double max_width = 3.0;       // 最大横向偏移限制
        
        // 碰撞检测半径 (包含膨胀)
        double collision_radius = 0.8; 

        // 权重
        double w_collision = 1000.0;
        double w_offset = 1.0;        // 偏离中心线的代价
        double w_consistency = 2.0;   // 保持车道的代价
    };

    struct RobotState {
        double x, y, yaw, v;
    };

    LatticePlanner() = default;

    void update_config(const Config& cfg) { cfg_ = cfg; }

    double get_last_best_offset() const { return last_best_offset_; }
    std::vector<CandidatePath> get_last_candidates() const { return last_candidates_; }

    /**
     * @brief 核心规划函数
     */
    CandidatePath plan(const RobotState& robot, 
                       const std::vector<Point2D>& global_path, 
                       const std::vector<Point2D>& obstacles) 
    {
        last_candidates_.clear();
        
        int start_idx = find_closest_index(robot, global_path);
        
        // 1. 生成横向偏移量
        std::vector<double> offsets;
        offsets.push_back(0.0); 
        // 确保 num_samples 是奇数以保持对称，或者向下取整
        int side_samples = (cfg_.num_samples - 1) / 2;
        for (int i = 1; i <= side_samples; ++i) {
            double d = i * cfg_.sample_width;
            if (d <= cfg_.max_width) {
                offsets.push_back(d);
                offsets.push_back(-d);
            }
        }
        // 历史一致性启发
        if (std::abs(last_best_offset_) > 0.01) {
            // 如果上一帧的 offset 不在标准采样中，额外加入它
            bool exists = false;
            for(double o : offsets) if(std::abs(o - last_best_offset_) < 0.01) exists = true;
            if(!exists) offsets.insert(offsets.begin(), last_best_offset_);
        }

        CandidatePath best_path;
        best_path.cost = std::numeric_limits<double>::infinity();
        bool found_valid = false;

        // 2. 采样与评分
        for (double offset : offsets) {
            CandidatePath candidate;
            candidate.lateral_offset = offset;
            candidate.collision = false;
            
            generate_offset_path(global_path, start_idx, offset, candidate.points);
            
            if (candidate.points.empty()) continue;

            if (check_collision(candidate.points, obstacles)) {
                candidate.collision = true;
                candidate.cost = std::numeric_limits<double>::infinity(); // 依然记录，用于可视化(红色)
            } else {
                // Cost Function
                candidate.cost = cfg_.w_offset * std::abs(offset) + 
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
            if (!last_candidates_.empty()) return last_candidates_[0]; // 返回中心线（即使碰撞）
            return CandidatePath();
        }
    }

private:
    Config cfg_;
    double last_best_offset_ = 0.0;
    std::vector<CandidatePath> last_candidates_;

    void generate_offset_path(const std::vector<Point2D>& ref_path, 
                              int start_idx, 
                              double offset, 
                              std::vector<Point2D>& out_points) 
    {
        double current_dist = 0.0;
        out_points.clear();
        
        if (start_idx >= (int)ref_path.size()) return;

        // 使用配置的分辨率进行重采样逻辑
        double accum_segment = 0.0;
        
        for (int i = start_idx; i < (int)ref_path.size() - 1; ++i) {
            const auto& p_curr = ref_path[i];
            const auto& p_next = ref_path[i+1];
            
            // 当前点的平移坐标
            double nx = -std::sin(p_curr.yaw);
            double ny =  std::cos(p_curr.yaw);
            Point2D pt_shifted;
            pt_shifted.x = p_curr.x + nx * offset;
            pt_shifted.y = p_curr.y + ny * offset;
            pt_shifted.yaw = p_curr.yaw;
            
            // 第一个点直接加入
            if (out_points.empty()) {
                out_points.push_back(pt_shifted);
            } else {
                // 计算与上一个加入点的距离
                const auto& last_pt = out_points.back();
                double dist = std::hypot(pt_shifted.x - last_pt.x, pt_shifted.y - last_pt.y);
                
                // 只有当距离超过设定的 resolution 时才加入
                // (这是一个简单的降采样，如果 path_resolution 很大，可以起到稀疏化的作用)
                if (dist >= cfg_.path_resolution) {
                    out_points.push_back(pt_shifted);
                }
            }

            double seg_len = std::hypot(p_next.x - p_curr.x, p_next.y - p_curr.y);
            current_dist += seg_len;
            if (current_dist > cfg_.lookahead_dist) break;
        }
    }

    int find_closest_index(const RobotState& robot, const std::vector<Point2D>& path) {
        double min_dist = 1e9;
        int idx = 0;
        for (int i = 0; i < (int)path.size(); ++i) {
            double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
            if (d < min_dist) {
                min_dist = d;
                idx = i;
            }
        }
        return idx;
    }

    bool check_collision(const std::vector<Point2D>& pts, const std::vector<Point2D>& obstacles) {
        if (obstacles.empty()) return false;
        
        // 使用配置的碰撞半径
        double r = cfg_.collision_radius;
        double safety_margin_sq = r * r; 
        
        // 降采样检测以提高性能
        int step = 1;
        if (cfg_.path_resolution < 0.1) step = 2; // 如果分辨率太密，跳着检

        for (size_t i = 0; i < pts.size(); i += step) { 
            for (const auto& obs : obstacles) {
                double dx = pts[i].x - obs.x;
                double dy = pts[i].y - obs.y;
                if (dx*dx + dy*dy < safety_margin_sq) return true;
            }
        }
        return false;
    }
};

#endif