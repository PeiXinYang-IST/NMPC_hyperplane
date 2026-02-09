#include "sfc_generator.hpp"
#include <limits>
#include <iostream>

SFCGenerator::SFCGenerator(const SFC_Config& config) : cfg_(config) {}

void SFCGenerator::update_config(const SFC_Config& config) {
    cfg_ = config;
}

std::vector<SFC_Constraint> SFCGenerator::generate_corridor(
    const std::vector<Eigen::Vector2d>& path_points,
    const std::vector<Eigen::Vector2d>& obstacles) 
{
    size_t N = path_points.size();
    std::vector<SFC_Constraint> constraints;
    constraints.reserve(N);

    if (N == 0) return constraints;

    // 1. 计算每个点的 Yaw 角
    std::vector<double> yaws(N);
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector2d diff;
        if (i < N - 1) {
            diff = path_points[i+1] - path_points[i];
        } else {
            // 最后一个点沿用前一个方向
            diff = path_points[i] - path_points[i-1];
        }
        
        // 防止重合点导致的计算错误
        if (diff.norm() < 1e-3) {
            yaws[i] = (i > 0) ? yaws[i-1] : 0.0;
        } else {
            yaws[i] = std::atan2(diff.y(), diff.x());
        }
    }

    // 2. 遍历路径点生成约束
    for (size_t i = 0; i < N; ++i) {
        const Eigen::Vector2d& seed = path_points[i];
        double yaw = yaws[i];
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);

        // 构建旋转矩阵 R_world_to_local (2x2)
        // Local X 指向切线方向
        Eigen::Matrix2d R_w2l;
        R_w2l << cos_yaw, sin_yaw,
                -sin_yaw, cos_yaw;

        // 搜索附近的障碍物并转到局部坐标系
        std::vector<Eigen::Vector2d> local_obs;
        double search_r_sq = cfg_.search_radius * cfg_.search_radius;
        
        // 注意：这里使用暴力搜索。如果 obstacles 数量 > 500，建议使用 KDTree。
        // 对于 NMPC 局部规划，通常只处理局部地图的点云，数量有限。
        for (const auto& obs : obstacles) {
            Eigen::Vector2d diff = obs - seed;
            if (diff.squaredNorm() < search_r_sq) {
                // 转换到局部坐标系: p_local = R * (p_world - seed)
                local_obs.push_back(R_w2l * diff);
            }
        }

        // 计算收缩后的边界 [front, back, left, right]
        Eigen::Vector4d bounds = shrink_box(local_obs);
        double d_front = bounds[0];
        double d_back  = bounds[1];
        double d_left  = bounds[2];
        double d_right = bounds[3];

        // 构建局部 A, b
        // n_front = (1, 0), n_back = (-1, 0), n_left = (0, 1), n_right = (0, -1)
        Eigen::Matrix<double, 4, 2> A_local;
        A_local <<  1.0,  0.0,
                   -1.0,  0.0,
                    0.0,  1.0,
                    0.0, -1.0;
        
        Eigen::Vector4d b_local;
        b_local << d_front, -d_back, d_left, -d_right;

        // 转换回世界坐标系
        // A_world = A_local * R_w2l
        Eigen::Matrix<double, 4, 2> A_world = A_local * R_w2l;

        // b_world = b_local + A_world * seed
        // 这一步是因为: A_world * (x - seed) <= b_local  =>  A_world * x <= b_local + A_world * seed
        Eigen::Vector4d b_world = b_local + A_world * seed;

        // 计算顶点 (Local -> World)
        // Local Vertices: (front, left), (back, left), (back, right), (front, right)
        // 为了绘图顺序通常是逆时针
        Eigen::Matrix<double, 4, 2> v_local;
        v_local << d_front, d_left,  // Top-Right (relative to orientation)
                   d_back,  d_left,  // Top-Left
                   d_back,  d_right, // Bottom-Left
                   d_front, d_right; // Bottom-Right
        
        // v_world = (R_w2l^T * v_local^T)^T + seed = v_local * R_w2l + seed
        // 因为 R^T = R^-1, 我们要把局部点转回世界点
        // p_world = R_w2l^T * p_local + seed
        Eigen::Matrix<double, 4, 2> v_world;
        for(int k=0; k<4; ++k) {
             v_world.row(k) = (R_w2l.transpose() * v_local.row(k).transpose()).transpose() + seed.transpose();
        }

        SFC_Constraint cons;
        cons.A = A_world;
        cons.b = b_world;
        cons.vertices = v_world;
        constraints.push_back(cons);
    }

    return constraints;
}

Eigen::Vector4d SFCGenerator::shrink_box(const std::vector<Eigen::Vector2d>& local_obs) {
    // 初始边界
    double d_front = cfg_.longitudinal_length / 2.0;
    double d_back  = -cfg_.longitudinal_length / 2.0;
    double d_left  = cfg_.search_radius;  // +Y
    double d_right = -cfg_.search_radius; // -Y

    double margin = cfg_.robot_radius + 0.1;

    // 遍历局部障碍物进行收缩
    for (const auto& p : local_obs) {
        double px = p.x();
        double py = p.y();

        // 1. 左右收缩 (Lateral Contraction)
        // 只有当障碍物位于当前的 [d_back, d_front] 范围内时，才考虑收缩左右边界
        if (px > d_back && px < d_front) {
            // 在左侧 (+Y)
            if (py > 0 && py < d_left) {
                d_left = std::max(0.0, py - margin);
            }
            // 在右侧 (-Y)
            else if (py < 0 && py > d_right) {
                d_right = std::min(0.0, py + margin);
            }
        }

        // 2. 前后收缩 (Longitudinal Contraction)
        // 只有当障碍物位于当前的 [d_right, d_left] 范围内时，才考虑收缩前后边界
        if (py > d_right && py < d_left) {
            // 在前方 (+X)
            if (px > 0 && px < d_front) {
                d_front = std::max(0.0, px - margin);
            }
            // 在后方 (-X)
            else if (px < 0 && px > d_back) {
                d_back = std::min(0.0, px + margin);
            }
        }
    }

    return Eigen::Vector4d(d_front, d_back, d_left, d_right);
}