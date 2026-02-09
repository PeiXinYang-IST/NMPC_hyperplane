#ifndef SFC_GENERATOR_HPP
#define SFC_GENERATOR_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <Eigen/Dense>

// 使用 KDTree 需要引入额外的库，或者简单的暴力搜索
// 为了保持库的轻量级和独立性，这里我们使用简单的空间哈希或暴力搜索（取决于点数）
// 如果点数较多，建议引入 nanoflann 等轻量级 KDTree 库
// 这里为了演示，假设障碍物数量不多，使用暴力搜索或简单的网格加速

struct SFC_Config {
    double robot_radius = 0.5;
    double search_radius = 6.0;
    double longitudinal_length = 4.0; // 沿路径方向的最大前后搜索距离
};

// 对应 Ax <= b 的线性约束
struct SFC_Constraint {
    // A 矩阵: 4x2 (4个平面的法向量)
    Eigen::Matrix<double, 4, 2> A; 
    // b 向量: 4x1
    Eigen::Vector4d b; 
    // 矩形的四个顶点 (用于可视化调试), 4x2
    Eigen::Matrix<double, 4, 2> vertices; 
};

class SFCGenerator {
public:
    explicit SFCGenerator(const SFC_Config& config);
    ~SFCGenerator() = default;

    /**
     * @brief 生成固定维度的凸包（定向矩形走廊）
     * @param path_points 路径点集 [N, 2]
     * @param obstacles 障碍物点云 [M, 2]
     * @return 约束列表，长度为 N
     */
    std::vector<SFC_Constraint> generate_corridor(
        const std::vector<Eigen::Vector2d>& path_points,
        const std::vector<Eigen::Vector2d>& obstacles
    );

    void update_config(const SFC_Config& config);

private:
    SFC_Config cfg_;

    // 内部辅助函数：在局部坐标系下根据障碍物收缩边界
    // 返回 {d_front, d_back, d_left, d_right}
    Eigen::Vector4d shrink_box(
        const std::vector<Eigen::Vector2d>& local_obs
    );
};

#endif // SFC_GENERATOR_HPP