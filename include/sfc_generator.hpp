#ifndef SFC_GENERATOR_HPP
#define SFC_GENERATOR_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

// 基础几何结构
struct SFC_Point { double x, y; };
struct SFC_Pose { double x, y, yaw; };

// 输出的墙体参数 (对应 NMPC 的超平面)
struct SFC_WallParams {
    double ox_left, oy_left, nx_left, ny_left;   // 左墙
    double ox_right, oy_right, nx_right, ny_right; // 右墙
    double width; // 实际计算出的单侧宽度(用于调试)
};

class SFCGenerator {
public:
    struct Config {
        double max_road_width = 5.0;   // 赛道半宽 (上限)
        double min_valid_width = 0.6;  // 最小通行半宽 (下限，防死锁)
        double security_margin = 0.5;  // 基础膨胀半径 (车辆半径 + 余量)
        double smoothing_factor = 0.3; // 走廊宽度平滑系数 (0~1, 越小越平滑)
    };

    explicit SFCGenerator(Config cfg) : cfg_(cfg) {}

    /**
     * @brief 核心函数：生成整个预测视界的安全走廊
     * @param horizon_refs NMPC 预测视界内的参考点列表 (Size = N)
     * @param obstacles 障碍物点云列表
     * @return 对应的虚拟墙参数列表 (Size = N)
     */
    std::vector<SFC_WallParams> generate_corridor(
        const std::vector<SFC_Pose>& horizon_refs,
        const std::vector<SFC_Point>& obstacles
    );

    // 允许动态更新配置
    void update_config(const Config& cfg) { cfg_ = cfg; }

private:
    Config cfg_;
    
    // 内部状态：用于平滑处理
    double last_first_step_width_ = 5.0; 
};

#endif // SFC_GENERATOR_HPP