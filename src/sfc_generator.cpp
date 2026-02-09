#include "sfc_generator.hpp"
#include <limits>

std::vector<SFC_WallParams> SFCGenerator::generate_corridor(
    const std::vector<SFC_Pose>& horizon_refs,
    const std::vector<SFC_Point>& obstacles) 
{
    size_t N = horizon_refs.size();
    std::vector<SFC_WallParams> corridor(N);
    
    // 临时存储左右宽度
    std::vector<double> left_widths(N, cfg_.max_road_width);
    std::vector<double> right_widths(N, cfg_.max_road_width);

    // ---------------------------------------------------------
    // Step 1: 左右分离的宽度计算 (Left/Right Independent Calculation)
    // ---------------------------------------------------------
    for (size_t i = 0; i < N; ++i) {
        const auto& ref = horizon_refs[i];
        double yaw = ref.yaw;
        double cy = std::cos(yaw);
        double sy = std::sin(yaw);

        // 如果没有障碍物，保持 max_width
        if (obstacles.empty()) continue;

        // 构建局部坐标系变换矩阵 (Global -> Local)
        // Local X: 沿车道方向 (Longitudinal)
        // Local Y: 垂直车道方向 (Lateral, 左正右负)
        
        for (const auto& obs : obstacles) {
            double dx = obs.x - ref.x;
            double dy = obs.y - ref.y;

            // 投影到局部坐标系
            // local_x = dx * cos(yaw) + dy * sin(yaw)
            // local_y = -dx * sin(yaw) + dy * cos(yaw)
            double local_x = dx * cy + dy * sy;
            double local_y = -dx * sy + dy * cy;

            // 关键优化：只考虑“当前横截面”附近的障碍物
            // 例如只考虑前后 1.0m 范围内的障碍物对当前宽度的影响
            // 避免弯道时计算到远处的点
            if (std::abs(local_x) > 1.5) continue; 

            double dist = std::sqrt(dx*dx + dy*dy); // 或者直接用 abs(local_y) 近似

            if (local_y > 0) {
                // 障碍物在左侧 -> 限制左宽
                // 有效宽度 = 横向距离 - 安全余量
                // 这里用 abs(local_y) 比 dist 更准确地描述“横向空间”
                double valid_w = std::abs(local_y) - cfg_.security_margin;
                if (valid_w < left_widths[i]) left_widths[i] = valid_w;
            } else {
                // 障碍物在右侧 -> 限制右宽
                double valid_w = std::abs(local_y) - cfg_.security_margin;
                if (valid_w < right_widths[i]) right_widths[i] = valid_w;
            }
        }

        // 钳位 (Clamp)
        left_widths[i] = std::clamp(left_widths[i], cfg_.min_valid_width, cfg_.max_road_width);
        right_widths[i] = std::clamp(right_widths[i], cfg_.min_valid_width, cfg_.max_road_width);
    }

    // ---------------------------------------------------------
    // Step 2: 安全平滑 (Safe Smoothing)
    // ---------------------------------------------------------
    auto apply_safe_smoothing = [&](std::vector<double>& widths) {
        if (widths.empty()) return;
        std::vector<double> smoothed = widths;
        // 前向滤波
        for (size_t i = 1; i < N; ++i) {
            double smooth_val = cfg_.smoothing_factor * widths[i] + 
                                (1.0 - cfg_.smoothing_factor) * smoothed[i-1];
            // [重要] 必须取 Min，确保不会把墙推到障碍物里面去
            smoothed[i] = std::min(widths[i], smooth_val);
        }
        // 反向滤波 (可选，消除相位滞后)
        for (int i = N - 2; i >= 0; --i) {
            double smooth_val = cfg_.smoothing_factor * widths[i] + 
                                (1.0 - cfg_.smoothing_factor) * smoothed[i+1];
            smoothed[i] = std::min(smoothed[i], smooth_val);
        }
        widths = smoothed;
    };

    apply_safe_smoothing(left_widths);
    apply_safe_smoothing(right_widths);

    // ---------------------------------------------------------
    // Step 3: 生成墙体参数
    // ---------------------------------------------------------
    for (size_t i = 0; i < N; ++i) {
        const auto& ref = horizon_refs[i];
        double cy = std::cos(ref.yaw);
        double sy = std::sin(ref.yaw);

        // 法向量指向路中心 (Constraint: n * (p_car - p_wall) >= 0)
        // 左墙法向量: (sy, -cy) 指向右
        double nx_left = sy;
        double ny_left = -cy;
        
        // 右墙法向量: (-sy, cy) 指向左
        double nx_right = -sy;
        double ny_right = cy;

        SFC_WallParams& wall = corridor[i];

        // 左墙位置：沿法向量反方向延伸 left_width
        // P_wall_left = P_ref - n_left * w_left (错，这样会指向左边)
        // 你的原代码逻辑：
        // wall.ox_left = ref.x - nx_left * w; 
        // nx_left 是 (sy, -cy)。 若 yaw=0, nx=(0, -1). 
        // P = (x, y) - (0, -1)*w = (x, y+w). 正确，在左边。
        
        wall.ox_left = ref.x - nx_left * left_widths[i];
        wall.oy_left = ref.y - ny_left * left_widths[i];
        wall.nx_left = nx_left;
        wall.ny_left = ny_left;

        wall.ox_right = ref.x - nx_right * right_widths[i];
        wall.oy_right = ref.y - ny_right * right_widths[i];
        wall.nx_right = nx_right;
        wall.ny_right = ny_right;
        
        // 调试用：记录总宽度
        wall.width = left_widths[i] + right_widths[i];
    }

    return corridor;
}