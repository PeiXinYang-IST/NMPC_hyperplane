#ifndef ESO_HPP
#define ESO_HPP

#include <cmath>
#include <algorithm>

/**
 * @brief 线性扩张状态观测器 (LESO)
 * 用于估计: x_dot = b0 * u + f (总扰动)
 */
class ESO {
public:
    struct Config {
        double b0;          // 系统标称增益
        double omega_o;     // 观测带宽
        double dt;          // 周期
        double max_dist;    // 扰动限幅
    };

    ESO(Config cfg) : cfg_(cfg) {
        beta1_ = 2.0 * cfg_.omega_o;
        beta2_ = cfg_.omega_o * cfg_.omega_o;
        reset();
    }

    void reset() {
        z1_ = 0.0; // 状态估计
        z2_ = 0.0; // 扰动估计
    }

    void update(double u, double y) {
        double err = y - z1_;
        z1_ += (z2_ + cfg_.b0 * u + beta1_ * err) * cfg_.dt;
        z2_ += (beta2_ * err) * cfg_.dt;
        z2_ = std::clamp(z2_, -cfg_.max_dist, cfg_.max_dist);
    }

    double get_disturbance() const { return z2_; }
    double get_estimated_state() const { return z1_; }

private:
    Config cfg_;
    double z1_, z2_;
    double beta1_, beta2_;
};
#endif