#ifndef ESO_HPP
#define ESO_HPP

#include <cmath>
#include <algorithm>

class ESO {
public:
    struct Config {
        double b0;          
        double omega_o;    
        double dt = 0.1;    
        double max_dist;    
        double alpha = 0.5; 
        double delta = 0.05; 
        double lpf_tau = 0.2; 
    };

    ESO(Config cfg) : cfg_(cfg) {
        beta1_ = 2.0 * cfg_.omega_o; 
        beta2_ = cfg_.omega_o * cfg_.omega_o;
        reset();
    }

    void reset() {
        z1_ = 0.0; 
        z2_ = 0.0; 
        raw_z2_ = 0.0;
    }

    double fal(double e, double alpha, double delta) {
        double abs_e = std::abs(e);
        if (abs_e <= delta) {
            return e / std::pow(delta, 1.0 - alpha);
        } else {
            return std::pow(abs_e, alpha) * (e > 0 ? 1.0 : -1.0);
        }
    }

    void update(double u, double y) {
        double err = y - z1_;
        double fe = fal(err, cfg_.alpha, cfg_.delta);

        double prev_z1 = z1_;
        z1_ += (z2_ + cfg_.b0 * u + beta1_ * fe) * cfg_.dt;

        double dz2 = (beta2_ * fe) * cfg_.dt;
        raw_z2_ += dz2;

        double k = cfg_.dt / (cfg_.lpf_tau + cfg_.dt);
        z2_ = (1.0 - k) * z2_ + k * raw_z2_;

        // 限幅保护
        z2_ = std::clamp(z2_, -cfg_.max_dist, cfg_.max_dist);
        raw_z2_ = std::clamp(raw_z2_, -cfg_.max_dist, cfg_.max_dist);
    }

    double get_disturbance() const { return z2_; }
    double get_estimated_state() const { return z1_; }

private:
    Config cfg_;
    double z1_, z2_, raw_z2_;
    double beta1_, beta2_;
};
#endif