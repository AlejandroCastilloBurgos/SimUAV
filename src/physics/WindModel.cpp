#include "simuav/physics/WindModel.h"
#include <cmath>

namespace simuav::physics {

WindModel::WindModel(WindParams params, double dt, uint64_t seed)
    : params_(std::move(params)), dt_(dt), rng_(seed)
{
    if (!params_.use_dryden) return;

    const double V = params_.dryden_airspeed;

    // --- u/N: 1st-order AR(1) ---
    // Exact discretisation preserving variance σ_u²: x = a·x + b·ξ
    const double tau_u = params_.dryden_length_u / V;
    dry_au_ = std::exp(-dt_ / tau_u);
    dry_bu_ = params_.dryden_sigma_u * std::sqrt(1.0 - dry_au_ * dry_au_);

    // --- v/E: 2-state Euler (controllable canonical form) ---
    // Continuous: ẋ = A·x + B·w(t),  w ~ N(0, π) per unit time
    //   A = [[0, 1], [-1/τ², -2/τ]],  B = [0, 1]ᵀ
    //   y = K·[1/τ², √3/τ]·x,  K = σ·√(τ/π)
    // Output variance = σ²  (verified via Lyapunov equation)
    dry_tau_v_   = params_.dryden_length_v / V;
    const double Kv   = params_.dryden_sigma_v * std::sqrt(dry_tau_v_ / M_PI);
    dry_out_v1_  = Kv / (dry_tau_v_ * dry_tau_v_);
    dry_out_v2_  = Kv * std::sqrt(3.0) / dry_tau_v_;

    // --- w/D: same form as v ---
    dry_tau_w_   = params_.dryden_length_w / V;
    const double Kw   = params_.dryden_sigma_w * std::sqrt(dry_tau_w_ / M_PI);
    dry_out_w1_  = Kw / (dry_tau_w_ * dry_tau_w_);
    dry_out_w2_  = Kw * std::sqrt(3.0) / dry_tau_w_;

    // Shared noise gain for both 2nd-order filters: √(π·dt)
    dry_noise_vw_ = std::sqrt(M_PI * dt_);
}

Eigen::Vector3d WindModel::drydenSample() {
    // --- u/N: AR(1) ---
    dry_xu_ = dry_au_ * dry_xu_ + dry_bu_ * normal_(rng_);

    // --- v/E: 2-state Euler step ---
    // x1_dot = x2,  x2_dot = -x1/τ² - 2x2/τ + w(t)
    {
        const double x1 = dry_xv_[0];
        const double x2 = dry_xv_[1];
        dry_xv_[0] = x1 + dt_ * x2;
        dry_xv_[1] = x2 + dt_ * (-x1 / (dry_tau_v_ * dry_tau_v_)
                                  - 2.0 * x2 / dry_tau_v_)
                       + dry_noise_vw_ * normal_(rng_);
    }

    // --- w/D: 2-state Euler step ---
    {
        const double x1 = dry_xw_[0];
        const double x2 = dry_xw_[1];
        dry_xw_[0] = x1 + dt_ * x2;
        dry_xw_[1] = x2 + dt_ * (-x1 / (dry_tau_w_ * dry_tau_w_)
                                  - 2.0 * x2 / dry_tau_w_)
                       + dry_noise_vw_ * normal_(rng_);
    }

    return {
        dry_xu_,
        dry_out_v1_ * dry_xv_[0] + dry_out_v2_ * dry_xv_[1],
        dry_out_w1_ * dry_xw_[0] + dry_out_w2_ * dry_xw_[1]
    };
}

Eigen::Vector3d WindModel::sample() {
    // Turbulence
    const Eigen::Vector3d turbulence = params_.use_dryden
        ? drydenSample()
        : Eigen::Vector3d{
              params_.turbulence_std * normal_(rng_),
              params_.turbulence_std * normal_(rng_),
              params_.turbulence_std * normal_(rng_)};

    // Discrete gust: random horizontal direction, exponential decay
    if (uniform_(rng_) < params_.gust_probability) {
        Eigen::Vector3d dir(normal_(rng_), normal_(rng_), 0.0);
        if (dir.norm() > 1e-6)
            current_gust_ = params_.gust_magnitude * dir.normalized();
    }
    current_gust_ *= 0.95;

    return params_.mean_ned + turbulence + current_gust_;
}

}  // namespace simuav::physics
