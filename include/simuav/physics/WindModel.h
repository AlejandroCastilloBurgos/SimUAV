#pragma once
#include <Eigen/Dense>
#include <random>

namespace simuav::physics {

struct WindParams {
    // --- Steady wind + gusts (always active) ---
    Eigen::Vector3d mean_ned{Eigen::Vector3d::Zero()}; // m/s, constant mean
    double gust_probability{0.005}; // probability per step of a discrete gust
    double gust_magnitude{3.0};     // m/s, peak gust speed

    // --- Turbulence mode ---
    // use_dryden = true : Dryden frequency-shaped turbulence (MIL-HDBK-1797)
    // use_dryden = false: white-noise turbulence (legacy)
    bool   use_dryden{true};
    double turbulence_std{0.5};     // m/s, white-noise std (use_dryden = false only)

    // --- Dryden parameters (use_dryden = true) ---
    double dryden_airspeed{15.0};   // m/s, equivalent airspeed
    double dryden_length_u{200.0};  // m, scale length for N/u component
    double dryden_length_v{200.0};  // m, scale length for E/v component
    double dryden_length_w{50.0};   // m, scale length for D/w component
    double dryden_sigma_u{1.5};     // m/s, turbulence intensity, N/u
    double dryden_sigma_v{1.5};     // m/s, turbulence intensity, E/v
    double dryden_sigma_w{0.75};    // m/s, turbulence intensity, D/w
};

// Produces a wind velocity vector in NED each simulation step.
// Models: constant mean + Dryden (or white-noise) turbulence + random discrete gusts.
//
// Dryden shaping filters (MIL-HDBK-1797):
//   u/N (1st-order AR):  H_u(s) = σ·√(2τ/π) / (1 + τ·s),  τ = L/V
//   v/E, w/D (2nd-order): H(s)  = σ·√(τ/π) · (1 + √3·τ·s) / (1 + τ·s)²
class WindModel {
public:
    explicit WindModel(WindParams params = {}, double dt = 0.004, uint64_t seed = 0);

    // Call once per physics step. Returns wind velocity in NED (m/s).
    Eigen::Vector3d sample();

private:
    Eigen::Vector3d drydenSample();

    WindParams params_;
    double dt_;

    // Gust state
    Eigen::Vector3d current_gust_{Eigen::Vector3d::Zero()};

    // Dryden state
    double          dry_xu_{0.0};                        // AR(1) state, u/N
    Eigen::Vector2d dry_xv_{Eigen::Vector2d::Zero()};    // 2-state, v/E
    Eigen::Vector2d dry_xw_{Eigen::Vector2d::Zero()};    // 2-state, w/D

    // Precomputed Dryden coefficients
    double dry_au_{0.0}, dry_bu_{0.0};       // AR(1): x = a·x + b·ξ
    double dry_tau_v_{1.0};                  // τ_v = L_v/V
    double dry_out_v1_{0.0}, dry_out_v2_{0.0}; // output weights for xv
    double dry_tau_w_{1.0};
    double dry_out_w1_{0.0}, dry_out_w2_{0.0};
    double dry_noise_vw_{0.0};               // √(π·dt): noise gain for v/w states

    std::mt19937_64 rng_;
    std::normal_distribution<double>           normal_{0.0, 1.0};
    std::uniform_real_distribution<double>     uniform_{0.0, 1.0};
};

}  // namespace simuav::physics
