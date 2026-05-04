#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

namespace simuav::physics {

// Motor indices and spin directions (top view, body FRD: x=forward, y=right, z=down):
//
//   FL(0) -------- FR(1)
//      \            /
//       \          /
//   RL(2) -------- RR(3)
//
// FL=CW, FR=CCW, RL=CCW, RR=CW  (viewed from above)
// Torque mixing:
//   roll  (+right)  = L * kt * ( ω₀² + ω₂² − ω₁² − ω₃² )
//   pitch (+nose-up) = L * kt * ( ω₀² + ω₁² − ω₂² − ω₃² )
//   yaw   (+right)  = kd    * ( ω₀² − ω₁² − ω₂² + ω₃² )

static constexpr int kNumMotors = 4;

struct QuadrotorParams {
    double mass{1.5};               // kg
    double arm_length{0.225};       // m, motor-to-CoM distance
    double k_thrust{9.18e-6};       // N / (rad/s)²
    double k_drag{1.35e-7};         // N·m / (rad/s)²  (reactive yaw torque)
    double aero_drag{0.25};         // N·s/m, translational drag coefficient
    // Diagonal inertia tensor [Ixx, Iyy, Izz] (kg·m²)
    Eigen::Vector3d inertia_diag{0.029, 0.029, 0.055};
    double max_motor_speed{838.0};  // rad/s (~8 000 RPM)
    double esc_exponent{0.5};       // throttle→speed power law: 0.5 = thrust-linear, 1.0 = speed-linear
    double motor_spin_min{0.0};        // rad/s — motor speed at zero throttle (idle)
    bool   use_rk4{true};             // true = RK4 (default), false = semi-implicit Euler
    double motor_time_constant_s{0.04}; // s — first-order electromechanical lag (τ); 0 = instantaneous
    bool   enable_ground_constraint{true}; // prevent vehicle from penetrating z=0 (NED ground plane)
};

// Full rigid-body state expressed in NED world frame.
struct State {
    Eigen::Vector3d    position{Eigen::Vector3d::Zero()};         // m
    Eigen::Vector3d    velocity{Eigen::Vector3d::Zero()};         // m/s
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};  // body → NED
    Eigen::Vector3d    angular_velocity{Eigen::Vector3d::Zero()}; // rad/s, body FRD
    double             time{0.0};                                  // s
};

class QuadrotorModel {
public:
    explicit QuadrotorModel(QuadrotorParams params = {});

    // Advance state by dt seconds.
    // motor_speeds: rad/s per motor (indices 0–3 as above).
    // wind_ned:     external wind velocity in NED frame (m/s).
    void integrate(const std::array<double, kNumMotors>& motor_speeds,
                   double dt,
                   const Eigen::Vector3d& wind_ned = Eigen::Vector3d::Zero());

    const State&           state()              const { return state_; }
    const QuadrotorParams& params()             const { return params_; }
    const Eigen::Vector3d& lastAccelWorld()     const { return last_accel_world_; }
    const std::array<double, kNumMotors>& motorSpeedsActual() const { return motor_speeds_actual_; }
    void setState(const State& s) { state_ = s; }

private:
    // Returns linear acceleration (world NED) and angular acceleration (body FRD).
    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    computeAccelerations(const State& s,
                         const std::array<double, kNumMotors>& motor_speeds,
                         const Eigen::Vector3d& wind_ned) const;

    QuadrotorParams                  params_;
    State                            state_;
    Eigen::Vector3d                  last_accel_world_{Eigen::Vector3d::Zero()};
    std::array<double, kNumMotors>   motor_speeds_actual_{};
};

}  // namespace simuav::physics
