#include "simuav/physics/QuadrotorModel.h"
#include <cmath>

namespace simuav::physics {

static constexpr double kGravity = 9.80665; // m/s²

QuadrotorModel::QuadrotorModel(QuadrotorParams params)
    : params_(std::move(params)) {}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
QuadrotorModel::computeAccelerations(const State& s,
                                      const std::array<double, kNumMotors>& w,
                                      const Eigen::Vector3d& wind_ned) const
{
    // --- Thrust and body torques ----------------------------------------
    const double L  = params_.arm_length;
    const double kt = params_.k_thrust;
    const double kd = params_.k_drag;

    const double w0sq = w[0] * w[0];
    const double w1sq = w[1] * w[1];
    const double w2sq = w[2] * w[2];
    const double w3sq = w[3] * w[3];

    const double total_thrust = kt * (w0sq + w1sq + w2sq + w3sq);

    // Body FRD: thrust is along -Z (upward)
    const Eigen::Vector3d thrust_body(0.0, 0.0, -total_thrust);

    // Rotate thrust into NED world frame
    const Eigen::Matrix3d R = s.attitude.toRotationMatrix(); // body → NED
    const Eigen::Vector3d thrust_ned = R * thrust_body;

    // --- Translational drag against relative airspeed -------------------
    const Eigen::Vector3d rel_vel = s.velocity - wind_ned;
    const Eigen::Vector3d drag_ned = -params_.aero_drag * rel_vel;

    // --- Gravity (NED: positive Z = down) --------------------------------
    const Eigen::Vector3d gravity_ned(0.0, 0.0, params_.mass * kGravity);

    const Eigen::Vector3d lin_accel =
        (thrust_ned + drag_ned + gravity_ned) / params_.mass;

    // --- Rotational dynamics (body FRD) ----------------------------------
    const Eigen::Vector3d torque_body(
        L * kt * (w0sq + w2sq - w1sq - w3sq),   // roll  (+right)
        L * kt * (w0sq + w1sq - w2sq - w3sq),   // pitch (+nose-up)
        kd      * (w0sq - w1sq - w2sq + w3sq)   // yaw   (+right, reactive)
    );

    const Eigen::Array3d I = params_.inertia_diag.array();
    const Eigen::Vector3d& omega = s.angular_velocity;

    // Euler's rigid-body equation: α = I⁻¹ (τ − ω × (I ω))
    const Eigen::Vector3d gyro_term(
        (I[1] - I[2]) * omega[1] * omega[2],
        (I[2] - I[0]) * omega[2] * omega[0],
        (I[0] - I[1]) * omega[0] * omega[1]
    );

    const Eigen::Vector3d ang_accel =
        (torque_body + gyro_term).array() / I;

    return {lin_accel, ang_accel};
}

void QuadrotorModel::integrate(const std::array<double, kNumMotors>& motor_speeds,
                                double dt,
                                const Eigen::Vector3d& wind_ned)
{
    // Clamp motor speeds to physical limits
    std::array<double, kNumMotors> w{};
    for (int i = 0; i < kNumMotors; ++i) {
        w[i] = std::max(0.0, std::min(motor_speeds[i], params_.max_motor_speed));
    }

    auto [lin_accel, ang_accel] = computeAccelerations(state_, w, wind_ned);
    last_accel_world_ = lin_accel;

    // Semi-implicit Euler integration
    state_.velocity         += dt * lin_accel;
    state_.position         += dt * state_.velocity;
    state_.angular_velocity += dt * ang_accel;

    // Quaternion kinematics: q_dot = 0.5 * q ⊗ [0, ω]
    const Eigen::Quaterniond omega_quat(
        0.0,
        state_.angular_velocity.x(),
        state_.angular_velocity.y(),
        state_.angular_velocity.z()
    );
    const Eigen::Quaterniond dq = state_.attitude * omega_quat;
    state_.attitude.coeffs() += 0.5 * dt * dq.coeffs();
    state_.attitude.normalize();

    state_.time += dt;
}

}  // namespace simuav::physics
