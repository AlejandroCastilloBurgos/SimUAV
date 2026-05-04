#include "simuav/physics/QuadrotorModel.h"
#include <cmath>

namespace simuav::physics {

static constexpr double kGravity = 9.80665; // m/s²

// State time-derivative used by the RK4 stepper (internal to this TU).
struct Deriv {
    Eigen::Vector3d vel;      // d(position)/dt
    Eigen::Vector3d accel;    // d(velocity)/dt
    Eigen::Vector4d dq;       // d(attitude.coeffs)/dt
    Eigen::Vector3d domega;   // d(angular_velocity)/dt
};

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
    // Clamp commanded speeds to physical limits.
    std::array<double, kNumMotors> w_cmd{};
    for (int i = 0; i < kNumMotors; ++i) {
        w_cmd[i] = std::max(0.0, std::min(motor_speeds[i], params_.max_motor_speed));
    }

    // First-order motor lag: ω_actual += (dt/τ) * (ω_cmd − ω_actual).
    // When τ ≤ 0 skip the filter so the model stays instantaneous.
    std::array<double, kNumMotors> w{};
    if (params_.motor_time_constant_s > 0.0) {
        const double alpha = dt / params_.motor_time_constant_s;
        for (int i = 0; i < kNumMotors; ++i) {
            motor_speeds_actual_[i] += alpha * (w_cmd[i] - motor_speeds_actual_[i]);
            w[i] = motor_speeds_actual_[i];
        }
    } else {
        for (int i = 0; i < kNumMotors; ++i) {
            motor_speeds_actual_[i] = w_cmd[i];
            w[i] = w_cmd[i];
        }
    }

    // Compute the full state derivative at state s.
    auto deriv = [&](const State& s) -> Deriv {
        auto [la, aa] = computeAccelerations(s, w, wind_ned);
        const Eigen::Quaterniond oq(0.0, s.angular_velocity.x(),
                                    s.angular_velocity.y(),
                                    s.angular_velocity.z());
        return {s.velocity, la, 0.5 * (s.attitude * oq).coeffs(), aa};
    };

    // Advance a state by h * d without updating time.
    auto advance = [](const State& s, const Deriv& d, double h) -> State {
        State out = s;
        out.position         += h * d.vel;
        out.velocity         += h * d.accel;
        out.attitude.coeffs() = (s.attitude.coeffs() + h * d.dq).normalized();
        out.angular_velocity += h * d.domega;
        return out;
    };

    if (params_.use_rk4) {
        const Deriv k1 = deriv(state_);
        const Deriv k2 = deriv(advance(state_, k1, 0.5 * dt));
        const Deriv k3 = deriv(advance(state_, k2, 0.5 * dt));
        const Deriv k4 = deriv(advance(state_, k3, dt));

        last_accel_world_ = k1.accel;

        const Deriv combined {
            k1.vel    + 2.0*k2.vel    + 2.0*k3.vel    + k4.vel,
            k1.accel  + 2.0*k2.accel  + 2.0*k3.accel  + k4.accel,
            k1.dq     + 2.0*k2.dq     + 2.0*k3.dq     + k4.dq,
            k1.domega + 2.0*k2.domega + 2.0*k3.domega + k4.domega,
        };
        state_ = advance(state_, combined, dt / 6.0);
        state_.attitude.normalize();
    } else {
        // Semi-implicit Euler (legacy path — kept for benchmarking)
        const Deriv d = deriv(state_);
        last_accel_world_ = d.accel;
        state_.velocity         += dt * d.accel;
        state_.position         += dt * state_.velocity;  // semi-implicit: uses updated velocity
        state_.angular_velocity += dt * d.domega;
        state_.attitude.coeffs() = (state_.attitude.coeffs() + dt * d.dq).normalized();
    }

    // Ground constraint (NED: z > 0 is below ground level).
    if (params_.enable_ground_constraint && state_.position.z() > 0.0) {
        state_.position.z() = 0.0;
        state_.velocity.z() = std::min(state_.velocity.z(), 0.0);
        if (last_accel_world_.z() > 0.0)
            last_accel_world_.z() = 0.0;
    }

    state_.time += dt;
}

}  // namespace simuav::physics
