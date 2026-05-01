#include "simuav/sensors/IMU.h"
#include <cmath>

namespace simuav::sensors {

static constexpr double kGravity = 9.80665;
static constexpr double kTwoPi   = 6.283185307179586;

IMU::IMU(IMUParams params, uint64_t seed)
    : SensorBase(seed), params_(std::move(params)) {}

// Advance one Gauss-Markov axis: b_new = (1 - dt/τ)*b + sqrt(2σ²/τ)*N(0,1)*sqrt(dt)
static double gaussMarkovStep(double b, double dt, double tau, double sigma, double noise) {
    if (tau <= 0.0) return b; // degenerate — leave unchanged
    const double decay    = 1.0 - dt / tau;
    const double drive_sd = std::sqrt(2.0 * sigma * sigma / tau) * std::sqrt(dt);
    return decay * b + drive_sd * noise;
}

IMUSample IMU::sample(const physics::State& state,
                       const Eigen::Vector3d& accel_world_ned)
{
    const Eigen::Matrix3d R = state.attitude.toRotationMatrix(); // body → NED

    // Specific force: total NED acceleration minus gravity, rotated to body frame
    const Eigen::Vector3d gravity_ned(0.0, 0.0, kGravity);
    const Eigen::Vector3d specific_force_ned = accel_world_ned - gravity_ned;
    const Eigen::Vector3d accel_true = R.transpose() * specific_force_ned;

    // Gauss-Markov bias update (requires dt from state time, defaulting 0.004 s if unknown)
    const double dt = last_time_ < 0.0 ? 0.004 : (state.time - last_time_);
    last_time_ = state.time;

    const double at = params_.accel_bias_instability_tc_s;
    const double as = params_.accel_bias_instability;
    for (int i = 0; i < 3; ++i) {
        accel_bias_[i] = gaussMarkovStep(accel_bias_[i], dt, at, as, white(1.0));
    }

    const double gt = params_.gyro_bias_instability_tc_s;
    const double gs = params_.gyro_bias_instability;
    for (int i = 0; i < 3; ++i) {
        gyro_bias_[i] = gaussMarkovStep(gyro_bias_[i], dt, gt, gs, white(1.0));
    }

    // Vibration: sinusoidal on body-Z accel
    double vibration_z = 0.0;
    if (params_.vibration_amplitude_mps2 > 0.0 && params_.vibration_frequency_hz > 0.0) {
        vibration_z = params_.vibration_amplitude_mps2 *
                      std::sin(kTwoPi * params_.vibration_frequency_hz * state.time);
    }

    IMUSample out;
    out.timestamp = state.time;
    out.accel_body = accel_true + accel_bias_ +
                     Eigen::Vector3d(white(params_.accel_arw_std),
                                     white(params_.accel_arw_std),
                                     white(params_.accel_arw_std) + vibration_z);
    out.gyro_body  = state.angular_velocity + gyro_bias_ +
                     Eigen::Vector3d(white(params_.gyro_arw_std),
                                     white(params_.gyro_arw_std),
                                     white(params_.gyro_arw_std));
    return out;
}

}  // namespace simuav::sensors
