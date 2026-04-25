#include "simuav/sensors/IMU.h"

namespace simuav::sensors {

static constexpr double kGravity = 9.80665;

IMU::IMU(IMUParams params, uint64_t seed)
    : SensorBase(seed), params_(std::move(params)) {}

IMUSample IMU::sample(const physics::State& state,
                       const Eigen::Vector3d& accel_world_ned)
{
    const Eigen::Matrix3d R = state.attitude.toRotationMatrix(); // body → NED

    // Specific force: total NED acceleration minus gravity, rotated to body frame
    const Eigen::Vector3d gravity_ned(0.0, 0.0, kGravity);
    const Eigen::Vector3d specific_force_ned = accel_world_ned - gravity_ned;
    const Eigen::Vector3d accel_true = R.transpose() * specific_force_ned;

    // Bias random walk
    accel_bias_ += Eigen::Vector3d(white(params_.accel_bias_std),
                                    white(params_.accel_bias_std),
                                    white(params_.accel_bias_std));
    gyro_bias_  += Eigen::Vector3d(white(params_.gyro_bias_std),
                                    white(params_.gyro_bias_std),
                                    white(params_.gyro_bias_std));

    IMUSample out;
    out.timestamp = state.time;
    out.accel_body = accel_true + accel_bias_ +
                     Eigen::Vector3d(white(params_.accel_noise_std),
                                     white(params_.accel_noise_std),
                                     white(params_.accel_noise_std));
    out.gyro_body  = state.angular_velocity + gyro_bias_ +
                     Eigen::Vector3d(white(params_.gyro_noise_std),
                                     white(params_.gyro_noise_std),
                                     white(params_.gyro_noise_std));
    return out;
}

}  // namespace simuav::sensors
