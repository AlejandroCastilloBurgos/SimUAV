#pragma once
#include "SensorBase.h"
#include "simuav/physics/QuadrotorModel.h"
#include <Eigen/Dense>

namespace simuav::sensors {

struct IMUParams {
    double accel_noise_std{0.005};    // m/s², white noise per axis
    double accel_bias_std{0.0001};    // m/s², bias random-walk std per step
    double gyro_noise_std{0.0003};    // rad/s, white noise per axis
    double gyro_bias_std{0.000005};   // rad/s, bias random-walk std per step
};

struct IMUSample {
    double          timestamp{0.0};
    Eigen::Vector3d accel_body{Eigen::Vector3d::Zero()}; // m/s², specific force
    Eigen::Vector3d gyro_body{Eigen::Vector3d::Zero()};  // rad/s
};

// Models accelerometer + gyroscope with additive white noise and bias random walk.
// Accelerometer output is specific force (non-gravitational acceleration), body FRD.
class IMU : public SensorBase {
public:
    explicit IMU(IMUParams params = {}, uint64_t seed = 1);

    IMUSample sample(const physics::State& state,
                     const Eigen::Vector3d& accel_world_ned);

private:
    IMUParams       params_;
    Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
};

}  // namespace simuav::sensors
