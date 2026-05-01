#pragma once
#include "SensorBase.h"
#include "simuav/physics/QuadrotorModel.h"
#include <Eigen/Dense>

namespace simuav::sensors {

struct IMUParams {
    // Accelerometer
    double accel_arw_std{0.005};               // m/s², angle random walk (white noise) per axis
    double accel_bias_instability{0.0001};     // m/s², Gauss-Markov bias instability σ
    double accel_bias_instability_tc_s{300.0}; // s, Gauss-Markov correlation time

    // Gyroscope
    double gyro_arw_std{0.0003};               // rad/s, white noise per axis
    double gyro_bias_instability{0.000005};    // rad/s, Gauss-Markov bias instability σ
    double gyro_bias_instability_tc_s{300.0};  // s, Gauss-Markov correlation time

    // Rotor-frequency vibration injected on body-Z accelerometer
    double vibration_amplitude_mps2{0.0};  // m/s², 0 = disabled
    double vibration_frequency_hz{0.0};    // Hz
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
    double          last_time_{-1.0}; // negative sentinel: dt defaults to 0.004 on first call
};

}  // namespace simuav::sensors
