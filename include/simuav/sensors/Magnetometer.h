#pragma once
#include "SensorBase.h"
#include "simuav/physics/QuadrotorModel.h"
#include <Eigen/Dense>

namespace simuav::sensors {

struct MagParams {
    // Earth's magnetic field at reference location in NED frame (Gauss).
    // Default: Zurich, Switzerland.
    Eigen::Vector3d earth_field_ned{0.21462, 0.00571, 0.42663};
    double noise_std{0.005}; // Gauss, per axis
};

struct MagSample {
    double          timestamp{0.0};
    Eigen::Vector3f field_body{Eigen::Vector3f::Zero()}; // Gauss, body FRD
};

// Rotates Earth's field vector into the body frame and adds white noise.
class Magnetometer : public SensorBase {
public:
    explicit Magnetometer(MagParams params = {}, uint64_t seed = 4);

    MagSample sample(const physics::State& state);

private:
    MagParams params_;
};

}  // namespace simuav::sensors
