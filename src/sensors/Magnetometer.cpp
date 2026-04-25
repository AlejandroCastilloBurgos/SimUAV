#include "simuav/sensors/Magnetometer.h"

namespace simuav::sensors {

Magnetometer::Magnetometer(MagParams params, uint64_t seed)
    : SensorBase(seed), params_(std::move(params)) {}

MagSample Magnetometer::sample(const physics::State& state) {
    const Eigen::Matrix3d R = state.attitude.toRotationMatrix(); // body → NED
    // Earth field is in NED; rotate to body frame
    const Eigen::Vector3d field_body = R.transpose() * params_.earth_field_ned;

    MagSample out;
    out.timestamp = state.time;
    out.field_body = field_body.cast<float>() +
                     Eigen::Vector3f(static_cast<float>(white(params_.noise_std)),
                                     static_cast<float>(white(params_.noise_std)),
                                     static_cast<float>(white(params_.noise_std)));
    return out;
}

}  // namespace simuav::sensors
