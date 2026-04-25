#include "simuav/sensors/GPS.h"
#include <cmath>

namespace simuav::sensors {

GPS::GPS(GPSParams params, uint64_t seed)
    : SensorBase(seed), params_(std::move(params)) {}

bool GPS::sample(const physics::State& state, GPSSample& out) {
    const double interval = 1.0 / params_.update_rate_hz;
    if (last_update_time_ >= 0.0 && (state.time - last_update_time_) < interval) {
        out = last_sample_;
        return false;
    }
    last_update_time_ = state.time;

    const double lat_ref_rad = params_.lat_ref_deg * M_PI / 180.0;

    // NED to geodetic (flat-Earth approximation, valid within ~100 km of origin)
    const double north = state.position.x() + white(params_.pos_noise_std_m);
    const double east  = state.position.y() + white(params_.pos_noise_std_m);
    const double down  = state.position.z() + white(params_.alt_noise_std_m);

    last_sample_.timestamp     = state.time;
    last_sample_.latitude_deg  = params_.lat_ref_deg +
                                  (north / kEarthRadius) * (180.0 / M_PI);
    last_sample_.longitude_deg = params_.lon_ref_deg +
                                  (east  / (kEarthRadius * std::cos(lat_ref_rad))) *
                                  (180.0 / M_PI);
    last_sample_.altitude_m    = static_cast<float>(params_.alt_ref_m - down);

    last_sample_.velocity_n = static_cast<float>(state.velocity.x() + white(params_.vel_noise_std));
    last_sample_.velocity_e = static_cast<float>(state.velocity.y() + white(params_.vel_noise_std));
    last_sample_.velocity_d = static_cast<float>(state.velocity.z() + white(params_.vel_noise_std));

    out = last_sample_;
    return true;
}

}  // namespace simuav::sensors
