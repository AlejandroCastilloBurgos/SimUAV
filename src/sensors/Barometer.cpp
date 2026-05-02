#include "simuav/sensors/Barometer.h"
#include <cmath>

namespace simuav::sensors {

Barometer::Barometer(BaroParams params, uint64_t seed)
    : SensorBase(seed), params_(std::move(params)) {}

BaroSample Barometer::sample(const physics::State& state) {
    // NED: down is positive; altitude above origin = -position.z()
    const double altitude_msl = params_.alt_ref_m + (-state.position.z())
                                 + white(params_.noise_std_m);

    // ISA barometric formula
    const double exponent = (kG * kM) / (kR * kL);
    const double pressure = kP0 * std::pow(1.0 - (kL * altitude_msl) / kT0, exponent);

    // ISA temperature at altitude, plus optional non-standard day offset.
    const double temperature_c = (kT0 - kL * altitude_msl) - 273.15
                                  + params_.ground_temp_offset_c;

    BaroSample out;
    out.timestamp     = state.time;
    out.pressure_pa   = static_cast<float>(pressure);
    out.altitude_m    = static_cast<float>(altitude_msl);
    out.temperature_c = static_cast<float>(temperature_c);
    return out;
}

}  // namespace simuav::sensors
