#pragma once
#include "SensorBase.h"
#include "simuav/physics/QuadrotorModel.h"

namespace simuav::sensors {

struct BaroParams {
    double alt_ref_m{488.0};              // MSL altitude of reference origin
    double noise_std_m{0.3};             // m equivalent altitude noise
    double ground_temp_offset_c{0.0};    // non-standard day offset added to ISA temperature (°C)
};

struct BaroSample {
    double timestamp{0.0};
    float  pressure_pa{101325.0f};
    float  altitude_m{0.0f};
    float  temperature_c{20.0f};
};

// International standard atmosphere model. Pressure derives from NED-down (altitude).
class Barometer : public SensorBase {
public:
    explicit Barometer(BaroParams params = {}, uint64_t seed = 3);

    BaroSample sample(const physics::State& state);

private:
    BaroParams params_;

    static constexpr double kP0   = 101325.0; // Pa, sea-level standard pressure
    static constexpr double kT0   = 288.15;   // K, sea-level standard temperature
    static constexpr double kL    = 0.0065;   // K/m, temperature lapse rate
    static constexpr double kG    = 9.80665;  // m/s²
    static constexpr double kM    = 0.0289644; // kg/mol, molar mass of air
    static constexpr double kR    = 8.31446;   // J/(mol·K), gas constant
};

}  // namespace simuav::sensors
