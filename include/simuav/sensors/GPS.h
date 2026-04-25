#pragma once
#include "SensorBase.h"
#include "simuav/physics/QuadrotorModel.h"

namespace simuav::sensors {

struct GPSParams {
    double lat_ref_deg{47.397742};    // Reference origin latitude
    double lon_ref_deg{8.545594};     // Reference origin longitude
    double alt_ref_m{488.0};         // Reference origin altitude MSL
    double pos_noise_std_m{1.5};     // m, horizontal position noise (1-sigma)
    double alt_noise_std_m{2.5};     // m, vertical position noise
    double vel_noise_std{0.1};       // m/s, velocity noise per axis
    double update_rate_hz{5.0};      // Hz, GPS update rate
};

struct GPSSample {
    double   timestamp{0.0};
    double   latitude_deg{0.0};
    double   longitude_deg{0.0};
    float    altitude_m{0.0f};
    float    velocity_n{0.0f};       // m/s NED
    float    velocity_e{0.0f};
    float    velocity_d{0.0f};
    float    eph{1.5f};              // horizontal position uncertainty (m)
    float    epv{2.5f};             // vertical position uncertainty (m)
    uint8_t  num_sats{12};
    uint8_t  fix_type{3};           // 3 = 3D fix
};

// Converts NED position to geodetic coordinates and adds positional noise.
class GPS : public SensorBase {
public:
    explicit GPS(GPSParams params = {}, uint64_t seed = 2);

    // Returns a new sample only when the GPS update interval has elapsed.
    // Returns false if it is not yet time for a new fix.
    bool sample(const physics::State& state, GPSSample& out);

private:
    GPSParams  params_;
    GPSSample  last_sample_;
    double     last_update_time_{-1.0};

    static constexpr double kEarthRadius = 6378137.0; // m, WGS84
};

}  // namespace simuav::sensors
