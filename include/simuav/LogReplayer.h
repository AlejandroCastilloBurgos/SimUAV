#pragma once
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Magnetometer.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace simuav {

struct ReplayEntry {
    physics::State  state;
    // World-frame linear acceleration (NED, m/s²), finite-differenced from
    // consecutive velocity entries. Zero for the first entry (no predecessor).
    Eigen::Vector3d accel_world{Eigen::Vector3d::Zero()};
};

struct ReplaySamples {
    sensors::IMUSample  imu;
    sensors::BaroSample baro;
    sensors::MagSample  mag;
    sensors::GPSSample  gps;
    bool                gps_valid{false};
};

// Parses an NDJSON telemetry file (as written by JSONLogger) into a vector of
// ReplayEntry. Throws std::runtime_error on I/O or parse failure.
// accel_world for entry[0] is Zero() — logs starting mid-flight should treat
// the first entry's IMU output as unreliable.
[[nodiscard]] std::vector<ReplayEntry> loadLog(const std::string& path);

// Drives one replay step: feeds entry through the four sensor models and
// returns all samples. GPS::sample() may return false (rate-throttled); gps_out
// is unchanged in that case (ReplaySamples::gps_valid == false).
ReplaySamples replayStep(const ReplayEntry&      entry,
                         sensors::IMU&           imu,
                         sensors::GPS&           gps,
                         sensors::Barometer&     baro,
                         sensors::Magnetometer&  mag);

}  // namespace simuav
