#pragma once
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Magnetometer.h"
#include "simuav/sensors/Battery.h"

#include <array>
#include <fstream>
#include <string>

namespace simuav::logging {

// Writes telemetry as newline-delimited JSON (NDJSON).
// One JSON object per call to log(), flushed lazily.
// Fields: t, pos[3], vel[3], att[4], omega[3], accel[3], gyro[3], baro_alt,
//         gps_lat, gps_lon, motor_rad_s[4], bat_v, bat_soc, wind_ned[3]
class JSONLogger {
public:
    explicit JSONLogger(const std::string& path);
    ~JSONLogger();

    bool isOpen() const { return file_.is_open(); }

    void log(const physics::State&                         state,
             const sensors::IMUSample&                     imu,
             const sensors::BaroSample&                    baro,
             const sensors::GPSSample&                     gps,
             const std::array<double, physics::kNumMotors>& motor_rad_s,
             const sensors::BatterySample&                 bat,
             const Eigen::Vector3d&                        wind_ned);

private:
    std::ofstream file_;
    bool          first_{true};
};

}  // namespace simuav::logging
