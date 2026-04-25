#include "simuav/logging/JSONLogger.h"
#include <iomanip>

namespace simuav::logging {

JSONLogger::JSONLogger(const std::string& path)
    : file_(path, std::ios::out | std::ios::trunc) {}

JSONLogger::~JSONLogger() {
    if (file_.is_open()) file_.close();
}

void JSONLogger::log(const physics::State&      state,
                      const sensors::IMUSample&  imu,
                      const sensors::BaroSample& baro,
                      const sensors::GPSSample&  gps)
{
    if (!file_.is_open()) return;

    const auto& p = state.position;
    const auto& v = state.velocity;
    const auto& q = state.attitude;
    const auto& w = state.angular_velocity;
    const auto& a = imu.accel_body;
    const auto& g = imu.gyro_body;

    file_ << std::fixed << std::setprecision(6)
          << "{\"t\":"        << state.time
          << ",\"pos\":["     << p.x() << ',' << p.y() << ',' << p.z() << ']'
          << ",\"vel\":["     << v.x() << ',' << v.y() << ',' << v.z() << ']'
          << ",\"att\":["     << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ']'
          << ",\"omega\":["   << w.x() << ',' << w.y() << ',' << w.z() << ']'
          << ",\"accel\":["   << a.x() << ',' << a.y() << ',' << a.z() << ']'
          << ",\"gyro\":["    << g.x() << ',' << g.y() << ',' << g.z() << ']'
          << ",\"baro_alt\":" << baro.altitude_m
          << ",\"gps_lat\":"  << std::setprecision(9) << gps.latitude_deg
          << ",\"gps_lon\":"  << gps.longitude_deg
          << "}\n";
}

}  // namespace simuav::logging
