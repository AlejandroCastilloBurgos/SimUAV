#pragma once
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Battery.h"

#include <array>
#include <fstream>
#include <string>
#include <cstdint>

namespace simuav::logging {

// Writes telemetry in PX4 uLog binary format.
// Spec: https://docs.px4.io/main/en/dev_log/ulog_file_format.html
//
// Topics (msg_id order):
//   0  vehicle_local_position
//   1  vehicle_imu
//   2  vehicle_gps_position
//   3  vehicle_air_data
//   4  vehicle_actuator_outputs   (motor speeds rad/s)
//   5  vehicle_battery_status     (voltage, current, remaining)
class ULogLogger {
public:
    explicit ULogLogger(const std::string& path);
    ~ULogLogger();

    bool isOpen() const { return file_.is_open(); }

    void log(const physics::State&                         state,
             const sensors::IMUSample&                     imu,
             const sensors::BaroSample&                    baro,
             const sensors::GPSSample&                     gps,
             const std::array<double, physics::kNumMotors>& motor_rad_s,
             const sensors::BatterySample&                 bat);

private:
    void writeFileHeader();
    void writeFlagBits();
    void writeFormatMessage(const char* fmt_str);
    void writeSubscriptionMessage(const char* topic_name, uint16_t msg_id);
    void writeU16(uint16_t v);
    void writeU64(uint64_t v);
    void writeFloat(float v);
    void writeByte(uint8_t v);
    void writeBytes(const void* data, std::size_t len);

    std::ofstream file_;
    bool          header_written_{false};
};

}  // namespace simuav::logging
