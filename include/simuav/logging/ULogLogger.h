#pragma once
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/Barometer.h"

#include <fstream>
#include <string>
#include <cstdint>

namespace simuav::logging {

// Writes telemetry in PX4 uLog binary format.
// Spec: https://docs.px4.io/main/en/dev_log/ulog_file_format.html
//
// Layout written:
//   File header (16 bytes magic + timestamp)
//   FLAG_BITS message
//   FORMAT message  (defines "vehicle_local_position" struct)
//   DATA messages   (one per log() call, msg_id 0)
class ULogLogger {
public:
    explicit ULogLogger(const std::string& path);
    ~ULogLogger();

    bool isOpen() const { return file_.is_open(); }

    void log(const physics::State&     state,
             const sensors::IMUSample& imu,
             const sensors::BaroSample& baro);

private:
    void writeFileHeader();
    void writeFlagBits();
    void writeFormatMessage();
    void writeU16(uint16_t v);
    void writeU64(uint64_t v);
    void writeFloat(float v);
    void writeByte(uint8_t v);
    void writeBytes(const void* data, std::size_t len);

    std::ofstream file_;
    bool          header_written_{false};
};

}  // namespace simuav::logging
