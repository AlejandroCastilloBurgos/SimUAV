#include "simuav/logging/ULogLogger.h"
#include <cstring>
#include <chrono>

namespace simuav::logging {

// ULog magic bytes + version
static constexpr uint8_t kMagic[] = {0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35};
static constexpr uint8_t kVersion  = 1;

// Message types
static constexpr uint8_t kMsgFlagBits = 0x42; // 'B'
static constexpr uint8_t kMsgFormat   = 0x46; // 'F'
static constexpr uint8_t kMsgData     = 0x44; // 'D'

// vehicle_local_position fields logged (subset)
static constexpr char kFormatStr[] =
    "vehicle_local_position:float x;float y;float z;"
    "float vx;float vy;float vz;"
    "float ax;float ay;float az;"
    "float baro_alt;";

static constexpr uint16_t kMsgId = 0;

ULogLogger::ULogLogger(const std::string& path)
    : file_(path, std::ios::out | std::ios::binary | std::ios::trunc) {}

ULogLogger::~ULogLogger() {
    if (file_.is_open()) file_.close();
}

void ULogLogger::writeByte(uint8_t v) {
    file_.write(reinterpret_cast<const char*>(&v), 1);
}

void ULogLogger::writeU16(uint16_t v) {
    file_.write(reinterpret_cast<const char*>(&v), 2);
}

void ULogLogger::writeU64(uint64_t v) {
    file_.write(reinterpret_cast<const char*>(&v), 8);
}

void ULogLogger::writeFloat(float v) {
    file_.write(reinterpret_cast<const char*>(&v), 4);
}

void ULogLogger::writeBytes(const void* data, std::size_t len) {
    file_.write(static_cast<const char*>(data), static_cast<std::streamsize>(len));
}

void ULogLogger::writeFileHeader() {
    writeBytes(kMagic, sizeof(kMagic));
    writeByte(kVersion);
    const uint64_t ts = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    writeU64(ts);
}

void ULogLogger::writeFlagBits() {
    // FLAG_BITS message: 8 bytes of zeroed compat/incompat flags
    constexpr uint8_t payload[8] = {};
    const uint16_t msg_size = sizeof(payload) + 1; // +1 for msg_type byte
    writeU16(msg_size);
    writeByte(kMsgFlagBits);
    writeBytes(payload, sizeof(payload));
}

void ULogLogger::writeFormatMessage() {
    const uint16_t fmt_len = static_cast<uint16_t>(std::strlen(kFormatStr));
    const uint16_t msg_size = fmt_len + 1; // +1 for msg_type byte
    writeU16(msg_size);
    writeByte(kMsgFormat);
    writeBytes(kFormatStr, fmt_len);
}

void ULogLogger::log(const physics::State&      state,
                      const sensors::IMUSample&  imu,
                      const sensors::BaroSample& baro)
{
    if (!file_.is_open()) return;

    if (!header_written_) {
        writeFileHeader();
        writeFlagBits();
        writeFormatMessage();
        header_written_ = true;
    }

    // DATA message payload: msg_id (2 bytes) + struct fields
    struct __attribute__((packed)) Payload {
        uint16_t msg_id;
        float x, y, z;
        float vx, vy, vz;
        float ax, ay, az;
        float baro_alt;
    };

    Payload pl{};
    pl.msg_id   = kMsgId;
    pl.x        = static_cast<float>(state.position.x());
    pl.y        = static_cast<float>(state.position.y());
    pl.z        = static_cast<float>(state.position.z());
    pl.vx       = static_cast<float>(state.velocity.x());
    pl.vy       = static_cast<float>(state.velocity.y());
    pl.vz       = static_cast<float>(state.velocity.z());
    pl.ax       = static_cast<float>(imu.accel_body.x());
    pl.ay       = static_cast<float>(imu.accel_body.y());
    pl.az       = static_cast<float>(imu.accel_body.z());
    pl.baro_alt = baro.altitude_m;

    const uint16_t msg_size = sizeof(pl) + 1; // +1 for msg_type byte
    writeU16(msg_size);
    writeByte(kMsgData);
    writeBytes(&pl, sizeof(pl));
}

}  // namespace simuav::logging
