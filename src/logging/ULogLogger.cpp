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

static constexpr uint8_t  kMsgSubscription = 0x53;
static constexpr uint16_t kMsgIdLocalPos   = 0;
static constexpr uint16_t kMsgIdImu        = 1;
static constexpr uint16_t kMsgIdGps        = 2;
static constexpr uint16_t kMsgIdAirData    = 3;

// vehicle_local_position fields logged (subset)
static constexpr char kFormatLocalPos[] =
    "vehicle_local_position:float x;float y;float z;"
    "float vx;float vy;float vz;"
    "float ax;float ay;float az;"
    "float baro_alt;";

static constexpr char kFormatImu[] =
    "vehicle_imu:float ax;float ay;float az;float gx;float gy;float gz;";

static constexpr char kFormatGps[] =
    "vehicle_gps_position:double lat;double lon;float alt;"
    "float vn;float ve;float vd;float eph;float epv;";

static constexpr char kFormatAirData[] =
    "vehicle_air_data:float pressure_pa;float altitude_m;float temperature_c;";

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

void ULogLogger::writeFormatMessage(const char* fmt_str) {
    const uint16_t fmt_len = static_cast<uint16_t>(std::strlen(fmt_str));
    const uint16_t msg_size = fmt_len + 1; // +1 for msg_type byte
    writeU16(msg_size);
    writeByte(kMsgFormat);
    writeBytes(fmt_str, fmt_len);
}

void ULogLogger::writeSubscriptionMessage(const char* topic_name, uint16_t msg_id) {
    auto name_len = static_cast<uint16_t>(std::strlen(topic_name));
    uint16_t msg_size = 1 + 1 + 2 + name_len;
    writeU16(msg_size);
    writeByte(kMsgSubscription);
    writeByte(0);        // multi_id
    writeU16(msg_id);
    writeBytes(topic_name, name_len);
}

void ULogLogger::log(const physics::State&      state,
                      const sensors::IMUSample&  imu,
                      const sensors::BaroSample& baro,
                      const sensors::GPSSample&  gps)
{
    if (!file_.is_open()) return;

    if (!header_written_) {
        writeFileHeader();
        writeFlagBits();
        writeFormatMessage(kFormatLocalPos);
        writeFormatMessage(kFormatImu);
        writeFormatMessage(kFormatGps);
        writeFormatMessage(kFormatAirData);
        writeSubscriptionMessage("vehicle_local_position", kMsgIdLocalPos);
        writeSubscriptionMessage("vehicle_imu",            kMsgIdImu);
        writeSubscriptionMessage("vehicle_gps_position",   kMsgIdGps);
        writeSubscriptionMessage("vehicle_air_data",       kMsgIdAirData);
        header_written_ = true;
    }

    // Each call writes one DATA record per subscribed topic.
    // msg_size = sizeof(payload) + 1 (for the msg_type byte).

    {
        struct __attribute__((packed)) LocalPosPayload {
            uint16_t msg_id;
            float x, y, z;
            float vx, vy, vz;
            float ax, ay, az;
            float baro_alt;
        };
        LocalPosPayload pl{};
        pl.msg_id   = kMsgIdLocalPos;
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
        writeU16(static_cast<uint16_t>(sizeof(pl) + 1));
        writeByte(kMsgData);
        writeBytes(&pl, sizeof(pl));
    }

    {
        struct __attribute__((packed)) ImuPayload {
            uint16_t msg_id;
            float ax, ay, az;
            float gx, gy, gz;
        };
        ImuPayload pl{};
        pl.msg_id = kMsgIdImu;
        pl.ax     = static_cast<float>(imu.accel_body.x());
        pl.ay     = static_cast<float>(imu.accel_body.y());
        pl.az     = static_cast<float>(imu.accel_body.z());
        pl.gx     = static_cast<float>(imu.gyro_body.x());
        pl.gy     = static_cast<float>(imu.gyro_body.y());
        pl.gz     = static_cast<float>(imu.gyro_body.z());
        writeU16(static_cast<uint16_t>(sizeof(pl) + 1));
        writeByte(kMsgData);
        writeBytes(&pl, sizeof(pl));
    }

    {
        struct __attribute__((packed)) GpsPayload {
            uint16_t msg_id;
            double   lat, lon;
            float    alt;
            float    vn, ve, vd;
            float    eph, epv;
        };
        GpsPayload pl{};
        pl.msg_id = kMsgIdGps;
        pl.lat    = gps.latitude_deg;
        pl.lon    = gps.longitude_deg;
        pl.alt    = gps.altitude_m;
        pl.vn     = gps.velocity_n;
        pl.ve     = gps.velocity_e;
        pl.vd     = gps.velocity_d;
        pl.eph    = gps.eph;
        pl.epv    = gps.epv;
        writeU16(static_cast<uint16_t>(sizeof(pl) + 1));
        writeByte(kMsgData);
        writeBytes(&pl, sizeof(pl));
    }

    {
        struct __attribute__((packed)) AirDataPayload {
            uint16_t msg_id;
            float    pressure_pa;
            float    altitude_m;
            float    temperature_c;
        };
        AirDataPayload pl{};
        pl.msg_id      = kMsgIdAirData;
        pl.pressure_pa = baro.pressure_pa;
        pl.altitude_m  = baro.altitude_m;
        pl.temperature_c = baro.temperature_c;
        writeU16(static_cast<uint16_t>(sizeof(pl) + 1));
        writeByte(kMsgData);
        writeBytes(&pl, sizeof(pl));
    }
}

}  // namespace simuav::logging
