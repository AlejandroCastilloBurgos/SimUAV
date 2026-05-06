#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>

#include "simuav/logging/ULogLogger.h"
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Battery.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/IMU.h"

static const std::array<double, simuav::physics::kNumMotors> kZeroMotors{};
static const simuav::sensors::BatterySample kZeroBat{};

static void writeTestEntry(simuav::logging::ULogLogger& logger,
                           const simuav::physics::State& state) {
    simuav::sensors::IMUSample  imu;
    simuav::sensors::BaroSample baro{};
    simuav::sensors::GPSSample  gps{};
    logger.log(state, imu, baro, gps, kZeroMotors, kZeroBat);
}

// ── helpers ──────────────────────────────────────────────────────────────────

static bool readExact(std::ifstream& f, void* buf, std::size_t n) {
    return static_cast<bool>(
        f.read(static_cast<char*>(buf), static_cast<std::streamsize>(n)));
}

static uint16_t readU16(std::ifstream& f) {
    uint16_t v = 0;
    readExact(f, &v, 2);
    return v;
}

static uint8_t readU8(std::ifstream& f) {
    uint8_t v = 0;
    readExact(f, &v, 1);
    return v;
}

static void skipBytes(std::ifstream& f, std::size_t n) {
    f.seekg(static_cast<std::streamoff>(n), std::ios::cur);
}

// ── test ─────────────────────────────────────────────────────────────────────

TEST(ULogLogger, SubscriptionMessagesPresent) {
    const std::string path = "/tmp/simuav_test_sub.ulg";

    // 1. Write one log entry and close the file.
    {
        simuav::logging::ULogLogger logger(path);
        ASSERT_TRUE(logger.isOpen());

        simuav::physics::State state;          // Eigen members initialise to zero/identity
        writeTestEntry(logger, state);
    }  // destructor flushes and closes

    // 2. Open for binary reading.
    std::ifstream f(path, std::ios::binary);
    ASSERT_TRUE(f.is_open());

    // 3. Skip 16-byte file header: 7 magic + 1 version + 8 timestamp.
    skipBytes(f, 16);

    // 4. Read and verify FLAG_BITS message (msg_type == 0x42 'B').
    {
        uint16_t msg_size = readU16(f);
        uint8_t  msg_type = readU8(f);
        ASSERT_EQ(0x42, msg_type);
        // Skip remaining msg_size-1 bytes of FLAG_BITS payload.
        skipBytes(f, static_cast<std::size_t>(msg_size) - 1);
    }

    // 5. Expect exactly 6 FORMAT messages (msg_type == 0x46 'F').
    for (int i = 0; i < 6; ++i) {
        uint16_t msg_size = readU16(f);
        uint8_t  msg_type = readU8(f);
        ASSERT_EQ(0x46, msg_type) << "FORMAT message " << i << " has wrong type";
        skipBytes(f, static_cast<std::size_t>(msg_size) - 1);
    }

    // 6. Expect exactly 6 SUBSCRIPTION messages (msg_type == 0x53 'S').
    //    Layout after the 2-byte msg_size:
    //      [0]      msg_type  (1 byte, == 0x53)
    //      [1]      multi_id  (1 byte, == 0)
    //      [2–3]    msg_id    (2 bytes, little-endian)
    //      [4…]     topic_name (msg_size - 1 - 1 - 2 bytes, NOT null-terminated)

    struct ExpectedSub {
        uint16_t    msg_id;
        const char* name;
    };

    const std::array<ExpectedSub, 6> expected{{
        {0, "vehicle_local_position"},
        {1, "vehicle_imu"},
        {2, "vehicle_gps_position"},
        {3, "vehicle_air_data"},
        {4, "vehicle_actuator_outputs"},
        {5, "vehicle_battery_status"},
    }};

    for (int i = 0; i < 6; ++i) {
        uint16_t msg_size = readU16(f);
        uint8_t  msg_type = readU8(f);
        ASSERT_EQ(0x53, msg_type) << "SUBSCRIPTION message " << i << " has wrong type";

        uint8_t  multi_id = readU8(f);
        EXPECT_EQ(0, multi_id) << "SUBSCRIPTION " << i << ": multi_id mismatch";

        uint16_t msg_id = readU16(f);
        EXPECT_EQ(expected[i].msg_id, msg_id) << "SUBSCRIPTION " << i << ": msg_id mismatch";

        // topic_name occupies the rest: msg_size - 1(type) - 1(multi_id) - 2(msg_id) bytes
        std::size_t name_len = static_cast<std::size_t>(msg_size) - 1 - 1 - 2;
        std::string topic_name(name_len, '\0');
        ASSERT_TRUE(readExact(f, topic_name.data(), name_len))
            << "SUBSCRIPTION " << i << ": failed to read topic name";

        EXPECT_EQ(std::string(expected[i].name), topic_name)
            << "SUBSCRIPTION " << i << ": topic name mismatch";
    }
}

TEST(ULogLogger, AllFourTopicsHaveDataMessages) {
    const std::string path = "/tmp/simuav_test_data.ulg";

    {
        simuav::logging::ULogLogger logger(path);
        ASSERT_TRUE(logger.isOpen());

        simuav::physics::State      state;
        writeTestEntry(logger, state);
    }

    std::ifstream f(path, std::ios::binary);
    ASSERT_TRUE(f.is_open());

    // Skip file header (16 bytes).
    skipBytes(f, 16);

    // Skip FLAG_BITS message.
    {
        uint16_t sz = readU16(f);
        skipBytes(f, sz);
    }

    // Skip 6 FORMAT messages.
    for (int i = 0; i < 6; ++i) {
        uint16_t sz = readU16(f);
        skipBytes(f, sz);
    }

    // Skip 6 SUBSCRIPTION messages.
    for (int i = 0; i < 6; ++i) {
        uint16_t sz = readU16(f);
        skipBytes(f, sz);
    }

    // Collect msg_ids from all DATA messages (msg_type == 0x44 'D').
    std::array<bool, 6> seen{};
    while (f) {
        uint16_t msg_size = readU16(f);
        if (!f) break;
        uint8_t msg_type = readU8(f);
        if (!f) break;

        if (msg_type == 0x44) {
            uint16_t msg_id = readU16(f);
            if (msg_id < 6) seen[msg_id] = true;
            skipBytes(f, static_cast<std::size_t>(msg_size) - 1 - 2);
        } else {
            skipBytes(f, static_cast<std::size_t>(msg_size) - 1);
        }
    }

    EXPECT_TRUE(seen[0]) << "DATA msg_id 0 (vehicle_local_position) missing";
    EXPECT_TRUE(seen[1]) << "DATA msg_id 1 (vehicle_imu) missing";
    EXPECT_TRUE(seen[2]) << "DATA msg_id 2 (vehicle_gps_position) missing";
    EXPECT_TRUE(seen[3]) << "DATA msg_id 3 (vehicle_air_data) missing";
    EXPECT_TRUE(seen[4]) << "DATA msg_id 4 (vehicle_actuator_outputs) missing";
    EXPECT_TRUE(seen[5]) << "DATA msg_id 5 (vehicle_battery_status) missing";
}

TEST(ULogLogger, FormatStringsContainTimestampFirstField) {
    // Each FORMAT body is a raw string; verify "uint64_t timestamp" appears
    // before any other field separator for all four topics.
    const std::string path = "/tmp/simuav_test_fmt_ts.ulg";
    {
        simuav::logging::ULogLogger logger(path);
        simuav::physics::State      state;
        writeTestEntry(logger, state);
    }

    std::ifstream f(path, std::ios::binary);
    ASSERT_TRUE(f.is_open());
    skipBytes(f, 16);  // file header

    // Skip FLAG_BITS
    { uint16_t sz = readU16(f); skipBytes(f, sz); }

    int fmt_with_timestamp = 0;
    for (int i = 0; i < 6; ++i) {
        uint16_t msg_size = readU16(f);
        uint8_t  msg_type = readU8(f);
        ASSERT_EQ(0x46, msg_type);
        std::string body(static_cast<std::size_t>(msg_size) - 1, '\0');
        ASSERT_TRUE(readExact(f, body.data(), body.size()));

        // FORMAT body: "topic_name:field1;field2;..."
        // The first field after ':' must be "uint64_t timestamp"
        auto colon = body.find(':');
        ASSERT_NE(std::string::npos, colon) << "FORMAT " << i << " has no ':'";
        const std::string fields = body.substr(colon + 1);
        EXPECT_EQ(0u, fields.find("uint64_t timestamp;"))
            << "FORMAT " << i << " first field is not 'uint64_t timestamp': " << body;
        ++fmt_with_timestamp;
    }
    EXPECT_EQ(6, fmt_with_timestamp);
}

TEST(ULogLogger, DataTimestampMatchesStateTime) {
    // Write two log entries at known simulation times and verify that the
    // timestamp field in each DATA message encodes state.time in microseconds.
    const std::string path = "/tmp/simuav_test_ts.ulg";
    constexpr double kTime1 = 1.5;   // seconds
    constexpr double kTime2 = 3.25;
    {
        simuav::logging::ULogLogger logger(path);
        simuav::physics::State state;
        state.time = kTime1;
        writeTestEntry(logger, state);
        state.time = kTime2;
        writeTestEntry(logger, state);
    }

    std::ifstream f(path, std::ios::binary);
    ASSERT_TRUE(f.is_open());
    skipBytes(f, 16);
    { uint16_t sz = readU16(f); skipBytes(f, sz); }       // FLAG_BITS
    for (int i = 0; i < 6; ++i) { uint16_t sz = readU16(f); skipBytes(f, sz); } // FORMATs
    for (int i = 0; i < 6; ++i) { uint16_t sz = readU16(f); skipBytes(f, sz); } // SUBs

    // First DATA message (local_position, msg_id 0) written for kTime1.
    {
        uint16_t msg_size = readU16(f);
        uint8_t  msg_type = readU8(f);
        ASSERT_EQ(0x44, msg_type);
        uint16_t msg_id = readU16(f);
        EXPECT_EQ(0u, msg_id);
        uint64_t ts = 0;
        ASSERT_TRUE(readExact(f, &ts, 8));
        EXPECT_EQ(static_cast<uint64_t>(kTime1 * 1.0e6), ts);
        // Skip rest of payload
        skipBytes(f, static_cast<std::size_t>(msg_size) - 1 - 2 - 8);
    }
}
