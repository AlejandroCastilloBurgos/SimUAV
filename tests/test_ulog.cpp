#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>

#include "simuav/logging/ULogLogger.h"
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/IMU.h"

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
        simuav::sensors::IMUSample imu;        // Eigen members initialise to zero
        simuav::sensors::BaroSample baro{};    // POD aggregate — zero by value-init
        simuav::sensors::GPSSample  gps{};     // POD aggregate — zero by value-init

        logger.log(state, imu, baro, gps);
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

    // 5. Expect exactly 4 FORMAT messages (msg_type == 0x46 'F').
    for (int i = 0; i < 4; ++i) {
        uint16_t msg_size = readU16(f);
        uint8_t  msg_type = readU8(f);
        ASSERT_EQ(0x46, msg_type) << "FORMAT message " << i << " has wrong type";
        skipBytes(f, static_cast<std::size_t>(msg_size) - 1);
    }

    // 6. Expect exactly 4 SUBSCRIPTION messages (msg_type == 0x53 'S').
    //    Layout after the 2-byte msg_size:
    //      [0]      msg_type  (1 byte, == 0x53)
    //      [1]      multi_id  (1 byte, == 0)
    //      [2–3]    msg_id    (2 bytes, little-endian)
    //      [4…]     topic_name (msg_size - 1 - 1 - 2 bytes, NOT null-terminated)

    struct ExpectedSub {
        uint16_t    msg_id;
        const char* name;
    };

    const std::array<ExpectedSub, 4> expected{{
        {0, "vehicle_local_position"},
        {1, "vehicle_imu"},
        {2, "vehicle_gps_position"},
        {3, "vehicle_air_data"},
    }};

    for (int i = 0; i < 4; ++i) {
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
        simuav::sensors::IMUSample  imu;
        simuav::sensors::BaroSample baro{};
        simuav::sensors::GPSSample  gps{};

        logger.log(state, imu, baro, gps);
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

    // Skip 4 FORMAT messages.
    for (int i = 0; i < 4; ++i) {
        uint16_t sz = readU16(f);
        skipBytes(f, sz);
    }

    // Skip 4 SUBSCRIPTION messages.
    for (int i = 0; i < 4; ++i) {
        uint16_t sz = readU16(f);
        skipBytes(f, sz);
    }

    // Collect msg_ids from all DATA messages (msg_type == 0x44 'D').
    std::array<bool, 4> seen{};
    while (f) {
        uint16_t msg_size = readU16(f);
        if (!f) break;
        uint8_t msg_type = readU8(f);
        if (!f) break;

        if (msg_type == 0x44) {
            uint16_t msg_id = readU16(f);
            if (msg_id < 4) seen[msg_id] = true;
            // Skip remaining payload bytes (msg_size - 1 type - 2 msg_id).
            skipBytes(f, static_cast<std::size_t>(msg_size) - 1 - 2);
        } else {
            skipBytes(f, static_cast<std::size_t>(msg_size) - 1);
        }
    }

    EXPECT_TRUE(seen[0]) << "DATA msg_id 0 (vehicle_local_position) missing";
    EXPECT_TRUE(seen[1]) << "DATA msg_id 1 (vehicle_imu) missing";
    EXPECT_TRUE(seen[2]) << "DATA msg_id 2 (vehicle_gps_position) missing";
    EXPECT_TRUE(seen[3]) << "DATA msg_id 3 (vehicle_air_data) missing";
}
