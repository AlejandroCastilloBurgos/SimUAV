#include <gtest/gtest.h>

#include "simuav/LogReplayer.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/Magnetometer.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

namespace {

static constexpr double kDt      = 0.004;
static constexpr int    kSteps   = 250;
static constexpr double kGravity = 9.80665;
static const std::string kFixture = "/tmp/simuav_replay_test.ndjson";

// Writes kSteps NDJSON lines for a free-fall trajectory starting from rest.
// Only pos/vel/att/omega/t are used by the replayer; the sensor fields are
// placeholders that satisfy the JSON schema but are not read back.
void writeFixture() {
    std::ofstream f(kFixture);
    f << std::fixed << std::setprecision(6);
    for (int i = 0; i < kSteps; ++i) {
        const double t  = (i + 1) * kDt;
        const double vz = kGravity * t;
        const double pz = 0.5 * kGravity * t * t;
        f << "{\"t\":"    << t
          << ",\"pos\":["  << "0.000000,0.000000," << pz << ']'
          << ",\"vel\":["  << "0.000000,0.000000," << vz << ']'
          << ",\"att\":[1.000000,0.000000,0.000000,0.000000]"
          << ",\"omega\":[0.000000,0.000000,0.000000]"
          << ",\"accel\":[0.000000,0.000000,0.000000]"
          << ",\"gyro\":[0.000000,0.000000,0.000000]"
          << ",\"baro_alt\":488.000000"
          << ",\"gps_lat\":" << std::setprecision(9) << 47.397742000
          << ",\"gps_lon\":" << 8.545594000
          << "}\n";
        f << std::setprecision(6);
    }
}

// Returns sensor param sets with all noise and bias zeroed for deterministic tests.
simuav::sensors::IMUParams zeroIMU() {
    simuav::sensors::IMUParams p;
    p.accel_noise_std = 0.0;
    p.accel_bias_std  = 0.0;
    p.gyro_noise_std  = 0.0;
    p.gyro_bias_std   = 0.0;
    return p;
}

simuav::sensors::GPSParams zeroGPS(double rate_hz = 5.0) {
    simuav::sensors::GPSParams p;
    p.pos_noise_std_m = 0.0;
    p.alt_noise_std_m = 0.0;
    p.vel_noise_std   = 0.0;
    p.update_rate_hz  = rate_hz;
    return p;
}

simuav::sensors::BaroParams zeroBaro() {
    simuav::sensors::BaroParams p;
    p.noise_std_m = 0.0;
    return p;
}

simuav::sensors::MagParams zeroMag() {
    simuav::sensors::MagParams p;
    p.noise_std = 0.0;
    return p;
}

} // namespace

class ReplayIntegration : public ::testing::Test {
protected:
    void SetUp() override { writeFixture(); }
};

TEST_F(ReplayIntegration, LoadLogParsesAllEntries) {
    const auto entries = simuav::loadLog(kFixture);
    ASSERT_EQ(static_cast<int>(entries.size()), kSteps);

    // First entry: no predecessor → accel_world must be Zero
    EXPECT_NEAR(entries[0].accel_world.norm(), 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(entries[0].state.time, kDt);

    // Second entry onwards: finite-diff should recover gravity in NED z.
    // 6-decimal-place velocity truncation gives ~1.5e-4 m/s² quantisation error.
    EXPECT_NEAR(entries[1].accel_world.x(), 0.0,      1e-3);
    EXPECT_NEAR(entries[1].accel_world.y(), 0.0,      1e-3);
    EXPECT_NEAR(entries[1].accel_world.z(), kGravity, 2e-4);
}

TEST_F(ReplayIntegration, ReplayedIMUMatchesFreeFall) {
    const auto entries = simuav::loadLog(kFixture);

    simuav::sensors::IMU          imu(zeroIMU());
    simuav::sensors::GPS          gps(zeroGPS());
    simuav::sensors::Barometer    baro(zeroBaro());
    simuav::sensors::Magnetometer mag(zeroMag());

    // In free fall, specific force = accel_world - gravity = 0 → all three axes
    // are zero for every entry after the first (where accel_world is recovered).
    for (int i = 1; i < kSteps; ++i) {
        const auto s = simuav::replayStep(entries[i], imu, gps, baro, mag);
        EXPECT_NEAR(s.imu.accel_body.x(), 0.0, 1e-3) << "step " << i;
        EXPECT_NEAR(s.imu.accel_body.y(), 0.0, 1e-3) << "step " << i;
        EXPECT_NEAR(s.imu.accel_body.z(), 0.0, 1e-3) << "step " << i;
        EXPECT_NEAR(s.imu.gyro_body.x(),  0.0, 1e-9) << "step " << i;
        EXPECT_NEAR(s.imu.gyro_body.y(),  0.0, 1e-9) << "step " << i;
        EXPECT_NEAR(s.imu.gyro_body.z(),  0.0, 1e-9) << "step " << i;
    }
}

TEST_F(ReplayIntegration, ReplayedBaroTracksAltitude) {
    const auto entries = simuav::loadLog(kFixture);

    simuav::sensors::IMU          imu(zeroIMU());
    simuav::sensors::GPS          gps(zeroGPS());
    simuav::sensors::Barometer    baro(zeroBaro());
    simuav::sensors::Magnetometer mag(zeroMag());

    for (int i = 0; i < kSteps; ++i) {
        const auto s = simuav::replayStep(entries[i], imu, gps, baro, mag);
        const double t            = (i + 1) * kDt;
        const double expected_alt = 488.0 - 0.5 * kGravity * t * t;
        EXPECT_NEAR(static_cast<double>(s.baro.altitude_m), expected_alt, 1e-3)
            << "step " << i;
    }
}

TEST_F(ReplayIntegration, ReplayedGPSMatchesReferenceOrigin) {
    const auto entries = simuav::loadLog(kFixture);

    // Drive GPS at 1000 Hz (interval < dt) so every step produces a fix,
    // avoiding floating-point boundary issues at update_rate_hz == 250 Hz.
    simuav::sensors::IMU          imu(zeroIMU());
    simuav::sensors::GPS          gps(zeroGPS(1000.0));
    simuav::sensors::Barometer    baro(zeroBaro());
    simuav::sensors::Magnetometer mag(zeroMag());

    // North/East position is zero throughout → lat/lon must equal reference exactly
    for (int i = 0; i < kSteps; ++i) {
        const auto s = simuav::replayStep(entries[i], imu, gps, baro, mag);
        ASSERT_TRUE(s.gps_valid) << "expected GPS fix at step " << i;
        EXPECT_NEAR(s.gps.latitude_deg,  47.397742, 2e-7) << "step " << i;
        EXPECT_NEAR(s.gps.longitude_deg,  8.545594, 2e-7) << "step " << i;
    }
}
