#include <gtest/gtest.h>
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Magnetometer.h"
#include "simuav/physics/QuadrotorModel.h"

using namespace simuav;

static physics::State makeHoverState(double time = 0.0) {
    physics::State s;
    s.time = time;
    return s;
}

// ── IMU ─────────────────────────────────────────────────────────────────────

TEST(IMU, OutputIsFinite) {
    sensors::IMU imu;
    const physics::State s = makeHoverState();
    const auto sample = imu.sample(s, Eigen::Vector3d::Zero());
    EXPECT_TRUE(sample.accel_body.allFinite());
    EXPECT_TRUE(sample.gyro_body.allFinite());
}

TEST(IMU, AccelNearGravityAtRest) {
    // At rest in NED, specific force in body z should be ~ +g after many samples
    // (gravity component projected onto body -Z = -9.81 → specific force = 0 by definition)
    // Actually at rest: accel_world = 0, so specific_force_body = R^T*(0 - g) = -g * body_z
    // For level attitude that is approximately (0, 0, -9.81).
    sensors::IMU imu(sensors::IMUParams{0.0, 0.0, 0.0, 0.0}); // no noise/bias
    const physics::State s = makeHoverState();
    const auto sample = imu.sample(s, Eigen::Vector3d::Zero());
    EXPECT_NEAR(sample.accel_body.z(), -9.80665, 1e-3);
}

// ── GPS ──────────────────────────────────────────────────────────────────────

TEST(GPS, FirstSampleAlwaysProduced) {
    sensors::GPS gps;
    const physics::State s = makeHoverState();
    sensors::GPSSample out;
    EXPECT_TRUE(gps.sample(s, out));
}

TEST(GPS, ThrottlesAtUpdateRate) {
    sensors::GPS gps(sensors::GPSParams{47.397742, 8.545594, 488.0,
                                         0.0, 0.0, 0.0, 5.0});
    physics::State s = makeHoverState(0.0);
    sensors::GPSSample out;
    EXPECT_TRUE(gps.sample(s, out));

    // 0.1 s later (< 0.2 s GPS interval)
    s.time = 0.1;
    EXPECT_FALSE(gps.sample(s, out));

    // 0.2 s later (= 1/5 Hz interval)
    s.time = 0.2;
    EXPECT_TRUE(gps.sample(s, out));
}

// ── Barometer ────────────────────────────────────────────────────────────────

TEST(Barometer, SeaLevelPressureNearP0) {
    sensors::BaroParams p;
    p.alt_ref_m  = 0.0; // origin is at sea level
    p.noise_std_m = 0.0;
    sensors::Barometer baro(p);

    physics::State s = makeHoverState();
    s.position.z() = 0.0; // no altitude gain
    const auto sample = baro.sample(s);
    EXPECT_NEAR(sample.pressure_pa, 101325.0f, 10.0f);
}

TEST(Barometer, PressureDecreasesWithAltitude) {
    sensors::BaroParams p;
    p.alt_ref_m   = 0.0;
    p.noise_std_m = 0.0;
    sensors::Barometer baro(p);

    physics::State s_low  = makeHoverState(); s_low.position.z()  =    0.0; // 0 m
    physics::State s_high = makeHoverState(); s_high.position.z() = -1000.0; // 1000 m up (NED z negative)

    const auto low  = baro.sample(s_low);
    const auto high = baro.sample(s_high);
    EXPECT_LT(high.pressure_pa, low.pressure_pa);
}

// ── Magnetometer ─────────────────────────────────────────────────────────────

TEST(Magnetometer, LevelAttitudeMatchesEarthField) {
    sensors::MagParams p;
    p.noise_std = 0.0;
    sensors::Magnetometer mag(p);

    const physics::State s = makeHoverState();
    const auto sample = mag.sample(s);

    // At level attitude (identity quaternion), body field = earth field
    EXPECT_NEAR(sample.field_body.x(), static_cast<float>(p.earth_field_ned.x()), 1e-5f);
    EXPECT_NEAR(sample.field_body.y(), static_cast<float>(p.earth_field_ned.y()), 1e-5f);
    EXPECT_NEAR(sample.field_body.z(), static_cast<float>(p.earth_field_ned.z()), 1e-5f);
}
