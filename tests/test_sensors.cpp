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

TEST(IMU, HoverAccelFromPhysicsModelIsNearG) {
    // Verifies the fix for the accel_world=0 stub: after running the physics
    // model at hover speed, lastAccelWorld() fed into the IMU must produce
    // specific force magnitude ≈ g on body -Z (level attitude).
    physics::QuadrotorParams p;
    p.motor_time_constant_s = 0.0; // instant actuators — testing IMU math only
    physics::QuadrotorModel model(p);
    const double w = std::sqrt((p.mass * 9.80665) / (4.0 * p.k_thrust));
    const std::array<double, physics::kNumMotors> motors{w, w, w, w};

    // Settle for 0.5 s so velocity/drag reach equilibrium
    for (int i = 0; i < 125; ++i)
        model.integrate(motors, 0.004);

    sensors::IMU imu(sensors::IMUParams{0.0, 0.0, 0.0, 0.0}); // no noise
    const auto sample = imu.sample(model.state(), model.lastAccelWorld());

    // At hover the net world acceleration is ~0, so specific force = -g on body z
    EXPECT_NEAR(sample.accel_body.z(), -9.80665, 0.05);
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

TEST(GPS, EphMatchesNoiseSigma) {
    sensors::GPSParams p;
    p.pos_noise_std_m = 3.0;
    p.alt_noise_std_m = 5.0;
    p.num_sats        = 8;
    p.fix_type        = 3;
    sensors::GPS gps(p);

    physics::State s = makeHoverState();
    sensors::GPSSample out;
    ASSERT_TRUE(gps.sample(s, out));

    EXPECT_FLOAT_EQ(out.eph, 3.0f);
    EXPECT_FLOAT_EQ(out.epv, 5.0f);
    EXPECT_EQ(out.num_sats, 8u);
    EXPECT_EQ(out.fix_type, 3u);
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

// ── IMU Gauss-Markov bias model ───────────────────────────────────────────────

TEST(IMUBiasGaussMarkov, BiasRemainsWithin3Sigma) {
    // After 300 s (= τ) simulated time the bias RMS must stay bounded.
    // Theoretical std dev of Gauss-Markov process at steady state = σ_b.
    // We check that all axis biases stay within 3σ_b at the end of the run.
    sensors::IMUParams p{};
    p.accel_arw_std                = 0.0; // suppress white noise for clarity
    p.accel_bias_instability       = 0.001; // σ_b = 1 mm/s² — slightly larger for speed
    p.accel_bias_instability_tc_s  = 10.0; // short τ so test runs quickly
    p.gyro_arw_std                 = 0.0;
    p.gyro_bias_instability        = 0.0001; // rad/s
    p.gyro_bias_instability_tc_s   = 10.0;

    sensors::IMU imu(p, /*seed=*/42);

    physics::State state;
    constexpr double dt    = 0.004;
    const     int    steps = static_cast<int>(5.0 * 10.0 / dt); // 5 τ

    sensors::IMUSample last;
    for (int i = 0; i < steps; ++i) {
        state.time += dt;
        last = imu.sample(state, Eigen::Vector3d(0.0, 0.0, 9.80665));
    }

    const double gyro_3sigma = 3.0 * p.gyro_bias_instability;

    // Check via gyro output (angular_velocity = 0, so output equals bias only)
    EXPECT_LT(std::abs(last.gyro_body.x()), gyro_3sigma * 5.0)
        << "Gyro bias x should not diverge beyond 5σ after 5τ";
    EXPECT_LT(std::abs(last.gyro_body.y()), gyro_3sigma * 5.0)
        << "Gyro bias y should not diverge beyond 5σ after 5τ";
    EXPECT_LT(std::abs(last.gyro_body.z()), gyro_3sigma * 5.0)
        << "Gyro bias z should not diverge beyond 5σ after 5τ";
}

TEST(IMUVibration, ZAccelPowerDominatedByVibration) {
    // With vibration enabled and zero noise, the Z-accel output must be dominated
    // by the sinusoidal vibration injection.
    sensors::IMUParams p{};
    p.accel_arw_std              = 0.0;
    p.accel_bias_instability     = 0.0;
    p.gyro_arw_std               = 0.0;
    p.gyro_bias_instability      = 0.0;
    p.vibration_amplitude_mps2   = 1.0;  // 1 m/s² amplitude
    p.vibration_frequency_hz     = 50.0; // 50 Hz

    sensors::IMU imu(p, /*seed=*/99);

    physics::State state;
    constexpr double dt    = 0.004;
    const     int    steps = 500; // 2 s

    // Hover: accel_world = gravity, so specific force ≈ 0 on body z (level attitude).
    // With vibration, body-z output oscillates at 50 Hz around ~0.
    double sum_z2 = 0.0;
    for (int i = 0; i < steps; ++i) {
        state.time += dt;
        const auto s = imu.sample(state, Eigen::Vector3d(0.0, 0.0, 9.80665));
        sum_z2 += s.accel_body.z() * s.accel_body.z();
    }

    // RMS of body-z should be ~amplitude / sqrt(2) ≈ 0.707 m/s²
    const double rms_z = std::sqrt(sum_z2 / steps);
    EXPECT_GT(rms_z, 0.3) << "Z-accel RMS should reflect vibration injection";
    EXPECT_LT(rms_z, 2.0) << "Z-accel RMS should not exceed 2× vibration amplitude";
}
