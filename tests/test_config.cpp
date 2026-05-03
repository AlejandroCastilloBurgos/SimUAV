#include <gtest/gtest.h>
#include "simuav/ConfigLoader.h"
#include <fstream>
#include <cstdio>

using namespace simuav;

// Write a temporary JSON file and return its path.
static std::string writeTempJson(const std::string& content) {
    const std::string path = "/tmp/simuav_test_config.json";
    std::ofstream f(path);
    f << content;
    return path;
}

TEST(ConfigLoader, LoadsAllTopLevelFields) {
    const std::string path = writeTempJson(R"({
        "dt": 0.002,
        "mavlink_host": "192.168.1.1",
        "mavlink_port": 14550,
        "mavlink_local_port": 15001,
        "json_log_path": "out.json",
        "ulog_path": "out.ulg"
    })");

    const SimConfig cfg = loadConfig(path);

    EXPECT_DOUBLE_EQ(cfg.dt, 0.002);
    EXPECT_EQ(cfg.mavlink_host, "192.168.1.1");
    EXPECT_EQ(cfg.mavlink_port, 14550);
    EXPECT_EQ(15001u, cfg.mavlink_local_port);
    EXPECT_EQ(cfg.json_log_path, "out.json");
    EXPECT_EQ(cfg.ulog_path, "out.ulg");
}

TEST(ConfigLoader, LoadsQuadParams) {
    const std::string path = writeTempJson(R"({
        "quad_params": {
            "mass": 2.0,
            "arm_length": 0.3,
            "k_thrust": 1e-5,
            "k_drag": 2e-7,
            "aero_drag": 0.4,
            "inertia_diag": [0.05, 0.05, 0.09],
            "max_motor_speed": 900.0
        }
    })");

    const SimConfig cfg = loadConfig(path);

    EXPECT_DOUBLE_EQ(cfg.quad_params.mass, 2.0);
    EXPECT_DOUBLE_EQ(cfg.quad_params.arm_length, 0.3);
    EXPECT_DOUBLE_EQ(cfg.quad_params.max_motor_speed, 900.0);
    EXPECT_NEAR(cfg.quad_params.inertia_diag.x(), 0.05, 1e-10);
    EXPECT_NEAR(cfg.quad_params.inertia_diag.z(), 0.09, 1e-10);
}

TEST(ConfigLoader, MissingKeysRetainDefaults) {
    // Empty JSON object — every field must fall back to SimConfig defaults.
    const std::string path = writeTempJson("{}");

    const SimConfig cfg  = loadConfig(path);
    const SimConfig dflt{};

    EXPECT_DOUBLE_EQ(cfg.dt, dflt.dt);
    EXPECT_EQ(cfg.mavlink_host, dflt.mavlink_host);
    EXPECT_EQ(cfg.mavlink_port, dflt.mavlink_port);
    EXPECT_DOUBLE_EQ(cfg.quad_params.mass, dflt.quad_params.mass);
    EXPECT_DOUBLE_EQ(cfg.imu_params.accel_arw_std, dflt.imu_params.accel_arw_std);
    EXPECT_DOUBLE_EQ(cfg.gps_params.update_rate_hz, dflt.gps_params.update_rate_hz);
    EXPECT_DOUBLE_EQ(cfg.baro_params.noise_std_m, dflt.baro_params.noise_std_m);
    EXPECT_DOUBLE_EQ(cfg.mag_params.noise_std, dflt.mag_params.noise_std);
}

TEST(ConfigLoader, ThrowsOnMissingFile) {
    EXPECT_THROW(loadConfig("/tmp/does_not_exist_simuav.json"), std::runtime_error);
}

TEST(ConfigLoader, TryLoadConfigReturnDefaultOnMissingFile) {
    const SimConfig cfg  = tryLoadConfig("/tmp/does_not_exist_simuav.json");
    const SimConfig dflt{};
    EXPECT_DOUBLE_EQ(cfg.dt, dflt.dt);
    EXPECT_DOUBLE_EQ(cfg.quad_params.mass, dflt.quad_params.mass);
}

TEST(ConfigLoader, LoadsDefaultJsonFile) {
    // Loads the real config/default.json shipped with the repo.
    // Verifies key fields are within physically sane ranges.
    const SimConfig cfg = loadConfig(std::string(SIMUAV_SOURCE_DIR) + "/config/default.json");

    EXPECT_GT(cfg.dt, 0.0);
    EXPECT_LT(cfg.dt, 1.0);
    EXPECT_GT(cfg.quad_params.mass, 0.0);
    EXPECT_GT(cfg.quad_params.k_thrust, 0.0);
    EXPECT_GT(cfg.gps_params.update_rate_hz, 0.0);
    EXPECT_GT(cfg.imu_params.accel_arw_std, 0.0);
    EXPECT_GT(cfg.baro_params.noise_std_m, 0.0);
    EXPECT_GT(cfg.mag_params.noise_std, 0.0);

    // status_port: disabled (0) or unprivileged port
    EXPECT_TRUE(cfg.status_port == 0u || cfg.status_port >= 1024u);
    // motor_time_constant_s: 0 (instantaneous) to <1 s physically
    EXPECT_GE(cfg.quad_params.motor_time_constant_s, 0.0);
    EXPECT_LT(cfg.quad_params.motor_time_constant_s, 1.0);
    // IMU Gauss-Markov: instability ≥ 0, time constants > 0
    EXPECT_GE(cfg.imu_params.accel_bias_instability,      0.0);
    EXPECT_GT(cfg.imu_params.accel_bias_instability_tc_s, 0.0);
    EXPECT_GE(cfg.imu_params.gyro_bias_instability,       0.0);
    EXPECT_GT(cfg.imu_params.gyro_bias_instability_tc_s,  0.0);
    // IMU vibration: amplitude and frequency non-negative
    EXPECT_GE(cfg.imu_params.vibration_amplitude_mps2, 0.0);
    EXPECT_GE(cfg.imu_params.vibration_frequency_hz,   0.0);
}

TEST(ConfigLoader, LoadsGpsNumSatsAndFixType) {
    const std::string path = writeTempJson(R"({
        "gps_params": {
            "num_sats": 8,
            "fix_type": 2
        }
    })");

    const SimConfig cfg = loadConfig(path);
    EXPECT_EQ(cfg.gps_params.num_sats, 8u);
    EXPECT_EQ(cfg.gps_params.fix_type, 2u);
}

TEST(ConfigLoader, LoadsMavlinkLocalPort) {
    const std::string path = writeTempJson(R"({ "mavlink_local_port": 15000 })");
    const SimConfig cfg = loadConfig(path);
    EXPECT_EQ(15000u, cfg.mavlink_local_port);
}

TEST(ConfigLoader, MavlinkLocalPortDefaultWhenAbsent) {
    const std::string path = writeTempJson(R"({})");
    const SimConfig cfg  = loadConfig(path);
    const SimConfig dflt{};
    EXPECT_EQ(dflt.mavlink_local_port, cfg.mavlink_local_port);
}

TEST(ConfigLoader, LoadsStatusPort) {
    const std::string path = writeTempJson(R"({ "status_port": 9999 })");
    const SimConfig cfg = loadConfig(path);
    EXPECT_EQ(9999u, cfg.status_port);
}

TEST(ConfigLoader, StatusPortDefaultWhenAbsent) {
    const std::string path = writeTempJson(R"({})");
    const SimConfig cfg  = loadConfig(path);
    const SimConfig dflt{};
    EXPECT_EQ(dflt.status_port, cfg.status_port);
}

TEST(ConfigLoader, LoadsMotorTimeConstant) {
    const std::string path = writeTempJson(R"({
        "quad_params": { "motor_time_constant_s": 0.025 }
    })");
    const SimConfig cfg = loadConfig(path);
    EXPECT_DOUBLE_EQ(0.025, cfg.quad_params.motor_time_constant_s);
    // Other quad_params fields must retain their defaults
    EXPECT_DOUBLE_EQ(simuav::physics::QuadrotorParams{}.mass, cfg.quad_params.mass);
}

TEST(ConfigLoader, LoadsImuGaussMarkovFields) {
    const std::string path = writeTempJson(R"({
        "imu_params": {
            "accel_bias_instability":      0.0005,
            "accel_bias_instability_tc_s": 600.0,
            "gyro_bias_instability":       0.00002,
            "gyro_bias_instability_tc_s":  400.0
        }
    })");
    const SimConfig cfg = loadConfig(path);
    EXPECT_DOUBLE_EQ(0.0005,   cfg.imu_params.accel_bias_instability);
    EXPECT_DOUBLE_EQ(600.0,    cfg.imu_params.accel_bias_instability_tc_s);
    EXPECT_DOUBLE_EQ(0.00002,  cfg.imu_params.gyro_bias_instability);
    EXPECT_DOUBLE_EQ(400.0,    cfg.imu_params.gyro_bias_instability_tc_s);
    // Unrelated IMU field must not be zeroed by partial JSON
    EXPECT_DOUBLE_EQ(simuav::sensors::IMUParams{}.accel_arw_std, cfg.imu_params.accel_arw_std);
}

TEST(ConfigLoader, LoadsImuVibrationFields) {
    const std::string path = writeTempJson(R"({
        "imu_params": {
            "vibration_amplitude_mps2": 0.15,
            "vibration_frequency_hz":   200.0
        }
    })");
    const SimConfig cfg = loadConfig(path);
    EXPECT_DOUBLE_EQ(0.15,  cfg.imu_params.vibration_amplitude_mps2);
    EXPECT_DOUBLE_EQ(200.0, cfg.imu_params.vibration_frequency_hz);
}

TEST(ConfigLoader, LoadsArduPilotSitlJsonFile) {
    const SimConfig cfg = loadConfig(
        std::string(SIMUAV_SOURCE_DIR) + "/config/ardupilot_sitl.json");

    EXPECT_EQ(FirmwareTarget::ArduPilot, cfg.firmware_target);
    EXPECT_EQ(9002u, cfg.mavlink_port);
    EXPECT_GT(cfg.dt, 0.0);
    EXPECT_GT(cfg.quad_params.mass, 0.0);
}
