#include "simuav/ScenarioLoader.h"

#include <gtest/gtest.h>
#include <fstream>
#include <string>

namespace {

std::string writeTempScenario(const std::string& content) {
    const std::string path = "/tmp/simuav_test_scenario.json";
    std::ofstream f(path);
    f << content;
    return path;
}

} // namespace

TEST(ScenarioLoader, ParsesWellFormedFile) {
    const std::string path = writeTempScenario(R"([
        {"time_s": 5.0,  "type": "wind_mean",      "params": {"ned": [2.0, 0.0, 0.0]}},
        {"time_s": 1.0,  "type": "motor_lock",     "params": {"motor": 2}},
        {"time_s": 10.0, "type": "imu_bias_inject","params": {"accel": [0.1, 0.0, 0.0], "gyro": [0.0, 0.0, 0.01]}},
        {"time_s": 20.0, "type": "stop"}
    ])");

    const auto events = simuav::loadScenario(path);

    ASSERT_EQ(events.size(), 4u);

    // Sorted ascending by time_s
    EXPECT_DOUBLE_EQ(events[0].time_s, 1.0);
    EXPECT_EQ(events[0].type, "motor_lock");

    EXPECT_DOUBLE_EQ(events[1].time_s, 5.0);
    EXPECT_EQ(events[1].type, "wind_mean");

    EXPECT_DOUBLE_EQ(events[2].time_s, 10.0);
    EXPECT_EQ(events[2].type, "imu_bias_inject");

    EXPECT_DOUBLE_EQ(events[3].time_s, 20.0);
    EXPECT_EQ(events[3].type, "stop");
}

TEST(ScenarioLoader, ThrowsOnMissingFile) {
    EXPECT_THROW(simuav::loadScenario("/tmp/__nonexistent_scenario__.json"),
                 std::runtime_error);
}

TEST(ScenarioLoader, EmptyArrayIsNoOp) {
    const std::string path = writeTempScenario("[]");
    const auto events = simuav::loadScenario(path);
    EXPECT_TRUE(events.empty());
}

TEST(ScenarioLoader, ThrowsOnNonArrayRoot) {
    const std::string path = writeTempScenario(R"({"time_s": 1.0, "type": "stop"})");
    EXPECT_THROW(simuav::loadScenario(path), std::runtime_error);
}

TEST(ScenarioLoader, EventsWithPastTimeFire) {
    const std::string path = writeTempScenario(R"([
        {"time_s": 0.0, "type": "motor_lock", "params": {"motor": 0}}
    ])");
    const auto events = simuav::loadScenario(path);

    ASSERT_EQ(events.size(), 1u);
    EXPECT_DOUBLE_EQ(events[0].time_s, 0.0);
    EXPECT_EQ(events[0].type, "motor_lock");
}

TEST(ScenarioLoader, ParsesGpsFixTypeEvent) {
    const std::string path = writeTempScenario(R"([
        {"time_s": 5.0, "type": "gps_fix_type", "params": {"fix_type": 0, "num_sats": 0}}
    ])");
    const auto events = simuav::loadScenario(path);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].type, "gps_fix_type");
    EXPECT_EQ(events[0].params.at("fix_type").get<int>(), 0);
    EXPECT_EQ(events[0].params.at("num_sats").get<int>(), 0);
}

TEST(ScenarioLoader, ParsesGpsNoiseOverrideEvent) {
    const std::string path = writeTempScenario(R"([
        {"time_s": 10.0, "type": "gps_noise_override", "params": {"pos_std_m": 50.0, "alt_std_m": 80.0}}
    ])");
    const auto events = simuav::loadScenario(path);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].type, "gps_noise_override");
    EXPECT_DOUBLE_EQ(events[0].params.at("pos_std_m").get<double>(), 50.0);
}

TEST(ScenarioLoader, ParsesBaroNoiseOverrideEvent) {
    const std::string path = writeTempScenario(R"([
        {"time_s": 15.0, "type": "baro_noise_override", "params": {"noise_std_m": 5.0}}
    ])");
    const auto events = simuav::loadScenario(path);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].type, "baro_noise_override");
    EXPECT_DOUBLE_EQ(events[0].params.at("noise_std_m").get<double>(), 5.0);
}
