// simuav
#include "simuav/comms/MAVLinkBridge.h"

// third-party
#include <gtest/gtest.h>

using simuav::comms::MAVLinkBridge;

TEST(ESCCurve, LinearExponentGivesLinearSpeedMapping) {
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(0.0, 838.0, 0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(0.5, 838.0, 0.0, 1.0), 419.0);
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(1.0, 838.0, 0.0, 1.0), 838.0);
}

TEST(ESCCurve, SqrtExponentGivesThrustLinearBehavior) {
    // exponent=0.5: ω = max * sqrt(throttle), so thrust ∝ throttle
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(0.0,  838.0, 0.0, 0.5), 0.0);
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(1.0,  838.0, 0.0, 0.5), 838.0);
    // sqrt(0.25) = 0.5 → half max speed at quarter throttle
    EXPECT_NEAR(MAVLinkBridge::escToSpeed(0.25, 838.0, 0.0, 0.5), 419.0, 1e-9);
}

TEST(ESCCurve, ThrottleClampsAtBoundaries) {
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(-1.0, 838.0, 0.0, 0.5), 0.0);
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed( 2.0, 838.0, 0.0, 0.5), 838.0);
}

TEST(ESCCurve, SpinMinAppliedAtZeroThrottle) {
    constexpr double kSpinMin = 100.0;
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(0.0, 838.0, kSpinMin, 0.5), kSpinMin);
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(1.0, 838.0, kSpinMin, 0.5), 838.0);
}

TEST(ESCCurve, SpinMinOffsetScalesCorrectly) {
    // With spin_min=100, max=838, exponent=1.0, throttle=0.5:
    // ω = 100 + (838-100)*0.5 = 100 + 369 = 469
    EXPECT_DOUBLE_EQ(MAVLinkBridge::escToSpeed(0.5, 838.0, 100.0, 1.0), 469.0);
}

// ── Dialect: message ID regressions ──────────────────────────────────────────
//
// These tests pin the compile-time MAVLink constants that both firmware targets
// depend on. Switching the dialect header (ardupilotmega vs common) must not
// change any of these IDs.

TEST(MavLinkDialect, Px4HilMessageIds) {
    // Pack a HIL_SENSOR and confirm the resulting msgid matches the constant.
    mavlink_message_t msg{};
    mavlink_msg_hil_sensor_pack(1, 1, &msg,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    EXPECT_EQ(msg.msgid, static_cast<uint32_t>(MAVLINK_MSG_ID_HIL_SENSOR));
    EXPECT_EQ(static_cast<uint32_t>(MAVLINK_MSG_ID_HIL_SENSOR),           107u);
    EXPECT_EQ(static_cast<uint32_t>(MAVLINK_MSG_ID_HIL_GPS),              113u);
    EXPECT_EQ(static_cast<uint32_t>(MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS), 93u);
}

TEST(MavLinkDialect, ArduPilotRcChannelsOverrideId) {
    EXPECT_EQ(static_cast<uint32_t>(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE), 70u);
}

// ── HIL_SENSOR fields_updated bitmask ────────────────────────────────────────
//
// Regression test for the bitmask used in MAVLinkBridge::sendHilSensor.
// The old value (0b111111111111 = 4095) set DIFF_PRESSURE (bit 10) and
// cleared TEMPERATURE (bit 12). The corrected symbolic constant must invert
// that: DIFF_PRESSURE clear, TEMPERATURE set.

TEST(HilSensorBitmask, CorrectBitmaskValue) {
    constexpr uint32_t kCorrect =
        HIL_SENSOR_UPDATED_XACC         |
        HIL_SENSOR_UPDATED_YACC         |
        HIL_SENSOR_UPDATED_ZACC         |
        HIL_SENSOR_UPDATED_XGYRO        |
        HIL_SENSOR_UPDATED_YGYRO        |
        HIL_SENSOR_UPDATED_ZGYRO        |
        HIL_SENSOR_UPDATED_XMAG         |
        HIL_SENSOR_UPDATED_YMAG         |
        HIL_SENSOR_UPDATED_ZMAG         |
        HIL_SENSOR_UPDATED_ABS_PRESSURE |
        HIL_SENSOR_UPDATED_PRESSURE_ALT |
        HIL_SENSOR_UPDATED_TEMPERATURE;

    EXPECT_EQ(kCorrect, 7167u);  // 1+2+4+8+16+32+64+128+256+512+2048+4096

    EXPECT_NE(kCorrect & HIL_SENSOR_UPDATED_TEMPERATURE,   0u) << "TEMPERATURE bit must be set";
    EXPECT_EQ(kCorrect & HIL_SENSOR_UPDATED_DIFF_PRESSURE, 0u) << "DIFF_PRESSURE bit must be clear";
}

TEST(HilSensorBitmask, OldBitmaskWasWrong) {
    constexpr uint32_t kOld = 0b111111111111u; // 4095 — the pre-fix value
    EXPECT_NE(kOld & HIL_SENSOR_UPDATED_DIFF_PRESSURE, 0u) << "old value incorrectly set DIFF_PRESSURE";
    EXPECT_EQ(kOld & HIL_SENSOR_UPDATED_TEMPERATURE,   0u) << "old value incorrectly cleared TEMPERATURE";
}

TEST(MavLinkDialect, ArduPilotPwmNormalisation) {
    // 1000 µs → throttle 0.0, 1500 µs → 0.5, 2000 µs → 1.0
    auto pwmToSpeed = [](uint16_t pwm) {
        const double t = (static_cast<double>(pwm) - 1000.0) / 1000.0;
        return MAVLinkBridge::escToSpeed(t, 838.0, 0.0, 1.0);
    };
    EXPECT_DOUBLE_EQ(pwmToSpeed(1000),   0.0);
    EXPECT_DOUBLE_EQ(pwmToSpeed(1500), 419.0);
    EXPECT_DOUBLE_EQ(pwmToSpeed(2000), 838.0);
    // Values outside [1000,2000] clamp to [0,1] throttle range
    EXPECT_DOUBLE_EQ(pwmToSpeed(500),    0.0);
    EXPECT_DOUBLE_EQ(pwmToSpeed(3000), 838.0);
}
