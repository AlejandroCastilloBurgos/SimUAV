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
