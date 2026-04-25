#include <gtest/gtest.h>
#include "simuav/physics/QuadrotorModel.h"
#include <cmath>

using namespace simuav::physics;

// Motor speed that produces hover thrust for default 1.5 kg quad:
//   F = 4 * k_thrust * w^2 = mass * g
//   w = sqrt(mass * g / (4 * k_thrust))
static double hoverSpeed() {
    QuadrotorParams p;
    return std::sqrt((p.mass * 9.80665) / (4.0 * p.k_thrust));
}

TEST(QuadrotorModel, InitialisesToZeroState) {
    QuadrotorModel model;
    const State& s = model.state();
    EXPECT_DOUBLE_EQ(s.time, 0.0);
    EXPECT_NEAR(s.position.norm(), 0.0, 1e-12);
    EXPECT_NEAR(s.velocity.norm(), 0.0, 1e-12);
    EXPECT_NEAR(s.attitude.w(), 1.0, 1e-12);
}

TEST(QuadrotorModel, HoverKeepsAltitudeStable) {
    QuadrotorModel model;
    const double w = hoverSpeed();
    const std::array<double, kNumMotors> motors{w, w, w, w};

    // Run for 1 s at 250 Hz
    for (int i = 0; i < 250; ++i) {
        model.integrate(motors, 0.004);
    }

    // Altitude (NED z) should stay near 0 (allow numerical drift < 5 cm)
    EXPECT_NEAR(model.state().position.z(), 0.0, 0.05);
}

TEST(QuadrotorModel, ZeroThrotleDropsUnderGravity) {
    QuadrotorModel model;
    const std::array<double, kNumMotors> motors{};

    // 0.5 s free fall
    for (int i = 0; i < 125; ++i) {
        model.integrate(motors, 0.004);
    }

    // Should have fallen: z > 0 in NED (positive z = down)
    EXPECT_GT(model.state().position.z(), 0.5);
    EXPECT_GT(model.state().velocity.z(), 0.0);
}

TEST(QuadrotorModel, AttitudeQuaternionRemainsNormalised) {
    QuadrotorModel model;
    const double w = hoverSpeed() * 1.1; // slight roll perturbation via asymmetry
    const std::array<double, kNumMotors> motors{w, w * 0.9, w, w * 0.9};

    for (int i = 0; i < 500; ++i) {
        model.integrate(motors, 0.004);
    }

    EXPECT_NEAR(model.state().attitude.norm(), 1.0, 1e-9);
}

TEST(WindModel, SampleReturnsFiniteVector) {
    WindModel wind;
    for (int i = 0; i < 100; ++i) {
        const Eigen::Vector3d w = wind.sample();
        EXPECT_TRUE(w.allFinite());
    }
}
