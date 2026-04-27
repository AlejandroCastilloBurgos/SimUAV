#include <gtest/gtest.h>
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/physics/WindModel.h"
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

TEST(QuadrotorModel, RK4IsMoreAccurateThanEulerInFreeFall) {
    // With aero_drag = 0, free fall is dv/dt = g (constant), z(t) = 0.5*g*t^2.
    // RK4 integrates degree-2 polynomials exactly; semi-implicit Euler accumulates O(dt) error.
    static constexpr double kG  = 9.80665;
    static constexpr double kT  = 10.0;
    static constexpr double kDt = 0.004;
    static constexpr int    kN  = static_cast<int>(kT / kDt);

    const std::array<double, kNumMotors> motors{};

    QuadrotorParams p_rk4;
    p_rk4.use_rk4   = true;
    p_rk4.aero_drag = 0.0;
    QuadrotorModel rk4_model(p_rk4);

    QuadrotorParams p_euler;
    p_euler.use_rk4   = false;
    p_euler.aero_drag = 0.0;
    QuadrotorModel euler_model(p_euler);

    for (int i = 0; i < kN; ++i) {
        rk4_model.integrate(motors, kDt);
        euler_model.integrate(motors, kDt);
    }

    const double z_exact     = 0.5 * kG * kT * kT;
    const double rk4_error   = std::abs(rk4_model.state().position.z()  - z_exact);
    const double euler_error = std::abs(euler_model.state().position.z() - z_exact);

    EXPECT_LT(rk4_error,   1e-6);        // RK4 is exact for constant-force ODEs
    EXPECT_LT(rk4_error,   euler_error); // RK4 beats Euler
}

TEST(WindModel, SampleReturnsFiniteVector) {
    WindModel wind;
    for (int i = 0; i < 100; ++i) {
        const Eigen::Vector3d w = wind.sample();
        EXPECT_TRUE(w.allFinite());
    }
}
