#include <gtest/gtest.h>
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/Magnetometer.h"
#include <Eigen/Geometry>
#include <cmath>

using namespace simuav::physics;
using namespace simuav::sensors;

static constexpr double kG = 9.80665;

// Motor speed that produces hover thrust for the default quad params.
static double hoverSpeed() {
    QuadrotorParams p;
    return std::sqrt((p.mass * kG) / (4.0 * p.k_thrust));
}

// At level attitude with all motors at hover speed, thrust is along body -Z.
// Rotated to the NED world frame (identity rotation), thrust_world = (0, 0, -F).
// Net z-accel from lastAccelWorld() = -F/m + g ≈ 0, with x=y=0. A sign error
// in the thrust direction would give z > 0 at any motor speed above hover.
TEST(FrameConventions, ThrustDirectionIsBodyNegativeZ) {
    QuadrotorParams p;
    p.motor_time_constant_s = 0.0;
    p.aero_drag             = 0.0;
    QuadrotorModel model(p);

    // 10 % above hover so net z-accel is strictly negative if thrust is upward.
    const double w = hoverSpeed() * 1.1;
    model.integrate({w, w, w, w}, 1e-6);

    EXPECT_NEAR(model.lastAccelWorld().x(), 0.0, 1e-6); // no lateral force
    EXPECT_NEAR(model.lastAccelWorld().y(), 0.0, 1e-6);
    EXPECT_LT(model.lastAccelWorld().z(), 0.0);          // thrust wins → upward = NED -z
}

// With zero thrust and zero drag, the only force is gravity: [0, 0, +g] NED.
// Ground constraint is disabled so the clamp does not zero lastAccelWorld.
TEST(FrameConventions, GravityIsNEDPositiveZ) {
    QuadrotorParams p;
    p.motor_time_constant_s    = 0.0;
    p.aero_drag                = 0.0;
    p.enable_ground_constraint = false;
    QuadrotorModel model(p);

    model.integrate({}, 1e-6);

    EXPECT_NEAR(model.lastAccelWorld().z(), kG,  1e-9);
    EXPECT_NEAR(model.lastAccelWorld().x(), 0.0, 1e-9);
    EXPECT_NEAR(model.lastAccelWorld().y(), 0.0, 1e-9);
}

// Roll torque formula: L * k_thrust * (w0² + w2² − w1² − w3²).
// Raising FL(0) and RL(2) (left motors) above FR(1)/RR(3) gives positive roll
// torque → positive angular_velocity.x() (roll right in FRD).
TEST(FrameConventions, RollTorqueIncreasesLeftMotors) {
    QuadrotorParams p;
    p.motor_time_constant_s = 0.0;
    QuadrotorModel model(p);

    const double w    = hoverSpeed();
    const double w_hi = w * 1.2;
    const double w_lo = w * 0.8;
    model.integrate({w_hi, w_lo, w_hi, w_lo}, 0.001);

    EXPECT_GT(model.state().angular_velocity.x(), 0.0);
}

// At hover equilibrium the net world acceleration is zero.
// The accelerometer measures specific force = (world_accel − gravity) in body frame.
// With level attitude: specific_force_body = [0, 0, -g].
TEST(FrameConventions, IMUSpecificForceEqualsNegativeGravityAtHover) {
    QuadrotorParams qp;
    qp.motor_time_constant_s = 0.0;
    QuadrotorModel model(qp);

    const double w = hoverSpeed();
    model.integrate({w, w, w, w}, 1e-6);

    IMUParams ip;
    ip.accel_arw_std          = 0.0;
    ip.accel_bias_instability = 0.0;
    ip.gyro_arw_std           = 0.0;
    ip.gyro_bias_instability  = 0.0;
    IMU imu(ip);

    const IMUSample s = imu.sample(model.state(), model.lastAccelWorld());
    EXPECT_NEAR(s.accel_body.x(),  0.0, 0.01);
    EXPECT_NEAR(s.accel_body.y(),  0.0, 0.01);
    EXPECT_NEAR(s.accel_body.z(), -kG,  0.01);
}

// With a 90° right-roll attitude the NED down component of Earth's field maps
// to body Y, and the NED north component maps to body X.
// Rotation matrix (body→NED) for Rx(90°): body_field = R^T * earth_ned gives
//   body.x = north, body.y = down, body.z = -east.
TEST(FrameConventions, MagnetometerBodyFrameRotatesWithAttitude) {
    MagParams mp;
    mp.noise_std = 0.0;
    Magnetometer mag(mp);

    State s;
    s.attitude = Eigen::Quaterniond(
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));

    const MagSample ms = mag.sample(s);

    const float earth_x = static_cast<float>(mp.earth_field_ned.x()); // north
    const float earth_z = static_cast<float>(mp.earth_field_ned.z()); // down

    EXPECT_NEAR(ms.field_body.x(),  earth_x, 1e-4f); // body-X = NED-north
    EXPECT_NEAR(ms.field_body.y(),  earth_z, 1e-4f); // body-Y = NED-down
}
