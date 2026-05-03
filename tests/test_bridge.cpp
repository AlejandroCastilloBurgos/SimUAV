// simuav
#include "simuav/comms/MAVLinkBridge.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/Magnetometer.h"

// third-party
#include <gtest/gtest.h>

// POSIX
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

// std
#include <array>
#include <cmath>
#include <cstdint>

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

// ── UDP loopback tests ────────────────────────────────────────────────────────
//
// Each test uses two ephemeral UDP sockets on 127.0.0.1 — one for the bridge
// under test, one acting as the simulated firmware peer.  No external process
// is required.  The peer socket uses poll() with a 250 ms timeout to avoid
// hanging the suite on unexpected failures.

namespace {

// Receive one datagram, waiting up to timeout_ms. Returns byte count or -1.
static ssize_t recvWithTimeout(int fd, void* buf, std::size_t len,
                               sockaddr_in* src, int timeout_ms)
{
    pollfd pfd{fd, POLLIN, 0};
    if (::poll(&pfd, 1, timeout_ms) <= 0) return -1;
    socklen_t sl = sizeof(*src);
    return ::recvfrom(fd, buf, static_cast<socklen_t>(len), 0,
                      reinterpret_cast<sockaddr*>(src), &sl);
}

// Parse a raw UDP payload into a mavlink_message_t using channel 1 (separate
// parser state from the bridge's internal channel 0).
static bool parseMavlink(const uint8_t* data, ssize_t len, mavlink_message_t& out)
{
    mavlink_status_t st{};
    for (ssize_t i = 0; i < len; ++i)
        if (mavlink_parse_char(MAVLINK_COMM_1, data[i], &out, &st))
            return true;
    return false;
}

// Inject a MAVLink message from the peer socket to a given local port.
static void injectToBridge(int peer_fd, uint16_t bridge_port,
                           const mavlink_message_t& msg)
{
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
    const uint16_t n = mavlink_msg_to_send_buffer(buf.data(), &msg);
    sockaddr_in dst{};
    dst.sin_family      = AF_INET;
    dst.sin_port        = htons(bridge_port);
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    (void)::sendto(peer_fd, buf.data(), n, 0,
                   reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
}

// Allocate an ephemeral port by binding a throwaway socket, reading the port,
// then closing it — so the caller can hand the port to another component.
// There is a tiny TOCTOU window; this is acceptable on loopback in CI.
static uint16_t allocEphemeralPort()
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(0);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    (void)::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    socklen_t sl = sizeof(addr);
    (void)::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &sl);
    const uint16_t port = ntohs(addr.sin_port);
    (void)::close(fd);
    return port;
}

}  // namespace

class LoopbackFixture : public ::testing::Test {
protected:
    void SetUp() override {
        // Create the peer (simulated firmware) socket and bind it to loopback.
        peer_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        ASSERT_GE(peer_fd_, 0);
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(0);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ASSERT_EQ(0, ::bind(peer_fd_,
                            reinterpret_cast<const sockaddr*>(&addr),
                            sizeof(addr)));
        socklen_t sl = sizeof(addr);
        ASSERT_EQ(0, ::getsockname(peer_fd_,
                                   reinterpret_cast<sockaddr*>(&addr), &sl));
        peer_port_ = ntohs(addr.sin_port);

        // Allocate an ephemeral port for the bridge's receive socket.
        bridge_local_port_ = allocEphemeralPort();

        simuav::comms::BridgeParams p;
        p.remote_host      = "127.0.0.1";
        p.remote_port      = peer_port_;
        p.local_port       = bridge_local_port_;
        p.system_id        = 1;
        p.component_id     = MAV_COMP_ID_AUTOPILOT1;
        p.max_motor_speed  = 838.0;
        p.esc_exponent     = 1.0;   // linear: speed = throttle * max_motor_speed
        p.motor_spin_min   = 0.0;
        bridge_ = simuav::comms::MAVLinkBridge(p);
        ASSERT_TRUE(bridge_.open());
    }

    void TearDown() override {
        bridge_.close();
        if (peer_fd_ >= 0) {
            (void)::close(peer_fd_);
            peer_fd_ = -1;
        }
    }

    int      peer_fd_{-1};
    uint16_t peer_port_{0};
    uint16_t bridge_local_port_{0};
    simuav::comms::MAVLinkBridge bridge_;
};

TEST_F(LoopbackFixture, SendHilSensorDeliversMavlinkPacket) {
    simuav::sensors::IMUSample imu;
    imu.timestamp      = 1.0;
    imu.accel_body     = {1.1, 2.2, 3.3};
    imu.gyro_body      = {0.1, 0.2, 0.3};

    simuav::sensors::BaroSample baro{};
    baro.pressure_pa   = 101325.0f;
    baro.altitude_m    = 488.0f;
    baro.temperature_c = 15.0f;

    simuav::sensors::MagSample mag{};
    mag.field_body = {0.21f, 0.005f, 0.42f};

    bridge_.sendHilSensor(imu, baro, mag);

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
    sockaddr_in src{};
    const ssize_t n = recvWithTimeout(peer_fd_, buf.data(), buf.size(),
                                      &src, 250);
    ASSERT_GT(n, 0) << "HIL_SENSOR datagram not received within 250 ms";

    mavlink_message_t msg{};
    ASSERT_TRUE(parseMavlink(buf.data(), n, msg));
    EXPECT_EQ(static_cast<uint32_t>(MAVLINK_MSG_ID_HIL_SENSOR), msg.msgid);

    mavlink_hil_sensor_t s{};
    mavlink_msg_hil_sensor_decode(&msg, &s);

    EXPECT_EQ(1'000'000ULL, s.time_usec);
    EXPECT_NEAR(1.1f,   s.xacc,        1e-4f);
    EXPECT_NEAR(2.2f,   s.yacc,        1e-4f);
    EXPECT_NEAR(3.3f,   s.zacc,        1e-4f);
    EXPECT_NEAR(0.1f,   s.xgyro,       1e-5f);
    EXPECT_NEAR(0.2f,   s.ygyro,       1e-5f);
    EXPECT_NEAR(0.3f,   s.zgyro,       1e-5f);
    EXPECT_NEAR(0.21f,  s.xmag,        1e-5f);
    EXPECT_NEAR(0.005f, s.ymag,        1e-5f);
    EXPECT_NEAR(0.42f,  s.zmag,        1e-5f);
    EXPECT_NEAR(1013.25f, s.abs_pressure, 0.01f); // 101325 Pa / 100 = hPa
    EXPECT_NEAR(488.0f, s.pressure_alt, 1e-4f);
    EXPECT_NEAR(15.0f,  s.temperature,  1e-4f);
    EXPECT_EQ(7167u,    s.fields_updated);
    EXPECT_EQ(0u,       s.id);
}

TEST_F(LoopbackFixture, SendHilGpsDeliversMavlinkPacket) {
    simuav::sensors::GPSSample gps{};
    gps.timestamp      = 2.0;
    gps.fix_type       = 3;
    gps.latitude_deg   = 47.397742;
    gps.longitude_deg  = 8.545594;
    gps.altitude_m     = 488.0f;
    gps.velocity_n     = 1.0f;
    gps.velocity_e     = 2.0f;
    gps.velocity_d     = -0.5f;
    gps.eph            = 1.5f;
    gps.epv            = 2.5f;
    gps.num_sats       = 12;

    bridge_.sendHilGps(gps);

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
    sockaddr_in src{};
    const ssize_t n = recvWithTimeout(peer_fd_, buf.data(), buf.size(),
                                      &src, 250);
    ASSERT_GT(n, 0) << "HIL_GPS datagram not received within 250 ms";

    mavlink_message_t msg{};
    ASSERT_TRUE(parseMavlink(buf.data(), n, msg));
    EXPECT_EQ(static_cast<uint32_t>(MAVLINK_MSG_ID_HIL_GPS), msg.msgid);

    mavlink_hil_gps_t g{};
    mavlink_msg_hil_gps_decode(&msg, &g);

    EXPECT_EQ(2'000'000ULL, g.time_usec);
    EXPECT_EQ(static_cast<int32_t>(47.397742 * 1e7), g.lat);
    EXPECT_EQ(static_cast<int32_t>(8.545594  * 1e7), g.lon);
    EXPECT_EQ(488000,                                  g.alt); // mm
    EXPECT_EQ(static_cast<int16_t>(1.0f  * 100), g.vn);
    EXPECT_EQ(static_cast<int16_t>(2.0f  * 100), g.ve);
    EXPECT_EQ(static_cast<int16_t>(-0.5f * 100), g.vd);
    // ground speed = sqrt(1²+2²)*100 ≈ 223 cm/s
    const uint16_t expected_vel = static_cast<uint16_t>(
        std::sqrt(1.0f * 1.0f + 2.0f * 2.0f) * 100.0f);
    EXPECT_EQ(expected_vel,               g.vel);
    EXPECT_EQ(static_cast<uint16_t>(150), g.eph); // 1.5*100
    EXPECT_EQ(static_cast<uint16_t>(250), g.epv); // 2.5*100
    EXPECT_EQ(12u,                        g.satellites_visible);
    EXPECT_EQ(3u,                         g.fix_type);
}

TEST_F(LoopbackFixture, ReceiveActuatorsDecodesMotorSpeeds) {
    // Build a HIL_ACTUATOR_CONTROLS packet with known throttle values.
    float controls[16]{};
    controls[0] = 0.25f;
    controls[1] = 0.50f;
    controls[2] = 0.75f;
    controls[3] = 1.00f;

    mavlink_message_t msg{};
    mavlink_msg_hil_actuator_controls_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &msg,
        /*time_usec=*/0, controls, /*mode=*/0, /*flags=*/0);

    // Send from peer to bridge's bound port — arrives in bridge's receive
    // buffer before receiveActuators() is called (loopback is synchronous).
    injectToBridge(peer_fd_, bridge_local_port_, msg);

    std::array<double, 4> speeds{};
    ASSERT_TRUE(bridge_.receiveActuators(speeds));

    // esc_exponent=1.0, motor_spin_min=0.0 → speed = throttle * 838.0
    EXPECT_NEAR(speeds[0], 0.25 * 838.0, 1e-6);
    EXPECT_NEAR(speeds[1], 0.50 * 838.0, 1e-6);
    EXPECT_NEAR(speeds[2], 0.75 * 838.0, 1e-6);
    EXPECT_NEAR(speeds[3], 1.00 * 838.0, 1e-6);
}
