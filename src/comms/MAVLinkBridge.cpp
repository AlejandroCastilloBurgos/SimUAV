#include "simuav/comms/MAVLinkBridge.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cmath>

namespace simuav::comms {

MAVLinkBridge::MAVLinkBridge(BridgeParams params)
    : params_(std::move(params)) {}

MAVLinkBridge::~MAVLinkBridge() { close(); }

bool MAVLinkBridge::open() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::perror("MAVLinkBridge: socket");
        return false;
    }

    // Non-blocking so receiveActuators() never stalls the physics loop
    const int flags = ::fcntl(sock_fd_, F_GETFL, 0);
    ::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in local{};
    local.sin_family      = AF_INET;
    local.sin_port        = htons(params_.local_port);
    local.sin_addr.s_addr = INADDR_ANY;

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        std::perror("MAVLinkBridge: bind");
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }
    return true;
}

void MAVLinkBridge::close() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void MAVLinkBridge::sendMessage(const mavlink_message_t& msg) {
    if (sock_fd_ < 0) return;

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
    const uint16_t len = mavlink_msg_to_send_buffer(buf.data(), &msg);

    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port   = htons(params_.remote_port);
    ::inet_pton(AF_INET, params_.remote_host.c_str(), &remote.sin_addr);

    ::sendto(sock_fd_, buf.data(), len, 0,
             reinterpret_cast<const sockaddr*>(&remote), sizeof(remote));
}

void MAVLinkBridge::sendHilSensor(const sensors::IMUSample&  imu,
                                   const sensors::BaroSample& baro,
                                   const sensors::MagSample&  mag)
{
    mavlink_message_t msg{};
    const uint64_t time_us = static_cast<uint64_t>(imu.timestamp * 1e6);

    // fields_updated bitmask: accel(0-2) + gyro(3-5) + mag(6-8) + baro(9-11)
    constexpr uint32_t kAllSensors = 0b111111111111u;

    mavlink_msg_hil_sensor_pack(
        params_.system_id, params_.component_id, &msg,
        time_us,
        static_cast<float>(imu.accel_body.x()),
        static_cast<float>(imu.accel_body.y()),
        static_cast<float>(imu.accel_body.z()),
        static_cast<float>(imu.gyro_body.x()),
        static_cast<float>(imu.gyro_body.y()),
        static_cast<float>(imu.gyro_body.z()),
        mag.field_body.x(),
        mag.field_body.y(),
        mag.field_body.z(),
        baro.pressure_pa / 100.0f,  // hPa
        0.0f,                        // diff_pressure (not modelled)
        baro.altitude_m,
        baro.temperature_c,
        kAllSensors
    );
    sendMessage(msg);
}

void MAVLinkBridge::sendHilGps(const sensors::GPSSample& gps) {
    mavlink_message_t msg{};
    const uint64_t time_us = static_cast<uint64_t>(gps.timestamp * 1e6);

    // MAVLink HIL_GPS expects lat/lon in 1e7 degrees, alt in mm
    const int32_t lat_e7  = static_cast<int32_t>(gps.latitude_deg  * 1e7);
    const int32_t lon_e7  = static_cast<int32_t>(gps.longitude_deg * 1e7);
    const int32_t alt_mm  = static_cast<int32_t>(gps.altitude_m * 1000.0f);

    const float ground_speed = std::sqrt(gps.velocity_n * gps.velocity_n +
                                          gps.velocity_e * gps.velocity_e);

    mavlink_msg_hil_gps_pack(
        params_.system_id, params_.component_id, &msg,
        time_us,
        gps.fix_type,
        lat_e7, lon_e7, alt_mm,
        static_cast<uint16_t>(gps.eph * 100),
        static_cast<uint16_t>(gps.epv * 100),
        static_cast<uint16_t>(ground_speed * 100),
        static_cast<int16_t>(gps.velocity_n * 100),
        static_cast<int16_t>(gps.velocity_e * 100),
        static_cast<int16_t>(gps.velocity_d * 100),
        UINT16_MAX,  // course over ground (unknown)
        gps.num_sats,
        0  // id (single GPS)
    );
    sendMessage(msg);
}

bool MAVLinkBridge::receiveActuators(std::array<double, 4>& out_speeds) {
    if (sock_fd_ < 0) return false;

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
    mavlink_status_t status{};
    mavlink_message_t msg{};

    bool got_actuators = false;

    ssize_t n;
    while ((n = ::recv(sock_fd_, buf.data(), buf.size(), 0)) > 0) {
        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;

            if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
                mavlink_hil_actuator_controls_t act{};
                mavlink_msg_hil_actuator_controls_decode(&msg, &act);

                // Map normalised [0,1] PWM output → motor angular speed in rad/s.
                // Linear mapping: 0 → 0 rad/s, 1 → max_motor_speed.
                // Actual ESC calibration tables should replace this.
                constexpr double kMaxSpeed = 838.0; // rad/s, must match QuadrotorParams
                for (int i = 0; i < 4; ++i) {
                    const double pwm = std::max(0.0f, std::min(1.0f, act.controls[i]));
                    out_speeds[i] = pwm * kMaxSpeed;
                }
                got_actuators = true;
            }
        }
    }
    return got_actuators;
}

}  // namespace simuav::comms
