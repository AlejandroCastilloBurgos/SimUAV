#pragma once
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Magnetometer.h"

#include <array>
#include <string>
#include <cstdint>

// MAVLink common dialect (C headers from mavlink/c_library_v2)
#include "common/mavlink.h"

namespace simuav::comms {

struct BridgeParams {
    std::string remote_host{"127.0.0.1"}; // firmware address
    uint16_t    remote_port{14560};       // firmware listens here (PX4 default)
    uint16_t    local_port{14561};        // we bind here
    uint8_t     system_id{1};
    uint8_t     component_id{MAV_COMP_ID_AUTOPILOT1};
};

// Non-blocking UDP bridge between the simulator and drone firmware.
//
// Outbound (sim → firmware):
//   HIL_SENSOR   – accel, gyro, mag, baro  (every physics step)
//   HIL_GPS      – position + velocity     (at GPS update rate)
//
// Inbound (firmware → sim):
//   HIL_ACTUATOR_CONTROLS – motor PWM outputs → converted to rad/s
class MAVLinkBridge {
public:
    explicit MAVLinkBridge(BridgeParams params = {});
    ~MAVLinkBridge();

    // Opens the UDP socket. Call once before the simulation loop.
    bool open();
    void close();

    void sendHilSensor(const sensors::IMUSample& imu,
                       const sensors::BaroSample& baro,
                       const sensors::MagSample& mag);

    void sendHilGps(const sensors::GPSSample& gps);

    // Drains the receive buffer. Returns true if new actuator data was read.
    // out_speeds: motor angular speeds in rad/s (indices match QuadrotorModel).
    bool receiveActuators(std::array<double, 4>& out_speeds);

private:
    void sendMessage(const mavlink_message_t& msg);

    BridgeParams params_;
    int          sock_fd_{-1};
};

}  // namespace simuav::comms
