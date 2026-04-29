#pragma once
#include "simuav/FirmwareTarget.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Magnetometer.h"

#include <array>
#include <string>
#include <cstdint>

// ardupilotmega is a strict superset of common; including it here gives
// access to all common HIL messages plus ArduPilot-specific ones (e.g.
// RC_CHANNELS_OVERRIDE used by ArduPilot SITL to send actuator commands).
#include "ardupilotmega/mavlink.h"

namespace simuav::comms {

struct BridgeParams {
    std::string    remote_host{"127.0.0.1"}; // firmware address
    uint16_t       remote_port{14560};       // firmware listens here (PX4 default)
    uint16_t       local_port{14561};        // we bind here
    uint8_t        system_id{1};
    uint8_t        component_id{MAV_COMP_ID_AUTOPILOT1};
    double         max_motor_speed{838.0};   // rad/s — must match QuadrotorParams
    double         esc_exponent{0.5};        // power-law exponent for throttle→speed curve
    double         motor_spin_min{0.0};      // rad/s — idle speed at zero throttle
    FirmwareTarget firmware_target{FirmwareTarget::PX4};
};

// Non-blocking UDP bridge between the simulator and drone firmware.
//
// Outbound (sim → firmware):
//   HIL_SENSOR   – accel, gyro, mag, baro  (every physics step)
//   HIL_GPS      – position + velocity     (at GPS update rate)
//
// Inbound (firmware → sim):
//   HIL_ACTUATOR_CONTROLS  – normalised [0,1] per motor (PX4 and ArduPilot)
//   RC_CHANNELS_OVERRIDE   – PWM [1000–2000 µs] per channel (ArduPilot SITL)
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

    // Maps normalised throttle [0,1] → rad/s using a power-law ESC curve.
    // ω = motor_spin_min + (max_motor_speed − motor_spin_min) × clamp(throttle,0,1)^exponent
    static double escToSpeed(double throttle, double max_motor_speed,
                             double motor_spin_min, double exponent);

private:
    void sendMessage(const mavlink_message_t& msg);

    BridgeParams params_;
    int          sock_fd_{-1};
};

}  // namespace simuav::comms
