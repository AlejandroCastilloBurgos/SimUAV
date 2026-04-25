#pragma once
#include "simuav/physics/QuadrotorModel.h"
#include "simuav/physics/WindModel.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/GPS.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Magnetometer.h"
#include "simuav/comms/MAVLinkBridge.h"
#include "simuav/logging/JSONLogger.h"
#include "simuav/logging/ULogLogger.h"

#include <atomic>
#include <string>

namespace simuav {

struct SimConfig {
    double      dt{0.004};                  // s, physics timestep (250 Hz)
    std::string mavlink_host{"127.0.0.1"};
    uint16_t    mavlink_port{14560};
    std::string json_log_path{"telemetry.json"};
    std::string ulog_path{"telemetry.ulg"};

    physics::QuadrotorParams quad_params{};
    physics::WindParams      wind_params{};
    sensors::IMUParams       imu_params{};
    sensors::GPSParams       gps_params{};
    sensors::BaroParams      baro_params{};
    sensors::MagParams       mag_params{};
};

// Top-level orchestrator. Owns all subsystems and drives the simulation loop.
//
// Loop order per step:
//   1. receiveActuators()  – pull latest motor commands from firmware (non-blocking)
//   2. integrate()         – advance physics by dt
//   3. sampleSensors()     – apply noise models
//   4. sendHilMessages()   – push sensor data to firmware
//   5. log()               – write JSON + uLog records
class Simulator {
public:
    explicit Simulator(SimConfig config);

    // Blocking: runs until stop() is called from another thread (or SIGINT).
    void run();
    void stop();

    const physics::State& state() const { return model_.state(); }

private:
    void step();

    SimConfig config_;

    physics::QuadrotorModel model_;
    physics::WindModel      wind_;
    sensors::IMU            imu_;
    sensors::GPS            gps_;
    sensors::Barometer      baro_;
    sensors::Magnetometer   mag_;
    comms::MAVLinkBridge    mavlink_;
    logging::JSONLogger     json_log_;
    logging::ULogLogger     ulog_;

    std::array<double, physics::kNumMotors> motor_speeds_{};
    std::atomic<bool> running_{false};
};

}  // namespace simuav
