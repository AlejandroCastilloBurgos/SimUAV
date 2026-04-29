#include "simuav/Simulator.h"

#include <chrono>
#include <thread>
#include <cstdio>

namespace simuav {

Simulator::Simulator(SimConfig config)
    : config_(std::move(config))
    , model_(config_.quad_params)
    , wind_(config_.wind_params, config_.dt)
    , imu_(config_.imu_params)
    , gps_(config_.gps_params)
    , baro_(config_.baro_params)
    , mag_(config_.mag_params)
    , mavlink_([this] {
        comms::BridgeParams bp;
        bp.remote_host     = config_.mavlink_host;
        bp.remote_port     = config_.mavlink_port;
        bp.max_motor_speed = config_.quad_params.max_motor_speed;
        bp.esc_exponent    = config_.quad_params.esc_exponent;
        bp.motor_spin_min  = config_.quad_params.motor_spin_min;
        bp.firmware_target = config_.firmware_target;
        return bp;
    }())
    , json_log_(config_.json_log_path)
    , ulog_(config_.ulog_path)
{}

void Simulator::run() {
    if (!mavlink_.open()) {
        std::fprintf(stderr, "Simulator: failed to open MAVLink bridge\n");
        return;
    }
    if (!json_log_.isOpen()) {
        std::fprintf(stderr, "Simulator: failed to open JSON log\n");
    }
    if (!ulog_.isOpen()) {
        std::fprintf(stderr, "Simulator: failed to open uLog file\n");
    }

    running_ = true;
    std::printf("SimUAV running at %.0f Hz. Ctrl-C to stop.\n",
                1.0 / config_.dt);

    using Clock     = std::chrono::steady_clock;
    using Duration  = std::chrono::duration<double>;
    const Duration step_dur{config_.dt};
    auto next_wake  = Clock::now() + step_dur;

    while (running_) {
        step();
        std::this_thread::sleep_until(next_wake);
        next_wake += step_dur;
    }

    mavlink_.close();
}

void Simulator::stop() {
    running_ = false;
}

void Simulator::step() {
    // 1. Pull actuator commands (non-blocking; keep last if none arrives)
    mavlink_.receiveActuators(motor_speeds_);

    // 2. Wind disturbance
    const Eigen::Vector3d wind_ned = wind_.sample();

    // 3. Physics integration
    model_.integrate(motor_speeds_, config_.dt, wind_ned);
    const physics::State& s = model_.state();

    // 4. World-frame linear acceleration produced by the last physics step.
    const Eigen::Vector3d accel_world = model_.lastAccelWorld();

    // 5. Sample sensors
    const sensors::IMUSample  imu_s  = imu_.sample(s, accel_world);
    const sensors::BaroSample baro_s = baro_.sample(s);
    const sensors::MagSample  mag_s  = mag_.sample(s);

    sensors::GPSSample gps_s{};
    const bool new_gps = gps_.sample(s, gps_s);

    // 6. Send HIL messages
    mavlink_.sendHilSensor(imu_s, baro_s, mag_s);
    if (new_gps) mavlink_.sendHilGps(gps_s);

    // 7. Log
    json_log_.log(s, imu_s, baro_s, gps_s);
    ulog_.log(s, imu_s, baro_s);
}

}  // namespace simuav
