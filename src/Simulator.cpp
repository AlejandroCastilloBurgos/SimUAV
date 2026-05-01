#include "simuav/Simulator.h"

#include <algorithm>
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
    , status_server_(config_.status_port)
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
    using FpSeconds = std::chrono::duration<double>;
    using FpMicros  = std::chrono::duration<double, std::micro>;
    const FpSeconds step_dur{config_.dt};
    auto next_wake  = Clock::now() + step_dur;

    // Overrun warning throttle: emit at most once per second.
    auto last_warn = Clock::now();

    while (running_) {
        const auto t0 = Clock::now();
        step();
        const auto t1 = Clock::now();

        const double elapsed_us = FpMicros(t1 - t0).count();
        ++stats_.step_count;
        stats_.max_step_us = std::max(stats_.max_step_us, elapsed_us);

        if (elapsed_us > config_.dt * 1e6) {
            ++stats_.overrun_count;
            // Warn at most once per second when overrun ratio exceeds 1%.
            if (stats_.step_count >= 100 &&
                static_cast<double>(stats_.overrun_count) /
                    static_cast<double>(stats_.step_count) > 0.01 &&
                FpSeconds(t1 - last_warn).count() >= 1.0) {
                std::fprintf(stderr,
                    "SimUAV: loop overruns %.1f%% (%llu / %llu steps), "
                    "max step %.0f µs\n",
                    100.0 * static_cast<double>(stats_.overrun_count) /
                        static_cast<double>(stats_.step_count),
                    static_cast<unsigned long long>(stats_.overrun_count),
                    static_cast<unsigned long long>(stats_.step_count),
                    stats_.max_step_us);
                last_warn = t1;
            }
        }

        // Publish status at ~1 Hz (every 250 steps)
        if (stats_.step_count % 250 == 0 && config_.status_port != 0) {
            StatusSnapshot snap;
            snap.sim_time          = model_.state().time;
            snap.step_count        = stats_.step_count;
            snap.overrun_count     = stats_.overrun_count;
            snap.hil_sensor_sent   = hil_sensor_sent_;
            snap.actuator_received = actuator_received_;
            snap.baro_alt_m        = last_baro_alt_m_;
            status_server_.publish(snap);
        }

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
    const std::array<double, physics::kNumMotors> prev = motor_speeds_;
    mavlink_.receiveActuators(motor_speeds_);
    if (motor_speeds_ != prev) ++actuator_received_;

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

    last_baro_alt_m_ = baro_s.altitude_m;

    // 6. Send HIL messages
    mavlink_.sendHilSensor(imu_s, baro_s, mag_s);
    ++hil_sensor_sent_;
    if (new_gps) mavlink_.sendHilGps(gps_s);

    // 7. Log
    json_log_.log(s, imu_s, baro_s, gps_s);
    ulog_.log(s, imu_s, baro_s, gps_s);
}

}  // namespace simuav
