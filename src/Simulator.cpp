#include "simuav/Simulator.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdio>

namespace simuav {

// When rng_seed is non-zero, derive each subsystem seed from it to make the
// entire run reproducible with a single user-facing value.
static uint64_t deriveSeed(uint64_t base, uint64_t offset) {
    return base == 0 ? offset : base + offset;
}

Simulator::Simulator(SimConfig config)
    : config_(std::move(config))
    , model_(config_.quad_params)
    , wind_(config_.wind_params, config_.dt, deriveSeed(config_.rng_seed, 0))
    , imu_(config_.imu_params,    deriveSeed(config_.rng_seed, 1))
    , gps_(config_.gps_params,    deriveSeed(config_.rng_seed, 2))
    , baro_(config_.baro_params,  deriveSeed(config_.rng_seed, 3))
    , mag_(config_.mag_params,    deriveSeed(config_.rng_seed, 4))
    , battery_(config_.battery_params, deriveSeed(config_.rng_seed, 5))
    , mavlink_([this] {
        comms::BridgeParams bp;
        bp.remote_host     = config_.mavlink_host;
        bp.remote_port     = config_.mavlink_port;
        bp.local_port      = config_.mavlink_local_port;
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

void Simulator::loadScenario(const std::string& path) {
    scenario_events_ = simuav::loadScenario(path);
    scenario_idx_    = 0;
}

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

        // Heartbeat + status at ~1 Hz (every 250 steps at 250 Hz)
        if (stats_.step_count % 250 == 0) {
            mavlink_.sendHeartbeat();
        }
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

        if (config_.run_duration_s > 0.0 && model_.state().time >= config_.run_duration_s) {
            running_ = false;
            break;
        }

        std::this_thread::sleep_until(next_wake);
        next_wake += step_dur;
    }

    mavlink_.close();
}

void Simulator::stop() {
    running_ = false;
}

void Simulator::dispatchScenarioEvents() {
    const double sim_time = model_.state().time;
    while (scenario_idx_ < scenario_events_.size() &&
           scenario_events_[scenario_idx_].time_s <= sim_time) {
        const ScenarioEvent& ev = scenario_events_[scenario_idx_];
        if (ev.type == "wind_mean") {
            const auto& ned = ev.params.at("ned");
            wind_.setMeanNed({ned[0].get<double>(),
                              ned[1].get<double>(),
                              ned[2].get<double>()});
        } else if (ev.type == "motor_lock") {
            const int m = ev.params.at("motor").get<int>();
            if (m >= 0 && m < static_cast<int>(physics::kNumMotors))
                locked_motors_[static_cast<std::size_t>(m)] = true;
        } else if (ev.type == "imu_bias_inject") {
            const auto& a = ev.params.at("accel");
            const auto& g = ev.params.at("gyro");
            imu_.injectBias(
                {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()},
                {g[0].get<double>(), g[1].get<double>(), g[2].get<double>()}
            );
        } else if (ev.type == "stop") {
            stop();
        } else if (ev.type == "gps_fix_type") {
            const uint8_t ft   = static_cast<uint8_t>(ev.params.at("fix_type").get<int>());
            const uint8_t sats = static_cast<uint8_t>(ev.params.at("num_sats").get<int>());
            gps_.setFixType(ft, sats);
        } else if (ev.type == "gps_noise_override") {
            gps_.setPosNoiseStd(
                ev.params.at("pos_std_m").get<double>(),
                ev.params.at("alt_std_m").get<double>()
            );
        } else if (ev.type == "baro_noise_override") {
            baro_.setNoiseStd(ev.params.at("noise_std_m").get<double>());
        }
        ++scenario_idx_;
    }
}

void Simulator::step() {
    // 1. Pull actuator commands (non-blocking; keep last if none arrives)
    const std::array<double, physics::kNumMotors> prev = motor_speeds_;
    mavlink_.receiveActuators(motor_speeds_);
    if (motor_speeds_ != prev) ++actuator_received_;

    // 2. Dispatch any scenario events due at this simulation time
    dispatchScenarioEvents();

    // 3. Apply locked motors before physics integration
    for (std::size_t i = 0; i < physics::kNumMotors; ++i) {
        if (locked_motors_[i]) motor_speeds_[i] = 0.0;
    }

    // 4. Wind disturbance
    const Eigen::Vector3d wind_ned = wind_.sample();

    // 5. Physics integration
    model_.integrate(motor_speeds_, config_.dt, wind_ned);
    const physics::State& s = model_.state();

    // 6. World-frame linear acceleration produced by the last physics step.
    const Eigen::Vector3d accel_world = model_.lastAccelWorld();

    // 7. Sample sensors
    const sensors::IMUSample  imu_s  = imu_.sample(s, accel_world);
    const sensors::BaroSample baro_s = baro_.sample(s);
    const sensors::MagSample  mag_s  = mag_.sample(s);

    sensors::GPSSample gps_s{};
    const bool new_gps = gps_.sample(s, gps_s);

    last_baro_alt_m_ = baro_s.altitude_m;

    // 8. Sample battery and send HIL messages
    const sensors::BatterySample bat_s =
        battery_.sample(motor_speeds_, config_.quad_params, config_.dt);

    mavlink_.sendHilSensor(imu_s, baro_s, mag_s);
    mavlink_.sendBatteryStatus(bat_s);
    ++hil_sensor_sent_;
    if (new_gps) mavlink_.sendHilGps(gps_s);

    // 9. Log
    json_log_.log(s, imu_s, baro_s, gps_s, model_.motorSpeedsActual(), bat_s, wind_ned);
    ulog_.log(s, imu_s, baro_s, gps_s, model_.motorSpeedsActual(), bat_s);
}

}  // namespace simuav
