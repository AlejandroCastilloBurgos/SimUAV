// simuav
#include "simuav/Simulator.h"
#include "simuav/ConfigLoader.h"
#include "simuav/LogReplayer.h"

// std
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <string>
#include <thread>

namespace {
simuav::Simulator* g_sim = nullptr;

void onSignal(int) {
    if (g_sim) g_sim->stop();
}
} // namespace

int main(int argc, char* argv[]) {
    std::string config_path;
    std::string replay_path;
    int         status_port_override = -1; // -1 = not set on command line

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--config") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--config requires a path argument\n");
                return EXIT_FAILURE;
            }
            config_path = argv[++i];
        } else if (arg == "--replay") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--replay requires a path argument\n");
                return EXIT_FAILURE;
            }
            replay_path = argv[++i];
        } else if (arg == "--status-port") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--status-port requires a port number\n");
                return EXIT_FAILURE;
            }
            status_port_override = std::atoi(argv[++i]);
        }
    }

    simuav::SimConfig cfg;
    if (!config_path.empty()) {
        try {
            cfg = simuav::loadConfig(config_path);
            std::printf("Config loaded from %s\n", config_path.c_str());
        } catch (const std::exception& e) {
            std::fprintf(stderr, "Error loading config '%s': %s\n",
                         config_path.c_str(), e.what());
            return EXIT_FAILURE;
        }
    } else {
        cfg = simuav::tryLoadConfig("config/default.json");
    }

    if (!replay_path.empty()) {
        std::vector<simuav::ReplayEntry> entries;
        try {
            entries = simuav::loadLog(replay_path);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "Error loading log '%s': %s\n",
                         replay_path.c_str(), e.what());
            return EXIT_FAILURE;
        }

        std::printf("Replay mode: %zu entries from %s\n",
                    entries.size(), replay_path.c_str());

        simuav::sensors::IMU          imu(cfg.imu_params);
        simuav::sensors::GPS          gps(cfg.gps_params);
        simuav::sensors::Barometer    baro(cfg.baro_params);
        simuav::sensors::Magnetometer mag(cfg.mag_params);

        for (std::size_t i = 0; i < entries.size(); ++i) {
            simuav::replayStep(entries[i], imu, gps, baro, mag);

            if (i + 1 < entries.size()) {
                const double delta_s = entries[i + 1].state.time - entries[i].state.time;
                if (delta_s > 0.0) {
                    std::this_thread::sleep_for(
                        std::chrono::duration<double>(delta_s));
                }
            }
        }

        std::printf("Replay complete.\n");
        return EXIT_SUCCESS;
    }

    if (status_port_override >= 0)
        cfg.status_port = static_cast<uint16_t>(status_port_override);

    simuav::Simulator sim(cfg);
    g_sim = &sim;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    std::printf("SimUAV v0.1 — headless SIL quadrotor simulator\n");
    std::printf("Connecting to firmware at %s:%u\n",
                cfg.mavlink_host.c_str(), cfg.mavlink_port);

    sim.run();

    const simuav::LoopStats& st = sim.stats();
    std::printf("Simulation ended. Logs written.\n");
    std::printf("Loop summary: %llu steps, %llu overruns (%.2f%%), max step %.0f µs\n",
                static_cast<unsigned long long>(st.step_count),
                static_cast<unsigned long long>(st.overrun_count),
                st.step_count > 0
                    ? 100.0 * static_cast<double>(st.overrun_count) /
                          static_cast<double>(st.step_count)
                    : 0.0,
                st.max_step_us);
    return EXIT_SUCCESS;
}
