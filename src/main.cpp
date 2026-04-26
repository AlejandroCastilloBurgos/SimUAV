// simuav
#include "simuav/Simulator.h"
#include "simuav/ConfigLoader.h"

// std
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <string>

namespace {
simuav::Simulator* g_sim = nullptr;

void onSignal(int) {
    if (g_sim) g_sim->stop();
}
} // namespace

int main(int argc, char* argv[]) {
    // Resolve config path: --config <path> > config/default.json > hardcoded defaults
    std::string config_path;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--config") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "--config requires a path argument\n");
                return EXIT_FAILURE;
            }
            config_path = argv[i + 1];
            break;
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

    simuav::Simulator sim(cfg);
    g_sim = &sim;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    std::printf("SimUAV v0.1 — headless SIL quadrotor simulator\n");
    std::printf("Connecting to firmware at %s:%u\n",
                cfg.mavlink_host.c_str(), cfg.mavlink_port);

    sim.run();

    std::printf("Simulation ended. Logs written.\n");
    return EXIT_SUCCESS;
}
