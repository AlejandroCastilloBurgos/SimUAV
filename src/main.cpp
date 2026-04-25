#include "simuav/Simulator.h"

#include <csignal>
#include <cstdlib>
#include <cstdio>

namespace {
simuav::Simulator* g_sim = nullptr;

void onSignal(int) {
    if (g_sim) g_sim->stop();
}
}

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;

    simuav::SimConfig cfg;
    // TODO: parse cfg from argv or config/default.json

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
