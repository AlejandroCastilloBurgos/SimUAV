#include "simuav/LogReplayer.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

namespace simuav {

std::vector<ReplayEntry> loadLog(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("loadLog: cannot open '" + path + "'");

    std::vector<ReplayEntry> entries;
    std::string line;

    while (std::getline(f, line)) {
        if (line.empty()) continue;

        const nlohmann::json j = nlohmann::json::parse(line);

        ReplayEntry e;
        e.state.time = j.at("t").get<double>();

        const auto& pos = j.at("pos");
        e.state.position = {pos[0].get<double>(), pos[1].get<double>(), pos[2].get<double>()};

        const auto& vel = j.at("vel");
        e.state.velocity = {vel[0].get<double>(), vel[1].get<double>(), vel[2].get<double>()};

        const auto& att = j.at("att");
        e.state.attitude = Eigen::Quaterniond(
            att[0].get<double>(), att[1].get<double>(),
            att[2].get<double>(), att[3].get<double>());

        const auto& omega = j.at("omega");
        e.state.angular_velocity = {
            omega[0].get<double>(), omega[1].get<double>(), omega[2].get<double>()};

        entries.push_back(std::move(e));
    }

    // Compute accel_world via finite difference of consecutive velocity entries.
    // Entry 0 keeps the default Zero() — no predecessor available.
    for (std::size_t i = 1; i < entries.size(); ++i) {
        const double dt = entries[i].state.time - entries[i - 1].state.time;
        if (dt > 0.0)
            entries[i].accel_world =
                (entries[i].state.velocity - entries[i - 1].state.velocity) / dt;
    }

    return entries;
}

ReplaySamples replayStep(const ReplayEntry&      entry,
                         sensors::IMU&           imu,
                         sensors::GPS&           gps,
                         sensors::Barometer&     baro,
                         sensors::Magnetometer&  mag)
{
    ReplaySamples out;
    out.imu       = imu.sample(entry.state, entry.accel_world);
    out.baro      = baro.sample(entry.state);
    out.mag       = mag.sample(entry.state);
    out.gps_valid = gps.sample(entry.state, out.gps);
    return out;
}

}  // namespace simuav
