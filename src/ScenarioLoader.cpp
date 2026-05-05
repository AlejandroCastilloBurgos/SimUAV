#include "simuav/ScenarioLoader.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace simuav {

std::vector<ScenarioEvent> loadScenario(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("ScenarioLoader: cannot open '" + path + "'");

    const nlohmann::json j = nlohmann::json::parse(f);
    if (!j.is_array())
        throw std::runtime_error("ScenarioLoader: root element must be a JSON array");

    std::vector<ScenarioEvent> events;
    events.reserve(j.size());

    for (const auto& item : j) {
        ScenarioEvent ev;
        ev.time_s = item.at("time_s").get<double>();
        ev.type   = item.at("type").get<std::string>();
        ev.params = item.value("params", nlohmann::json::object());
        events.push_back(std::move(ev));
    }

    std::sort(events.begin(), events.end(),
              [](const ScenarioEvent& a, const ScenarioEvent& b) {
                  return a.time_s < b.time_s;
              });

    return events;
}

} // namespace simuav
