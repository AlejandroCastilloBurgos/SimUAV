#pragma once
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace simuav {

struct ScenarioEvent {
    double          time_s{0.0};
    std::string     type{};
    nlohmann::json  params{};
};

// Parses a JSON array of scenario events from `path`.
// Returns events sorted ascending by time_s.
// Throws std::runtime_error if the file cannot be opened or is malformed.
[[nodiscard]] std::vector<ScenarioEvent> loadScenario(const std::string& path);

} // namespace simuav
