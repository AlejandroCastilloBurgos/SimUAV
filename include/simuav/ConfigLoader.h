#pragma once

// simuav
#include "simuav/Simulator.h"

// third-party
#include <nlohmann/json.hpp>

// std
#include <fstream>
#include <string>

namespace simuav {

// Loads a SimConfig from a JSON file.
// Missing keys silently fall back to SimConfig defaults — no exception is thrown
// for partial configs. Throws std::runtime_error only if the file cannot be opened.
inline SimConfig loadConfig(const std::string& path);

// Attempts to load config from path. Returns default SimConfig{} on any error.
inline SimConfig tryLoadConfig(const std::string& path);

} // namespace simuav

#include "simuav/ConfigLoader.inl"
