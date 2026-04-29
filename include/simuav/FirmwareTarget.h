#pragma once
#include <cstdint>

namespace simuav {

enum class FirmwareTarget : uint8_t {
    PX4       = 0,
    ArduPilot = 1,
};

}  // namespace simuav
