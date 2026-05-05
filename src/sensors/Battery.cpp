#include "simuav/sensors/Battery.h"
#include <algorithm>

namespace simuav::sensors {

Battery::Battery(BatteryParams params, uint64_t seed)
    : SensorBase(seed), params_(std::move(params)) {}

BatterySample Battery::sample(
    const std::array<double, physics::kNumMotors>& motor_speeds,
    const physics::QuadrotorParams& quad,
    double dt)
{
    elapsed_ += dt;

    // Mechanical power through drag torque: P_i = k_drag × ωᵢ³  [W]
    double p_total = 0.0;
    for (const double w : motor_speeds)
        p_total += quad.k_drag * w * w * w;

    // Open-circuit voltage using current SoC (linear interpolation)
    const auto ocv = [&](double soc) {
        return (params_.voltage_empty_v +
                soc * (params_.voltage_full_v - params_.voltage_empty_v))
               * params_.cell_count;
    };

    const double v_oc    = ocv(remaining_);
    const double current = (v_oc > 0.0) ? p_total / v_oc : 0.0;

    // Coulomb counting — deplete SoC; clamp to [0, 1]
    remaining_ -= current * dt / (params_.capacity_mah * 3.6);
    remaining_  = std::max(0.0, std::min(1.0, remaining_));

    // Terminal voltage under load (recompute OCV with updated SoC)
    const double voltage = std::max(0.0, ocv(remaining_) - current * params_.resistance_ohm);

    return {elapsed_, voltage, current, remaining_};
}

}  // namespace simuav::sensors
