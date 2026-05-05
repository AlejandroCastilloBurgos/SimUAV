#pragma once
#include "SensorBase.h"
#include "simuav/physics/QuadrotorModel.h"
#include <array>

namespace simuav::sensors {

struct BatteryParams {
    int    cell_count{4};
    double capacity_mah{5000.0};   // mAh
    double resistance_ohm{0.05};   // Ω — total pack internal resistance
    double voltage_full_v{4.2};    // V per cell at 100% SoC
    double voltage_empty_v{3.5};   // V per cell at 0% SoC
};

struct BatterySample {
    double timestamp{0.0};
    double voltage_v{0.0};   // terminal voltage (OCV − IR drop)
    double current_a{0.0};   // instantaneous draw
    double remaining{1.0};   // state of charge [0, 1]
};

// First-order battery model: Coulomb counting + internal-resistance voltage sag.
// Power draw is estimated from motor drag torque: P = Σ k_drag × ωᵢ³ (W).
// Terminal voltage: V_oc − I × R, where V_oc = cell_count × lerp(empty, full, SoC).
class Battery : public SensorBase {
public:
    explicit Battery(BatteryParams params = {}, uint64_t seed = 5);

    BatterySample sample(const std::array<double, physics::kNumMotors>& motor_speeds,
                         const physics::QuadrotorParams& quad,
                         double dt);

    double stateOfCharge() const { return remaining_; }

private:
    BatteryParams params_;
    double        remaining_{1.0};
    double        elapsed_{0.0};
};

}  // namespace simuav::sensors
