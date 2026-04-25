#pragma once
#include <cstdint>
#include <random>

namespace simuav::sensors {

// Shared RNG infrastructure for all sensor noise models.
// Each sensor gets its own seeded generator to keep noise streams independent.
class SensorBase {
protected:
    explicit SensorBase(uint64_t seed)
        : rng_(seed), normal_(0.0, 1.0) {}

    double white(double std_dev) const {
        return std_dev * normal_(rng_);
    }

    mutable std::mt19937_64 rng_;
    mutable std::normal_distribution<double> normal_;
};

}  // namespace simuav::sensors
