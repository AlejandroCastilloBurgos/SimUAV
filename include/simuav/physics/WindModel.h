#pragma once
#include <Eigen/Dense>
#include <random>

namespace simuav::physics {

struct WindParams {
    Eigen::Vector3d mean_ned{Eigen::Vector3d::Zero()}; // m/s, steady component
    double turbulence_std{0.5};   // m/s, white-noise turbulence per axis
    double gust_probability{0.005}; // probability per step of a discrete gust
    double gust_magnitude{3.0};   // m/s, peak gust speed
};

// Produces a wind velocity vector in NED each simulation step.
// Models: constant mean + white-noise turbulence + random discrete gusts.
class WindModel {
public:
    explicit WindModel(WindParams params = {}, uint64_t seed = 0);

    // Call once per physics step. Returns wind velocity in NED (m/s).
    Eigen::Vector3d sample();

private:
    WindParams params_;
    Eigen::Vector3d current_gust_{Eigen::Vector3d::Zero()};
    std::mt19937_64 rng_;
    std::normal_distribution<double>  normal_{0.0, 1.0};
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};
};

}  // namespace simuav::physics
