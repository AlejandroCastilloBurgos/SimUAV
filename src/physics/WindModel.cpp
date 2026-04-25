#include "simuav/physics/WindModel.h"

namespace simuav::physics {

WindModel::WindModel(WindParams params, uint64_t seed)
    : params_(std::move(params)), rng_(seed) {}

Eigen::Vector3d WindModel::sample() {
    // Turbulence: independent white noise per axis
    Eigen::Vector3d turbulence(
        params_.turbulence_std * normal_(rng_),
        params_.turbulence_std * normal_(rng_),
        params_.turbulence_std * normal_(rng_)
    );

    // Discrete gust: random direction, fixed magnitude, decays over ~20 steps
    if (uniform_(rng_) < params_.gust_probability) {
        Eigen::Vector3d dir(normal_(rng_), normal_(rng_), 0.0);
        if (dir.norm() > 1e-6) {
            current_gust_ = params_.gust_magnitude * dir.normalized();
        }
    }
    current_gust_ *= 0.95; // exponential decay

    return params_.mean_ned + turbulence + current_gust_;
}

}  // namespace simuav::physics
