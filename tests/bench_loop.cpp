#include "simuav/physics/QuadrotorModel.h"
#include "simuav/sensors/IMU.h"
#include "simuav/sensors/Barometer.h"
#include "simuav/sensors/Magnetometer.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

int main() {
    constexpr int    kSteps     = 10000;
    constexpr double kDt        = 0.004;
    constexpr double kThreshold = 3000.0; // µs — 75% of 4 ms budget

    simuav::physics::QuadrotorModel model;
    simuav::sensors::IMU            imu;
    simuav::sensors::Barometer      baro;
    simuav::sensors::Magnetometer   mag;

    // Hover at half throttle: each motor at ~593 rad/s (sqrt(0.5) * 838)
    std::array<double, simuav::physics::kNumMotors> motors{};
    motors.fill(593.0);

    const Eigen::Vector3d wind_zero = Eigen::Vector3d::Zero();

    std::vector<double> step_times;
    step_times.reserve(kSteps);

    using Clock   = std::chrono::steady_clock;
    using FpMicro = std::chrono::duration<double, std::micro>;

    for (int i = 0; i < kSteps; ++i) {
        const auto t0 = Clock::now();

        model.integrate(motors, kDt, wind_zero);
        const simuav::physics::State& s = model.state();
        const Eigen::Vector3d accel     = model.lastAccelWorld();

        imu.sample(s, accel);
        baro.sample(s);
        mag.sample(s);

        const auto t1 = Clock::now();
        step_times.push_back(FpMicro(t1 - t0).count());
    }

    std::sort(step_times.begin(), step_times.end());
    const double median_us = step_times[kSteps / 2];

    std::printf("bench_loop: %d steps, median=%.2f µs, p95=%.2f µs, max=%.2f µs\n",
                kSteps,
                median_us,
                step_times[static_cast<std::size_t>(kSteps * 0.95)],
                step_times.back());

    if (median_us > kThreshold) {
        std::fprintf(stderr,
            "FAIL: median step time %.2f µs exceeds threshold %.0f µs\n",
            median_us, kThreshold);
        return EXIT_FAILURE;
    }

    std::printf("PASS: median %.2f µs < %.0f µs threshold\n", median_us, kThreshold);
    return EXIT_SUCCESS;
}
