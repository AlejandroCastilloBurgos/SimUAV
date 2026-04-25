# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

SimUAV is a headless Software-in-the-Loop (SIL) quadrotor simulator written in C++17. It replaces physical hardware in CI/CD pipelines for drone firmware testing. It connects to real PX4 or ArduPilot firmware over UDP, feeds it simulated sensor data, and receives motor commands back — creating a closed-loop hardware-in-the-loop session without any physical drone.

## Build

**Prerequisites (WSL Ubuntu):**
```bash
sudo apt install cmake libeigen3-dev build-essential
# MAVLink C headers (one-time):
git clone https://github.com/mavlink/c_library_v2.git third_party/mavlink-c-library
```

**Configure and build:**
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

**Release build:**
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

**Run the simulator:**
```bash
./build/simuav
```

## Tests

```bash
cmake --build build --target simuav_tests
ctest --test-dir build --output-on-failure
# Run a single test binary directly (faster, no ctest overhead):
./build/tests/simuav_tests --gtest_filter="QuadrotorModel.*"
```

GoogleTest is fetched automatically by CMake via `FetchContent` the first time — requires internet.

## Architecture

```
src/main.cpp
    └── Simulator           orchestrates all subsystems, owns the 250 Hz loop
          ├── physics/
          │   ├── QuadrotorModel   rigid-body dynamics (Euler integration, NED/FRD frames)
          │   └── WindModel        mean wind + turbulence + discrete gusts
          ├── sensors/
          │   ├── IMU              specific force + angular rate, bias random walk
          │   ├── GPS              geodetic position, throttled to update_rate_hz
          │   ├── Barometer        ISA pressure → altitude
          │   └── Magnetometer     Earth field rotated to body frame
          ├── comms/
          │   └── MAVLinkBridge    non-blocking UDP socket; sends HIL_SENSOR + HIL_GPS,
          │                        receives HIL_ACTUATOR_CONTROLS
          └── logging/
              ├── JSONLogger       NDJSON, one line per step
              └── ULogLogger       PX4 uLog binary format
```

### Frame conventions

| Frame | Axes | Used for |
|-------|------|----------|
| World | NED — North/East/Down, Z positive downward | position, velocity, gravity |
| Body  | FRD — Forward/Right/Down | forces, torques, sensor outputs |
| Quaternion | body → NED rotation | `State::attitude` |

Gravity in NED: `[0, 0, +9.80665]` m/s².
Thrust in body: `[0, 0, −F_total]` (upward = negative body-Z).

### Motor layout and mixing (X-frame, top view)

```
  FL(0) ──── FR(1)     FL=CW,  FR=CCW
     \          /       RL=CCW, RR=CW   (viewed from above)
  RL(2) ──── RR(3)
```

| Output | Formula |
|--------|---------|
| Total thrust | `k_thrust × (ω₀² + ω₁² + ω₂² + ω₃²)` |
| Roll  (+right)    | `arm × k_thrust × (ω₀² + ω₂² − ω₁² − ω₃²)` |
| Pitch (+nose-up)  | `arm × k_thrust × (ω₀² + ω₁² − ω₂² − ω₃²)` |
| Yaw   (+right)    | `k_drag        × (ω₀² − ω₁² − ω₂² + ω₃²)` |

### MAVLink bridge

- **Outbound**: `HIL_SENSOR` every physics step (250 Hz); `HIL_GPS` at `gps_params.update_rate_hz` (default 5 Hz).
- **Inbound**: `HIL_ACTUATOR_CONTROLS` — normalized `[0, 1]` per motor, linearly mapped to `[0, max_motor_speed]` rad/s.
- Socket is **non-blocking** (`O_NONBLOCK`). If no packet arrives in a step, the previous motor command is reused.
- Default ports: simulator binds `14561`, sends to firmware at `14560` (PX4 SITL default).

### Sensor noise model

Each sensor class inherits `SensorBase` (a seeded `std::mt19937_64`). Sensors use:
- **White noise**: `std_dev × N(0,1)` added to each output per sample.
- **Bias random walk** (IMU only): bias vector incremented each step by `N(0, bias_std)`.
- Seeds are fixed constants (1–4) so simulation runs are **deterministic by default**. Pass a different seed to the constructor to vary.

### Logging

- **JSON** (`telemetry.json`): NDJSON, one JSON object per physics step. Fields: `t`, `pos[3]`, `vel[3]`, `att[4]` (w,x,y,z), `omega[3]`, `accel[3]`, `gyro[3]`, `baro_alt`, `gps_lat`, `gps_lon`.
- **uLog** (`telemetry.ulg`): PX4 binary format. Currently writes file header + `FLAG_BITS` + one `FORMAT` message (`vehicle_local_position`) + one `DATA` message per step. Open with `pyulog` or Flight Review.

## Language rules

- C++17 strict: no C++20 features. Nested namespaces (`namespace a::b`) and structured bindings (`auto [x, y]`) are fine.
- `-Wall -Wextra -Wpedantic -Werror` are enforced by CMake — zero warnings allowed.
- All public headers live under `include/simuav/` and are included as `#include "simuav/..."`
- MAVLink headers are included as `#include "common/mavlink.h"` (the dialect subdirectory is on the include path).

## Key files for new contributors

| File | Why read it first |
|------|------------------|
| `include/simuav/Simulator.h` | Defines `SimConfig` and the loop order |
| `include/simuav/physics/QuadrotorModel.h` | Motor layout, `State` struct, frame conventions |
| `src/physics/QuadrotorModel.cpp` | Full force/torque derivation |
| `src/comms/MAVLinkBridge.cpp` | PWM→rad/s mapping; HIL message packing |
| `config/default.json` | All tuneable parameters with their default values |
