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

**End-to-end test (PX4 SITL):**
```bash
# Requires PX4-Autopilot source and pymavlink
pip install pymavlink
export PX4_SRC=/path/to/PX4-Autopilot
python3 scripts/e2e_px4_sitl.py
```
Starts PX4 SITL + SimUAV, asserts GPS lock within 30 s and successful arm. Runs nightly on CI (`.github/workflows/e2e.yml`).

## Architecture

```
src/main.cpp
    └── Simulator           orchestrates all subsystems, owns the 250 Hz loop
          ├── physics/
          │   ├── QuadrotorModel   rigid-body dynamics (RK4 integration, NED/FRD frames,
          │   │                    first-order motor lag via motor_time_constant_s)
          │   └── WindModel        mean wind + Dryden turbulence + discrete gusts
          ├── sensors/
          │   ├── IMU              specific force + angular rate; Gauss-Markov bias model
          │   │                    + sinusoidal vibration injection on body-Z accel
          │   ├── GPS              geodetic position, throttled to update_rate_hz;
          │   │                    reports eph/epv/num_sats/fix_type
          │   ├── Barometer        ISA lapse-rate temperature + pressure → altitude
          │   └── Magnetometer     Earth field rotated to body frame
          ├── comms/
          │   └── MAVLinkBridge    non-blocking UDP socket; sends HIL_SENSOR + HIL_GPS,
          │                        receives HIL_ACTUATOR_CONTROLS / RC_CHANNELS_OVERRIDE
          ├── observability/
          │   └── StatusServer     UDP broadcast of JSON status snapshots at status_port
          └── logging/
              ├── JSONLogger       NDJSON, one line per step
              └── ULogLogger       PX4 uLog binary format (with uint64_t timestamp per message)
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
- **Inbound**: `HIL_ACTUATOR_CONTROLS` (normalized [0,1], PX4 and ArduPilot) or `RC_CHANNELS_OVERRIDE` (PWM [1000–2000 µs], ArduPilot SITL only). Motor channels 1–4 map to motor indices 0–3.
- Socket is **non-blocking** (`O_NONBLOCK`). If no packet arrives in a step, the previous motor command is reused.
- Default ports: simulator binds `mavlink_local_port` (default `14561`), sends to firmware at `mavlink_port` (default `14560`, PX4 SITL default). Both are configurable in the JSON config.
- Firmware target is selected via `"firmware_target": "px4"` or `"firmware_target": "ardupilot"` in the config file (default: `"px4"`). The `ardupilotmega` MAVLink dialect header (a strict superset of `common`) is always included.

### Sensor noise model

Each sensor class inherits `SensorBase` (a seeded `std::mt19937_64`). Sensors use:
- **White noise**: `std_dev × N(0,1)` added to each output per sample.
- **Gauss-Markov bias** (IMU only): each bias axis follows `b[k] = exp(-dt/τ)·b[k-1] + N(0, σ_b)` so the bias is bounded at steady state (stationary std ≈ σ_b/√(1−exp(−2dt/τ))). Configurable via `accel_bias_instability`, `accel_bias_instability_tc_s`, `gyro_bias_instability`, `gyro_bias_instability_tc_s`.
- **Vibration injection** (IMU only): sinusoidal signal added to body-Z accelerometer at `vibration_frequency_hz` with amplitude `vibration_amplitude_mps2`. Set to 0 to disable.
- Seeds are fixed constants (1–4) so simulation runs are **deterministic by default**. Pass a different seed to the constructor to vary.

### Logging

- **JSON** (`telemetry.json`): NDJSON, one JSON object per physics step. Fields: `t`, `pos[3]`, `vel[3]`, `att[4]` (w,x,y,z), `omega[3]`, `accel[3]`, `gyro[3]`, `baro_alt`, `gps_lat`, `gps_lon`.
- **uLog** (`telemetry.ulg`): PX4 binary format. Writes file header + `FLAG_BITS` + four `FORMAT` messages (`vehicle_local_position`, `vehicle_imu`, `vehicle_gps_position`, `vehicle_air_data`) + four `SUBSCRIPTION` messages (msg_id 0–3) + four `DATA` messages per step (one per subscribed topic, each with a `uint64_t timestamp` field in microseconds). Open with `pyulog` or Flight Review.
- **Replay** (`--replay <path>`): reads an NDJSON log, reconstructs `physics::State` from each entry, derives `accel_world` by finite-differencing consecutive velocity entries, and feeds the states through the sensor pipeline without connecting to firmware. See `include/simuav/LogReplayer.h`.

## Language rules

- C++17 strict: no C++20 features. Nested namespaces (`namespace a::b`) and structured bindings (`auto [x, y]`) are fine.
- `-Wall -Wextra -Wpedantic -Werror` are enforced by CMake — zero warnings allowed.
- All public headers live under `include/simuav/` and are included as `#include "simuav/..."`
- MAVLink headers are included as `#include "ardupilotmega/mavlink.h"` (the dialect subdirectory is on the include path). This is a strict superset of `common`; all common message IDs remain unchanged.

## Key files for new contributors

| File | Why read it first |
|------|------------------|
| `include/simuav/Simulator.h` | Defines `SimConfig` and the loop order |
| `include/simuav/physics/QuadrotorModel.h` | Motor layout, `State` struct, frame conventions, motor lag |
| `src/physics/QuadrotorModel.cpp` | Full force/torque derivation and RK4 integrator |
| `src/comms/MAVLinkBridge.cpp` | PWM→rad/s mapping; HIL message packing |
| `include/simuav/StatusServer.h` | UDP status broadcast; `StatusSnapshot` struct |
| `config/default.json` | All tuneable parameters with their default values |


## Core Engineering Principles

We build production-grade software.

**Priority order:**
1. Correctness
2. Maintainability
3. Simplicity
4. Security
5. Performance
6. Scalability

**Always avoid:** spaghetti code, overengineering, premature optimization, hidden complexity, fragile systems, unclear ownership.

**Prefer:** readable code, modular design, predictable behavior, strong naming, clean architecture, testability, small safe iterations.

**Apply pragmatically:** KISS, SOLID, DRY, YAGNI.

**Simulation-specific invariants:** determinism over raw speed, numerical stability over convenience, explicit behavior over implicit assumptions.

## Multi-Agent Workflow

Each agent has strictly isolated responsibilities. No agent performs another agent's role. Execution always follows the order below.

### Agent order

```
Architect → Developer → Reviewer → Tester → Security → Performance → Integration → Refactorer → Documentation → DevOps
```

### Per-task shortcuts

| Task type | Agents |
|-----------|--------|
| Feature | Architect → Developer → Reviewer → Tester → Security → Documentation → DevOps |
| Bug fix | Developer → Reviewer → Tester → Documentation |
| Technical debt | Reviewer → Refactorer → Tester → Documentation |
| CI/infra | DevOps → Tester |

### Agent responsibilities

**Architect** — design before implementation. Outputs: class responsibilities, data flow, module boundaries, design rationale. Does NOT write full implementations.

**Developer** — implement exactly what the Architect defined. C++17, RAII, `const`-correctness, `[[nodiscard]]` where applicable, clear ownership. No architectural decisions.

**Reviewer** — critically evaluate the implementation. Detects bugs, edge cases, memory issues, design violations, unnecessary complexity. Does NOT rewrite — analyzes and points out.

**Tester** — validate correctness via GoogleTest. Unit tests, edge cases, invalid inputs, regression tests. Simulation focus: numerical correctness, stability under extreme conditions, deterministic outputs.

**Security** — identify undefined behavior, input validation gaps, memory safety issues, overflow/underflow. Outputs risk assessment and hardening recommendations.

**Performance** — optimize real performance (not theoretical). Analyzes cache locality, data layout, hot paths, frame-time cost, hidden allocations. Simulation focus: deterministic timing, avoiding per-step heap allocation.

**Integration** — verify modules work correctly together. Detects interface mismatches, incorrect coupling, broken cross-module assumptions.

**Refactorer** — improve code quality without changing behavior: naming, structure, readability, duplication. No functional changes.

**Documentation** — produce documentation that reflects actual code: headers, design rationale, config reference. Not theory.

**DevOps** — CMake, CI/CD pipelines, reproducible builds, Linux parity. Ensures nightly e2e passes and caches are valid.