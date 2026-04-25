# SimUAV Changelog

All notable changes to this project will be documented here.
Format: `[version/date] — description (branch, PR#)`

---

## [2026-04-25] — Initial commit

**Branch:** `main` (first commit `aeda642`)

### What was implemented

#### Physics (`src/physics/`)
- `QuadrotorModel`: full rigid-body dynamics — thrust, gravity, aerodynamic drag, reactive yaw torques. NED/FRD frame convention. Semi-implicit Euler integration.
- `WindModel`: mean wind vector + white-noise turbulence + random gust injection.

#### Sensors (`src/sensors/`)
- `IMU`: specific force + angular rate with white noise and bias random walk. Deterministic seeding.
- `GPS`: NED → geodetic conversion (flat-Earth approximation), rate-throttled to 5 Hz.
- `Barometer`: ISA pressure formula from altitude.
- `Magnetometer`: Earth magnetic field vector rotated into body frame.

#### Communications (`src/comms/`)
- `MAVLinkBridge`: non-blocking UDP socket. Sends `HIL_SENSOR` at 250 Hz and `HIL_GPS` at 5 Hz. Receives `HIL_ACTUATOR_CONTROLS` and converts PWM → motor rad/s (linear approximation).

#### Logging (`src/logging/`)
- `JSONLogger`: NDJSON output — one line per simulation step with full state + sensor readings.
- `ULogLogger`: valid PX4 binary `.ulog` format with file header, FLAG_BITS message, FORMAT messages, and DATA messages.

#### Orchestration (`src/`)
- `Simulator`: owns the 250 Hz main loop, wires all subsystems, handles SIGINT/SIGTERM gracefully.
- `main.cpp`: entry point, constructs and runs Simulator.

#### Tests (`tests/`)
- 9 GoogleTest cases covering: hover stability, free-fall trajectory, quaternion normalization, sensor noise bounds, GPS rate throttling, barometric pressure accuracy, magnetometer field at level attitude.

### Known stubs (tracked as issues)

| Issue | Stub |
|-------|------|
| #1 | `Simulator::step()` passes `accel_world = {0,0,0}` to IMU — real acceleration not yet piped from physics |
| #2 | `main.cpp` has `// TODO: parse cfg from argv or config/default.json` |
| #3 | PWM → rad/s is a linear approximation, no real ESC curve |

---

<!-- New entries go above this line, newest first -->
