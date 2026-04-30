#!/usr/bin/env python3
"""
End-to-end integration test: SimUAV + ArduPilot SITL.

Starts ArduPilot Copter SITL (headless) and SimUAV in HIL mode with the
ardupilot firmware target, then uses pymavlink to assert that the vehicle
acquires a GPS lock and arms successfully.

Prerequisites:
  pip install pymavlink
  export ARDUPILOT_SRC=/path/to/ardupilot   (or pass --ardupilot-src)
  cmake --build build (SimUAV binary must exist at ./build/simuav)

ArduPilot SITL port mapping (default):
  14550 — GCS UDP port (pymavlink connects here)
  9002  — HIL sensor input (SimUAV sends HIL_SENSOR here)
  9003  — HIL actuator output (SimUAV receives RC_CHANNELS_OVERRIDE here)

Exit codes:
  0  all assertions passed
  1  assertion failed or timeout
  2  prerequisite missing (binary not found, etc.)

Usage:
  python3 scripts/e2e_ardupilot_sitl.py [--ardupilot-src <path>]
                                         [--simuav <path>]
                                         [--gps-timeout <s>]
                                         [--arm-timeout <s>]
                                         [--dry-run]
"""

import argparse
import os
import signal
import subprocess
import sys
import time

GPS_FIX_TIMEOUT_DEFAULT  = 60   # seconds — ArduPilot pre-arm checks take longer
ARM_TIMEOUT_DEFAULT      = 90   # seconds
GCS_PORT                 = 14550
HEARTBEAT_TIMEOUT        = 15   # seconds to wait for first heartbeat

# MAVLink command IDs (common dialect)
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_RESULT_ACCEPTED          = 0

# Path to the SimUAV config for ArduPilot SITL
_SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT   = os.path.dirname(_SCRIPT_DIR)
ARDUPILOT_CONFIG = os.path.join(_REPO_ROOT, "config", "ardupilot_sitl.json")


def _die(msg: str, code: int = 1) -> None:
    print(f"[e2e] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def _info(msg: str) -> None:
    print(f"[e2e] {msg}", flush=True)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ardupilot-src", default=os.environ.get("ARDUPILOT_SRC", ""),
                   help="Path to ardupilot source tree (default: $ARDUPILOT_SRC)")
    p.add_argument("--simuav",        default="./build/simuav",
                   help="Path to SimUAV binary (default: ./build/simuav)")
    p.add_argument("--gps-timeout",   type=int, default=GPS_FIX_TIMEOUT_DEFAULT,
                   help=f"Seconds to wait for GPS fix (default: {GPS_FIX_TIMEOUT_DEFAULT})")
    p.add_argument("--arm-timeout",   type=int, default=ARM_TIMEOUT_DEFAULT,
                   help=f"Seconds to wait for arming ack (default: {ARM_TIMEOUT_DEFAULT})")
    p.add_argument("--dry-run",       action="store_true",
                   help="Validate arguments and print commands without launching processes")
    return p.parse_args()


def _terminate(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def wait_for_gps_fix(mav, timeout: int) -> bool:
    """Return True when GPS_RAW_INT reports fix_type >= 3 within timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
        if msg and msg.fix_type >= 3:
            return True
    return False


def send_arm(mav) -> None:
    """Send MAV_CMD_COMPONENT_ARM_DISARM (arm=1)."""
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,   # confirmation
        1,   # param1: 1 = arm
        0, 0, 0, 0, 0, 0,
    )


def wait_for_arm_ack(mav, timeout: int) -> bool:
    """Return True when COMMAND_ACK for arm command arrives with result ACCEPTED."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
        if msg and msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
            return msg.result == MAV_RESULT_ACCEPTED
    return False


def main() -> int:
    args = parse_args()

    # ── Prerequisite checks ───────────────────────────────────────────────────

    if not args.ardupilot_src:
        _die("ArduPilot source path not set — pass --ardupilot-src or "
             "export ARDUPILOT_SRC=<path>", code=2)

    if not os.path.isdir(args.ardupilot_src):
        _die(f"ArduPilot source path does not exist: {args.ardupilot_src}", code=2)

    simuav_bin = os.path.abspath(args.simuav)
    if not os.path.isfile(simuav_bin):
        _die(f"SimUAV binary not found: {simuav_bin}\n"
             "Build with: cmake --build build -j$(nproc)", code=2)

    if not os.path.isfile(ARDUPILOT_CONFIG):
        _die(f"SimUAV ArduPilot config not found: {ARDUPILOT_CONFIG}", code=2)

    # ArduPilot SITL binary: Tools/autotest/sim_vehicle.py wrapper or direct
    sim_vehicle = os.path.join(args.ardupilot_src, "Tools", "autotest", "sim_vehicle.py")
    if not os.path.isfile(sim_vehicle):
        _die(f"ArduPilot sim_vehicle.py not found at: {sim_vehicle}", code=2)

    ap_cmd = [
        sys.executable, sim_vehicle,
        "--vehicle", "ArduCopter",
        "--frame", "quad",
        "--no-rebuild",
        "--sim-address", "127.0.0.1",
        "--out", f"udpout:127.0.0.1:{GCS_PORT}",
    ]
    simuav_cmd = [simuav_bin, "--config", ARDUPILOT_CONFIG]

    if args.dry_run:
        _info("Dry-run mode — commands that would be executed:")
        _info(f"  ArduPilot: {' '.join(ap_cmd)}")
        _info(f"  SimUAV:    {' '.join(simuav_cmd)}")
        _info("Argument validation passed.")
        return 0

    try:
        from pymavlink import mavutil  # type: ignore
    except ImportError:
        _die("pymavlink not installed — run: pip install pymavlink", code=2)

    ap_proc     = None
    simuav_proc = None

    try:
        # 1. Start ArduPilot SITL (headless)
        _info(f"Starting ArduPilot SITL from {args.ardupilot_src} ...")
        ap_proc = subprocess.Popen(
            ap_cmd,
            cwd=args.ardupilot_src,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Give ArduPilot time to bind UDP ports before SimUAV connects.
        time.sleep(5)

        if ap_proc.poll() is not None:
            _die(f"ArduPilot SITL exited early with code {ap_proc.returncode}")

        # 2. Start SimUAV with the ArduPilot config
        _info(f"Starting SimUAV ({simuav_bin}) with ArduPilot config ...")
        simuav_proc = subprocess.Popen(
            simuav_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        time.sleep(1)

        if simuav_proc.poll() is not None:
            _die(f"SimUAV exited early with code {simuav_proc.returncode}")

        # 3. Connect GCS via pymavlink
        _info(f"Connecting to GCS port {GCS_PORT} ...")
        mav = mavutil.mavlink_connection(f"udpin:0.0.0.0:{GCS_PORT}")

        _info(f"Waiting for heartbeat (timeout {HEARTBEAT_TIMEOUT} s) ...")
        mav.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
        _info(f"Heartbeat received (system {mav.target_system}, "
              f"component {mav.target_component})")

        # 4. Assert GPS fix
        _info(f"Waiting for GPS fix (timeout {args.gps_timeout} s) ...")
        if not wait_for_gps_fix(mav, args.gps_timeout):
            _die(f"GPS fix not acquired within {args.gps_timeout} s")
        _info("GPS fix acquired.")

        # 5. Arm
        _info("Sending ARM command ...")
        send_arm(mav)

        _info(f"Waiting for arm acknowledgement (timeout {args.arm_timeout} s) ...")
        if not wait_for_arm_ack(mav, args.arm_timeout):
            _die(f"Arm not acknowledged within {args.arm_timeout} s")
        _info("Vehicle armed successfully.")

        _info("All assertions passed.")
        return 0

    finally:
        _info("Tearing down processes ...")
        if simuav_proc:
            _terminate(simuav_proc)
        if ap_proc:
            _terminate(ap_proc)


if __name__ == "__main__":
    sys.exit(main())
