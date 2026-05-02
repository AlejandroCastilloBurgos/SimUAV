#!/usr/bin/env python3
"""
End-to-end integration test: SimUAV + PX4 SITL.

Starts PX4 SITL (headless) and SimUAV in HIL mode, then uses pymavlink to
assert that the vehicle acquires a GPS lock and arms successfully.

Prerequisites:
  pip install pymavlink
  export PX4_SRC=/path/to/PX4-Autopilot   (or pass --px4-src)
  cmake --build build (SimUAV binary must exist at ./build/simuav)

Exit codes:
  0  all assertions passed
  1  assertion failed or timeout
  2  prerequisite missing (binary not found, etc.)

Usage:
  python3 scripts/e2e_px4_sitl.py [--px4-src <path>] [--simuav <path>]
                                   [--gps-timeout <s>] [--arm-timeout <s>]
"""

import argparse
import os
import signal
import subprocess
import sys
import time

GPS_FIX_TIMEOUT_DEFAULT  = 30   # seconds
ARM_TIMEOUT_DEFAULT      = 15   # seconds
GCS_PORT                 = 14550
HEARTBEAT_TIMEOUT        = 10   # seconds to wait for first heartbeat

# MAVLink command IDs (common dialect)
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_RESULT_ACCEPTED          = 0


def _die(msg: str, code: int = 1) -> None:
    print(f"[e2e] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def _info(msg: str) -> None:
    print(f"[e2e] {msg}", flush=True)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--px4-src",      default=os.environ.get("PX4_SRC", ""),
                   help="Path to PX4-Autopilot source (default: $PX4_SRC)")
    p.add_argument("--simuav",       default="./build/simuav",
                   help="Path to SimUAV binary (default: ./build/simuav)")
    p.add_argument("--gps-timeout",  type=int, default=GPS_FIX_TIMEOUT_DEFAULT,
                   help=f"Seconds to wait for GPS fix (default: {GPS_FIX_TIMEOUT_DEFAULT})")
    p.add_argument("--arm-timeout",  type=int, default=ARM_TIMEOUT_DEFAULT,
                   help=f"Seconds to wait for arming ack (default: {ARM_TIMEOUT_DEFAULT})")
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


def send_arm(mav) -> bool:
    """Send MAV_CMD_COMPONENT_ARM_DISARM and return True on ACCEPTED ack."""
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,   # confirmation
        1,   # param1: 1 = arm
        0, 0, 0, 0, 0, 0,
    )
    return True


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

    try:
        from pymavlink import mavutil  # type: ignore
    except ImportError:
        _die("pymavlink not installed — run: pip install pymavlink", code=2)

    if not args.px4_src:
        _die("PX4 source path not set — pass --px4-src or export PX4_SRC=<path>", code=2)

    if not os.path.isdir(args.px4_src):
        _die(f"PX4 source path does not exist: {args.px4_src}", code=2)

    simuav_bin = os.path.abspath(args.simuav)
    if not os.path.isfile(simuav_bin):
        _die(f"SimUAV binary not found: {simuav_bin}\n"
             "Build with: cmake --build build -j$(nproc)", code=2)

    px4_proc    = None
    simuav_proc = None
    px4_stderr_log = None

    try:
        # 1. Start PX4 SITL (headless, iris airframe, no UI)
        _info(f"Starting PX4 SITL from {args.px4_src} ...")
        px4_env = {**os.environ, "HEADLESS": "1", "PX4_SIM_MODEL": "iris"}
        px4_stderr_log = open("/tmp/px4_sitl_stderr.log", "w")
        px4_proc = subprocess.Popen(
            ["make", "px4_sitl_default", "none_iris"],
            cwd=args.px4_src,
            env=px4_env,
            stdout=subprocess.DEVNULL,
            stderr=px4_stderr_log,
        )

        # Give PX4 a moment to bind its UDP ports before SimUAV connects
        time.sleep(3)

        if px4_proc.poll() is not None:
            px4_stderr_log.flush()
            px4_stderr_log.close()
            try:
                with open("/tmp/px4_sitl_stderr.log") as f:
                    tail = f.read()[-4000:]
                _info(f"PX4 stderr (last 4000 chars):\n{tail}")
            except OSError:
                pass
            _die(f"PX4 SITL exited early with code {px4_proc.returncode}")

        # 2. Start SimUAV
        _info(f"Starting SimUAV ({simuav_bin}) ...")
        simuav_proc = subprocess.Popen(
            [simuav_bin],
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
        if px4_proc:
            _terminate(px4_proc)
        try:
            if px4_stderr_log is not None:
                px4_stderr_log.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
