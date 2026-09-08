#!/usr/bin/env python3
"""
force_matek_connect.py
======================
Deep Serial Hardware Scanner & Force Arming Tool for Matek H743 + Jetson Orin Nano.

Fixes common Jetson serial issues:
 1. Disables 'nvgetty.service' (Jetson serial console conflict on /dev/ttyTHS1).
 2. Exhaustive scan across all ports (/dev/ttyTHS1, /dev/ttyUSB0, /dev/ttyACM0) and baud rates (921600, 57600, 115200).
 3. Continuous 10Hz stream loop that keeps ArduPilot armed and rotors spinning continuously.

Usage:
  sudo python3 scripts/force_matek_connect.py
"""

import sys
import os
import time
import threading
import subprocess

try:
    from pymavlink import mavutil
except ImportError:
    print("[INFO] Installing pymavlink library...")
    subprocess.run([sys.executable, "-m", "pip", "install", "pymavlink"], check=False)
    from pymavlink import mavutil


def fix_jetson_serial():
    print("[1/4] Checking & Clearing Jetson Serial Port Locks...")
    try:
        subprocess.run(["sudo", "systemctl", "stop", "nvgetty.service"], check=False, stderr=subprocess.DEVNULL)
        subprocess.run(["sudo", "systemctl", "disable", "nvgetty.service"], check=False, stderr=subprocess.DEVNULL)
        print("  ✅ Stopped 'nvgetty.service' (Released /dev/ttyTHS1 for MAVLink).")
    except Exception:
        pass

    for p in ["/dev/ttyTHS1", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyTHS0"]:
        if os.path.exists(p):
            try:
                os.chmod(p, 0o666)
                print(f"  ✅ Applied serial permissions on {p}")
            except Exception:
                pass


def scan_ports_and_bauds():
    print("\n[2/4] Scanning Serial Ports & Baud Rates for MAVLink Heartbeat...")
    ports = ["/dev/ttyTHS1", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyTHS0"]
    bauds = [921600, 57600, 115200, 38400, 19200]

    existing_ports = [p for p in ports if os.path.exists(p)]
    if not existing_ports:
        print("  ❌ \033[1;31mNO SERIAL PORTS FOUND ON JETSON!\033[0m")
        return None, None

    for p in existing_ports:
        for b in bauds:
            print(f"  🔍 Testing Port \033[1;33m{p}\033[0m @ \033[1;33m{b} baud\033[0m...", end="", flush=True)
            try:
                conn = mavutil.mavlink_connection(p, baud=b)
                msg = conn.wait_heartbeat(timeout=1.5)
                if msg is not None:
                    print(f" -> 🎉 \033[1;32mHEARTBEAT RECEIVED! System {conn.target_system}, Component {conn.target_component}\033[0m")
                    return conn, (p, b)
                else:
                    print(" (No response)")
                conn.close()
            except Exception as err:
                print(f" (Error: {err})")

    return None, None


def force_arm_and_spin_continuous(conn, port_info):
    port, baud = port_info
    print(f"\n[3/4] Connected to Matek FC on {port} @ {baud} baud.")
    print("----------------------------------------------------------------")
    print("  ⚠️ SAFETY WARNING: CONTINUOUS MOTOR ARMING & ROTOR SPIN ACTIVE!")
    print("----------------------------------------------------------------")

    is_running = True
    current_pwm = 1150  # ~15% Warmup Spin

    def _stream_thread():
        while is_running:
            try:
                # Force arming signal
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 21196, 0, 0, 0, 0, 0
                )
                # Continuous 10Hz RC Override
                conn.mav.rc_channels_override_send(
                    conn.target_system, conn.target_component,
                    1500, 1500, current_pwm, 1500, 0, 0, 0, 0
                )
            except Exception:
                pass
            time.sleep(0.1)

    t = threading.Thread(target=_stream_thread, daemon=True)
    t.start()

    print("\n[4/4] Motors ARMED Continuously at ~15% Throttle (1150 PWM).")
    print("  Press Enter to DISARM and stop motors...")
    input()

    is_running = False
    print("  🛑 Disarming motors and cutting throttle...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    conn.mav.rc_channels_override_send(
        conn.target_system, conn.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("  ✅ Motors Disarmed cleanly.")


def main():
    print("================================================================")
    print("  🛸 MATEK H743 — JETSON NANO FORCE CONNECT & MOTOR TESTER 🛸    ")
    print("================================================================")

    fix_jetson_serial()
    conn, port_info = scan_ports_and_bauds()

    if conn is None:
        print("\n❌ Matek H743 is not responding. Check physical connections.")
        sys.exit(1)

    try:
        force_arm_and_spin_continuous(conn, port_info)
    except KeyboardInterrupt:
        print("\n[INFO] Emergency Disarming...")
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
        sys.exit(0)

if __name__ == "__main__":
    main()
