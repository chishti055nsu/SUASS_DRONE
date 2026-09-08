#!/usr/bin/env python3
"""
force_matek_connect.py
======================
Deep Serial Hardware Scanner & Force Arming Tool for Matek H743 + Jetson Orin Nano.

Fixes common Jetson serial issues:
 1. Disables 'nvgetty.service' (Jetson serial console conflict on /dev/ttyTHS1).
 2. Exhaustive scan across all ports (/dev/ttyTHS1, /dev/ttyUSB0, /dev/ttyACM0) and baud rates (57600, 115200, 921600).
 3. Forces MAVLink arming override (param2=21196) and spins rotors immediately.

Usage:
  sudo python3 scripts/force_matek_connect.py
"""

import sys
import os
import time
import subprocess

try:
    from pymavlink import mavutil
except ImportError:
    print("[INFO] Installing pymavlink library...")
    subprocess.run([sys.executable, "-m", "pip", "install", "pymavlink"], check=False)
    from pymavlink import mavutil


def fix_jetson_serial():
    print("[1/4] Checking & Clearing Jetson Serial Port Locks...")
    # Stop nvgetty service if active (it locks /dev/ttyTHS1 on Jetson)
    try:
        subprocess.run(["sudo", "systemctl", "stop", "nvgetty.service"], check=False, stderr=subprocess.DEVNULL)
        subprocess.run(["sudo", "systemctl", "disable", "nvgetty.service"], check=False, stderr=subprocess.DEVNULL)
        print("  ✅ Stopped 'nvgetty.service' (Released /dev/ttyTHS1 for MAVLink).")
    except Exception:
        pass

    # Fix device permissions
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
    bauds = [57600, 115200, 921600, 38400, 19200]

    existing_ports = [p for p in ports if os.path.exists(p)]
    if not existing_ports:
        print("  ❌ \033[1;31mNO SERIAL PORTS FOUND ON JETSON!\033[0m")
        print("  👉 Check physical wiring: USB cable or Jetson 40-pin header (Pin 8 TX -> FC RX, Pin 10 RX -> FC TX, Pin 6 GND).")
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


def force_arm_and_spin(conn, port_info):
    port, baud = port_info
    print(f"\n[3/4] Connected to Matek FC on {port} @ {baud} baud.")
    print("----------------------------------------------------------------")
    print("  ⚠️ SAFETY WARNING: FORCING MOTOR ARMING & ROTOR SPIN NOW!")
    print("----------------------------------------------------------------")
    
    print("  👉 Sending MAV_CMD_COMPONENT_ARM_DISARM with Force Override (21196)...")
    
    # Try 3 times to send force arm command
    for i in range(3):
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,      # param1: 1 = arm
            21196,  # param2: 21196 = force arming magic override code
            0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    print("  👉 Sending RC Channel Throttle Override (PWM 1150 = ~12% Warmup Spin)...")
    for _ in range(25):
        conn.mav.rc_channels_override_send(
            conn.target_system,
            conn.target_component,
            1500,  # Roll (Ch1)
            1500,  # Pitch (Ch2)
            1150,  # Throttle (Ch3: Warmup Spin)
            1500,  # Yaw (Ch4)
            0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n[4/4] Motor Warmup Spin Test Active!")
    print("  Press Enter to DISARM and stop motors...")
    input()

    print("  🛑 Disarming motors and cutting throttle...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
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
        print("\n================================================================")
        print("  ❌ \033[1;31mMATEK H743 IS NOT RESPONDING ON ANY PORT OR BAUD RATE!\033[0m")
        print("================================================================")
        print("  🔧 HARDWARE TROUBLESHOOTING CHECKLIST:")
        print("  1. TX/RX SWAP CHECK (Most Common Issue):")
        print("     - Jetson Pin 8 (TX)  -->  Matek RX (e.g. RX6 or RX1)")
        print("     - Jetson Pin 10 (RX) -->  Matek TX (e.g. TX6 or TX1)")
        print("     - Jetson Pin 6 (GND) -->  Matek GND")
        print("     *(If connected TX->TX, swap the two signal wires!)*")
        print("\n  2. USB TEST:")
        print("     - Plug a USB-C cable directly from Jetson Nano USB to Matek H743 USB port.")
        print("     - Re-run this script: sudo python3 scripts/force_matek_connect.py")
        print("\n  3. FLIGHT BATTERY POWER:")
        print("     - Ensure 4S-6S LiPo battery is plugged in so FC & ESCs have power.")
        print("================================================================")
        sys.exit(1)

    try:
        force_arm_and_spin(conn, port_info)
    except KeyboardInterrupt:
        print("\n[INFO] Emergency Disarming...")
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
        sys.exit(0)

if __name__ == "__main__":
    main()
