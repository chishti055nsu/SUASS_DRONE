#!/usr/bin/env python3
"""
matek_motor_warmup.py
======================
Standalone Motor Warmup & Manual Rotor Test Tool for Matek H743-Wing V3.

Allows direct hardware arming, ESC/motor warmup, and manual throttle testing 
without running the main ROS 2 / Autonomy software stack.

Usage:
  python3 scripts/matek_motor_warmup.py [--port /dev/ttyTHS1] [--baud 57600]
"""

import sys
import os
import time
import argparse

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] pymavlink is required for standalone motor warmup.")
    print("        Install using: pip3 install pymavlink")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Matek H743 Standalone Motor Warmup Tool")
    parser.add_argument("--port", type=str, default=None, help="Serial port (/dev/ttyTHS1, /dev/ttyUSB0, /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (921600, 57600, or 115200)")
    return parser.parse_args()


def detect_port():
    candidates = ["/dev/ttyTHS1", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyTHS0"]
    for p in candidates:
        if os.path.exists(p):
            return p
    return "/dev/ttyTHS1"


def main():
    args = parse_args()
    port = args.port or detect_port()
    baud = args.baud

    print("================================================================")
    print("  🛸 MATEK H743 FLIGHT CONTROLLER — STANDALONE MOTOR WARMUP 🛸 ")
    print("================================================================")
    print(f"  Target Serial Port : {port}")
    print(f"  Baud Rate          : {baud}")
    print("================================================================")
    print("  Connecting to Flight Controller... Please wait...")

    try:
        master = mavutil.mavlink_connection(port, baud=baud)
        master.wait_heartbeat(timeout=10)
        print(f"  ✅ [CONNECTED] MAVLink Heartbeat received from System {master.target_system}, Component {master.target_component}")
    except Exception as e:
        print(f"  ❌ [CONNECTION ERROR] Failed to connect on {port}: {e}")
        print("  👉 Check if cable is connected and user has dialout permission (sudo usermod -a -G dialout $USER).")
        sys.exit(1)

    print("\n----------------------------------------------------------------")
    print("  ⚠️ SAFETY WARNING: PROPELLERS WILL SPIN! KEEP CLEAR OF ROTORS!")
    print("----------------------------------------------------------------")
    print("  Controls:")
    print("    [1] ARM MOTORS & WARMUP IDLE SPIN (~10% Throttle / 1100 PWM)")
    print("    [2] LOW ROTOR THRUST (~25% Throttle / 1250 PWM)")
    print("    [3] MEDIUM ROTOR THRUST (~40% Throttle / 1400 PWM)")
    print("    [0] IDLE PWM (1000 PWM)")
    print("    [D] DISARM MOTORS IMMEDIATELY")
    print("    [Q] QUIT TOOL & CUT MOTOR POWER")
    print("----------------------------------------------------------------\n")

    try:
        while True:
            cmd = input("Select Action (1=Warmup, 2=Low, 3=Med, 0=Idle, D=Disarm, Q=Quit) > ").strip().lower()

            if cmd in ['q', 'exit']:
                print("[INFO] Disarming motors and exiting...")
                master.arducopter_disarm() if hasattr(master, 'arducopter_disarm') else master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
                )
                break

            elif cmd == '1':
                print("[WARMUP] Arming Flight Controller & initiating rotor idle spin...")
                # Force arm command (param1=1 arm, param2=21196 force arm override)
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
                )
                time.sleep(0.5)
                # Send RC PWM throttle override for 4 motors (1100 PWM = ~10% Idle Warmup)
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1500, 1500, 1100, 1500, 0, 0, 0, 0
                )
                print("  ✅ Motors ARMED and spinning at Warmup Idle (~10% Throttle).")

            elif cmd == '2':
                print("[THRUST] Increasing rotor speed to Low Thrust (~25%)...")
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1500, 1500, 1250, 1500, 0, 0, 0, 0
                )
                print("  ✅ Rotor speed set to ~25% Throttle (1250 PWM).")

            elif cmd == '3':
                print("[THRUST] Increasing rotor speed to Medium Thrust (~40%)...")
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1500, 1500, 1400, 1500, 0, 0, 0, 0
                )
                print("  ✅ Rotor speed set to ~40% Throttle (1400 PWM).")

            elif cmd == '0':
                print("[IDLE] Setting throttle to minimum idle (1000 PWM)...")
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1500, 1500, 1000, 1500, 0, 0, 0, 0
                )

            elif cmd == 'd':
                print("[DISARM] Emergency motor disarm requested...")
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
                )
                # Release RC override
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                print("  🛑 MOTORS DISARMED & POWER CUT.")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted. Emergency disarming motors...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
        sys.exit(0)

if __name__ == "__main__":
    main()
