#!/usr/bin/env python3
"""
matek_motor_warmup.py
======================
Standalone Motor Warmup & Manual Rotor Test Tool for Matek H743-Wing V3.

Fixes premature disarming by streaming RC override & Arm commands continuously in a 10Hz background thread.

Usage:
  python3 scripts/matek_motor_warmup.py [--port /dev/ttyTHS1] [--baud 921600]
"""

import sys
import os
import time
import threading
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


class MotorWarmupController:
    def __init__(self, master):
        self.master = master
        self.is_armed = False
        self.current_pwm = 1000
        self.is_running = True
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()

    def _stream_loop(self):
        while self.is_running:
            if self.is_armed and self.master:
                try:
                    # Keep force arming signal active every cycle
                    self.master.mav.command_long_send(
                        self.master.target_system, self.master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
                    )
                    # Send RC Override continuously (10Hz)
                    self.master.mav.rc_channels_override_send(
                        self.master.target_system, self.master.target_component,
                        1500, 1500, int(self.current_pwm), 1500, 0, 0, 0, 0
                    )
                except Exception:
                    pass
            time.sleep(0.1)  # 10Hz continuous stream

    def arm(self, initial_pwm=1150):
        self.is_armed = True
        self.current_pwm = initial_pwm

    def disarm(self):
        self.is_armed = False
        self.current_pwm = 1000
        if self.master:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
                )
                self.master.mav.rc_channels_override_send(
                    self.master.target_system, self.master.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            except Exception:
                pass


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

    controller = MotorWarmupController(master)

    print("\n----------------------------------------------------------------")
    print("  ⚠️ SAFETY WARNING: PROPELLERS MUST BE DISCONNECTED FOR BENCH TEST!")
    print("----------------------------------------------------------------")
    print("  Controls:")
    print("    [1] ARM MOTORS & WARMUP IDLE SPIN (~15% Throttle / 1150 PWM)")
    print("    [2] LOW ROTOR THRUST (~30% Throttle / 1300 PWM)")
    print("    [3] MEDIUM ROTOR THRUST (~50% Throttle / 1500 PWM)")
    print("    [4] HIGH ROTOR THRUST (~75% Throttle / 1750 PWM)")
    print("    [5] FULL THROTTLE THRUST (100% Throttle / 1950 PWM)")
    print("    [0] IDLE PWM (1000 PWM)")
    print("    [D] DISARM MOTORS IMMEDIATELY")
    print("    [Q] QUIT TOOL & CUT MOTOR POWER")
    print("----------------------------------------------------------------\n")

    try:
        while True:
            cmd = input("Select Action (1=Warmup, 2=Low, 3=Med, 4=High, 5=Full, 0=Idle, D=Disarm, Q=Quit) > ").strip().lower()

            if cmd in ['q', 'exit']:
                print("[INFO] Disarming motors and exiting...")
                controller.disarm()
                controller.is_running = False
                break

            elif cmd == '1':
                print("[WARMUP] Arming & spinning rotors at Warmup Idle (1150 PWM)...")
                controller.arm(1150)
                print("  ✅ Motors ARMED continuously at Warmup Idle (~15%).")

            elif cmd == '2':
                print("[THRUST] Low Thrust (1300 PWM / ~30%)...")
                controller.arm(1300)
                print("  ✅ Motors spinning continuously at 1300 PWM.")

            elif cmd == '3':
                print("[THRUST] Medium Thrust (1500 PWM / ~50%)...")
                controller.arm(1500)
                print("  ✅ Motors spinning continuously at 1500 PWM.")

            elif cmd == '4':
                print("[THRUST] High Thrust (1750 PWM / ~75%)...")
                controller.arm(1750)
                print("  ✅ Motors spinning continuously at 1750 PWM.")

            elif cmd == '5':
                print("[THRUST] FULL THROTTLE THRUST (1950 PWM / 100%)...")
                controller.arm(1950)
                print("  🔥 FULL THROTTLE CONTINUOUS THRUST ACTIVE!")

            elif cmd == '0':
                print("[IDLE] Setting throttle to minimum idle (1000 PWM)...")
                controller.current_pwm = 1000

            elif cmd == 'd':
                print("[DISARM] Disarming motors...")
                controller.disarm()
                print("  🛑 MOTORS DISARMED & POWER CUT.")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted. Emergency disarming motors...")
        controller.disarm()
        controller.is_running = False
        sys.exit(0)

if __name__ == "__main__":
    main()
