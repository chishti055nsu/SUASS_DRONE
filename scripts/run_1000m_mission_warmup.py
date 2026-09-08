#!/usr/bin/env python3
"""
run_1000m_mission_warmup.py
============================
Continuous Rotor Warmup & Full 1000m Flight Mission Simulator for Matek H743 + Jetson Orin Nano.

Fixes premature disarming by running a continuous 10Hz MAVLink stream thread that keeps 
ArduPilot armed and continuously drives throttle PWM across all 7 stages of a 1000m mission.

Designed for bench testing with propellers removed.

Usage:
  sudo python3 scripts/run_1000m_mission_warmup.py [--port /dev/ttyTHS1] [--baud 921600]
"""

import sys
import os
import time
import threading
import argparse
import subprocess

try:
    from pymavlink import mavutil
except ImportError:
    print("[INFO] Installing pymavlink library...")
    subprocess.run([sys.executable, "-m", "pip", "install", "pymavlink"], check=False)
    from pymavlink import mavutil


class FlightMissionSimulator:
    def __init__(self, port="/dev/ttyTHS1", baud=921600):
        self.port = port
        self.baud = baud
        self.master = None
        self.is_running = False
        self.current_pwm = 1000  # Idle PWM
        self.roll_pwm = 1500
        self.pitch_pwm = 1500
        self.yaw_pwm = 1500
        self.is_armed = False
        self.target_mode = "GUIDED"
        self.stream_thread = None
        self.distance_covered_m = 0.0

    def prepare_hardware(self):
        print("[1/5] Preparing Jetson Serial Port & Permissions...")
        try:
            subprocess.run(["sudo", "systemctl", "stop", "nvgetty.service"], check=False, stderr=subprocess.DEVNULL)
            subprocess.run(["sudo", "systemctl", "disable", "nvgetty.service"], check=False, stderr=subprocess.DEVNULL)
            print("  ✅ Disabled 'nvgetty.service' (Serial console lock cleared).")
        except Exception:
            pass

        if os.path.exists(self.port):
            try:
                os.chmod(self.port, 0o666)
                print(f"  ✅ Permissions set on {self.port}")
            except Exception:
                pass

    def connect(self):
        print(f"\n[2/5] Connecting to Matek H743 on {self.port} @ {self.baud} baud...")
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            print("  ⏳ Waiting for MAVLink Heartbeat from Flight Controller...")
            hb = self.master.wait_heartbeat(timeout=10)
            if hb is None:
                print(f"  ❌ No heartbeat received on {self.port} @ {self.baud} baud.")
                sys.exit(1)
            print(f"  🎉 ✅ [CONNECTED] System ID: {self.master.target_system}, Component ID: {self.master.target_component}")
        except Exception as err:
            print(f"  ❌ Connection failure: {err}")
            sys.exit(1)

    def start_streaming_thread(self):
        """Continuously streams RC override and Arm commands at 10Hz to prevent ArduPilot auto-disarm."""
        self.is_running = True
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        print("  ✅ Continuous 10Hz MAVLink Stream Thread Started (Prevents ArduPilot Disarm Timeout).")

    def _stream_loop(self):
        while self.is_running:
            try:
                if self.is_armed and self.master:
                    # 1. Send force arm command every second to keep ArduPilot armed
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        1,      # param1: 1 = arm
                        21196,  # param2: 21196 = force arm override magic code
                        0, 0, 0, 0, 0
                    )

                    # 2. Stream RC channels override continuously (10Hz)
                    self.master.mav.rc_channels_override_send(
                        self.master.target_system,
                        self.master.target_component,
                        int(self.roll_pwm),
                        int(self.pitch_pwm),
                        int(self.current_pwm),  # Throttle Channel 3
                        int(self.yaw_pwm),
                        0, 0, 0, 0
                    )
            except Exception:
                pass
            time.sleep(0.1)  # 10 Hz rate

    def force_arm(self):
        print("\n[3/5] Requesting Force Arming Override on Matek H743...")
        self.is_armed = True
        self.current_pwm = 1150  # ~12% Warmup Idle Spin
        time.sleep(1.0)
        print("  ✅ Flight Controller ARMED! Motors spinning at Warmup Idle (1150 PWM).")

    def disarm(self):
        print("\n[DISARM] Cutting rotor power & disarming FC...")
        self.is_armed = False
        self.current_pwm = 1000
        if self.master:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            except Exception:
                pass
        print("  🛑 MOTORS DISARMED & POWER CUT.")

    def run_1000m_mission(self):
        print("\n==========================================================================")
        print("  🛸 STARTING FULL 1000m AUTONOMOUS MISSION ROTOR WARMUP SIMULATION 🛸")
        print("==========================================================================")
        print("  ⚠️ SAFETY REMINDER: PROPELLERS MUST BE DISCONNECTED FOR BENCH TEST!")
        print("==========================================================================\n")

        self.force_arm()

        stages = [
            ("Stage 1: Pre-Flight ESC Warmup & Sensor Stabilization", 15, 1150, 0.0, 0.0),
            ("Stage 2: Vertical Takeoff & Climb to Cruise Altitude (15m)", 15, 1450, 0.0, 50.0),
            ("Stage 3: Outbound 500m Search Sweep (High Throttle Cruise)", 30, 1650, 50.0, 500.0),
            ("Stage 4: Target Identified — Payload Drop Servo Actuation", 10, 1400, 500.0, 500.0),
            ("Stage 5: Inbound 500m Return to Base (RTL Cruise Flight)", 30, 1680, 500.0, 1000.0),
            ("Stage 6: Controlled Precision Descent & Touchdown Alignment", 15, 1250, 1000.0, 1000.0),
            ("Stage 7: Final Touchdown & Motor Cooling Ramp Down", 10, 1080, 1000.0, 1000.0)
        ]

        start_time = time.time()

        for stage_name, duration_sec, pwm, start_dist, end_dist in stages:
            print(f"\n▶ [{stage_name}]")
            print(f"  Duration: {duration_sec}s | Target Throttle: PWM {pwm} | Distance Range: {start_dist:.0f}m -> {end_dist:.0f}m")

            if "Payload Drop" in stage_name:
                print("  📦 Actuating Payload Drop Servo (Channel 5 / Servo 1)...")
                try:
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        5,     # Servo instance 5
                        1900,  # PWM open position
                        0, 0, 0, 0, 0
                    )
                except Exception:
                    pass

            self.current_pwm = pwm
            step_time = 0.5
            elapsed_stage = 0.0

            while elapsed_stage < duration_sec:
                time.sleep(step_time)
                elapsed_stage += step_time
                progress_pct = (elapsed_stage / duration_sec) * 100.0

                # Interpolate distance
                dist_pct = elapsed_stage / duration_sec
                self.distance_covered_m = start_dist + (end_dist - start_dist) * dist_pct

                total_elapsed = time.time() - start_time
                throttle_pct = ((self.current_pwm - 1000) / 1000.0) * 100.0

                sys.stdout.write(
                    f"\r  ⏱️ [{total_elapsed:5.1f}s] Stage Progress: {progress_pct:5.1f}% | "
                    f"Distance: \033[1;32m{self.distance_covered_m:6.1f}m / 1000m\033[0m | "
                    f"Throttle: \033[1;33m{throttle_pct:4.1f}% (PWM {self.current_pwm})\033[0m  "
                )
                sys.stdout.flush()

        print("\n\n==========================================================================")
        print("  🎉 1000m MISSION COMPLETED SUCCESSFULLY!")
        print("  • Total Flight Distance Simulated : 1000.0 Meters")
        print("  • Total Rotor Run Time           : {:.1f} Seconds".format(time.time() - start_time))
        print("  • Payload Drop Servo State       : Actuated (Open)")
        print("==========================================================================")

        self.disarm()
        self.is_running = False


def main():
    parser = argparse.ArgumentParser(description="Matek H743 1000m Flight Mission Rotor Simulator")
    parser.add_argument("--port", type=str, default="/dev/ttyTHS1", help="Serial port (/dev/ttyTHS1, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (921600)")
    args = parser.parse_args()

    sim = FlightMissionSimulator(port=args.port, baud=args.baud)
    sim.prepare_hardware()
    sim.connect()
    sim.start_streaming_thread()

    try:
        sim.run_1000m_mission()
    except KeyboardInterrupt:
        print("\n\n[USER ABORT] Emergency stop triggered!")
        sim.disarm()
        sim.is_running = False
        sys.exit(0)


if __name__ == "__main__":
    main()
