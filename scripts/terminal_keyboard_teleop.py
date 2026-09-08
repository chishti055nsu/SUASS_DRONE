#!/usr/bin/env python3
"""
terminal_keyboard_teleop.py
============================
Live Keyboard Manual Teleoperation Tool for IUB Drone SUAS 2026.
Allows operators to pilot the quadcopter directly from any terminal window using standard keyboard keys:

Control Scheme:
  • W / S          : Pitch Forward (+N) / Pitch Backward (-N)
  • A / D          : Roll Left (-E) / Roll Right (+E)
  • Up / Down      : Climb (+Altitude) / Descend (-Altitude)
  • Left / Right   : Yaw Left / Yaw Right
  • SPACEBAR       : Emergency Position Hold (Brake)
  • SHIFT + SPACE  : Arm & Offboard Takeoff
  • ESC / Q        : Emergency Disarm / Exit

Usage:
  python3 scripts/terminal_keyboard_teleop.py
"""

import sys
import os
import time
import tty
import termios
import select
import urllib.request
import urllib.parse
import json

SERVER_URL = "http://127.0.0.1:8080/api/teleop"
CMD_URL = "http://127.0.0.1:8080/api/command"


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                # Arrow key sequence
                rlist_seq, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist_seq:
                    seq = sys.stdin.read(2)
                    return key + seq
            return key
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def send_teleop(vn: float, ve: float, vu: float, vyaw: float = 0.0):
    try:
        url = f"{SERVER_URL}?vn={vn}&ve={ve}&vu={vu}&vyaw={vyaw}"
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=0.5) as resp:
            data = json.loads(resp.read().decode('utf-8'))
            return data.get("message", "OK")
    except Exception as e:
        return f"Error: {e}"


def send_cmd(cmd: str):
    try:
        url = f"{CMD_URL}?cmd={cmd}"
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=0.5) as resp:
            data = json.loads(resp.read().decode('utf-8'))
            return data.get("message", "OK")
    except Exception as e:
        return f"Error: {e}"


def main():
    print("================================================================")
    print("   🛸 IUB DRONE SUAS 2026 — TERMINAL KEYBOARD TELEOPERATION 🛸  ")
    print("================================================================")
    print("  Controls:")
    print("    • W / S          : Pitch Forward (+N) / Backward (-N)")
    print("    • A / D          : Roll Left (-E) / Roll Right (+E)")
    print("    • ↑ / ↓ (or I/K) : Altitude Climb (+1m) / Descend (-1m)")
    print("    • ← / → (or J/L) : Yaw Left / Yaw Right")
    print("    • SPACEBAR       : Emergency Position Hold (Brake)")
    print("    • SHIFT + SPACE  : Arm & Offboard Takeoff")
    print("    • ESC / Q        : Disarm & Quit Teleop")
    print("================================================================")
    print("  Listening for keyboard keypresses... Press ESC or Q to exit.\n")

    try:
        while True:
            key = get_key(0.15)
            if key is None:
                continue

            key_lower = key.lower()

            if key in ['\x1b', 'q', 'Q']:
                print("\n[TELEOP] Exiting Keyboard Teleoperation.")
                send_cmd("hold")
                break
            elif key == ' ':
                msg = send_cmd("hold")
                print(f"[KEY: SPACEBAR] -> {msg}")
            elif key_lower == 'w':
                msg = send_teleop(vn=1.5, ve=0.0, vu=0.0)
                print(f"[KEY: W (PITCH FORWARD)] -> {msg}")
            elif key_lower == 's':
                msg = send_teleop(vn=-1.5, ve=0.0, vu=0.0)
                print(f"[KEY: S (PITCH BACKWARD)] -> {msg}")
            elif key_lower == 'a':
                msg = send_teleop(vn=0.0, ve=-1.5, vu=0.0)
                print(f"[KEY: A (ROLL LEFT)] -> {msg}")
            elif key_lower == 'd':
                msg = send_teleop(vn=0.0, ve=1.5, vu=0.0)
                print(f"[KEY: D (ROLL RIGHT)] -> {msg}")
            elif key in ['\x1b[A', 'i', 'I']:
                msg = send_teleop(vn=0.0, ve=0.0, vu=1.0)
                print(f"[KEY: ↑ / I (CLIMB)] -> {msg}")
            elif key in ['\x1b[B', 'k', 'K']:
                msg = send_teleop(vn=0.0, ve=0.0, vu=-1.0)
                print(f"[KEY: ↓ / K (DESCEND)] -> {msg}")
            elif key in ['\x1b[D', 'j', 'J']:
                msg = send_teleop(vn=0.0, ve=0.0, vu=0.0, vyaw=-15.0)
                print(f"[KEY: ← / J (YAW LEFT)] -> {msg}")
            elif key in ['\x1b[C', 'l', 'L']:
                msg = send_teleop(vn=0.0, ve=0.0, vu=0.0, vyaw=15.0)
                print(f"[KEY: → / L (YAW RIGHT)] -> {msg}")

    except KeyboardInterrupt:
        print("\n[TELEOP] Interrupted.")
    finally:
        send_cmd("hold")

if __name__ == "__main__":
    main()
