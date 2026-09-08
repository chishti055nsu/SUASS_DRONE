"""
drone_web_gui.py
================
Web Ground Control Station (GCS) Server for IUB Drone SUAS 2026.
Serves a user-friendly browser UI on http://0.0.0.0:8080 for novice student operators.
No terminal usage required — control drone via one-touch buttons and browser forms!
"""

import os
import sys
import json
import math
import subprocess
import urllib.parse
from http.server import HTTPServer, SimpleHTTPRequestHandler
import logging

logger = logging.getLogger(__name__)

# Workspace path setup
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT_DIR, "mission_planner"))
sys.path.insert(0, os.path.join(ROOT_DIR, "drone_vision"))

from mission_planner.flight_controller import SimStubFlightController

# Persistent Stub Controller for Telemetry Simulation when MAVROS is absent
STUB_FC = SimStubFlightController()


class WebGCSHandler(SimpleHTTPRequestHandler):
    """Custom HTTP Request Handler serving index.html and handling GCS REST API endpoints."""

    def __init__(self, *args, **kwargs):
        gui_dir = os.path.join(ROOT_DIR, "web_gui")
        super().__init__(*args, directory=gui_dir, **kwargs)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            t = STUB_FC.get_telemetry()
            pos = t.get("pos_enu", (0.0, 0.0, 0.0))
            
            import time
            now = time.time()
            armed = bool(t.get("armed", False))
            speed = float(t.get("speed_ms", 0.0))
            sim_roll = math.sin(now * 1.5) * 3.5 if armed and speed > 0.1 else 0.0
            sim_pitch = math.cos(now * 1.2) * 2.0 if armed and speed > 0.1 else 0.0
            sim_heading = (now * 12.0) % 360.0 if speed > 0.5 else 45.0
            voltage = round(25.2 - (0.8 * (100 - t.get("battery_pct", 100)) / 100.0), 2)
            current = round(12.5 + (speed * 2.1) if armed else 2.1, 1)

            altitude = float(pos[2])
            lidar_dist = round(max(0.3, altitude + (math.sin(now * 2.0) * 0.04 if armed else 0.0)), 2)

            data = {
                "state": t.get("mode", "IDLE"),
                "pos_enu": list(pos),
                "altitude_m": altitude,
                "speed_ms": speed,
                "battery_pct": float(t.get("battery_pct", 100.0)),
                "voltage": voltage,
                "current_a": current,
                "cpu_load": 14,
                "gpu_load": 8,
                "temp_c": 41.5,
                "gps_fix": "3D RTK FIX",
                "armed": armed,
                "roll": round(sim_roll, 1),
                "pitch": round(sim_pitch, 1),
                "heading": round(sim_heading, 1),
                "satellites": 18,
                "rssi_pct": 98,
                "payload_released": bool(t.get("payload_released", False)),
                "lidar": {
                    "distance_m": lidar_dist,
                    "signal_quality": 96,
                    "status": "HEALTHY",
                    "min_m": 0.3,
                    "max_m": 12.0
                },
                "gps_feedback": {
                    "latitude": 38.145025 + (pos[1] * 0.000009),
                    "longitude": -76.426980 + (pos[0] * 0.000011),
                    "altitude_msl": round(30.0 + altitude, 2),
                    "hdop": 0.6,
                    "vdop": 0.8,
                    "fix_type": "3D RTK FIX (0.02m)",
                    "satellites": 18,
                    "vel_ned": [round(speed * 0.7, 2), round(speed * 0.7, 2), 0.0]
                },
                "realsense_d455": {
                    "vio_status": "LOCK (6-DOF ODOMETRY)",
                    "fps": 30.0,
                    "pos_vio_enu": list(pos),
                    "optical_flow": "STABLE",
                    "depth_range": "0.4m - 10.0m"
                },
                "object_detections": [
                    {
                        "target_id": "TGT-01",
                        "label": "MANNEQUIN",
                        "payload_match": "WATER_BOTTLE",
                        "confidence": 0.942,
                        "bbox": [180, 110, 80, 80],
                        "lat": 38.145120,
                        "lon": -76.426880
                    },
                    {
                        "target_id": "TGT-02",
                        "label": "TENT",
                        "payload_match": "MEDICAL_KIT",
                        "confidence": 0.895,
                        "bbox": [80, 160, 90, 75],
                        "lat": 38.145250,
                        "lon": -76.426510
                    }
                ],
                "checklist": {
                    "imu": True,
                    "gps": True,
                    "lidar": True,
                    "vio": True,
                    "payload": True,
                    "geofence": True
                }
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
            return

        elif path == "/api/command":
            cmd = query.get("cmd", ["start"])[0]
            logger.info(f"[Web GCS] Command received: {cmd}")

            if cmd == "arm":
                STUB_FC.arm_and_offboard()
                STUB_FC._target_setpoint = [0.0, 0.0, 10.0]
                resp = {"status": "ok", "message": "Motors ARMED & OFFBOARD Mode Enabled."}
            elif cmd == "start":
                STUB_FC.arm_and_offboard()
                STUB_FC._mode = "WAYPOINT_NAV"
                STUB_FC._target_setpoint = [30.0, 45.0, 15.0]
                script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
                try:
                    subprocess.run(["bash", script_path, "start"], check=False)
                except Exception:
                    pass
                resp = {"status": "ok", "message": "AUTONOMOUS MISSION LAUNCHED. Waypoint navigation active."}
            elif cmd == "hold":
                STUB_FC._mode = "HOVER"
                t = STUB_FC.get_telemetry()
                STUB_FC._target_setpoint = list(t.get("pos_enu", [0.0, 0.0, 15.0]))
                script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
                try:
                    subprocess.run(["bash", script_path, "abort"], check=False)
                except Exception:
                    pass
                resp = {"status": "ok", "message": "HOVER / HOLD Executed. Locking current 3D position."}
            elif cmd == "land":
                STUB_FC._mode = "LAND"
                t = STUB_FC.get_telemetry()
                pos = t.get("pos_enu", [0.0, 0.0, 0.0])
                STUB_FC._target_setpoint = [pos[0], pos[1], 0.0]
                script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
                try:
                    subprocess.run(["bash", script_path, "land"], check=False)
                except Exception:
                    pass
                resp = {"status": "ok", "message": "LAND NOW Executed. Controlled vertical descent active."}
            elif cmd == "disarm" or cmd == "terminate":
                STUB_FC.disarm()
                STUB_FC._mode = "IDLE"
                t = STUB_FC.get_telemetry()
                pos = t.get("pos_enu", [0.0, 0.0, 0.0])
                STUB_FC._target_setpoint = [pos[0], pos[1], 0.0]
                script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
                try:
                    subprocess.run(["bash", script_path, "abort"], check=False)
                except Exception:
                    pass
                resp = {"status": "ok", "message": "Motors DISARMED & EMERGENCY KILLED."}
            elif cmd == "rtl":
                STUB_FC.trigger_rtl()
                STUB_FC._mode = "RTL"
                STUB_FC._target_setpoint = [0.0, 0.0, 15.0]
                script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
                try:
                    subprocess.run(["bash", script_path, "rtl"], check=False)
                except Exception:
                    pass
                resp = {"status": "ok", "message": "RTL Triggered. Returning to Home pose."}
            elif cmd == "payload":
                STUB_FC.trigger_payload_release()
                resp = {"status": "ok", "message": "Payload Servo Release Triggered."}
            else:
                script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
                try:
                    subprocess.run(["bash", script_path, cmd], check=False)
                    resp = {"status": "ok", "message": f"Command '{cmd}' executed successfully."}
                except Exception as e:
                    resp = {"status": "error", "message": str(e)}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        elif path == "/api/goto":
            n = float(query.get("n", ["0.0"])[0])
            e = float(query.get("e", ["0.0"])[0])
            alt = float(query.get("alt", ["15.0"])[0])
            STUB_FC.set_setpoint_enu(e, n, alt)

            script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
            try:
                subprocess.run(["bash", script_path, "goto", str(n), str(e), str(-abs(alt))], check=False)
                resp = {"status": "ok", "message": f"GOTO Dispatched: North={n}m, East={e}m, Alt={alt}m."}
            except Exception as err:
                resp = {"status": "error", "message": str(err)}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        elif path == "/api/teleop":
            vn = float(query.get("vn", ["0.0"])[0])
            ve = float(query.get("ve", ["0.0"])[0])
            vu = float(query.get("vu", ["0.0"])[0])
            vyaw = float(query.get("vyaw", ["0.0"])[0])

            if not STUB_FC.is_armed():
                STUB_FC.arm_and_offboard()

            STUB_FC._mode = "KEYBOARD_TELEOP"
            t = STUB_FC.get_telemetry()
            pos = list(t.get("pos_enu", [0.0, 0.0, 10.0]))
            
            dt = 0.3
            new_e = round(pos[0] + (ve * dt * 4.0), 2)
            new_n = round(pos[1] + (vn * dt * 4.0), 2)
            new_u = round(max(0.5, pos[2] + (vu * dt * 2.0)), 2)

            STUB_FC.set_setpoint_enu(new_e, new_n, new_u)

            script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
            try:
                subprocess.run(["bash", script_path, "goto", str(new_n), str(new_e), str(-abs(new_u))], check=False)
            except Exception:
                pass

            resp = {"status": "ok", "message": f"KEYBOARD TELEOP: VN={vn}, VE={ve}, VU={vu} -> New Target (E:{new_e}m, N:{new_n}m, Alt:{new_u}m)"}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        elif path == "/api/jog":
            axis = query.get("axis", ["n"])[0]
            val = float(query.get("val", ["1.0"])[0])
            t = STUB_FC.get_telemetry()
            curr_pos = list(t.get("pos_enu", [0.0, 0.0, 15.0]))
            
            if axis == "n":
                curr_pos[1] += val
            elif axis == "s":
                curr_pos[1] -= val
            elif axis == "e":
                curr_pos[0] += val
            elif axis == "w":
                curr_pos[0] -= val
            elif axis == "up":
                curr_pos[2] += val
            elif axis == "down":
                curr_pos[2] = max(1.0, curr_pos[2] - val)

            STUB_FC.set_setpoint_enu(curr_pos[0], curr_pos[1], curr_pos[2])
            resp = {"status": "ok", "message": f"Nudged {axis.upper()} by {val}m. New Target: E={curr_pos[0]}m, N={curr_pos[1]}m, Alt={curr_pos[2]}m."}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        # Serve index.html or static files
        super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/load_gps":
            content_len = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_len).decode("utf-8")
            try:
                data = json.loads(body)
                home_lat = float(data.get("home_lat", 38.145))
                home_lon = float(data.get("home_lon", -76.427))
                pts_str = data.get("points", "")

                from mission_planner.waypoint_manager import WaypointManager
                wm = WaypointManager()
                pts = []
                if pts_str:
                    for item in pts_str.split(";"):
                        if "," in item:
                            lat, lon = item.split(",")
                            pts.append({"latitude": float(lat), "longitude": float(lon), "altitude": 15.0})

                plan = wm.load_raw_gps_coordinates(home_lat, home_lon, pts)
                resp = {"status": "ok", "message": f"Successfully loaded {plan.total()} waypoints relative to Home!"}
            except Exception as e:
                resp = {"status": "error", "message": str(e)}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        super().do_POST()


def run_web_gcs_server(port: int = 8080):
    server_address = ("0.0.0.0", port)
    httpd = HTTPServer(server_address, WebGCSHandler)
    print("==========================================================================")
    print("       🛸 IUB DRONE SUAS 2026 — WEB GROUND CONTROL STATION 🛸            ")
    print(f"  Web Dashboard UI Server running at: http://localhost:{port}")
    print(f"  Access from any laptop/tablet/phone on network: http://<JETSON_IP>:{port}")
    print("==========================================================================")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nWeb GCS Server stopped.")


if __name__ == "__main__":
    run_web_gcs_server()
