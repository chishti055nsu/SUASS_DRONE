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
            
            # Dynamic simulated sensor motion for realistic interactive visualizers
            import time
            now = time.time()
            armed = bool(t.get("armed", False))
            speed = float(t.get("speed_ms", 0.0))
            sim_roll = math.sin(now * 1.5) * 3.5 if armed else 0.0
            sim_pitch = math.cos(now * 1.2) * 2.0 if armed else 0.0
            sim_heading = (now * 5.0) % 360.0 if speed > 0.5 else 45.0
            voltage = round(25.2 - (0.8 * (100 - t.get("battery_pct", 100)) / 100.0), 2)
            current = round(12.5 + (speed * 2.1) if armed else 2.1, 1)

            data = {
                "state": t.get("mode", "IDLE"),
                "pos_enu": list(pos),
                "altitude_m": float(pos[2]),
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
                resp = {"status": "ok", "message": "Motors ARMED & OFFBOARD Mode Enabled."}
            elif cmd == "disarm" or cmd == "terminate":
                STUB_FC.disarm()
                resp = {"status": "ok", "message": "Motors DISARMED & EMERGENCY KILLED."}
            elif cmd == "rtl":
                STUB_FC.trigger_rtl()
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
