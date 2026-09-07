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
            data = {
                "state": t.get("mode", "IDLE"),
                "altitude_m": float(t.get("pos_enu", (0, 0, 0))[2]),
                "speed_ms": float(t.get("speed_ms", 0.0)),
                "battery_pct": float(t.get("battery_pct", 100.0)),
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
            return

        elif path == "/api/command":
            cmd = query.get("cmd", ["start"])[0]
            logger.info(f"[Web GCS] Command received: {cmd}")

            # Execute send_command.sh script
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
            n = query.get("n", ["0.0"])[0]
            e = query.get("e", ["0.0"])[0]
            alt = query.get("alt", ["15.0"])[0]
            down = str(-abs(float(alt)))

            script_path = os.path.join(ROOT_DIR, "scripts", "send_command.sh")
            try:
                subprocess.run(["bash", script_path, "goto", n, e, down], check=False)
                resp = {"status": "ok", "message": f"GOTO Dispatched: North={n}m, East={e}m, Alt={alt}m."}
            except Exception as err:
                resp = {"status": "error", "message": str(err)}

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
