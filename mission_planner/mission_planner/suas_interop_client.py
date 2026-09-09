"""
suas_interop_client.py
======================
Official SUAS Interop Server Client Module for SUAS Competition.

Handles:
1. Authentication via POST /api/login.
2. 10Hz background telemetry streaming (latitude, longitude, altitude_msl, heading) as required by SUAS rules.
3. Submitting classified ODLC targets via POST /api/odlcs.
4. Fetching stationary and moving cylinder obstacles via GET /api/obstacles.
5. Fetching mission waypoints via GET /api/missions/<id>.
"""

import time
import logging
import threading
from typing import Dict, List, Optional, Any, Callable
import requests

logger = logging.getLogger(__name__)


class SuasInteropClient:
    """
    Official SUAS Interop Server Client & 10Hz Telemetry Streamer.
    """
    def __init__(
        self,
        url: str = "http://192.168.1.10:8000",
        username: str = "iub_drone",
        password: str = "suas2026",
        mission_id: int = 1
    ):
        self.url = url.rstrip('/')
        self.username = username
        self.password = password
        self.mission_id = mission_id
        self.session = requests.Session()

        self._running = False
        self._telemetry_thread: Optional[threading.Thread] = None
        self._is_authenticated = False

        # Attempt initial login
        self.login()

    def login(self) -> bool:
        """Authenticates with the SUAS Interop Server."""
        try:
            resp = self.session.post(
                f"{self.url}/api/login",
                json={"username": self.username, "password": self.password},
                timeout=3.0
            )
            if resp.status_code in (200, 201):
                self._is_authenticated = True
                logger.info(f"[Interop SUCCESS] Authenticated as '{self.username}' with server at {self.url}")
                return True
            else:
                logger.warning(f"[Interop Warning] Login failed with status code {resp.status_code}")
                return False
        except Exception as e:
            logger.warning(f"[Interop Connection Warning] Could not reach Interop server at {self.url}: {e}")
            self._is_authenticated = False
            return False

    def start_telemetry_stream(self, get_telemetry_fn: Callable[[], Dict[str, float]], hz: float = 10.0) -> None:
        """
        Starts a background daemon thread that streams drone telemetry to the Interop server at 10Hz.
        """
        if self._running:
            return

        self._running = True

        def _stream():
            period = 1.0 / max(hz, 1.0)
            while self._running:
                t0 = time.time()
                try:
                    telem = get_telemetry_fn()
                    if telem and self._is_authenticated:
                        payload = {
                            "latitude": float(telem.get("lat", 38.145000)),
                            "longitude": float(telem.get("lon", -76.427000)),
                            "altitude": float(telem.get("alt_msl", 30.0)),
                            "heading": float(telem.get("heading", 0.0))
                        }
                        self.session.post(
                            f"{self.url}/api/telemetry",
                            json=payload,
                            timeout=0.5
                        )
                except Exception:
                    pass

                elapsed = time.time() - t0
                time.sleep(max(0.01, period - elapsed))

        self._telemetry_thread = threading.Thread(target=_stream, daemon=True)
        self._telemetry_thread.start()
        logger.info(f"[Interop] Started 10Hz background telemetry streaming to {self.url}")

    def submit_odlc(
        self,
        latitude: float,
        longitude: float,
        shape: str,
        shape_color: str,
        alphanumeric: str,
        alphanumeric_color: str,
        orientation: str,
        autonomous: bool = True
    ) -> Dict[str, Any]:
        """
        Submits an identified ODLC target to the Interop server.
        """
        payload = {
            "mission": self.mission_id,
            "type": "standard",
            "latitude": float(latitude),
            "longitude": float(longitude),
            "orientation": orientation,
            "shape": shape,
            "shapeColor": shape_color,
            "alphanumeric": alphanumeric,
            "alphanumericColor": alphanumeric_color,
            "autonomous": autonomous
        }

        try:
            resp = self.session.post(f"{self.url}/api/odlcs", json=payload, timeout=2.0)
            if resp.status_code in (200, 201):
                logger.info(f"[Interop ODLC Submitted] Target '{alphanumeric}' ({shape}, {shape_color}) at ({latitude:.6f}, {longitude:.6f})")
                return resp.json() if resp.text else {"status": "ok"}
            else:
                logger.warning(f"[Interop ODLC Submit Failed] Code {resp.status_code}: {resp.text}")
                return {"status": "error", "code": resp.status_code}
        except Exception as e:
            logger.error(f"[Interop ODLC Exception] {e}")
            return {"status": "exception", "error": str(e)}

    def get_obstacles(self) -> Dict[str, Any]:
        """Fetches stationary and moving cylinder obstacles from Interop server."""
        try:
            resp = self.session.get(f"{self.url}/api/obstacles?mission={self.mission_id}", timeout=2.0)
            if resp.status_code == 200:
                return resp.json()
        except Exception as e:
            logger.warning(f"[Interop Obstacles Warning] Failed to fetch obstacles: {e}")

        # Default synthetic fallback obstacles for testing
        return {
            "stationary_obstacles": [
                {"latitude": 38.145100, "longitude": -76.426900, "radius": 8.0, "height": 30.0},
                {"latitude": 38.145250, "longitude": -76.426500, "radius": 10.0, "height": 25.0}
            ],
            "moving_obstacles": []
        }

    def get_waypoints(self) -> List[Dict[str, float]]:
        """Fetches mission waypoints from Interop server."""
        try:
            resp = self.session.get(f"{self.url}/api/missions/{self.mission_id}", timeout=2.0)
            if resp.status_code == 200:
                data = resp.json()
                return data.get("waypoints", [])
        except Exception as e:
            logger.warning(f"[Interop Waypoints Warning] Failed to fetch waypoints: {e}")

        return []

    def stop(self) -> None:
        """Stops the telemetry streaming thread."""
        self._running = False
