"""
skydio_x2_sim.py
================
IUB Drone — Skydio X2 Native MuJoCo Simulation (1 KM Multi-Target SUAS Edition).

Features:
  - Official DeepMind Skydio X2 model with 1.5 KM Extended Airspace & Multi-Storey City Skyscrapers
  - Target 1: 500-Meter Drop Zone Bullseye [500m, 100m, 3.5m]
  - Target 2: 1-Kilometer Extended Inspection Target [1000m, -50m, 3.5m]
  - Airborne Weather Balloons (25m - 60m altitude) & Flying Birds
  - Dual Windows: Native 3D MuJoCo Interactive Viewer + Downward Camera Feed with HUD Overlay
  - Smooth 18 m/s Trajectory Setpoint Generator (High-Speed 1km Transit)
  - Key Callbacks in BOTH 3D Viewer & Terminal (Auto-arms on S/D/L/1/2 keys)

Usage:
    mjpython skydio_x2_sim.py

Controls (Press in 3D Window OR Terminal):
    S    → Start mission (TAKEOFF → SEARCH pattern)
    D / 1→ Fly to Target 1: 500m Drop Target [500m, 100m, 3.5m]
    2    → Fly to Target 2: 1km Extended Target [1000m, -50m, 3.5m]
    L / 3→ Return Home & Land on White H-marker [0m, 0m, 0.08m]
    A    → Abort / hold position
    C    → Toggle Camera View (Chase Cam <-> Downward Vision <-> Spotter Cam)
    R    → Reset simulation
    Q    → Quit
"""

import os
import sys
import time
import math
import argparse
import logging
import select
import termios
import tty
import numpy as np
import cv2

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "drone_vision"))
sys.path.insert(0, os.path.join(ROOT, "mission_planner"))

from drone_vision.yolo_detector            import YOLODetector
from drone_vision.gemma_analyzer           import GemmaAnalyzer
from mission_planner.mission_state_machine import MissionStateMachine, MissionState

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("skydio_x2_sim")

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    sys.exit("MuJoCo not installed: pip install mujoco")

# ─────────────────────────────────────────────────────────────────────────────
# Physical Constants & Rotor Config
# ─────────────────────────────────────────────────────────────────────────────
SKYDIO_MASS       = 1.325         # kg
GRAVITY           = 9.81
HOVER_THRUST_EACH = SKYDIO_MASS * GRAVITY / 4.0   # ≈ 3.2496 N/rotor
CTRL_MAX          = 13.0          # N max per motor

CAM_W, CAM_H      = 640, 480      # drone camera feed resolution

# State HUD overlay colors
STATE_COLORS = {
    "IDLE":            (160, 160, 160),
    "ARMING":          (0,   200, 255),
    "TAKEOFF":         (0,   255, 200),
    "SEARCH":          (0,   220,  80),
    "LOITER":          (0,   165, 255),
    "APPROACH_TARGET": (30,  180, 255),
    "DROP_PAYLOAD":    (0,   100, 255),
    "RETURN_HOME":     (200,  80, 255),
    "LAND":            (80,  255,  80),
    "ABORT":           (0,    50, 255),
    "COMPLETE":        (0,   255,   0),
}


# ─────────────────────────────────────────────────────────────────────────────
# Rock-Solid Skydio X2 Controller (500 Hz)
# ─────────────────────────────────────────────────────────────────────────────
class SkydioX2Controller:
    """
    High-precision PD attitude + position controller for Skydio X2.
    Acts on motor array:
      t1: Rear-Right  (x=-.14, y=-.18)
      t2: Rear-Left   (x=-.14, y=+.18)
      t3: Front-Left  (x=+.14, y=+.18)
      t4: Front-Right (x=+.14, y=-.18)
    """

    def compute(
        self,
        pos:    np.ndarray,   # world [x, y, z]
        vel:    np.ndarray,   # world velocity
        quat:   np.ndarray,   # [w, x, y, z]
        gyro:   np.ndarray,   # body angular rates
        target: np.ndarray,   # desired setpoint [x, y, z]
    ) -> np.ndarray:
        # Altitude PID
        err_z = target[2] - pos[2]
        thrust_total = HOVER_THRUST_EACH * 4 + 12.0 * err_z - 6.0 * vel[2]
        thrust_total = np.clip(thrust_total, 0.0, 4 * CTRL_MAX)

        # Orientation
        w, x, y, z = quat
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = math.asin(np.clip(2*(w*y - z*x), -1.0, 1.0))

        err_x, err_y = target[0] - pos[0], target[1] - pos[1]
        des_pitch = np.clip( 0.8 * err_x - 0.6 * vel[0], -0.25, 0.25)
        des_roll  = np.clip(-0.8 * err_y + 0.6 * vel[1], -0.25, 0.25)

        r_cmd = 3.5 * (des_roll - roll)   - 0.8 * gyro[0]
        p_cmd = 3.5 * (des_pitch - pitch) - 0.8 * gyro[1]
        y_cmd = -0.6 * gyro[2]

        base = thrust_total / 4.0
        t1 = np.clip(base + p_cmd - r_cmd - y_cmd, 0.0, CTRL_MAX) # Rear-Right
        t2 = np.clip(base + p_cmd + r_cmd + y_cmd, 0.0, CTRL_MAX) # Rear-Left
        t3 = np.clip(base - p_cmd + r_cmd - y_cmd, 0.0, CTRL_MAX) # Front-Left
        t4 = np.clip(base - p_cmd - r_cmd + y_cmd, 0.0, CTRL_MAX) # Front-Right

        return np.array([t1, t2, t3, t4])


# ─────────────────────────────────────────────────────────────────────────────
# Camera HUD Drawing
# ─────────────────────────────────────────────────────────────────────────────
def draw_cam_hud(
    frame: np.ndarray,
    state: str,
    gemma: dict,
    pos: np.ndarray,
    target: np.ndarray,
    fps: float,
    gemma_ms: float,
    sim_time: float,
) -> np.ndarray:
    """Draw telemetry & mission state HUD on camera feed."""
    h, w = frame.shape[:2]
    dist_target = np.linalg.norm(target[:2] - pos[:2])

    # Top banner
    color = STATE_COLORS.get(state, (200, 200, 200))
    cv2.rectangle(frame, (0, 0), (w, 36), (20, 20, 20), -1)
    cv2.circle(frame, (20, 18), 8, color, -1)
    cv2.putText(frame, f"STATE: {state}", (36, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    # Telemetry
    tele = f"Alt: {pos[2]:4.1f}m | TargetDist: {dist_target:5.1f}m | t: {sim_time:.1f}s | FPS: {fps:.0f}"
    cv2.putText(frame, tele, (w - 420, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    # Gemma Panel
    if gemma:
        rec = gemma.get("mission_recommendation", {})
        action = rec.get("action", "N/A")
        desc   = gemma.get("scene_description", "")
        obs    = gemma.get("obstacles", {})

        cv2.rectangle(frame, (10, h - 110), (w - 10, h - 10), (10, 10, 10), -1)
        cv2.rectangle(frame, (10, h - 110), (w - 10, h - 10), (0, 200, 255), 1)

        rows = [
            (f"Gemma 4 e4b Reasoning ({gemma_ms:.0f}ms): {action.upper()}", (0, 255, 200), True),
            (f"Scene: {desc[:60]}...", (220, 220, 220), False),
            (f"Obstacles: {obs.get('summary','Clear')} | Density: {obs.get('density',0):.1f}", (200, 200, 100), False),
        ]
        for i, (txt, c, bold) in enumerate(rows):
            cv2.putText(frame, txt, (18, h - 85 + i * 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, c, 2 if bold else 1)

    # Controls footer
    cv2.putText(frame, "[S]Start [D/1]500m Target [2]1km Target [L/3]Land [C]Cam [R]Reset [Q]Quit",
                (10, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 120, 120), 1)

    return frame


# ─────────────────────────────────────────────────────────────────────────────
# Skydio X2 Native MuJoCo Simulation Class (1km Multi-Target)
# ─────────────────────────────────────────────────────────────────────────────
class SkydioX2Simulation:

    # Mission waypoints [x, y, z] — 1 KM Multi-Target Scale
    SEARCH_WPS   = [[200.0, 50.0, 15.0], [500.0, 100.0, 15.0], [750.0, -50.0, 15.0], [1000.0, -50.0, 15.0]]
    HOME_POS     = [   0.0,    0.0,  8.0]
    DROP_500M    = [ 500.0,  100.0,  3.5]    # Target 1: 500m Drop Bullseye
    TARGET_1KM   = [1000.0,  -50.0,  3.5]    # Target 2: 1km Extended Target
    LAND_POS     = [   0.0,    0.0,  0.08]   # Base Station White H-Marker

    def __init__(self, yolo_model="yolov8n.pt", gemma_model="gemma4:e4b",
                 gemma_interval=40, ollama_url="http://localhost:11434"):

        banner = "─" * 60
        print(f"\n\033[1m\033[96m{banner}")
        print("  IUB Drone  ×  Skydio X2  ×  1 KM Multi-Target MuJoCo Suite")
        print(f"{banner}\033[0m\n")

        # ── MuJoCo Model Load ─────────────────────────────────────────
        print("\033[93m[1/4] Loading 1km Skydio X2 MuJoCo model...\033[0m")
        cwd = os.getcwd()
        os.chdir(os.path.join(ROOT, "mujoco_sim"))
        try:
            self.model = mujoco.MjModel.from_xml_path("skydio_x2_mission.xml")
            self.model.opt.timestep = 0.002   # 500 Hz physics step
            self.data  = mujoco.MjData(self.model)
        finally:
            os.chdir(cwd)

        self._x2_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "x2")

        # Offscreen renderer for drone downward camera
        self.drone_renderer = mujoco.Renderer(self.model, CAM_H, CAM_W)
        self._down_cam = mujoco.MjvCamera()
        self._down_cam.type        = mujoco.mjtCamera.mjCAMERA_TRACKING
        self._down_cam.trackbodyid = self._x2_body_id
        self._down_cam.distance    = 0.01
        self._down_cam.elevation   = -90.0
        self._down_cam.azimuth     = 90.0

        print(f"\033[92m✅ Skydio X2 loaded: {self.model.nbody} bodies, {self.model.nu} actuators\033[0m")

        # ── YOLO ──────────────────────────────────────────────────────
        print("\033[93m[2/4] Loading YOLOv8...\033[0m")
        self.detector = YOLODetector(
            model_path=yolo_model, conf_threshold=0.35, device="mps")
        print("\033[92m✅ YOLO ready\033[0m")

        # ── Gemma ─────────────────────────────────────────────────────
        print(f"\033[93m[3/4] Connecting to Gemma ({gemma_model})...\033[0m")
        self.analyzer = GemmaAnalyzer(
            model=gemma_model, ollama_url=ollama_url,
            timeout=60.0, jpeg_quality=30,
            inference_width=160, inference_height=120)
        self.gemma_interval = gemma_interval
        print("\033[92m✅ Gemma ready\033[0m")

        # ── Mission SM ────────────────────────────────────────────────
        print("\033[93m[4/4] Initialising mission planner...\033[0m")
        self.sm = MissionStateMachine(
            mission_type="search_and_drop",
            on_state_change=self._on_state_change,
            loiter_confirm_frames=3)
        self.ctrl = SkydioX2Controller()
        print("\033[92m✅ Mission planner + controller ready\033[0m\n")

        self._frame_n   = 0
        self._target    = np.array(self.HOME_POS, dtype=float)
        self._active_setpoint = np.array(self.HOME_POS, dtype=float)
        self._max_speed = 18.0  # 18 m/s max trajectory glide speed for 1km transit
        self._wp_idx    = 0
        self._gemma_res = None
        self._gemma_ms  = 0.0
        self._fps_times = []
        self._dropped   = False
        self._should_quit = False
        self._camera_mode = 0   # 0=Chase Cam, 1=Downward Vision, 2=Target Spotter

        self._reset()

    # ── Reset ─────────────────────────────────────────────────────────
    def _reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3]  = [0.0, 0.0, 5.0]   # Hover start
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)
        self._target  = np.array(self.HOME_POS, dtype=float)
        self._active_setpoint = np.array(self.HOME_POS, dtype=float)
        self._dropped = False
        log.info("Skydio X2 simulation reset")

    # ── Sensors ───────────────────────────────────────────────────────
    def _sensors(self):
        gyro = self.data.sensordata[:3].copy() if self.data.sensordata.size >= 3 else np.zeros(3)
        quat = self.data.qpos[3:7].copy()
        pos  = self.data.qpos[:3].copy()
        vel  = self.data.qvel[:3].copy()
        return pos, vel, quat, gyro

    # ── Smooth trajectory glide step (18 m/s max) ─────────────────────
    def _smooth_trajectory_step(self, dt=0.002):
        diff = self._target - self._active_setpoint
        dist = np.linalg.norm(diff)
        if dist > 0.001:
            step_size = min(dist, self._max_speed * dt)
            self._active_setpoint += (diff / dist) * step_size
        else:
            self._active_setpoint = self._target.copy()

    # ── Physics Step (500 Hz) ─────────────────────────────────────────
    def _step(self):
        self._smooth_trajectory_step(dt=0.002)
        pos, vel, quat, gyro = self._sensors()
        thrusts = self.ctrl.compute(pos, vel, quat, gyro, self._active_setpoint)
        self.data.ctrl[:] = thrusts
        mujoco.mj_step(self.model, self.data)

    # ── Render downward camera ────────────────────────────────────────
    def _render_down(self) -> np.ndarray:
        self.drone_renderer.update_scene(self.data, camera=self._down_cam)
        return cv2.cvtColor(self.drone_renderer.render(), cv2.COLOR_RGB2BGR)

    # ── Keyboard Command Event Dispatcher ─────────────────────────────
    def process_command_key(self, key_str: str):
        ch = key_str.lower()
        if ch == "s":
            print(f"\n\033[92m▶ [COMMAND] START MISSION (TAKEOFF → SEARCH 1KM)\033[0m")
            self._dropped = False
            self.sm._transition(MissionState.ARMING, "start command received")
            self.sm.on_armed()
            self.sm.on_altitude_reached()
        elif ch in ("d", "1"):
            print(f"\n\033[93m📦 [COMMAND] FLY TO TARGET 1: 500-METER DROP ZONE (500m, 100m, 3.5m)\033[0m")
            self._dropped = False
            if self.sm.state == "IDLE":
                self.sm.on_start_command()
                self.sm.on_armed()
            self.sm._transition(MissionState.APPROACH_TARGET, "fly to 500m drop target command")
            self._target = np.array(self.DROP_500M)
        elif ch == "2":
            print(f"\n\033[93m🎯 [COMMAND] FLY TO TARGET 2: 1-KILOMETER EXTENDED BASE (1000m, -50m, 3.5m)\033[0m")
            self._dropped = False
            if self.sm.state == "IDLE":
                self.sm.on_start_command()
                self.sm.on_armed()
            self.sm._transition(MissionState.APPROACH_TARGET, "fly to 1km extended target command")
            self._target = np.array(self.TARGET_1KM)
        elif ch in ("l", "3"):
            print(f"\n\033[92m⬇ [COMMAND] RETURN HOME & LAND ON WHITE H-MARKER (0m, 0m, 0.08m)\033[0m")
            if self.sm.state == "IDLE":
                self.sm.on_start_command()
                self.sm.on_armed()
            self.sm._transition(MissionState.LAND, "land command received")
            self._target = np.array(self.LAND_POS)
        elif ch == "a":
            print(f"\n\033[91m⛔ [COMMAND] ABORT / HOLD POSITION\033[0m")
            self.sm.on_abort_command()
            pos = self.data.qpos[:3].copy()
            self._target = np.array([pos[0], pos[1], max(pos[2], 2.0)])
        elif ch == "c":
            self._camera_mode = (self._camera_mode + 1) % 3
            modes = ["CHASE CAM (Follows Drone across 1km)", "ONBOARD DOWNWARD VISION", "SPOTTER CAM (Target 1/2 View)"]
            print(f"\n\033[96m📷 [CAMERA MODE] {modes[self._camera_mode]}\033[0m")
        elif ch == "r":
            print(f"\n\033[93m🔄 [COMMAND] RESET SIMULATION\033[0m")
            self._reset()
            self.sm = MissionStateMachine(
                mission_type="search_and_drop",
                on_state_change=self._on_state_change,
                loiter_confirm_frames=3)
        elif ch == "q":
            print(f"\n\033[91m🚪 [COMMAND] QUIT SIMULATION\033[0m")
            self._should_quit = True

    def _on_mujoco_key(self, keycode: int):
        try:
            self.process_command_key(chr(keycode))
        except (ValueError, OverflowError):
            pass

    def _check_terminal_key(self):
        try:
            fd = sys.stdin.fileno()
            if select.select([sys.stdin], [], [], 0.0001)[0]:
                ch = sys.stdin.read(1)
                if ch:
                    self.process_command_key(ch)
        except Exception:
            pass

    def _on_state_change(self, old: str, new: str):
        log.info(f"\033[1m{'='*50}\033[0m")
        log.info(f"\033[1m  SKYDIO X2: {old} → {new}\033[0m")
        log.info(f"\033[1m{'='*50}\033[0m")

    # ── Search waypoints ──────────────────────────────────────────────
    def _advance_wp(self, pos):
        if self.sm.state != "SEARCH":
            return
        wp = np.array(self.SEARCH_WPS[self._wp_idx])
        if np.linalg.norm(pos[:2] - wp[:2]) < 5.0:
            self._wp_idx = (self._wp_idx + 1) % len(self.SEARCH_WPS)
            self._target = np.array(self.SEARCH_WPS[self._wp_idx])
            log.info(f"Search WP {self._wp_idx}: {self._target}")

    # ── Auto altitude transitions ─────────────────────────────────────
    def _auto_transitions(self, pos):
        s = self.sm.state
        if s == "TAKEOFF" and pos[2] >= 8.0:
            self.sm.on_altitude_reached()

        elif s in ("APPROACH_TARGET", "DROP_PAYLOAD") and not self._dropped:
            dist_to_drop = np.linalg.norm(pos[:2] - self._target[:2])
            if dist_to_drop < 2.0 and pos[2] <= 4.2:
                print(f"\n\033[93m📦 PAYLOAD DELIVERED AT TARGET POSITION! ({pos[0]:.1f}m, {pos[1]:.1f}m, {pos[2]:.2f}m)\033[0m")
                self._dropped = True
                self.sm.on_payload_dropped()
                self.sm._transition(MissionState.RETURN_HOME, "payload dropped return home")
                self._target = np.array(self.HOME_POS)

        elif s == "LAND" and pos[2] <= 0.15:
            log.info("Landed safely on H-marker!")
            self.sm.on_landed()

        elif s == "RETURN_HOME":
            if np.linalg.norm(pos[:2]) < 2.0:
                self.sm.on_at_home()

    # ── Vision → Mission ──────────────────────────────────────────────
    def _vision_to_sm(self, res):
        if not res:
            return
        lz = res.get("landing_zone", {})
        dz = res.get("drop_zone", {})
        ob = res.get("obstacles", {})
        self.sm.on_vision_update(
            scene_analysis=res,
            landing_zone={"zone_detected": lz.get("detected", False),
                          "safety_assessment": lz.get("safety", "caution"),
                          "gemma_confidence": lz.get("clearance_score", 0.0),
                          "clearance_score": lz.get("clearance_score", 0.0)},
            drop_zone={"zone_detected": dz.get("detected", False),
                       "safety_assessment": "safe" if dz.get("confidence", 0) > 0.5 else "caution",
                       "gemma_confidence": dz.get("confidence", 0.0),
                       "area_ratio": 0.06 if dz.get("detected") else 0.0},
            obstacles={"density": ob.get("density", 0.0),
                       "center_clear": ob.get("center_clear", True)},
            battery_pct=100.0)

    # ── FPS counter ───────────────────────────────────────────────────
    def _fps(self):
        now = time.time()
        self._fps_times = [t for t in self._fps_times if now - t < 1.0]
        self._fps_times.append(now)
        return float(len(self._fps_times))

    # ─────────────────────────────────────────────────────────────────
    # Native MuJoCo Viewer Main Loop (Dual Windows + Zero-Crash Cam)
    # ─────────────────────────────────────────────────────────────────
    def run(self):
        print("\033[1mControls (Press S, D/1, 2, L/3, A, C, R, Q in 3D Window OR Terminal):\033[0m")
        for k, v in [("S", "Start mission (TAKEOFF -> SEARCH 1KM)"),
                    ("D / 1", "Fly to Target 1: 500m Drop Zone [500m, 100m, 3.5m]"),
                    ("2", "Fly to Target 2: 1km Extended Target [1000m, -50m, 3.5m]"),
                    ("L / 3", "Return Home & Land [0m, 0m, 0.08m]"),
                    ("A", "Abort / hold position"),
                    ("C", "Toggle Camera View (Chase Cam <-> Onboard Downward Vision <-> Spotter Cam)"),
                    ("R", "Reset"), ("Q", "Quit")]:
            print(f"  \033[96m{k:<6}\033[0m → {v}")
        print()

        has_cv2_gui = True
        try:
            cv2.namedWindow("Skydio X2 | Drone Camera Feed", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Skydio X2 | Drone Camera Feed", CAM_W, CAM_H)
        except Exception:
            has_cv2_gui = False

        # Launch native 3D MuJoCo Viewer with key_callback
        print("\033[93mLaunching Native MuJoCo Interactive 3D Viewer...\033[0m")

        # Set up non-blocking terminal input
        old_termios = None
        try:
            fd = sys.stdin.fileno()
            old_termios = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except Exception:
            pass

        try:
            with mujoco.viewer.launch_passive(
                self.model, self.data, key_callback=self._on_mujoco_key
            ) as viewer:
                print("\033[92m✅ Native MuJoCo Interactive 3D Viewer is open!\033[0m\n")

                PHYS_STEPS_PER_RENDER = 10

                while viewer.is_running() and not self._should_quit:
                    self._frame_n += 1

                    # Check terminal non-blocking keys
                    self._check_terminal_key()

                    # 500 Hz Physics Step
                    for _ in range(PHYS_STEPS_PER_RENDER):
                        self._step()

                    pos, vel, quat, gyro = self._sensors()
                    self._advance_wp(pos)
                    self._auto_transitions(pos)
                    sim_t = self.data.time

                    # ── Dynamic Camera Modes (mjCAMERA_FREE prevents any fixedcamid crash!) ─
                    if self._camera_mode == 0:
                        # Dynamic Chase Cam: Follows quadcopter 14m behind & elevated 5m
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        viewer.cam.trackbodyid = self._x2_body_id
                        viewer.cam.distance = 14.0
                        viewer.cam.elevation = -18.0
                        viewer.cam.azimuth = 90.0
                    elif self._camera_mode == 1:
                        # Downward Onboard Vision Cam
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        viewer.cam.trackbodyid = self._x2_body_id
                        viewer.cam.distance = 0.01
                        viewer.cam.elevation = -90.0
                        viewer.cam.azimuth = 90.0
                    elif self._camera_mode == 2:
                        # 1km Target Spotter Cam (FREE camera focused on active target)
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.lookat[:] = [float(self._target[0]), float(self._target[1]), 2.0]
                        viewer.cam.distance = 25.0
                        viewer.cam.elevation = -25.0
                        viewer.cam.azimuth = 45.0

                    # Sync Native 3D Viewer
                    viewer.sync()

                    # Downward Drone Camera -> YOLO + Gemma
                    down_frame = self._render_down()
                    detections, fps, annotated, _ = self.detector.detect(down_frame)

                    if self._frame_n % self.gemma_interval == 0:
                        self.analyzer.analyze_async(annotated, [])

                    result, _, gms = self.analyzer.get_latest_result()
                    if result is not None:
                        self._gemma_res = result
                        self._gemma_ms  = gms
                        self._vision_to_sm(result)

                    # Draw Camera HUD Overlay Window
                    cam_hud = draw_cam_hud(
                        annotated, self.sm.state, self._gemma_res,
                        pos, self._target, self._fps(), self._gemma_ms, sim_t)

                    if has_cv2_gui:
                        try:
                            cv2.imshow("Skydio X2 | Drone Camera Feed", cam_hud)
                            key = cv2.waitKey(1) & 0xFF
                            if key in (ord("q"), 27):
                                break
                            elif key in (ord("s"), ord("d"), ord("l"), ord("a"), ord("r"), ord("c"), ord("1"), ord("2"), ord("3")):
                                self.process_command_key(chr(key))
                        except Exception:
                            has_cv2_gui = False

                    # Status Telemetry
                    dist_to_target = np.linalg.norm(self._target[:2] - pos[:2])
                    if self._frame_n % 30 == 0:
                        print(
                            f"\r\033[96m[t={sim_t:7.2f}s]\033[0m"
                            f" {self.sm.state:<16}"
                            f" Pos=({pos[0]:5.1f}m, {pos[1]:5.1f}m, {pos[2]:4.1f}m)"
                            f" TargetDist={dist_to_target:5.1f}m"
                            f" Speed={np.linalg.norm(vel):4.1f}m/s"
                            f" FPS={fps:.0f}",
                            end="", flush=True)

        finally:
            if old_termios is not None:
                try:
                    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_termios)
                except Exception:
                    pass

        if has_cv2_gui:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

        print(f"\n\033[92m✅ Simulation finished. Final state: {self.sm.state}\033[0m")


def main():
    parser = argparse.ArgumentParser(description="Skydio X2 Native MuJoCo Simulation 1km Multi-Target")
    parser.add_argument("--yolo", default="yolov8n.pt", help="Path to YOLO weights")
    parser.add_argument("--gemma", default="gemma4:e4b", help="Ollama Gemma model name")
    parser.add_argument("--interval", type=int, default=40, help="Gemma frame interval")
    args = parser.parse_args()

    sim = SkydioX2Simulation(
        yolo_model=args.yolo,
        gemma_model=args.gemma,
        gemma_interval=args.interval,
    )
    sim.run()


if __name__ == "__main__":
    main()
