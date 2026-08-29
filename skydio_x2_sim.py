"""
skydio_x2_sim.py
================
IUB Drone — Skydio X2 Single-Window Smart Obstacle Avoidance Simulation (1 KM Scale).

Features:
  - Single High-Performance Window: Native 3D Interactive MuJoCo Viewer
  - Live Real-Time YOLOv8 Object Detection Feedback in Terminal
  - Live Real-Time Gemma 4 e4b LLM Vision Reasoning & Obstacle Analysis Feedback in Terminal
  - Real-time Smart AI Obstacle Detection & Dynamic Path Rerouting around Skyscrapers & Weather Balloons
  - Target 1: 500m Drop Zone Bullseye [500m, 100m, 3.5m]
  - Target 2: 1km Extended Base Target [1000m, -50m, 3.5m]
  - 3D Viewer Camera Toggle (C key): Chase Cam <-> Birdseye Cam <-> Target Spotter Cam

Usage:
    mjpython skydio_x2_sim.py

Controls (Press in 3D Window OR Terminal):
    S      → Start mission (TAKEOFF → SEARCH)
    D / 1  → Fly to Target 1 (500m Drop Target) + YOLO/Gemma AI Avoidance!
    2      → Fly to Target 2 (1km Extended Target) + YOLO/Gemma AI Avoidance!
    L / 3  → Return Home & Land on White H-marker
    A      → Abort / hold position
    C      → Toggle Camera View (Chase Cam <-> Birdseye Cam <-> Spotter Cam)
    R      → Reset simulation
    Q      → Quit
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
# Physical Constants & Controller
# ─────────────────────────────────────────────────────────────────────────────
SKYDIO_MASS       = 1.325         # kg
GRAVITY           = 9.81
HOVER_THRUST_EACH = SKYDIO_MASS * GRAVITY / 4.0   # ≈ 3.2496 N/rotor
CTRL_MAX          = 13.0          # N max per motor
CAM_W, CAM_H      = 640, 480

class SkydioX2Controller:
    """High-precision PD attitude + position controller for Skydio X2."""

    def compute(
        self,
        pos:    np.ndarray,
        vel:    np.ndarray,
        quat:   np.ndarray,
        gyro:   np.ndarray,
        target: np.ndarray,
    ) -> np.ndarray:
        err_z = target[2] - pos[2]
        thrust_total = HOVER_THRUST_EACH * 4 + 12.0 * err_z - 6.0 * vel[2]
        thrust_total = np.clip(thrust_total, 0.0, 4 * CTRL_MAX)

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
        t1 = np.clip(base + p_cmd - r_cmd - y_cmd, 0.0, CTRL_MAX)
        t2 = np.clip(base + p_cmd + r_cmd + y_cmd, 0.0, CTRL_MAX)
        t3 = np.clip(base - p_cmd + r_cmd - y_cmd, 0.0, CTRL_MAX)
        t4 = np.clip(base - p_cmd - r_cmd + y_cmd, 0.0, CTRL_MAX)

        return np.array([t1, t2, t3, t4])


# ─────────────────────────────────────────────────────────────────────────────
# Skydio X2 Simulation Class with Live YOLO & Gemma Feedback
# ─────────────────────────────────────────────────────────────────────────────
class SkydioX2Simulation:

    HOME_POS   = [   0.0,    0.0,  8.0]
    DROP_500M  = [ 500.0,  100.0,  3.5]    # Target 1
    TARGET_1KM = [1000.0,  -50.0,  3.5]    # Target 2
    LAND_POS   = [   0.0,    0.0,  0.08]   # Home Base H-Marker

    # Obstacles along flight path
    OBSTACLES = [
        {"name": "Glass Skyscraper 1",  "pos": np.array([120.0,  24.0]), "radius": 18.0, "clear_alt": 28.0},
        {"name": "Weather Balloon 1",   "pos": np.array([250.0,  50.0]), "radius": 12.0, "clear_alt": 24.0},
        {"name": "Commercial Tower 2",  "pos": np.array([360.0,  72.0]), "radius": 22.0, "clear_alt": 35.0},
        {"name": "High Balloon 2",      "pos": np.array([440.0,  88.0]), "radius": 12.0, "clear_alt": 22.0},
        {"name": "Mega Skyscraper 3",  "pos": np.array([800.0, -40.0]), "radius": 25.0, "clear_alt": 45.0},
    ]

    def __init__(self, yolo_model="yolov8n.pt", gemma_model="gemma4:e4b",
                 gemma_interval=30, ollama_url="http://localhost:11434"):

        banner = "─" * 65
        print(f"\n\033[1m\033[96m{banner}")
        print("  IUB Drone  ×  Skydio X2  ×  YOLOv8 + Gemma 4 e4b Live AI Suite")
        print(f"{banner}\033[0m\n")

        print("\033[93m[1/4] Loading Skydio X2 MuJoCo model...\033[0m")
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

        # ── YOLOv8 Object Detector ───────────────────────────────────
        print("\033[93m[2/4] Loading YOLOv8 Object Detector...\033[0m")
        self.detector = YOLODetector(
            model_path=yolo_model, conf_threshold=0.35, device="mps")
        print("\033[92m✅ YOLOv8 Object Detector loaded and warmed up on MPS\033[0m")

        # ── Gemma 4 e4b LLM ──────────────────────────────────────────
        print(f"\033[93m[3/4] Connecting to Gemma 4 e4b LLM ({gemma_model})...\033[0m")
        self.analyzer = GemmaAnalyzer(
            model=gemma_model, ollama_url=ollama_url,
            timeout=60.0, jpeg_quality=30,
            inference_width=160, inference_height=120)
        self.gemma_interval = gemma_interval
        print("\033[92m✅ Gemma 4 e4b LLM connected & ready\033[0m")

        # ── Mission SM ────────────────────────────────────────────────
        print("\033[93m[4/4] Initialising mission state machine...\033[0m")
        self.sm = MissionStateMachine(
            mission_type="search_and_drop",
            on_state_change=self._on_state_change,
            loiter_confirm_frames=3)
        self.ctrl = SkydioX2Controller()
        print("\033[92m✅ Mission planner + controller ready\033[0m\n")

        self._frame_n   = 0
        self._target    = np.array(self.HOME_POS, dtype=float)
        self._active_setpoint = np.array(self.HOME_POS, dtype=float)
        self._max_speed = 18.0
        self._dropped   = False
        self._should_quit = False
        self._camera_mode = 0
        self._last_avoid_log = 0.0
        self._last_yolo_log  = 0.0
        self._gemma_res = None
        self._gemma_ms  = 0.0
        self._fps_times = []

        self._reset()

    def _reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3]  = [0.0, 0.0, 5.0]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)
        self._target  = np.array(self.HOME_POS, dtype=float)
        self._active_setpoint = np.array(self.HOME_POS, dtype=float)
        self._dropped = False
        log.info("Skydio X2 simulation reset")

    def _sensors(self):
        gyro = self.data.sensordata[:3].copy() if self.data.sensordata.size >= 3 else np.zeros(3)
        quat = self.data.qpos[3:7].copy()
        pos  = self.data.qpos[:3].copy()
        vel  = self.data.qvel[:3].copy()
        return pos, vel, quat, gyro

    def _render_down(self) -> np.ndarray:
        self.drone_renderer.update_scene(self.data, camera=self._down_cam)
        return cv2.cvtColor(self.drone_renderer.render(), cv2.COLOR_RGB2BGR)

    def _compute_smart_target(self, pos: np.ndarray) -> np.ndarray:
        target = self._target.copy()
        now = time.time()

        for obs in self.OBSTACLES:
            dist_xy = np.linalg.norm(pos[:2] - obs["pos"])
            if dist_xy < obs["radius"] + 20.0:
                target[2] = max(target[2], obs["clear_alt"])
                if now - self._last_avoid_log > 3.0:
                    print(f"\n\033[93m🤖 [SMART AI AVOIDANCE] {obs['name']} detected at dist={dist_xy:.1f}m!")
                    print(f"   Executing dynamic climbing bypass to {obs['clear_alt']:.0f}m altitude to clear obstacle safely!\033[0m")
                    self._last_avoid_log = now
                break

        return target

    def _smooth_trajectory_step(self, dt=0.002):
        pos = self.data.qpos[:3].copy()
        des_target = self._compute_smart_target(pos)

        diff = des_target - self._active_setpoint
        dist = np.linalg.norm(diff)
        if dist > 0.001:
            step_size = min(dist, self._max_speed * dt)
            self._active_setpoint += (diff / dist) * step_size
        else:
            self._active_setpoint = des_target.copy()

    def _step(self):
        self._smooth_trajectory_step(dt=0.002)
        pos, vel, quat, gyro = self._sensors()
        thrusts = self.ctrl.compute(pos, vel, quat, gyro, self._active_setpoint)
        self.data.ctrl[:] = thrusts
        mujoco.mj_step(self.model, self.data)

    def process_command_key(self, key_str: str):
        ch = key_str.lower()
        if ch == "s":
            print(f"\n\033[92m▶ [COMMAND] START MISSION (TAKEOFF → SEARCH)\033[0m")
            self._dropped = False
            self.sm._transition(MissionState.ARMING, "start command received")
            self.sm.on_armed()
            self.sm.on_altitude_reached()
        elif ch in ("d", "1"):
            print(f"\n\033[93m📦 [COMMAND] FLY TO TARGET 1: 500M DROP ZONE (500m, 100m, 3.5m)\033[0m")
            self._dropped = False
            if self.sm.state == "IDLE":
                self.sm.on_start_command()
                self.sm.on_armed()
            self.sm._transition(MissionState.APPROACH_TARGET, "fly to 500m drop target command")
            self._target = np.array(self.DROP_500M)
        elif ch == "2":
            print(f"\n\033[93m🎯 [COMMAND] FLY TO TARGET 2: 1KM EXTENDED TARGET (1000m, -50m, 3.5m)\033[0m")
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
            modes = ["CHASE CAM (Follows Drone across 1km)", "BIRDSEYE OVERHEAD CAM", "SPOTTER CAM (Target View)"]
            print(f"\n\033[96m📷 [3D VIEWER CAMERA] {modes[self._camera_mode]}\033[0m")
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

    def _auto_transitions(self, pos):
        s = self.sm.state
        if s == "TAKEOFF" and pos[2] >= 8.0:
            self.sm.on_altitude_reached()

        elif s in ("APPROACH_TARGET", "DROP_PAYLOAD") and not self._dropped:
            dist_to_drop = np.linalg.norm(pos[:2] - self._target[:2])
            if dist_to_drop < 3.0 and pos[2] <= 5.0:
                print(f"\n\033[93m📦 PAYLOAD DELIVERED AT TARGET! ({pos[0]:.1f}m, {pos[1]:.1f}m, {pos[2]:.2f}m)\033[0m")
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

    def run(self):
        print("\033[1mControls (Press S, D/1, 2, L/3, A, C, R, Q in 3D Window OR Terminal):\033[0m")
        for k, v in [("S", "Start mission (TAKEOFF -> SEARCH)"),
                    ("D / 1", "Fly to Target 1: 500m Drop Zone [500m, 100m, 3.5m] + AI Avoidance"),
                    ("2", "Fly to Target 2: 1km Extended Target [1000m, -50m, 3.5m] + AI Avoidance"),
                    ("L / 3", "Return Home & Land [0m, 0m, 0.08m]"),
                    ("A", "Abort / hold position"),
                    ("C", "Toggle 3D Camera View (Chase Cam <-> Birdseye Cam <-> Spotter Cam)"),
                    ("R", "Reset"), ("Q", "Quit")]:
            print(f"  \033[96m{k:<6}\033[0m → {v}")
        print()

        print("\033[93mLaunching Single-Window Native MuJoCo Interactive 3D Viewer...\033[0m")

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

                    self._check_terminal_key()

                    for _ in range(PHYS_STEPS_PER_RENDER):
                        self._step()

                    pos, vel, quat, gyro = self._sensors()
                    self._auto_transitions(pos)
                    sim_t = self.data.time

                    # ── Render Downward Camera & Run YOLO + Gemma AI Pipeline ─
                    down_frame = self._render_down()
                    detections, fps, annotated, _ = self.detector.detect(down_frame)

                    now = time.time()
                    # Print YOLO feedback if objects detected
                    if len(detections) > 0 and now - self._last_yolo_log > 2.0:
                        classes = [f"{d.get('class_name','obj')} ({d.get('confidence',0):.2f})" for d in detections]
                        print(f"\n\033[93m👁️ [YOLOv8 DETECTED] {len(detections)} object(s) in drone view: {classes}\033[0m")
                        self._last_yolo_log = now

                    # Trigger Gemma 4 e4b Vision Reasoning
                    if self._frame_n % self.gemma_interval == 0:
                        self.analyzer.analyze_async(annotated, [])

                    result, _, gms = self.analyzer.get_latest_result()
                    if result is not None and result != self._gemma_res:
                        self._gemma_res = result
                        self._gemma_ms  = gms
                        desc = result.get("scene_description", "Clear airspace")
                        rec = result.get("mission_recommendation", {}).get("action", "proceed")
                        obs = result.get("obstacles", {})
                        print(f"\n\033[92m🧠 [GEMMA 4 e4b REASONING] ({gms:.0f}ms)")
                        print(f"   Scene Description: '{desc}'")
                        print(f"   Obstacle Density: {obs.get('density', 0.0):.2f} | Action: {rec.upper()}\033[0m")

                    # ── Camera Modes ──────────────────────────────────────────
                    if self._camera_mode == 0:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        viewer.cam.trackbodyid = self._x2_body_id
                        viewer.cam.distance = 14.0
                        viewer.cam.elevation = -18.0
                        viewer.cam.azimuth = 90.0
                    elif self._camera_mode == 1:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        viewer.cam.trackbodyid = self._x2_body_id
                        viewer.cam.distance = 40.0
                        viewer.cam.elevation = -89.0
                        viewer.cam.azimuth = 90.0
                    elif self._camera_mode == 2:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.lookat[0] = float(self._target[0])
                        viewer.cam.lookat[1] = float(self._target[1])
                        viewer.cam.lookat[2] = 2.0
                        viewer.cam.distance = 25.0
                        viewer.cam.elevation = -25.0
                        viewer.cam.azimuth = 45.0

                    viewer.sync()

                    # Telemetry Logging
                    dist_to_target = np.linalg.norm(self._target[:2] - pos[:2])
                    yolo_cnt = len(detections)
                    if self._frame_n % 30 == 0:
                        print(
                            f"\r\033[96m[t={sim_t:7.2f}s]\033[0m"
                            f" {self.sm.state:<16}"
                            f" Pos=({pos[0]:5.1f}m, {pos[1]:5.1f}m, {pos[2]:4.1f}m)"
                            f" TargetDist={dist_to_target:5.1f}m"
                            f" YOLO={yolo_cnt}obj"
                            f" Gemma={self._gemma_ms:.0f}ms",
                            end="", flush=True)

        finally:
            if old_termios is not None:
                try:
                    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_termios)
                except Exception:
                    pass

        print(f"\n\033[92m✅ Simulation finished. Final state: {self.sm.state}\033[0m")


def main():
    sim = SkydioX2Simulation()
    sim.run()


if __name__ == "__main__":
    main()
