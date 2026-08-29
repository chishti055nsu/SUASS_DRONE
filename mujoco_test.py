"""
mujoco_test.py
==============
IUB Drone — MuJoCo Quadcopter Simulation

Integrates:
  • MuJoCo 3.x physics simulation (quadcopter + scene)
  • Downward-facing camera feed → YOLOv8 (Apple MPS)
  • Gemma 4 e4b via Ollama → structured JSON scene analysis
  • Mission State Machine (IDLE→SEARCH→DROP→LAND)
  • PD attitude + altitude controller

Layout:
  ┌──────────────────────┬──────────────────────┐
  │  MuJoCo 3D View      │  Drone Camera + HUD  │
  │  (chase cam)         │  (YOLO + Gemma)      │
  └──────────────────────┴──────────────────────┘

Usage:
    python3 mujoco_test.py

Controls:
    S  → Start mission (SEARCH)
    L  → Land now
    D  → Drop payload
    A  → Abort / Emergency
    R  → Reset simulation
    Q  → Quit
"""

import os, sys, time, math, threading, argparse
import logging
import numpy as np
import cv2

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "drone_vision"))
sys.path.insert(0, os.path.join(ROOT, "mission_planner"))

from drone_vision.yolo_detector   import YOLODetector
from drone_vision.gemma_analyzer  import GemmaAnalyzer
from mission_planner.mission_state_machine import MissionStateMachine

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("mujoco_test")

# ── MuJoCo ───────────────────────────────────────────────────────────────────
try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("MuJoCo not installed. Run: pip install mujoco")
    sys.exit(1)

# ── Constants ─────────────────────────────────────────────────────────────────
MODEL_PATH   = os.path.join(ROOT, "mujoco_sim", "quadcopter.xml")
GRAVITY      = 9.81
DRONE_MASS   = 1.10          # kg (body + rotors)
HOVER_THRUST = DRONE_MASS * GRAVITY / 4.0  # per rotor

CAM_W, CAM_H = 640, 480      # drone camera render resolution
VIEW_W, VIEW_H = 800, 600    # 3D chase-cam render resolution

# State colours for HUD
STATE_COLORS = {
    "IDLE":            (160, 160, 160),
    "ARMING":          (0, 200, 255),
    "TAKEOFF":         (0, 255, 200),
    "SEARCH":          (0, 220, 80),
    "LOITER":          (0, 165, 255),
    "APPROACH_TARGET": (30, 180, 255),
    "DROP_PAYLOAD":    (0, 100, 255),
    "RETURN_HOME":     (200, 80, 255),
    "LAND":            (80, 255, 80),
    "ABORT":           (0, 50, 255),
    "COMPLETE":        (0, 255, 0),
}


# ─────────────────────────────────────────────────────────────────────────────
# PD Quadcopter Controller
# ─────────────────────────────────────────────────────────────────────────────
class PDController:
    """
    Simple PD position + attitude controller.
    Converts [x_target, y_target, z_target] setpoint
    into 4 rotor thrust commands.
    """

    def __init__(self):
        # Position PD gains
        self.kp_xy  = 1.2;  self.kd_xy  = 1.5
        self.kp_z   = 3.0;  self.kd_z   = 2.5
        # Attitude PD gains
        self.kp_att = 4.0;  self.kd_att = 1.2

        self._prev_pos = np.zeros(3)
        self._prev_vel = np.zeros(3)
        self._prev_t   = time.time()

    def compute(
        self,
        pos: np.ndarray,    # current [x, y, z]
        vel: np.ndarray,    # current velocity
        quat: np.ndarray,   # current quaternion [w,x,y,z]
        gyro: np.ndarray,   # body angular rates
        target: np.ndarray, # desired [x, y, z]
    ) -> np.ndarray:        # returns [fl, fr, rl, rr] thrust (N)

        # ── Altitude control ──────────────────────────────────────────
        err_z  = target[2] - pos[2]
        err_vz = -vel[2]
        thrust_total = (HOVER_THRUST * 4
                        + self.kp_z * err_z
                        + self.kd_z * err_vz)
        thrust_total = np.clip(thrust_total, 0.0, 4 * HOVER_THRUST * 2.0)

        # ── Lateral position → desired roll/pitch ─────────────────────
        err_x = target[0] - pos[0]
        err_y = target[1] - pos[1]

        # Yaw-compensated lateral errors using current heading
        w, x, y, z = quat
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        cy, sy = math.cos(yaw), math.sin(yaw)

        err_fwd  =  cy * err_x + sy * err_y
        err_side = -sy * err_x + cy * err_y

        des_pitch = np.clip(self.kp_xy * err_fwd  - self.kd_xy * vel[0], -0.4, 0.4)
        des_roll  = np.clip(self.kp_xy * err_side - self.kd_xy * vel[1], -0.4, 0.4)

        # ── Current roll/pitch from quaternion ────────────────────────
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = math.asin(np.clip(2*(w*y - z*x), -1, 1))

        err_roll  = des_roll  - roll
        err_pitch = des_pitch - pitch

        roll_cmd  = self.kp_att * err_roll  - self.kd_att * gyro[0]
        pitch_cmd = self.kp_att * err_pitch - self.kd_att * gyro[1]
        yaw_cmd   = -self.kd_att * gyro[2]   # damp yaw

        # ── Mix to 4 rotors (X-frame) ─────────────────────────────────
        # FL(+pitch,+roll,-yaw), FR(+pitch,-roll,+yaw)
        # RL(-pitch,+roll,+yaw), RR(-pitch,-roll,-yaw)
        base = thrust_total / 4.0
        fl = base + pitch_cmd + roll_cmd - yaw_cmd
        fr = base + pitch_cmd - roll_cmd + yaw_cmd
        rl = base - pitch_cmd + roll_cmd + yaw_cmd
        rr = base - pitch_cmd - roll_cmd - yaw_cmd

        return np.clip([fl, fr, rl, rr], 0.0, HOVER_THRUST * 2.5)


# ─────────────────────────────────────────────────────────────────────────────
# HUD Drawing
# ─────────────────────────────────────────────────────────────────────────────
def draw_drone_cam_hud(
    frame: np.ndarray,
    state: str,
    gemma_result: dict,
    detections: list,
    pos: np.ndarray,
    fps: float,
    gemma_ms: float,
) -> np.ndarray:
    h, w = frame.shape[:2]

    # Top bar
    overlay = frame.copy()
    cv2.rectangle(overlay, (0,0), (w, 58), (10,10,20), -1)
    cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

    state_col = STATE_COLORS.get(state, (200,200,200))
    cv2.putText(frame, f"STATE: {state}",
        (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, state_col, 2, cv2.LINE_AA)

    # Altitude + FPS
    cv2.putText(frame, f"ALT: {pos[2]:.1f}m",
        (w-200, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,220,255), 2)
    cv2.putText(frame, f"FPS: {fps:.0f}  YOLO",
        (w-200, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180,180,180), 1)

    # Position info
    cv2.putText(frame, f"X:{pos[0]:.1f} Y:{pos[1]:.1f}",
        (12, h-12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180,180,180), 1)

    # Crosshair
    cx, cy = w//2, h//2
    cv2.line(frame, (cx-20,cy), (cx+20,cy), (0,255,80), 1)
    cv2.line(frame, (cx,cy-20), (cx,cy+20), (0,255,80), 1)
    cv2.circle(frame, (cx,cy), 30, (0,255,80), 1)

    # Gemma panel
    if gemma_result:
        px, py = w-310, h-215
        cv2.rectangle(frame, (px-4,py-4), (w-4,h-4), (10,10,30), -1)
        cv2.rectangle(frame, (px-4,py-4), (w-4,h-4), (50,50,100), 1)

        rec = gemma_result.get("mission_recommendation",{})
        lz  = gemma_result.get("landing_zone",{})
        dz  = gemma_result.get("drop_zone",{})
        obs = gemma_result.get("obstacles",{})

        s_col = {"safe":(0,220,80),"caution":(0,200,255),"unsafe":(0,50,255)}
        rows = [
            (f"GEMMA ({gemma_ms:.0f}ms)",              (255,200,50), True),
            (f"Action: {rec.get('action','?').upper()}",(255,220,50), False),
            (f"Land:   {lz.get('safety','?').upper()} {lz.get('clearance_score',0):.0%}",
             s_col.get(lz.get('safety','caution'),(200,200,200)), False),
            (f"Drop:   {'FOUND ✓' if dz.get('detected') else 'NOT FOUND'}",
             (0,220,80) if dz.get('detected') else (120,120,120), False),
            (f"Obs:    {int(obs.get('density',0)*100)}% density",
             (0,50,255) if obs.get('density',0)>0.5 else (0,200,80), False),
            (f"Dir:    {rec.get('direction','none')}",  (180,180,255), False),
        ]
        for i,(text,col,bold) in enumerate(rows):
            cv2.putText(frame, text,
                (px, py+22+i*30), cv2.FONT_HERSHEY_SIMPLEX,
                0.56, col, 2 if bold else 1, cv2.LINE_AA)

    # Controls hint
    cv2.putText(frame, "[S]Start [L]Land [D]Drop [A]Abort [R]Reset [Q]Quit",
        (6, h-2), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80,80,80), 1)

    return frame


def draw_3d_hud(frame: np.ndarray, state: str, pos: np.ndarray,
                target: np.ndarray, fps: float) -> np.ndarray:
    """Overlay on the 3D chase-cam view."""
    cv2.putText(frame, "MuJoCo 3D View",
        (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,100), 2)
    state_col = STATE_COLORS.get(state, (200,200,200))
    cv2.putText(frame, f"{state}",
        (10,52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_col, 2)
    cv2.putText(frame, f"Pos: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}m)",
        (10, frame.shape[0]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
    cv2.putText(frame, f"Tgt: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}m)",
        (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100,200,255), 1)
    return frame


# ─────────────────────────────────────────────────────────────────────────────
# MuJoCo Simulation
# ─────────────────────────────────────────────────────────────────────────────
class DroneSimulation:
    """
    Integrates MuJoCo physics with YOLO + Gemma + Mission State Machine.
    """

    # Mission waypoints [x, y, z]
    SEARCH_WAYPOINTS = [
        [ 5,  5, 12],
        [ 5, -5, 12],
        [-5, -5, 12],
        [-5,  5, 12],
        [ 5,  5, 12],
    ]
    HOME_POS  = [0.0, 0.0, 10.0]
    DROP_POS  = [8.0,  5.0, 4.0]   # above drop circle
    LAND_POS  = [0.0,  0.0, 0.15]  # above H-marker

    def __init__(self, yolo_model="yolov8n.pt", gemma_model="gemma4:e4b",
                 gemma_interval=40, ollama_url="http://localhost:11434"):

        print("\n\033[1m\033[96m" + "─"*50)
        print("  IUB Drone — MuJoCo Simulation")
        print("─"*50 + "\033[0m\n")

        # ── Load MuJoCo model ─────────────────────────────────────────
        print(f"\033[93m[1/4] Loading MuJoCo model...\033[0m")
        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data  = mujoco.MjData(self.model)
        # Renderers
        self.cam_renderer  = mujoco.Renderer(self.model, height=CAM_H,  width=CAM_W)
        self.view_renderer = mujoco.Renderer(self.model, height=VIEW_H, width=VIEW_W)
        # Camera IDs
        self.drone_cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "drone_cam")
        self.chase_cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "chase_cam")
        print(f"\033[92m✅ MuJoCo model loaded (drone_cam={self.drone_cam_id})\033[0m")

        # ── YOLO ─────────────────────────────────────────────────────
        print(f"\033[93m[2/4] Loading YOLOv8...\033[0m")
        self.detector = YOLODetector(
            model_path=yolo_model,
            conf_threshold=0.35,
            device="mps",
        )
        print(f"\033[92m✅ YOLO ready\033[0m")

        # ── Gemma ─────────────────────────────────────────────────────
        print(f"\033[93m[3/4] Connecting to Gemma ({gemma_model})...\033[0m")
        self.analyzer = GemmaAnalyzer(
            model=gemma_model,
            ollama_url=ollama_url,
            timeout=60.0,
            jpeg_quality=30,
            inference_width=160,
            inference_height=120,
        )
        self.gemma_interval = gemma_interval
        print(f"\033[92m✅ Gemma ready\033[0m")

        # ── Mission state machine ─────────────────────────────────────
        print(f"\033[93m[4/4] Initialising mission state machine...\033[0m")
        self.sm = MissionStateMachine(
            mission_type="search_and_drop",
            on_state_change=self._on_state_change,
            loiter_confirm_frames=3,
        )
        self.ctrl = PDController()
        print(f"\033[92m✅ Mission SM + PD controller ready\033[0m\n")

        # ── State ─────────────────────────────────────────────────────
        self._frame_count   = 0
        self._target_pos    = np.array(self.HOME_POS, dtype=float)
        self._wp_idx        = 0
        self._latest_gemma  = None
        self._latest_raw    = "{}"
        self._gemma_ms      = 0.0
        self._latest_dets   = []
        self._fps_times     = []
        self._payload_dropped = False
        self._sim_time      = 0.0
        self._running       = True

        # Reset drone to initial position
        self._reset()

    # ── Reset ─────────────────────────────────────────────────────────
    def _reset(self):
        mujoco.mj_resetData(self.model, self.data)
        # Place drone above H-marker at start altitude
        self.data.qpos[:3] = [0.0, 0.0, 8.0]
        self.data.qpos[3]  = 1.0  # quaternion w=1 (upright)
        mujoco.mj_forward(self.model, self.data)
        self._target_pos  = np.array(self.HOME_POS, dtype=float)
        self._wp_idx      = 0
        self._frame_count = 0
        self._payload_dropped = False
        log.info("Simulation reset")

    # ── State change callback ─────────────────────────────────────────
    def _on_state_change(self, old: str, new: str):
        log.info(f"\033[1m{'='*44}\033[0m")
        log.info(f"\033[1m  MISSION: {old} → {new}\033[0m")
        log.info(f"\033[1m{'='*44}\033[0m")

        # Update setpoint when state changes
        if new == "TAKEOFF":
            self._target_pos = np.array([0, 0, 12.0])
        elif new == "SEARCH":
            self._wp_idx = 0
            self._target_pos = np.array(self.SEARCH_WAYPOINTS[0])
        elif new == "APPROACH_TARGET":
            self._target_pos = np.array([self.DROP_POS[0], self.DROP_POS[1], 10.0])
        elif new == "DROP_PAYLOAD":
            self._target_pos = np.array(self.DROP_POS)
        elif new == "RETURN_HOME":
            self._target_pos = np.array(self.HOME_POS)
        elif new == "LAND":
            self._target_pos = np.array(self.LAND_POS)
        elif new == "ABORT":
            # Hold current position
            pos = self.data.qpos[:3].copy()
            self._target_pos = np.array([pos[0], pos[1], max(pos[2], 3.0)])

    # ── Get sensor data ───────────────────────────────────────────────
    def _get_state(self):
        pos  = self.data.qpos[:3].copy()
        vel  = self.data.qvel[:3].copy()
        quat = self.data.qpos[3:7].copy()   # [w,x,y,z]
        gyro = self.data.sensordata[:3].copy() if self.data.sensordata.size >= 3 else np.zeros(3)
        return pos, vel, quat, gyro

    # ── Physics step ──────────────────────────────────────────────────
    def _physics_step(self):
        pos, vel, quat, gyro = self._get_state()
        thrusts = self.ctrl.compute(pos, vel, quat, gyro, self._target_pos)
        self.data.ctrl[:4] = thrusts
        mujoco.mj_step(self.model, self.data)

    # ── Render camera frame ───────────────────────────────────────────
    def _render_drone_cam(self) -> np.ndarray:
        self.cam_renderer.update_scene(self.data, camera=self.drone_cam_id)
        rgb = self.cam_renderer.render()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def _render_chase_cam(self) -> np.ndarray:
        self.view_renderer.update_scene(self.data, camera=self.chase_cam_id)
        rgb = self.view_renderer.render()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    # ── Search waypoint advancement ───────────────────────────────────
    def _advance_search_waypoint(self, pos: np.ndarray):
        if self.sm.state != "SEARCH":
            return
        wp = np.array(self.SEARCH_WAYPOINTS[self._wp_idx])
        dist = np.linalg.norm(pos[:2] - wp[:2])
        if dist < 2.0:
            self._wp_idx = (self._wp_idx + 1) % len(self.SEARCH_WAYPOINTS)
            self._target_pos = np.array(self.SEARCH_WAYPOINTS[self._wp_idx])
            log.info(f"Search WP {self._wp_idx}: {self._target_pos}")

    # ── Altitude-triggered transitions ────────────────────────────────
    def _check_auto_transitions(self, pos: np.ndarray):
        state = self.sm.state
        if state == "TAKEOFF" and pos[2] >= 10.0:
            self.sm.on_altitude_reached()
        elif state == "DROP_PAYLOAD" and pos[2] <= 5.0 and not self._payload_dropped:
            log.info("Payload dropped!")
            self._payload_dropped = True
            self.sm.on_payload_dropped()
        elif state == "LAND" and pos[2] <= 0.3:
            self.sm.on_landed()
        elif state == "RETURN_HOME":
            home = np.array([0, 0])
            if np.linalg.norm(pos[:2] - home) < 1.0:
                self.sm.on_at_home()

    # ── FPS ───────────────────────────────────────────────────────────
    def _fps(self) -> float:
        now = time.time()
        self._fps_times.append(now)
        self._fps_times = [t for t in self._fps_times if now - t < 1.0]
        return float(len(self._fps_times))

    # ── Vision → Mission update ───────────────────────────────────────
    def _update_mission_from_vision(self, gemma_result: dict):
        if not gemma_result:
            return
        lz  = gemma_result.get("landing_zone",{})
        dz  = gemma_result.get("drop_zone",{})
        obs = gemma_result.get("obstacles",{})
        self.sm.on_vision_update(
            scene_analysis=gemma_result,
            landing_zone={
                "zone_detected":     lz.get("detected", False),
                "safety_assessment": lz.get("safety","caution"),
                "gemma_confidence":  lz.get("clearance_score",0.0),
                "clearance_score":   lz.get("clearance_score",0.0),
            },
            drop_zone={
                "zone_detected":     dz.get("detected", False),
                "safety_assessment": "safe" if dz.get("confidence",0)>0.5 else "caution",
                "gemma_confidence":  dz.get("confidence",0.0),
                "area_ratio":        0.06 if dz.get("detected") else 0.0,
            },
            obstacles={
                "density":      obs.get("density",0.0),
                "center_clear": obs.get("center_clear",True),
            },
            battery_pct=100.0,
        )

    # ── Main loop ─────────────────────────────────────────────────────
    def run(self):
        print("\033[1mControls:\033[0m")
        print("  \033[96mS\033[0m → Start mission")
        print("  \033[96mL\033[0m → Land now")
        print("  \033[96mD\033[0m → Drop payload")
        print("  \033[96mA\033[0m → Abort (hold)")
        print("  \033[96mR\033[0m → Reset simulation")
        print("  \033[96mQ\033[0m → Quit\n")

        cv2.namedWindow("Drone Camera | YOLO + Gemma", cv2.WINDOW_NORMAL)
        cv2.namedWindow("MuJoCo 3D View",              cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Drone Camera | YOLO + Gemma", CAM_W, CAM_H)
        cv2.resizeWindow("MuJoCo 3D View",              VIEW_W, VIEW_H)
        cv2.moveWindow("MuJoCo 3D View",              0,     50)
        cv2.moveWindow("Drone Camera | YOLO + Gemma", VIEW_W+10, 50)

        # Physics runs N steps per rendered frame
        PHYSICS_STEPS_PER_FRAME = 10

        while self._running:
            self._frame_count += 1

            # ── Physics ────────────────────────────────────────────────
            for _ in range(PHYSICS_STEPS_PER_FRAME):
                self._physics_step()

            pos, vel, quat, gyro = self._get_state()
            self._sim_time = self.data.time

            # Advance search waypoints
            self._advance_search_waypoint(pos)

            # Auto altitude-based transitions
            self._check_auto_transitions(pos)

            # ── Drone camera render → YOLO ─────────────────────────────
            drone_frame = self._render_drone_cam()
            dets, fps, annotated, infer_ms = self.detector.detect(drone_frame)
            self._latest_dets = dets

            # ── Gemma (async, every N frames) ──────────────────────────
            if self._frame_count % self.gemma_interval == 0:
                self.analyzer.analyze_async(annotated, dets)

            result, raw_json, gemma_ms = self.analyzer.get_latest_result()
            if result is not None:
                self._latest_gemma = result
                self._latest_raw   = raw_json
                self._gemma_ms     = gemma_ms
                self._update_mission_from_vision(result)

            # ── 3D chase cam render ────────────────────────────────────
            view_frame = self._render_chase_cam()

            # ── Draw HUDs ──────────────────────────────────────────────
            cam_hud = draw_drone_cam_hud(
                annotated, self.sm.state, self._latest_gemma,
                dets, pos, self._fps(), self._gemma_ms,
            )
            view_hud = draw_3d_hud(
                view_frame, self.sm.state, pos, self._target_pos, fps,
            )

            # Sim time overlay on 3D view
            cv2.putText(view_hud, f"t={self._sim_time:.1f}s",
                (VIEW_W-120, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,100), 2)

            cv2.imshow("Drone Camera | YOLO + Gemma", cam_hud)
            cv2.imshow("MuJoCo 3D View",              view_hud)

            # Terminal status every 60 frames
            if self._frame_count % 60 == 0:
                print(
                    f"\r\033[96m[t={self._sim_time:6.1f}s]\033[0m"
                    f" {self.sm.state:<18}"
                    f" Alt={pos[2]:5.1f}m"
                    f" Tgt={self._target_pos[2]:.1f}m"
                    f" YOLO={len(dets):2d}"
                    f" Gemma={self._gemma_ms:.0f}ms"
                    f" FPS={fps:.0f}",
                    end="", flush=True,
                )

            # ── Key handling ───────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            elif key == ord("s"):
                print(f"\n\033[92m▶ START\033[0m")
                self.sm.on_start_command()
                self.sm.on_armed()
            elif key == ord("l"):
                print(f"\n\033[92m⬇ LAND\033[0m")
                self._target_pos = np.array(self.LAND_POS)
                self.sm.on_hold_command()
            elif key == ord("d"):
                print(f"\n\033[93m📦 DROP PAYLOAD\033[0m")
                self._target_pos = np.array(self.DROP_POS)
            elif key == ord("a"):
                print(f"\n\033[91m⛔ ABORT\033[0m")
                self.sm.on_abort_command()
            elif key == ord("r"):
                print(f"\n\033[93m🔄 RESET\033[0m")
                self._reset()
                self.sm = MissionStateMachine(
                    mission_type="search_and_drop",
                    on_state_change=self._on_state_change,
                    loiter_confirm_frames=3,
                )

        cv2.destroyAllWindows()
        print(f"\n\033[92mSimulation ended. Final state: {self.sm.state}\033[0m")


# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="IUB Drone MuJoCo Simulation")
    ap.add_argument("--yolo-model",    default="yolov8n.pt")
    ap.add_argument("--gemma-model",   default="gemma4:e4b")
    ap.add_argument("--gemma-interval",default=40, type=int,
                    help="Run Gemma every N rendered frames")
    ap.add_argument("--ollama-url",    default="http://localhost:11434")
    args = ap.parse_args()

    sim = DroneSimulation(
        yolo_model=args.yolo_model,
        gemma_model=args.gemma_model,
        gemma_interval=args.gemma_interval,
        ollama_url=args.ollama_url,
    )
    sim.run()


if __name__ == "__main__":
    main()
