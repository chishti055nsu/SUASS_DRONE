"""
mac_test.py
===========
Standalone Mac test for IUB Drone vision + mission system.
NO ROS2 required — runs YOLO + Gemma + Mission State Machine
with a live OpenCV display window.

Usage:
    # Demo mode (no camera needed — synthetic frames)
    python3 mac_test.py --demo

    # Webcam (requires camera permission in System Settings)
    python3 mac_test.py --source 0

    # Video file (recommended for quick testing)
    python3 mac_test.py --source /path/to/drone_video.mp4

    # Image folder
    python3 mac_test.py --source /path/to/images/

Controls (OpenCV window):
    S  → Send START mission command
    A  → Send ABORT command
    H  → Send HOLD command
    Q  → Quit

NOTE — Mac Camera Permission:
    If webcam fails, go to:
    System Settings → Privacy & Security → Camera
    and enable access for Terminal / your Python app.
    Alternatively use --demo or --source <video_file>
"""

import sys
import os
import time
import json
import argparse
import threading
import logging

import cv2
import numpy as np

# ── Add parent packages to path ───────────────────────────────────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "drone_vision"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "mission_planner"))

from drone_vision.yolo_detector import YOLODetector
from drone_vision.gemma_analyzer import GemmaAnalyzer
from mission_planner.mission_state_machine import MissionStateMachine

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("mac_test")

# ── ANSI colors for terminal ───────────────────────────────────────────────────
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


# ─────────────────────────────────────────────────────────────────────────────
# HUD Overlay
# ─────────────────────────────────────────────────────────────────────────────
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

def draw_hud(
    frame: np.ndarray,
    state: str,
    gemma_result: dict,
    fps: float,
    frame_count: int,
    gemma_ms: float,
) -> np.ndarray:
    """Draw full heads-up display overlay on frame."""
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # ── Top bar ──────────────────────────────────────────────────────────
    cv2.rectangle(overlay, (0, 0), (w, 64), (15, 15, 25), -1)
    cv2.addWeighted(overlay, 0.80, frame, 0.20, 0, frame)

    state_color = STATE_COLORS.get(state, (200, 200, 200))
    cv2.putText(frame, f"STATE: {state}",
        (14, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.1, state_color, 2, cv2.LINE_AA)

    cv2.putText(frame, f"FPS: {fps:.1f}",
        (w - 130, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Frame: {frame_count}",
        (w - 130, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1, cv2.LINE_AA)

    # ── Gemma panel (bottom right) ────────────────────────────────────────
    if gemma_result:
        panel_w, panel_h = 320, 200
        px, py = w - panel_w - 10, h - panel_h - 10
        panel = frame[py:py+panel_h, px:px+panel_w].copy()
        cv2.rectangle(panel, (0, 0), (panel_w, panel_h), (15, 15, 35), -1)
        frame[py:py+panel_h, px:px+panel_w] = panel
        cv2.rectangle(frame, (px, py), (px+panel_w, py+panel_h), (60, 60, 120), 1)

        cv2.putText(frame, f"GEMMA ({gemma_ms:.0f}ms)",
            (px+8, py+22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 50), 2)

        rec   = gemma_result.get("mission_recommendation", {})
        lz    = gemma_result.get("landing_zone", {})
        dz    = gemma_result.get("drop_zone", {})
        obs   = gemma_result.get("obstacles", {})

        safety_col = {"safe": (0, 220, 80), "caution": (0, 200, 255), "unsafe": (0, 50, 255)}
        lz_safety  = lz.get("safety", "caution")
        lz_col     = safety_col.get(lz_safety, (180, 180, 180))

        rows = [
            (f"Action:  {rec.get('action','?').upper()}",       (255, 220, 50)),
            (f"Land zone: {lz_safety.upper()} ({lz.get('clearance_score', 0):.0%})", lz_col),
            (f"Drop zone: {'✓ FOUND' if dz.get('detected') else '✗ NONE'}",
             (0, 220, 80) if dz.get("detected") else (120, 120, 120)),
            (f"Obstacles: {int(obs.get('density',0)*100)}%",
             (0, 50, 255) if obs.get("density", 0) > 0.5 else (0, 200, 80)),
            (f"Direction: {rec.get('direction', 'none')}",      (180, 180, 255)),
        ]
        for i, (text, color) in enumerate(rows):
            cv2.putText(frame, text,
                (px+8, py+48 + i*28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)

        # Scene description
        desc = gemma_result.get("scene_description", "")[:50]
        cv2.putText(frame, desc,
            (px+4, py+panel_h-10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 180), 1, cv2.LINE_AA)

    # ── Obstacle density bar (bottom left) ────────────────────────────────
    if gemma_result:
        density = gemma_result.get("obstacles", {}).get("density", 0.0)
        bar_w   = int(220 * density)
        bar_col = (0, 220, 80) if density < 0.3 else (0, 200, 255) if density < 0.6 else (0, 50, 255)
        cv2.rectangle(frame, (10, h - 36), (230, h - 16), (40, 40, 40), -1)
        if bar_w > 0:
            cv2.rectangle(frame, (10, h - 36), (10 + bar_w, h - 16), bar_col, -1)
        cv2.putText(frame, f"Obstacle density: {int(density*100)}%",
            (10, h - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    # ── Controls hint ──────────────────────────────────────────────────────
    hints = "[S] Start  [A] Abort  [H] Hold  [Q] Quit"
    cv2.putText(frame, hints,
        (10, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

    return frame


# ─────────────────────────────────────────────────────────────────────────────
# Main Test Runner
# ─────────────────────────────────────────────────────────────────────────────
class DroneVisionMacTest:

    def __init__(self, source=0, yolo_model="yolov8n.pt",
                 gemma_model="gemma4:e4b", gemma_interval=10,
                 ollama_url="http://localhost:11434", mission_type="search_and_drop"):

        self.gemma_interval = gemma_interval
        self.frame_count    = 0
        self._fps_times     = []

        print(f"\n{BOLD}{CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{RESET}")
        print(f"{BOLD}{CYAN}   IUB Drone — Mac Vision Test{RESET}")
        print(f"{CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{RESET}\n")

        # ── YOLO ─────────────────────────────────────────────────────────
        print(f"{YELLOW}[1/3] Loading YOLOv8...{RESET}")
        self.detector = YOLODetector(
            model_path=yolo_model,
            conf_threshold=0.45,
            device="mps",  # Apple Metal GPU — falls back to cpu automatically
        )
        print(f"{GREEN}✅ YOLO ready ({yolo_model}){RESET}")

        # ── Gemma ─────────────────────────────────────────────────────────
        print(f"{YELLOW}[2/3] Connecting to Gemma ({gemma_model})...{RESET}")
        self.analyzer = GemmaAnalyzer(
            model=gemma_model,
            ollama_url=ollama_url,
            timeout=60.0,        # gemma4:e4b needs ~3-30s on Mac
            jpeg_quality=30,     # small payload = faster
            inference_width=160,
            inference_height=120,
        )
        print(f"{GREEN}✅ Gemma ready via Ollama{RESET}")

        # ── Mission State Machine ─────────────────────────────────────────
        print(f"{YELLOW}[3/3] Initialising mission state machine...{RESET}")
        self.sm = MissionStateMachine(
            mission_type=mission_type,
            on_state_change=self._on_state_change,
            loiter_confirm_frames=3,
        )
        print(f"{GREEN}✅ Mission SM ready (type={mission_type}){RESET}\n")

        # ── Video source ──────────────────────────────────────────────────────
        self._demo_mode = (source == "demo")

        if self._demo_mode:
            self._image_dir = None
            self._cap       = None
            self._demo_frame_idx = 0
            print(f"🎮 Demo mode — synthetic frames (no camera needed)")

        elif isinstance(source, str) and source != "demo" and os.path.isdir(source):
            self._image_dir  = sorted([
                os.path.join(source, f) for f in os.listdir(source)
                if f.lower().endswith((".jpg", ".jpeg", ".png"))
            ])
            self._image_idx  = 0
            self._cap        = None
            print(f"📁 Image folder: {len(self._image_dir)} images")

        else:
            self._image_dir = None
            # Try AVFoundation (Mac native) first, then default
            self._cap = cv2.VideoCapture(source, cv2.CAP_AVFOUNDATION)
            if not self._cap.isOpened():
                self._cap = cv2.VideoCapture(source)
            if not self._cap.isOpened():
                print(f"\n{RED}❌ Cannot open camera/video: {source}{RESET}")
                print(f"{YELLOW}")
                print(f"Possible fixes:")
                print(f"  1. Grant camera permission:")
                print(f"     System Settings → Privacy & Security → Camera")
                print(f"     Enable access for Terminal")
                print(f"  2. Run in demo mode (no camera):")
                print(f"     python3 mac_test.py --demo")
                print(f"  3. Use a video file:")
                print(f"     python3 mac_test.py --source /path/to/video.mp4")
                print(f"{RESET}")
                raise RuntimeError(
                    f"Cannot open video source: {source}. "
                    f"Try --demo or --source <video_file>"
                )
            src_label = "Webcam" if source == 0 else f"Video: {source}"
            print(f"📷 Source: {src_label}")

        self._latest_gemma   = None
        self._latest_raw     = "{}"
        self._latest_gemma_ms = 0.0

    # ── State change callback ─────────────────────────────────────────────
    def _on_state_change(self, old: str, new: str):
        color = STATE_COLORS.get(new, (200, 200, 200))
        # Print with nearest ANSI color
        print(f"\n{BOLD}{'='*44}{RESET}")
        print(f"{BOLD}  MISSION STATE: {old}  →  {new}{RESET}")
        print(f"{'='*44}\n")

    # ── Read frame ────────────────────────────────────────────────────────
    def _make_demo_frame(self) -> np.ndarray:
        """Generate a synthetic outdoor scene frame for demo/testing."""
        t = self._demo_frame_idx / 30.0
        self._demo_frame_idx += 1

        # Sky gradient
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        for y in range(480):
            blend = y / 480.0
            frame[y] = [
                int(135 * (1 - blend) + 80 * blend),
                int(206 * (1 - blend) + 160 * blend),
                int(235 * (1 - blend) + 100 * blend),
            ]

        # Ground
        cv2.rectangle(frame, (0, 300), (640, 480), (60, 100, 40), -1)

        # Animated landing circle (pulsing)
        cx, cy = 320 + int(30 * np.sin(t * 0.5)), 380
        pulse  = int(5 * np.sin(t * 3))
        cv2.circle(frame, (cx, cy), 50 + pulse, (255, 255, 255), 3)
        cv2.circle(frame, (cx, cy), 25 + pulse, (255, 200, 0), 3)
        cv2.line(frame, (cx - 60, cy), (cx + 60, cy), (255, 255, 255), 2)
        cv2.line(frame, (cx, cy - 60), (cx, cy + 60), (255, 255, 255), 2)
        cv2.putText(frame, "DEMO", (cx - 20, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        # Simulated person (target)
        px = 100 + int(40 * np.sin(t * 0.3))
        cv2.rectangle(frame, (px, 260), (px + 30, 340), (0, 100, 180), -1)
        cv2.circle(frame, (px + 15, 250), 18, (200, 150, 100), -1)  # head

        # Obstacle tree
        cv2.rectangle(frame, (520, 200), (550, 360), (40, 80, 20), -1)
        cv2.circle(frame, (535, 190), 45, (20, 120, 30), -1)

        # Frame counter
        cv2.putText(frame, f"DEMO FRAME {self._demo_frame_idx}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        time.sleep(1/30.0)
        return frame

    def _read_frame(self):
        if self._demo_mode:
            return True, self._make_demo_frame()
        if self._image_dir is not None:
            if self._image_idx >= len(self._image_dir):
                self._image_idx = 0
            img = cv2.imread(self._image_dir[self._image_idx])
            self._image_idx += 1
            time.sleep(0.1)
            return True, img
        ret, frame = self._cap.read()
        if not ret and self._cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0:
            # Loop video file
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self._cap.read()
        return ret, frame

    # ── FPS ───────────────────────────────────────────────────────────────
    def _fps(self) -> float:
        now = time.time()
        self._fps_times.append(now)
        self._fps_times = [t for t in self._fps_times if now - t < 1.0]
        return float(len(self._fps_times))

    # ── Simulate mission update from vision ───────────────────────────────
    def _vision_to_mission(self, gemma_result: dict):
        if gemma_result is None:
            return
        lz  = gemma_result.get("landing_zone", {})
        dz  = gemma_result.get("drop_zone", {})
        obs = gemma_result.get("obstacles", {})

        landing_zone = {
            "zone_detected":     lz.get("detected", False),
            "safety_assessment": lz.get("safety", "caution"),
            "gemma_confidence":  lz.get("clearance_score", 0.0),
            "clearance_score":   lz.get("clearance_score", 0.0),
        }
        drop_zone = {
            "zone_detected":     dz.get("detected", False),
            "safety_assessment": "safe" if dz.get("confidence", 0) > 0.6 else "caution",
            "gemma_confidence":  dz.get("confidence", 0.0),
            "area_ratio":        0.05 if dz.get("detected") else 0.0,
        }
        obstacles = {
            "density":      obs.get("density", 0.0),
            "center_clear": obs.get("center_clear", True),
            "left_clear":   obs.get("left_clear", True),
            "right_clear":  obs.get("right_clear", True),
        }
        self.sm.on_vision_update(
            scene_analysis=gemma_result,
            landing_zone=landing_zone,
            drop_zone=drop_zone,
            obstacles=obstacles,
            battery_pct=100.0,
        )

    # ── Main loop ─────────────────────────────────────────────────────────
    def run(self):
        print(f"\n{BOLD}Controls:{RESET}")
        print(f"  {CYAN}S{RESET} → Start mission")
        print(f"  {CYAN}A{RESET} → Abort")
        print(f"  {CYAN}H{RESET} → Hold / Loiter")
        print(f"  {CYAN}Q{RESET} → Quit\n")
        print(f"{YELLOW}Opening display window...{RESET}")

        cv2.namedWindow("IUB Drone — Mac Test", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("IUB Drone — Mac Test", 1280, 720)

        while True:
            ret, frame = self._read_frame()
            if not ret or frame is None:
                logger.warning("No frame — retrying...")
                time.sleep(0.1)
                continue

            self.frame_count += 1

            # ── YOLO inference ────────────────────────────────────────────
            detections, fps, annotated, infer_ms = self.detector.detect(frame)

            # ── Gemma inference (async, every N frames) ───────────────────
            if self.frame_count % self.gemma_interval == 0:
                self.analyzer.analyze_async(annotated, detections)

            result, raw_json, gemma_ms = self.analyzer.get_latest_result()
            if result is not None:
                self._latest_gemma    = result
                self._latest_raw      = raw_json
                self._latest_gemma_ms = gemma_ms
                self._vision_to_mission(result)

            # ── Draw HUD ──────────────────────────────────────────────────
            display = draw_hud(
                annotated,
                self.sm.state,
                self._latest_gemma,
                self._fps(),
                self.frame_count,
                self._latest_gemma_ms,
            )

            cv2.imshow("IUB Drone — Mac Test", display)

            # ── Terminal status every 30 frames ───────────────────────────
            if self.frame_count % 30 == 0:
                st = self.sm.get_status_dict()
                print(
                    f"\r{CYAN}[Frame {self.frame_count:05d}]{RESET}"
                    f" State={BOLD}{st['state']}{RESET}"
                    f" | YOLO={len(detections)} objs"
                    f" | Gemma={self._latest_gemma_ms:.0f}ms"
                    f" | FPS={fps:.1f}",
                    end="", flush=True,
                )

            # ── Key handling ──────────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                print(f"\n{YELLOW}Quitting...{RESET}")
                break
            elif key == ord("s"):
                print(f"\n{GREEN}▶ START command sent{RESET}")
                self.sm.on_start_command()
                self.sm.on_armed()           # Simulate immediate arm on Mac
                self.sm.on_altitude_reached() # Simulate takeoff on Mac
            elif key == ord("a"):
                print(f"\n{RED}⛔ ABORT command sent{RESET}")
                self.sm.on_abort_command()
            elif key == ord("h"):
                print(f"\n{YELLOW}⏸ HOLD command sent{RESET}")
                self.sm.on_hold_command()

        # ── Cleanup ───────────────────────────────────────────────────────
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()
        print(f"\n{GREEN}Done. Final state: {self.sm.state}{RESET}")


# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="IUB Drone — Mac standalone test (no ROS2 needed)"
    )
    parser.add_argument(
        "--source", default=None,
        help="Video source: 0 (webcam) | /path/to/video.mp4 | /path/to/images/",
    )
    parser.add_argument("--demo",          action="store_true",          help="Run with synthetic demo frames (no camera)")
    parser.add_argument("--yolo-model",    default="yolov8n.pt",        help="YOLO weights path")
    parser.add_argument("--gemma-model",   default="gemma4:e4b",         help="Ollama model name")
    parser.add_argument("--gemma-interval",default=10, type=int,         help="Run Gemma every N frames")
    parser.add_argument("--ollama-url",    default="http://localhost:11434")
    parser.add_argument("--mission-type",  default="search_and_drop",
                        choices=["search_and_drop", "land_on_marker", "survey"])
    args = parser.parse_args()

    if args.demo:
        source = "demo"
    elif args.source is None:
        source = "demo"   # default to demo if no source given and no --source flag
    else:
        source = args.source
        if isinstance(source, str) and source.isdigit():
            source = int(source)

    try:
        test = DroneVisionMacTest(
            source=source,
            yolo_model=args.yolo_model,
            gemma_model=args.gemma_model,
            gemma_interval=args.gemma_interval,
            ollama_url=args.ollama_url,
            mission_type=args.mission_type,
        )
        test.run()
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Interrupted.{RESET}")
    except Exception as e:
        print(f"\n{RED}Error: {e}{RESET}")
        raise


if __name__ == "__main__":
    main()
