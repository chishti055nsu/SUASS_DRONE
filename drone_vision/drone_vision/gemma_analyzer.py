"""
gemma_analyzer.py
=================
Gemma 4 (e4b) multimodal analyzer via local Ollama.

Uses /api/chat endpoint (correct for gemma4 vision).
gemma4:e4b is a reasoning model — response comes in the
'thinking' field when 'content' is empty. Both are handled.

Sends annotated camera frames + YOLO detections to Gemma and receives
structured JSON describing:
  - Landing zone location and safety
  - Takeoff zone assessment
  - Payload drop zone identification
  - Scene-level obstacle density
  - Recommended mission action

Runs asynchronously so it never blocks the YOLO inference loop.
"""

import json
import time
import base64
import logging
import threading
from typing import Dict, List, Optional, Any

import cv2
import numpy as np
import requests

logger = logging.getLogger(__name__)


# ── Structured Output Schema ───────────────────────────────────────────────────
GEMMA_SYSTEM_PROMPT = """You are an AI vision system onboard a quadcopter drone.
You receive camera frames annotated with YOLO bounding boxes and a list of detected objects.

Your job is to analyze the scene and return ONLY valid JSON with the following structure.
Do not include any explanation or text outside the JSON.

Required JSON schema:
{
  "scene_description": "brief description of what you see",
  "landing_zone": {
    "detected": true/false,
    "location": "left|center|right|none",
    "description": "what the landing zone looks like",
    "clearance_score": 0.0-1.0,
    "safety": "safe|caution|unsafe",
    "reasoning": "why this safety assessment"
  },
  "takeoff_zone": {
    "detected": true/false,
    "location": "left|center|right|none",
    "description": "description of area suitable for takeoff",
    "clearance_score": 0.0-1.0,
    "safety": "safe|caution|unsafe"
  },
  "drop_zone": {
    "detected": true/false,
    "location": "left|center|right|none",
    "description": "description of drop target if any",
    "confidence": 0.0-1.0,
    "target_marker": "circle|X|H|numbered|none"
  },
  "obstacles": {
    "density": 0.0-1.0,
    "summary": "brief obstacle description",
    "left_clear": true/false,
    "center_clear": true/false,
    "right_clear": true/false,
    "primary_threat": "description of biggest threat or none"
  },
  "mission_recommendation": {
    "action": "land|takeoff|drop_payload|hold|avoid|search|descend|ascend",
    "direction": "left|right|forward|back|descend|ascend|none",
    "confidence": 0.0-1.0,
    "reasoning": "brief reasoning for recommendation"
  }
}"""


# ── Gemma Analyzer ────────────────────────────────────────────────────────────
class GemmaAnalyzer:
    """
    Async Gemma 4 e4b analyzer using Ollama's local REST API.

    Args:
        model:       Ollama model name, default "gemma4:e4b"
        ollama_url:  Base URL of local Ollama, default "http://localhost:11434"
        timeout:     HTTP request timeout in seconds
        jpeg_quality:JPEG compression for frame encoding (lower = faster)
    """

    def __init__(
        self,
        model: str = "gemma4:e4b",
        ollama_url: str = "http://localhost:11434",
        timeout: float = 60.0,    # gemma4:e4b needs up to 30s on Mac
        jpeg_quality: int = 30,   # lower = smaller payload = faster
        inference_width: int = 160,  # resize before sending to Gemma
        inference_height: int = 120,
    ):
        self.model = model
        self.ollama_url = ollama_url.rstrip("/")
        self.timeout = timeout
        self.jpeg_quality = jpeg_quality
        self.inference_width  = inference_width
        self.inference_height = inference_height

        # Thread-safe result storage
        self._lock = threading.Lock()
        self._latest_result: Optional[Dict] = None
        self._latest_raw_json: str = "{}"
        self._is_running = False
        self._inference_ms: float = 0.0
        self._thread: Optional[threading.Thread] = None

        # Check Ollama is reachable
        self._check_ollama()

    # ── Health Check ───────────────────────────────────────────────────────
    def _check_ollama(self) -> None:
        try:
            r = requests.get(f"{self.ollama_url}/api/tags", timeout=3.0)
            models = [m["name"] for m in r.json().get("models", [])]
            if self.model not in models:
                logger.warning(
                    f"Model '{self.model}' not found in Ollama. "
                    f"Available: {models}. "
                    f"Pull with: ollama pull {self.model}"
                )
            else:
                logger.info(f"Ollama ready. Model '{self.model}' found.")
        except Exception as e:
            logger.warning(f"Ollama unreachable at {self.ollama_url}: {e}")

    # ── Async Analyze ──────────────────────────────────────────────────────
    def analyze_async(
        self,
        frame: np.ndarray,
        detections: List[Dict],
    ) -> None:
        """
        Submit a frame for Gemma analysis in a background thread.
        Non-blocking — call get_latest_result() to read output.
        Only one analysis runs at a time (skips if already running).
        """
        with self._lock:
            if self._is_running:
                return  # Skip — previous analysis still running

        self._thread = threading.Thread(
            target=self._analyze_worker,
            args=(frame.copy(), list(detections)),
            daemon=True,
        )
        self._is_running = True
        self._thread.start()

    def _analyze_worker(self, frame: np.ndarray, detections: List[Dict]) -> None:
        """Background worker that calls Ollama and stores result."""
        try:
            t0 = time.time()
            result, raw_json = self._call_gemma(frame, detections)
            elapsed_ms = (time.time() - t0) * 1000.0

            with self._lock:
                self._latest_result = result
                self._latest_raw_json = raw_json
                self._inference_ms = elapsed_ms

            logger.debug(f"Gemma inference: {elapsed_ms:.0f}ms")

        except Exception as e:
            logger.error(f"Gemma analysis failed: {e}")
        finally:
            with self._lock:
                self._is_running = False

    # ── Ollama API Call ────────────────────────────────────────────────────
    def _call_gemma(
        self, frame: np.ndarray, detections: List[Dict]
    ) -> tuple[Optional[Dict], str]:
        """Encode frame, build prompt, call Ollama /api/chat, parse JSON."""

        # Resize frame before encoding — smaller = faster inference
        small = cv2.resize(
            frame,
            (self.inference_width, self.inference_height),
            interpolation=cv2.INTER_AREA,
        )
        _, buf = cv2.imencode(
            ".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        b64_image = base64.b64encode(buf.tobytes()).decode("utf-8")
        logger.debug(f"Image payload: {len(b64_image)} bytes base64")

        # Build detection summary for prompt
        det_summary = self._format_detections(detections)

        user_prompt = (
            f"YOLO detected: {det_summary}\n\n"
            f"Return ONLY valid JSON matching the schema. No explanation."
        )

        # Use /api/chat — correct endpoint for gemma4 vision
        payload = {
            "model": self.model,
            "stream": False,
            "system": GEMMA_SYSTEM_PROMPT,
            "messages": [
                {
                    "role": "user",
                    "content": user_prompt,
                    "images": [b64_image],
                }
            ],
            "options": {
                "temperature": 0.1,
                "num_predict": 600,
            },
        }

        response = requests.post(
            f"{self.ollama_url}/api/chat",
            json=payload,
            timeout=self.timeout,
        )
        response.raise_for_status()

        data     = response.json()
        message  = data.get("message", {})

        # gemma4:e4b is a reasoning model:
        # When it 'thinks', content may be empty and answer is in 'thinking'
        raw_text = message.get("content", "").strip()
        if not raw_text:
            raw_text = message.get("thinking", "").strip()
        if not raw_text:
            raw_text = data.get("response", "{}").strip()

        logger.debug(f"Gemma raw ({len(raw_text)} chars): {raw_text[:120]}")

        # Extract JSON even if Gemma wraps it in markdown
        raw_json = self._extract_json(raw_text)
        try:
            parsed = json.loads(raw_json)
        except json.JSONDecodeError:
            logger.warning(f"Could not parse Gemma JSON: {raw_json[:200]}")
            parsed = self.empty_result()
            raw_json = "{}"
        return parsed, raw_json

    # ── Result Access ──────────────────────────────────────────────────────
    def get_latest_result(self) -> tuple[Optional[Dict], str, float]:
        """
        Returns:
            (parsed_dict, raw_json_str, inference_ms)
            parsed_dict is None if no result yet.
        """
        with self._lock:
            return self._latest_result, self._latest_raw_json, self._inference_ms

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._is_running

    # ── Helpers ────────────────────────────────────────────────────────────
    @staticmethod
    def _format_detections(detections: List[Dict]) -> str:
        if not detections:
            return "No objects detected by YOLO."
        lines = []
        for i, d in enumerate(detections):
            lines.append(
                f"  [{i+1}] {d['class_name']} ({d['category']}) "
                f"conf={d['confidence']:.2f} "
                f"pos={d['placement_h']}/{d['placement_v']} "
                f"depth={d['depth_estimate']}"
            )
        return "\n".join(lines)

    @staticmethod
    def _extract_json(text: str) -> str:
        """Strip markdown code fences if present and extract raw JSON."""
        text = text.strip()
        if text.startswith("```"):
            lines = text.split("\n")
            # Drop first (```json) and last (```) lines
            inner = "\n".join(lines[1:-1]) if lines[-1].strip() == "```" else "\n".join(lines[1:])
            return inner.strip()
        # Find first { ... }
        start = text.find("{")
        end = text.rfind("}") + 1
        if start != -1 and end > start:
            return text[start:end]
        return text

    # ── Default empty result ───────────────────────────────────────────────
    @staticmethod
    def empty_result() -> Dict[str, Any]:
        return {
            "scene_description": "Awaiting Gemma analysis...",
            "landing_zone":  {"detected": False, "location": "none", "description": "", "clearance_score": 0.0, "safety": "caution", "reasoning": ""},
            "takeoff_zone":  {"detected": False, "location": "none", "description": "", "clearance_score": 0.0, "safety": "caution"},
            "drop_zone":     {"detected": False, "location": "none", "description": "", "confidence": 0.0, "target_marker": "none"},
            "obstacles":     {"density": 0.0, "summary": "", "left_clear": True, "center_clear": True, "right_clear": True, "primary_threat": "none"},
            "mission_recommendation": {"action": "hold", "direction": "none", "confidence": 0.0, "reasoning": "Awaiting analysis"},
        }
