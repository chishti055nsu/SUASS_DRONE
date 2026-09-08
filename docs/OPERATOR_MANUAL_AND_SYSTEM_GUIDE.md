# 🛸 IUB DRONE COMMERCIAL AUTONOMY PLATFORM
## Operator Manual, Installation & Network Setup Guide
**SUAS 2026 Aerospace-Grade Tactical Ground Control System**

---

## 1. System Overview & Architecture

The **IUB Drone Commercial Autonomy Platform** is a high-performance, aerospace-grade autonomous flight control and perception workstation designed for Jetson companion computers (NVIDIA Jetson Nano / Orin Nano) paired with Matek flight controllers (H743-WING V2 running ArduPilot/MAVROS).

### 🛠️ Hardware & Sensor Architecture
* **Flight Controller**: Matek H743-WING V2 connected via Serial MAVLink (`/dev/ttyTHS1` @ 921600 baud) for 400Hz real-time state estimation and actuator control.
* **Companion Computer**: NVIDIA Jetson Orin Nano / Jetson Nano 4GB running Ubuntu 22.04 LTS with MAXN Performance power mode and CUDA FP16 GPU hardware acceleration.
* **Primary AI Camera**: **SIYI A8 Mini 4K Gimbal Camera** streaming over RTSP (`rtsp://192.168.144.25:8554/main.264`) or USB V4L2 video capture for high-speed YOLOv8 object detection and target-payload matching.
* **Avoidance & VIO Camera**: **Intel RealSense D455 3D Depth Camera** (`/dev/video4`) for 6-DOF Visual Inertial Odometry (VIO) optical flow and 3D obstacle avoidance.
* **LiDAR Altimeter**: **TFmini-S Micro LiDAR** (`/dev/ttyUSB0` @ 115200 baud) for millimeter-precision vertical distance tracking.
* **Ground Control Station**: Lightweight, zero-dependency, asynchronous Web GCS server serving an interactive dark-mode HUD dashboard on port `8080`.

---

## 2. Installation & Software Environment Setup

### 2.1 Prerequisites
Ensure the companion computer has the following software installed:
- Ubuntu 22.04 LTS / JetPack 5.x+
- ROS 2 Humble Hawksbill
- Python 3.10+
- PyTorch with CUDA support (`torch`, `torchvision`)
- Ultralytics YOLOv8 (`pip install ultralytics`)
- PyMAVLink & MAVROS (`sudo apt install ros-humble-mavros ros-humble-mavros-extras python3-pymavlink`)
- OpenCV with CUDA/FFMPEG/GStreamer support

### 2.2 Workspace Installation
1. Clone the repository on the Jetson companion computer:
   ```bash
   cd ~
   git clone https://github.com/chishti055nsu/SUASS_DRONE.git IUB_DRONE
   cd ~/IUB_DRONE
   ```

2. Grant serial port permissions:
   ```bash
   sudo usermod -a -G dialout,video $USER
   sudo chmod 666 /dev/ttyTHS1 /dev/ttyUSB0 /dev/video* 2>/dev/null || true
   ```

3. Engage Maximum Hardware Performance (CPU & GPU Lock):
   ```bash
   bash scripts/maximize_jetson_performance.sh
   ```

---

## 3. Ground Station to Drone Network Connection Setup

To operate the drone remotely from a Ground Station computer (laptop/tablet) and stream telemetry + high-definition camera feeds:

```
 ┌────────────────────────┐      HM30 Datalink / WiFi      ┌──────────────────────────┐
 │ Ground Computer        │ ◄────────────────────────────► │ NVIDIA Jetson Onboard    │
 │ Browser:               │     IP: 192.168.144.x          │ Companion Computer       │
 │ http://192.168.144.x:8080│                                │ Web GCS Server (Port 8080)│
 └────────────────────────┘                                └─────────────┬────────────┘
                                                                         │ Serial MAVLink
                                                                         ▼
                                                           ┌──────────────────────────┐
                                                           │ Matek H743 FC            │
                                                           │ (ArduPilot / 400Hz MAV)  │
                                                           └──────────────────────────┘
```

### 3.1 Establishing Network Connectivity
1. **Datalink Connection (SIYI HM30 / WiFi Router)**:
   - Connect the Jetson companion computer Ethernet port to the SIYI HM30 air unit or WiFi router.
   - Assign static IP address to Jetson: `192.168.144.100` (Subnet: `255.255.255.0`, Gateway: `192.168.144.1`).
   - Connect Ground Station computer to the SIYI HM30 ground unit or WiFi network. Assign IP: `192.168.144.150`.
2. **Verify IP Connectivity**:
   - On Ground Station computer, ping the Jetson:
     ```bash
     ping 192.168.144.100
     ```

### 3.2 SIYI A8 Mini 4K RTSP Video Feed Configuration
- **Default RTSP Stream URL**: `rtsp://192.168.144.25:8554/main.264`
- If SIYI A8 Mini is connected via USB/HDMI capture card, the system automatically falls back to V4L2 USB capture (`/dev/video0` / `/dev/video2`) without blocking network threads.

---

## 4. Web GCS Control Center Operation Manual

### 4.1 Launching the Ground Control Station
On the Jetson companion computer, run:
```bash
cd ~/IUB_DRONE
bash scripts/start_web_gcs.sh
```
The server will start on port `8080` (or `8081` if `8080` is in use).

Open any web browser on your Ground Station computer:
👉 **`http://192.168.144.100:8080`** (Replace `192.168.144.100` with Jetson IP).

---

### 4.2 Web GCS Interface Tabs & Operating Features

#### 1. Top Header Telemetry Strip & Arming Badge
* **Telemetry Indicators**: Displays real-time MAVLink frequency (400Hz), GPS RTK Satellite count (18 SAT), 6S Battery Voltage (`25.2V`), and CPU/GPU load.
* **Clickable Arm/Disarm Status Badge**: 
  - **`DISARMED - READY`** (Cyan): Vehicle is disarmed and safe. Click to ARM.
  - **`ARMED - LIVE`** (Red Pulse): Motors are armed and live! Click to DISARM immediately.

#### 2. Tab 1: Tactical Dashboard & ENU 2D Radar
* **Interactive 2D Radar Canvas**: Shows real-time drone North/East position, target setpoint, flight trajectory trail, and geofence boundary.
  - **Click-to-Fly (GOTO)**: Click anywhere on the 2D radar grid to dispatch an instant autonomous 3D GOTO coordinate setpoint.
* **Primary Flight Instruments (PFD)**: Displays Roll/Pitch Horizon and Heading Compass.
* **Tactical Execution Commands Panel**:
  - `START AUTONOMOUS MISSION`: Arms motors and begins autonomous waypoint navigation.
  - `🔥 5.5KG HEAVY-LIFT 80%`: Engages 80% manual PWM throttle override (1800 PWM) for heavy payload lifting.
  - `RETURN HOME (RTL)`: Triggers automatic Return To Launch at 15m altitude.
  - `HOVER / HOLD`: Locks current 3D position in mid-air.
  - `LAND NOW`: Initiates controlled vertical descent and landing.
  - `🛑 DISARM MOTORS NOW`: **Instant Motor Cut-off** (Sends triple-redundant disarm).
  - `⚡ EMERGENCY KILL`: Displays confirmation modal for emergency flight termination.
* **Dynamic Hover Throttle & Altitude Lock Calibrator**:
  - Allows the operator to manually adjust hover throttle PWM (1000–2000 PWM) and target altitude (0–50m) with live JSON config saving (`mission_planner/config/hover_config.json`).

#### 3. Tab 2: SIYI A8 Mini 4K AI Vision Feed
* **Live Stream Window**: Displays HD video feed from SIYI A8 Mini with real-time YOLOv8 bounding boxes and classification labels.
* **Target & Payload Matching Table**: Lists identified targets (e.g. `MANNEQUIN`, `TENT`) and matched emergency payloads (`WATER_BOTTLE`, `MEDICAL_KIT`).
* **Gemma Multimodal AI Scene Reasoning**: Displays natural language scene analysis, obstacle density scores, and recommended tactical actions.

#### 4. Tab 3: RealSense D455 Avoidance & LiDAR HUD
* **RealSense D455 Live Feed**: Shows 3D depth perception and optical flow tracking.
* **Obstacle Threat Sector Clearance**: Displays Left/Center/Right sector clearance status and risk level (LOW/MEDIUM/CRITICAL).
* **TFmini-S LiDAR Tracking**: Millimeter-precision altitude readout.

#### 5. Tab 4: Virtual Manual Remote Controller
* **Nudge Control Pads**: Allows manual directional control nudges (Forward/Back/Left/Right/Yaw).
* **Direct Throttle Slider**: Precision PWM slider (1000 to 2000 PWM).

---

## 5. Step-by-Step Flight Operations & Emergency Procedures

### 5.1 Pre-Flight Safety Checklist
1. Inspect 6S LiPo Battery voltage ($\ge 24.0\text{V}$ for full charge).
2. Verify Matek Flight Controller serial connection (`MAVLink FC: 400Hz OK`).
3. Verify GPS 3D RTK Fix ($\ge 14$ Satellites).
4. Verify TFmini-S LiDAR altitude readout.
5. Verify SIYI A8 Mini camera video feed is clear.

### 5.2 Launch & Mission Execution
1. Click **`START AUTONOMOUS MISSION`** or click **`DISARMED - READY`** badge to arm motors.
2. Monitor waypoint progression on 2D Radar Canvas.
3. If payload release is needed, click **`5.5KG HEAVY-LIFT`** or dispatch GOTO target coordinate.

### 5.3 Emergency Motor Disarm Procedure
If an emergency occurs (runaway flight, obstacle threat, or manual intervention required):
1. Click **`🛑 DISARM MOTORS NOW`** on the control panel OR click the **`ARMED - LIVE`** header badge.
2. **What Happens Behind the Scenes**:
   - Web GCS transmits 5 MAVLink force-disarm packets (`MAV_CMD_COMPONENT_ARM_DISARM` param1=0, param2=21196 force-disarm magic).
   - Throttle channel is locked to 1000 PWM (0% throttle).
   - ROS 2 MAVROS service `/mavros/cmd/arming` `value: false` is executed.
   - Motors cut power immediately.

---

## 6. Troubleshooting & FAQs

| Symptom | Probable Cause | Recommended Action |
| :--- | :--- | :--- |
| **Web GCS Page Not Loading** | Server not started or port blocked. | Run `bash scripts/start_web_gcs.sh` on Jetson. Check IP with `ifconfig`. |
| **SIYI A8 Mini Feed Black** | Network IP mismatch or RTSP port closed. | Check Ethernet IP (`192.168.144.x`). System will auto-fallback to USB camera (`/dev/video0`). |
| **UI Stuttering on Tab Switch** | Dual streams loading simultaneously. | Updated software automatically pauses hidden stream on tab switch. Refresh browser (`Ctrl+F5`). |
| **Motors Not Arming** | Safety interlock / Arming check active. | Hardware bridge automatically sets `ARMING_CHECK=0`. Ensure battery $>22.0\text{V}$. |

---
*IUB Drone Commercial Autonomy System — Designed & Engineered for SUAS 2026 Competition.*
