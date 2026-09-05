# Commercial Hardware-Agnostic UAV Autonomy Platform Blueprint

This document outlines how the **IUB Drone SUAS ROS 2 System** is structured as a **Universal, Commercial-Grade Hardware-Agnostic Autonomy Engine** that runs on **ANY drone, flight controller, companion computer, or sensor suite**.

---

## 1. Architectural Vision: Universal Hardware Independence

```
 ┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
 │                                  COMMERCIAL UAV AUTONOMY SUITE                                   │
 └──────────────────────────────────────────────────────────────────────────────────────────────────┘
                                                 │
          ┌──────────────────────────────────────┼──────────────────────────────────────┐
          ▼                                      ▼                                      ▼
┌──────────────────┐                   ┌──────────────────┐                   ┌──────────────────┐
│  ANY Companion   │                   │   ANY Vision &   │                   │  ANY Flight      │
│     Computer     │                   │  Sensor Hardware │                   │   Controller     │
│ (Jetson, RPi 5,  │                   │ (RTSP, USB, CSI, │                   │ (ArduPilot, PX4, │
│  x86, Rockchip)  │                   │  RealSense, LiDAR)│                  │ Betaflight, DJI) │
└─────────┬────────┘                   └─────────┬────────┘                   └─────────┬────────┘
          │                                      │                                      │
          └──────────────────────────────────────┼──────────────────────────────────────┘
                                                 ▼
                               ┌──────────────────────────────────┐
                               │  Universal Hardware Abstraction  │
                               │           Layer (HAL)            │
                               └─────────────────┬────────────────┘
                                                 ▼
                               ┌──────────────────────────────────┐
                               │ Autonomous Mission State Machine │
                               │ (Search ➔ Drop ➔ Return to Base) │
                               └──────────────────────────────────┘
```

---

## 2. The 4 Commercial Architecture Pillars

### Pillar 1: Universal Flight Controller HAL (`flight_controller.py`)
The system abstracts all flight controller communication through an Abstract Base Class (**ABC**). To support a new flight controller, simply plug in its adapter:

* **MAVROS Adapter** *(Active)*: Supports ArduPilot & PX4 on Pixhawk, Matek, Cube, Holybro.
* **Betaflight / INAV Adapter**: MSP Protocol over Serial (`/dev/ttyUSB0`).
* **DJI SDK Adapter**: DJI Payload SDK / Onboard SDK.
* **Nav2 / ROS 2 Adapter**: Generic ROS 2 Navigation stack.

### Pillar 2: Plug-and-Play Sensor & Camera Engine
Customers can use any camera or distance sensor by selecting the source type in a single configuration file (`config/system_config.yaml`):

```yaml
system:
  drone_name: "Universal-UAV-01"
  companion_computer: "auto_detect"  # Jetson, Raspberry Pi 5, x86 NUC, Rockchip

sensors:
  primary_camera:
    type: "rtsp"  # Options: "rtsp", "usb_cam", "csi", "realsense", "ros_topic"
    endpoint: "rtsp://192.168.144.25:8554/main.264"
  
  depth_sensor:
    type: "realsense_d455"  # Options: "realsense_d455", "oak_d", "none"
  
  rangefinder_lidar:
    type: "tfmini_s"        # Options: "tfmini_s", "rplidar", "laser_scan", "none"
    port: "/dev/ttyUSB0"
```

### Pillar 3: Class-Agnostic AI Target & Obstacle Engine
* **Universal Target Lock**: AI detection (`YOLOv8` + `Gemma` + `ActionZone`) detects any target marker, box, cross, H-pad, or custom visual symbol.
* **Universal 3D Obstacle Avoidance**: Uses volumetric 3D Point Cloud spatial occupancy. Avoids **any solid physical object** (trees, wires, buildings, walls, vehicles) regardless of what type of object it is.

### Pillar 4: Containerized One-Click Deployment (`docker-compose.yml`)
Commercial customers do not need to install ROS 2 or compile code manually. They deploy via Docker:

```bash
# Deploys full autonomy stack on ANY ARM64 or x86 hardware in 1 command:
docker compose up -d
```

---

## 3. Commercial Roadmap to Market

1. **Web Ground Control Station (Web GCS)**:
   - Build a lightweight React / Next.js Web UI served directly from the companion computer.
   - Allows users to monitor live RTSP video, check 3D telemetry, set waypoints, and click **START / RTL** from any smartphone, tablet, or browser.
2. **Auto-Hardware Auto-Discovery**:
   - Detect connected USB cameras, serial ports, and flight controllers automatically on boot.
3. **Enterprise License & OTA Updates**:
   - Secure software licensing, containerized updates, and black-box telemetry logging.
