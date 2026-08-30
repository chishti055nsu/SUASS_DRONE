# IUB Drone System Architecture & Operational Safety Guide

## 1. System Architecture Overview

```
                      Camera Image Feed / Sim State (30 Hz)
                                  │
          ┌───────────────────────┴───────────────────────┐
          │                                               │
          ▼                                               ▼
┌──────────────────┐                           ┌─────────────────────┐
│  yolo_detector   │                           │  aruco_detector     │
│  (YOLOv8 Engine) │                           │  (Precision 3D Pose)│
└────────┬─────────┘                           └──────────┬──────────┘
         │                                                │
         ├───────────────────────┬────────────────────────┘
         │                       │
         ▼                       ▼
┌──────────────────┐   ┌──────────────────┐
│  drone_vision    │   │ precision_landing│
│  (ROS2 Node)     │   │ (ROS2 Node)      │
└────────┬─────────┘   └─────────┬────────┘
         │                       │
         └───────────┬───────────┘
                     │ ROS2 Topics (ActionZone, PoseStamped)
                     ▼
         ┌───────────────────────┐
         │ TargetDetection API   │
         │ (Perception Norm)     │
         └───────────┬───────────┘
                     │ Normalized Detections
                     ▼
        ┌─────────────────────────┐
        │  mission_planner_node   │
        │                         │
        │ • Valid Transition      │
        │   Matrix (13 States)    │
        │ • 20 Hz OFFBOARD Stream │
        │ • Multi-Condition Drop  │
        │ • Fail-Closed Watchdogs │
        └────────────┬────────────┘
                     │ FlightController HAL Interface
                     ▼
        ┌─────────────────────────┐
        │   FlightController HAL  │
        │ (Mavros | MuJoCo | Stub)│
        └────────────┬────────────┘
                     │
        ┌────────────┼────────────┐
        ▼            ▼            ▼
 ┌──────────┐ ┌──────────┐ ┌──────────┐
 │ MAVROS / │ │ MuJoCo   │ │ SimStub  │
 │ PX4 HW   │ │ Physics  │ │ In-Mem   │
 └──────────┘ └──────────┘ └──────────┘
```

---

## 2. Component Status & Integration Seams

| Component | Module Path | Status | Integration Seam |
|:---|:---|:---:|:---|
| **Mission State Machine** | `mission_planner.mission_state_machine` | ✅ Production Active | Pure Python class with deterministic rules and timeout watchdogs |
| **Waypoint Manager** | `mission_planner.waypoint_manager` | ✅ Production Active | Lawnmower pattern generator with ENU/NED coordinate transforms |
| **Flight Controller HAL** | `mission_planner.flight_controller` | ✅ Production Active | `FlightController` ABC supporting `Mavros`, `MuJoCo`, and `SimStub` backends |
| **YOLOv8 + Gemma Vision** | `drone_vision.vision_node` | ✅ Production Active | Subscribes to `/camera/image_raw`, publishes `/drone_vision/landing_zone` |
| **Precision Landing** | `precision_landing.precision_node` | ✅ Integrated | Subscribes to camera, publishes `/precision_landing/target_pose` & `target_locked` |
| **Perception Normalization**| `drone_vision.perception_interface` | ✅ Production Active | Unifies YOLO, Gemma, ArUco, and sim detections into `TargetDetection` |

---

## 3. Interface Contracts (ROS2 Topics)

| Topic | Message Type | Rate | Description | Safety Critical |
|:---|:---|:---:|:---|:---:|
| `/drone_vision/detections` | `DetectionArray` | 30 Hz | YOLOv8 bounding boxes + track IDs | ❌ |
| `/drone_vision/landing_zone` | `ActionZone` | 30 Hz | Geometry (`center_pixel`, `bbox_xyxy`, `area_ratio`, `confidence`) | ✅ |
| `/drone_vision/drop_zone` | `ActionZone` | 30 Hz | Payload target geometry & position error | ✅ |
| `/precision_landing/target_pose` | `geometry_msgs/PoseStamped` | 30 Hz | Camera frame 3D offset ($dx, dy, dz$) in meters | ✅ |
| `/precision_landing/target_locked` | `std_msgs/Bool` or `String` | 30 Hz | Lock status confirmation | ✅ |
| `/mission_planner/command` | `MissionCommand` | Event | Operator commands (`start`, `abort`, `rtl`, `terminate`) | ✅ |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 20 Hz | High-frequency continuous position setpoint | ✅ |

---

## 4. SUAS Safety Case & Failsafes

1. **Rule 5.3.1 (Manual Safety Pilot Override)**:
   - Continuously monitored via `/mavros/state`. If the human pilot flips the transmitter switch out of `OFFBOARD` into `POSCTL` or `MANUAL`, ROS2 setpoint control disengages immediately, and the state transitions to `MANUAL_OVERRIDE`.

2. **Rule 5.3.5 (Air-Gapped Onboard Control)**:
   - 100% of flight-critical perception (YOLOv8 + ArUco 3D Pose + State Machine) runs locally on the NVIDIA Jetson Orin Nano. Zero internet or cloud servers are required.

3. **Rule 5.3.8 (Emergency Flight Termination & RTL)**:
   - `terminate` command triggers instant disarm via `FlightController.disarm()`.
   - Low battery ($<15\%$), telemetry heartbeat loss ($>3.0\text{s}$), or state timeout automatically fails closed to `RETURN_HOME`.

---

## 5. Preflight & Flight Checklists

### 📋 Preflight Checklist
1. **Physical Inspection**:
   - Check propeller tightness and motor rotation direction.
   - Verify battery is wrapped in bright tape and securely latched in slide tray.
   - Confirm drone weight is $\le 35\text{ LBs}$.
   - Verify Remote ID module is powered and broadcasting.
2. **Software Verification**:
   - Run unit test suite: `pytest` or `python3 -m unittest discover -s tests`.
   - Verify camera index `/dev/video0` is live.
   - Confirm Pixhawk datalink connected at 921600 baud (`/dev/ttyTHS1`).
3. **Observability & Logging Setup**:
   - Start black-box recording before arming: `ros2 bag record -a -o rosbag_flight_$(date +%Y%m%d_%H%M%S)`.

### ✈️ Mission Operator Flight Checklist
1. Power up Jetson Orin Nano and Pixhawk.
2. Launch mission node with desired environment tier:
   - Hardware: `ros2 launch mission_planner mission_launch.py env:=hardware`
   - Simulation: `ros2 launch mission_planner mission_launch.py env:=sim`
3. Verify `MissionPlannerNode` is in `IDLE` state via `/mission_planner/status`.
4. Stand clear of propeller arc.
5. Safety Pilot issues `start` command to begin `ARMING` -> `TAKEOFF`.
6. Keep Safety Pilot RC transmitter in hand at all times for instant manual override if needed.
