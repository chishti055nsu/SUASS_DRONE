# IUB Drone System Architecture & Operational Safety Guide

## 1. System Architecture Overview

```
                      Camera Image Feed (30 Hz)
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
                     │ ROS2 Topics
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
                     │ MAVROS (921600 Baud)
                     ▼
            PX4 Flight Controller
```

---

## 2. Interface Contracts (ROS2 Topics)

| Topic | Message Type | Rate | Description | Safety Critical |
|:---|:---|:---:|:---|:---:|
| `/drone_vision/detections` | `DetectionArray` | 30 Hz | YOLOv8 bounding boxes + track IDs | ❌ |
| `/drone_vision/landing_zone` | `ActionZone` | 30 Hz | Geometry (`center_pixel`, `bbox_xyxy`, `area_ratio`, `confidence`) | ✅ |
| `/drone_vision/drop_zone` | `ActionZone` | 30 Hz | Payload target geometry & position error | ✅ |
| `/precision_landing/target_pose` | `geometry_msgs/PoseStamped` | 30 Hz | Camera frame 3D offset ($dx, dy, dz$) in meters | ✅ |
| `/precision_landing/alignment_error` | `geometry_msgs/Vector3` | 30 Hz | Precision 3D error vector | ✅ |
| `/mission_planner/command` | `MissionCommand` | Event | Operator commands (`start`, `abort`, `rtl`, `terminate`) | ✅ |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 20 Hz | High-frequency continuous position setpoint | ✅ |

---

## 3. SUAS Safety Case & Failsafes

1. **Rule 5.3.1 (Manual Safety Pilot Override)**:
   - Continuously monitored via `/mavros/state`. If the human pilot flips the transmitter switch out of `OFFBOARD` into `POSCTL` or `MANUAL`, ROS2 setpoint control disengages immediately, and the state transitions to `MANUAL_OVERRIDE`.

2. **Rule 5.3.5 (Air-Gapped Onboard Control)**:
   - 100% of flight-critical perception (YOLOv8 + ArUco 3D Pose + State Machine) runs locally on the NVIDIA Jetson Orin Nano. Zero internet or cloud servers are required.

3. **Rule 5.3.8 (Emergency Flight Termination & RTL)**:
   - `terminate` command triggers instant disarm via MAVROS `/mavros/cmd/arming` service call (`value = False`).
   - Low battery ($<15\%$), telemetry heartbeat loss ($>3.0\text{s}$), or stale vision ($>2.5\text{s}$) automatically fails closed to `RETURN_HOME`.

---

## 4. Preflight & Flight Checklists

### 📋 Preflight Checklist
1. **Physical Inspection**:
   - Check propeller tightness and motor rotation direction.
   - Verify battery is wrapped in bright tape and securely latched in slide tray.
   - Confirm drone weight is $\le 35\text{ LBs}$.
   - Verify Remote ID module is powered and broadcasting.
2. **Software Verification**:
   - Run unit test suite: `python3 -m unittest discover -s tests -p "test_*.py"`.
   - Verify camera index `/dev/video0` is live.
   - Confirm Pixhawk datalink connected at 921600 baud (`/dev/ttyTHS1`).

### ✈️ Mission Operator Flight Checklist
1. Power up Jetson Orin Nano and Pixhawk.
2. Verify `MissionPlannerNode` is in `IDLE` state via `/mission_planner/status`.
3. Stand clear of propeller arc.
4. Safety Pilot issues `start` command to begin `ARMING` -> `TAKEOFF`.
5. Keep Safety Pilot RC transmitter in hand at all times for instant manual override if needed.
