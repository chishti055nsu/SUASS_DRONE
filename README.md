# IUB Drone — Production SUAS ROS 2 System for Jetson Nano & Matek H743-Wing V3

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![NVIDIA Jetson](https://img.shields.io/badge/NVIDIA-Jetson%20Orin%20Nano-76B900.svg)](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
[![Matek H743](https://img.shields.io/badge/FCU-Matek%20H743--Wing%20V3-red.svg)](http://www.mateksys.com/?portfolio=h743-wing-v3)
[![SUAS Compliant](https://img.shields.io/badge/SUAS%20Rules-Section%205.3%20Compliant-success.svg)](https://robonation.gitbook.io/suas-resources/)

Autonomous quadcopter mission system combining **YOLOv8** (real-time GPU detection), **ArUco Precision Target Tracking**, and **Gemma 4** (multimodal LLM scene reasoning), publishing ROS 2 Humble topics and interfacing with the **Matek H743-Wing V3** flight controller via MAVROS.

---

## 🔌 Hardware Setup: Matek H743-Wing V3 + Jetson Nano

### 1. Physical Wiring

| Matek H743-Wing V3 Pad | Jetson Nano Pin / Header | Description |
|---|---|---|
| **`Tx1`** (UART1 TX) | **Pin 10** (`RXD` / `/dev/ttyTHS1`) | MAVLink Telemetry RX |
| **`Rx1`** (UART1 RX) | **Pin 8** (`TXD` / `/dev/ttyTHS1`) | MAVLink Telemetry TX |
| **`GND`** | **Pin 6** (`GND`) | Common Ground |

*(Alternatively, connect the Matek H743 USB-C port directly to the Jetson Nano USB port using `/dev/ttyACM0` at `115200` baud).*

### 2. ArduPilot Parameters (Set via Mission Planner / QGroundControl)
- `SERIAL1_PROTOCOL = 2` (MAVLink2)
- `SERIAL1_BAUD = 9216` (921,600 baud)
- `ARMING_CHECK = 1`

---

## 📦 Production ROS 2 Package Structure

```text
IUB_DRONE/
├── drone_vision_msgs/          # ROS 2 Custom Message Definitions
│   └── msg/
│       ├── ActionZone.msg
│       ├── MissionStatus.msg
│       ├── MissionCommand.msg
│       ├── DetectedObject.msg
│       ├── DetectionArray.msg
│       ├── ObstacleArray.msg
│       └── SceneAnalysis.msg
│
├── drone_vision/               # YOLOv8 + Gemma Vision Node
│   ├── drone_vision/
│   │   ├── vision_node.py      # Main ROS 2 vision publisher
│   │   ├── yolo_detector.py    # GPU / TensorRT YOLOv8 detector
│   │   ├── gemma_analyzer.py   # Onboard Gemma e4b LLM client
│   │   └── perception_interface.py # Unified target detection API
│   └── launch/
│       └── full_system.launch.py
│
├── precision_landing/          # ArUco Marker Detector & Precision Landing
│   └── precision_landing/
│       └── precision_node.py  # Publishes /precision_landing/target_pose & target_locked
│
├── mission_planner/            # SUAS State Machine & Flight Controller HAL
│   ├── mission_planner/
│   │   ├── mission_node.py          # ROS 2 Mission Planner Node
│   │   ├── mission_state_machine.py # Event-driven FSM
│   │   ├── flight_controller.py     # MAVROS / Hardware Abstraction Layer
│   │   └── waypoint_manager.py      # Search corridor grid generator
│   └── launch/
│       └── full_system.launch.py    # Production Launch File
│
└── simulation/                 # Optional Desktop/Mac MuJoCo Sim Suite
    ├── skydio_x2_sim.py        # 3D MuJoCo Physics Viewer
    └── sim_system.launch.py    # Unified Sim Launch
```

---

## ⚡ Deployment Instructions for Jetson Nano + Matek H743

### Step 1: Clone Repository into ROS 2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/chishti055nsu/SUASS_DRONE.git
```

### Step 2: Build Workspace with `colcon`
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash
```

---

## ✈️ Running on Matek H743-Wing V3 Hardware

### Step 1: Launch MAVROS (Connected to Matek H743)
- **Over UART1 (`/dev/ttyTHS1` @ 921,600 baud)**:
  ```bash
  ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyTHS1:921600
  ```
- **Over USB-C (`/dev/ttyACM0` @ 115,200 baud)**:
  ```bash
  ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyACM0:115200
  ```

### Step 2: Launch Autonomous Production System
In a new terminal:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mission_planner full_system.launch.py use_mavros:=true
```

---

## 🎮 Real-Time Commands & Monitoring

- **Check Mission Status Telemetry**:
  ```bash
  ros2 topic echo /mission_planner/status
  ```

- **Check ArUco Target Tracking**:
  ```bash
  ros2 topic echo /precision_landing/target_pose
  ```

- **Trigger Mission Commands**:
  - **Start Mission**:
    ```bash
    ros2 topic pub --once /mission_planner/command drone_vision_msgs/msg/MissionCommand "{command: 'start'}"
    ```
  - **Abort / Hold**:
    ```bash
    ros2 topic pub --once /mission_planner/command drone_vision_msgs/msg/MissionCommand "{command: 'abort'}"
    ```
  - **Return to Home (RTL)**:
    ```bash
    ros2 topic pub --once /mission_planner/command drone_vision_msgs/msg/MissionCommand "{command: 'rtl'}"
    ```

---

## 🧪 Unit & Integration Verification
```bash
python3 -m unittest discover -s tests -p "test_*.py"
```
**Result**: `21/21 Automated Tests Passed OK`.
