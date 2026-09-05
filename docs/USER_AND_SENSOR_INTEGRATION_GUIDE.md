# IUB Drone — User & Future Sensor Integration Guide

This guide details how to operate, test, and modify the **IUB Drone SUAS ROS 2 System** on Jetson Orin Nano / Jetson Nano.

---

## 1. Operating Modes: With or Without Quadcopter

The system is designed with a hardware abstraction layer (**HAL**) so you can develop and test the entire software stack without a physical drone attached.

```
                  ┌─────────────────────────────────────────┐
                  │          mission_planner_node           │
                  └────────────────────┬────────────────────┘
                                       │ FlightController HAL
                  ┌────────────────────┴────────────────────┐
                  │                                         │
                  ▼                                         ▼
      [SimStub / Offline Mode]                   [MAVROS / Hardware Mode]
      • No FCU or telemetry needed               • Matek H743 / Pixhawk connected
      • Simulated telemetry & waypoints          • Real sensors, GPS & actuators
```

### A. Testing WITHOUT a Quadcopter (Standalone / Offline Test Mode)

In this mode, `SimStub` simulates flight controller telemetry, GPS position, and battery status.

1. **Launch Test Mode**:
   ```bash
   cd ~/ros2_ws/src/SUASS_DRONE
   bash scripts/run_offline_test.sh
   ```
2. **In a second terminal, send the START command**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic pub --once /mission_planner/command drone_vision_msgs/msg/MissionCommand "{command: 'start'}"
   ```
3. **Monitor telemetry**:
   ```bash
   ros2 topic echo /mission_planner/status
   ```

### B. Testing & Operating WITH a Quadcopter (Wireless Ground Control Mode)

In this mode, `MAVROS` bridges ROS 2 topics directly to ArduPilot / PX4 on the Matek H743 via serial (`/dev/ttyTHS1`) or USB (`/dev/ttyACM0`).

1. **Hardware Connections & ArduPilot Params**:
   - `SERIAL1_PROTOCOL = 2` (MAVLink2)
   - `SERIAL1_BAUD = 9216` (921,600 baud)
2. **Launch Hardware Flight Mode (Onboard Jetson)**:
   ```bash
   cd ~/ros2_ws/src/SUASS_DRONE
   bash scripts/run_drone_flight.sh
   ```
3. **Wireless Field Operations (From Ground Laptop / Phone / Tablet via Wi-Fi / Telemetry Datalink)**:
   - **Start Autonomous Mission**:
     ```bash
     bash scripts/send_command.sh start
     ```
   - **Return to Base (RTL)**:
     ```bash
     bash scripts/send_command.sh rtl
     ```
   - **Emergency Hold/Abort**:
     ```bash
     bash scripts/send_command.sh abort
     ```


---

## 2. 500m Range Flight & Auto Return to Base (RTL)

The mission planner controls waypoint generation, search corridors, and return-to-base triggers.

### Waypoint & Distance Configuration
Open `mission_planner/mission_planner/waypoint_manager.py`:
- `max_distance_m`: Defines the total search boundary (e.g. 500 meters).
- `altitude_m`: Cruise altitude (default: `15.0` meters).
- `grid_spacing_m`: Distance between search grid passes.

### Auto Return-To-Launch (RTL) Triggers
The state machine (`mission_planner/mission_planner/mission_state_machine.py`) automatically triggers Return to Launch under the following conditions:
1. **Mission Completion**: All waypoints reached and payload operations completed.
2. **Operator Trigger**: Sending `command: 'rtl'` on `/mission_planner/command`.
3. **Failsafe Triggers**:
   - Battery falls below `15%`.
   - Heartbeat loss with Flight Controller > 3.0s.
   - Out-of-bounds boundary breach (e.g. > 500m from origin).

---

## 3. Future Sensor Integration Guide (LiDAR, Cameras, Radar)

The architecture uses standard ROS 2 topics and modular node design, making sensor additions seamless.

```
 ┌────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
 │ LiDAR / Depth  │     │  RGB / Stereo    │     │ Ultrasonic / Radar  │
 │ (PointCloud2)  │     │  Camera Image    │     │ Depth Sensor        │
 └───────┬────────┘     └────────┬─────────┘     └──────────┬──────────┘
         │                       │                          │
         ▼                       ▼                          ▼
 ┌───────────────┐      ┌──────────────────┐     ┌─────────────────────┐
 │ /sensor/lidar │      │  /camera/image   │     │   /sensor/range     │
 └───────┬───────┘      └────────┬─────────┘     └──────────┬──────────┘
         │                       │                          │
         └───────────────────────┼──────────────────────────┘
                                 │ ROS 2 Subscription
                                 ▼
                     ┌───────────────────────┐
                     │   drone_vision /      │
                     │  obstacle_avoidance   │
                     └───────────┬───────────┘
                                 │ /drone_vision/obstacles
                                 ▼
                     ┌───────────────────────┐
                     │ mission_planner_node  │
                     └───────────┬───────────┘
```

### A. Integrating a LiDAR (3D / 2D PointCloud2)
1. **Add LiDAR ROS 2 Driver**:
   - For RPLiDAR: `ros-humble-rplidar-ros`
   - For Velodyne: `ros-humble-velodyne`
   - For Livox: `livox_ros_driver2`
2. **Subscribe to LiDAR Topic in Perception**:
   In `drone_vision/drone_vision/perception_interface.py` or a new obstacle avoidance node:
   ```python
   from sensor_msgs.msg import PointCloud2, LaserScan

   self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

   def lidar_callback(self, msg):
       # Process distances and check for obstacles within safety radius
       min_distance = min(msg.ranges)
       if min_distance < 2.0: # 2 meter safety buffer
           self.publish_obstacle_warning(min_distance)
   ```

### B. Integrating Stereo / Thermal / RealSense Cameras
1. **Camera Publisher**: Use `realsense2_camera` or standard `image_publisher`.
2. **Vision Node Update**: Point `drone_vision/drone_vision/vision_node.py` to your camera topic:
   ```python
   # Default: /camera/image_raw
   self.declare_parameter('image_topic', '/camera/color/image_raw')
   ```

### C. Integrating Radar / Sonar / Rangefinders for Terrain Following
1. ArduPilot can handle rangefinders directly via MAVLink (`RANGEFINDER` message).
2. Alternatively, subscribe to `sensor_msgs/msg/Range` in `mission_planner/mission_planner/mission_node.py` for precision low-altitude hovering.

---

## 4. How to Modify & Extend the Codebase

### Key File Locations:
* **State Machine Rules**: `mission_planner/mission_planner/mission_state_machine.py`
  - Modify state transitions, add new states (e.g. `LIDAR_AVOIDANCE`, `MAPPING`).
* **Hardware Interfacing**: `mission_planner/mission_planner/flight_controller.py`
  - Modify velocity, waypoint, or takeoff altitude parameters.
* **Target Detection & Vision**: `drone_vision/drone_vision/yolo_detector.py`
  - Change YOLO weights (e.g. `yolov8n.pt`, custom trained models).
* **ROS 2 Custom Messages**: `drone_vision_msgs/msg/`
  - Add new message definitions for extra sensors.

### Verification after modifications:
Always run the unit test suite after making code edits:
```bash
python3 -m unittest discover -s tests -p "test_*.py"
```
