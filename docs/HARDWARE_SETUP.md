# Sensor & Camera Hardware Verification Guide — Jetson Orin Nano

This document describes the setup, configuration, and empirical verification of the three perception devices connected to the **NVIDIA Jetson Orin Nano Developer Kit**: the **SIYI A8 Mini camera/gimbal**, **Intel RealSense D455 depth camera**, and **Benewake TFmini-S LiDAR**.

---

## 1. Hardware & System Specifications

* **Primary Compute Platform**: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
* **Operating System / JetPack**: JetPack / L4T `R36.4.7` (Ubuntu 22.04 LTS)
* **Python Version**: `3.10.12`

```
                          ┌─────────────────────────────┐
                          │   NVIDIA Jetson Orin Nano   │
                          │   JetPack / L4T R36.4.7     │
                          └──────────────┬──────────────┘
                                         │
         ┌───────────────────────────────┼───────────────────────────────┐
         │ Ethernet                      │ USB 3.0                       │ USB 2.0 (CP210x)
         ▼                               ▼                               ▼
┌──────────────────┐            ┌──────────────────┐            ┌──────────────────┐
│  SIYI A8 Mini    │            │  Intel RealSense │            │ Benewake TFmini-S│
│  Camera / Gimbal │            │  D455 Depth Cam  │            │ LiDAR Rangefinder│
└──────────────────┘            └──────────────────┘            └──────────────────┘
```

---

## 2. SIYI A8 Mini Camera & Gimbal

### 2.1 Network Connection & Configuration
* **Physical Interface**: Ethernet port (`enP8p1s0`)
* **Network Subnet**: `192.168.144.x`
* **Jetson Static IP Setup**:
  ```bash
  sudo ip addr flush dev enP8p1s0
  sudo ip addr add 192.168.144.167/24 dev enP8p1s0
  sudo ip link set enP8p1s0 up
  ```
* **Device Addresses**:
  * **Jetson Address**: `192.168.144.167/24`
  * **A8 Mini Camera Address**: `192.168.144.25`
  * **RTSP Endpoint**: `rtsp://192.168.144.25:8554/main.264`

### 2.2 Network Verification
```bash
# ARP check
arping -I enP8p1s0 192.168.144.25

# Ping latency test
ping -c 5 192.168.144.25
```

### 2.3 RTSP Stream Specifications & GStreamer Pipeline
Although the RTSP path ends in `.264`, `ffprobe` confirms the stream is **HEVC/H.265**:
* **Codec**: `hevc` (Main Profile)
* **Resolution**: `1280x720`
* **Frame Rate**: `25 fps`

#### Hardware-Accelerated GStreamer Pipeline (NVIDIA NVMM Decoder):
```bash
gst-launch-1.0 -v \
  rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=100 protocols=tcp \
  ! rtph265depay \
  ! h265parse \
  ! nvv4l2decoder \
  ! nv3dsink
```

---

## 3. Intel RealSense D455 Depth Camera

### 3.1 USB 3.0 Connection Verification
Connected directly via USB 3.0:
```bash
lsusb -d 8086:0b5c
# Bus 002 Device 003: ID 8086:0b5c Intel Corp. Intel(R) RealSense(TM) Depth Camera 455
```
Check speed (must report `5000M` for USB 3 SuperSpeed):
```bash
lsusb -t
```

### 3.2 Building `librealsense` from Source (RSUSB Backend)
Build `librealsense` with the `RSUSB` backend for maximum stability on Jetson L4T:

```bash
# Install dependencies
sudo apt update && sudo apt install -y \
    git cmake build-essential libssl-dev libusb-1.0-0-dev \
    libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Clone & Build librealsense with Python Bindings
cd ~
git clone https://github.com/realsenseai/librealsense.git
cd ~/librealsense
mkdir build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_GRAPHICAL_EXAMPLES=ON \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DPYTHON_EXECUTABLE=$(which python3)

make -j$(nproc)
sudo make install
sudo ldconfig
```

### 3.3 Udev Rules Configuration
```bash
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3.4 Python Verification
```bash
python3 -c "import pyrealsense2 as rs; print('RealSense Python OK:', rs.__version__)"
# Output: RealSense Python OK: 2.58.4
```

---

## 4. Benewake TFmini-S LiDAR Rangefinder

### 4.1 USB-to-UART Serial Converter
* **Converter**: Silicon Labs `CP210x` USB-to-UART Bridge (`ID 10c4:ea60`)
* **Device Node**: `/dev/ttyUSB0`
* **User Permissions**:
  ```bash
  sudo usermod -aG dialout $USER
  ```

### 4.2 UART Protocol & 9-Byte Binary Frame Decoding
* **Baud Rate**: `115200` baud (8N1)
* **Frame Structure** (9 Bytes):
  `0x59 0x59 | Dist_L Dist_H | Strength_L Strength_H | Temp_L Temp_H | Checksum`
* **Distance Formula**: `Distance_cm = Dist_L + (Dist_H << 8)`
* **Checksum Verification**: `Checksum == sum(frame[0:8]) & 0xFF`

### 4.3 Python Reader (`pyserial`)
```python
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

while True:
    header = ser.read(2)
    if header == b'\x59\x59':
        data = ser.read(7)
        if len(data) == 7:
            raw_frame = b'\x59\x59' + data
            checksum = sum(raw_frame[:8]) & 0xFF
            if checksum == raw_frame[8]:
                distance_cm = raw_frame[2] | (raw_frame[3] << 8)
                distance_m = distance_cm / 100.0
                print(f"TFmini-S Distance: {distance_m:.2f} m")
```

---

## 5. Summary Perception Sensor Matrix

| Sensor Device | Hardware Interface | Jetson Port | Driver / Interface | Status |
|:---|:---|:---|:---|:---:|
| **SIYI A8 Mini** | Ethernet | `enP8p1s0` | RTSP HEVC/H.265 (`rtsp://192.168.144.25:8554/main.264`) | ✅ **OPERATIONAL** |
| **Intel RealSense D455** | USB 3.0 (5000M) | USB 3 Port | `pyrealsense2` v2.58.4 (RSUSB Backend) | ✅ **OPERATIONAL** |
| **Benewake TFmini-S** | USB-to-UART (CP210x) | `/dev/ttyUSB0` | `pyserial` (115200 8N1, 9-byte binary parser) | ✅ **OPERATIONAL** |

---

## 6. Official References & Manuals
* [SIYI A8 Mini User Manual v1.6](https://siyi.biz/siyi_file/A8%20mini/A8%20mini%20v1.6.pdf)
* [Benewake TFmini-S Product Manual](https://en.benewake.com/uploadfiles/2023/03/20230331102952719.pdf)
* [Intel RealSense Jetson Installation Guide](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)
