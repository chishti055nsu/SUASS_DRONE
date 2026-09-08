#!/usr/bin/env python3
"""
siyi_camera_diagnostic.py
=========================
SIYI A8 Mini & HM30 RTSP Video Stream Diagnostic Tool.
Tests network ping, ARP discovery, RTSP port binding, and OpenCV video frame capture.
"""

import os
import sys
import time
import socket
import subprocess

def run_cmd(cmd):
    try:
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=3)
        return res.returncode == 0, res.stdout.strip()
    except Exception as e:
        return False, str(e)

def main():
    print("=" * 70)
    print(" 🎥 SIYI A8 MINI 4K & HM30 DATALINK VIDEO STREAM DIAGNOSTIC 🎥")
    print("=" * 70)

    # 1. Jetson IP Check
    print("\n[1/4] Checking Jetson Network Interface IPs...")
    ok, out = run_cmd("ip -4 addr show")
    print(out if ok else "Failed to list IP addresses")

    # 2. Ping SIYI Camera & HM30 Air Unit
    print("\n[2/4] Pinging SIYI Devices on Subnet 192.168.144.x...")
    hosts = [
        ("SIYI A8 Mini Camera", "192.168.144.25"),
        ("SIYI HM30 Air Unit", "192.168.144.11"),
        ("SIYI HM30 Ground Unit", "192.168.144.1")
    ]
    for name, ip in hosts:
        ok, _ = run_cmd(f"ping -c 2 -w 2 {ip}")
        status = "✅ ONLINE" if ok else "❌ UNREACHABLE"
        print(f"  * {name} ({ip}): {status}")

    # 3. Test RTSP Port 8554 (TCP Socket Check)
    print("\n[3/4] Checking RTSP Service (Port 8554)...")
    for name, ip in hosts[:2]:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        result = sock.connect_ex((ip, 8554))
        sock.close()
        status = "✅ OPEN (Port 8554 listening)" if result == 0 else "❌ CLOSED / TIMEOUT"
        print(f"  * {name} ({ip}:8554): {status}")

    # 4. Attempt OpenCV RTSP Stream Grab (TCP Mode)
    print("\n[4/4] Testing OpenCV Video Frame Capture over TCP RTSP...")
    import cv2
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|max_delay;500000"
    
    rtsp_urls = [
        "rtsp://192.168.144.25:8554/main.264",
        "rtsp://192.168.144.25:8554/stream1",
        "rtsp://192.168.144.25:8554/live/0",
        "rtsp://192.168.144.11:8554/main.264"
    ]

    success = False
    for url in rtsp_urls:
        print(f"  Trying endpoint: {url} ... ", end="", flush=True)
        try:
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    h, w, c = frame.shape
                    print(f"✅ SUCCESS! Frame Captured: {w}x{h} ({c} channels)")
                    success = True
                    cap.release()
                    break
                else:
                    print("❌ Opened but failed to decode frame")
            else:
                print("❌ Failed to open RTSP stream")
            cap.release()
        except Exception as e:
            print(f"❌ Exception: {e}")

    print("\n" + "=" * 70)
    if success:
        print("🎉 DIAGNOSTIC PASSED: SIYI A8 Mini video feed is streaming correctly!")
    else:
        print("⚠️ DIAGNOSTIC WARNING: Camera hardware not detected over RTSP.")
        print("   Troubleshooting Recommendations:")
        print("   1. Verify Ethernet cable is clicked into SIYI HM30 Air Unit LAN port.")
        print("   2. Run: sudo ip addr add 192.168.144.167/24 dev eth0 (or enP8p1s0).")
        print("   3. Check SIYI FPV app on phone connected to HM30 Ground Unit to ensure camera IP is 192.168.144.25.")
    print("=" * 70)

if __name__ == "__main__":
    main()
