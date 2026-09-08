#!/usr/bin/env python3
"""
siyi_camera_diagnostic.py
=========================
Fast SIYI A8 Mini & HM30 RTSP Video Stream Diagnostic Tool.
Tests network ping, ARP discovery, RTSP port binding, and OpenCV video frame capture.
"""

import os
import sys
import time
import socket
import subprocess

def run_cmd(cmd):
    try:
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)
        return res.returncode == 0, res.stdout.strip()
    except Exception as e:
        return False, str(e)

def test_port(ip, port=8554, timeout=1.0):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except Exception:
        return False

def main():
    print("=" * 70)
    print(" 🎥 SIYI A8 MINI 4K & HM30 DATALINK VIDEO STREAM DIAGNOSTIC 🎥")
    print("=" * 70)

    # 1. Jetson / Host IP Check
    print("\n[1/4] Checking Local Network Interface IPs...")
    ok, out = run_cmd("ip -4 addr show || ifconfig")
    if ok:
        for line in out.splitlines():
            if "inet " in line or "inet192" in line:
                print(f"  * {line.strip()}")
    else:
        print("  * Network interface listing unavailable")

    # 2. Ping SIYI Devices on Subnet 192.168.144.x
    print("\n[2/4] Pinging SIYI Devices on Subnet 192.168.144.x...")
    hosts = [
        ("SIYI A8 Mini Camera", "192.168.144.25"),
        ("SIYI HM30 Air Unit", "192.168.144.11"),
        ("SIYI HM30 Ground Unit", "192.168.144.1")
    ]
    reachable_hosts = []
    for name, ip in hosts:
        ok, _ = run_cmd(f"ping -c 1 -W 1 {ip} || ping -c 1 -t 1 {ip}")
        status = "✅ ONLINE" if ok else "❌ UNREACHABLE"
        print(f"  * {name} ({ip}): {status}")
        if ok:
            reachable_hosts.append((name, ip))

    # 3. Test RTSP Port 8554 (TCP Socket Check)
    print("\n[3/4] Checking RTSP Service (Port 8554)...")
    open_rtsp_ips = []
    for name, ip in hosts[:2]:
        is_open = test_port(ip, 8554, timeout=1.0)
        status = "✅ OPEN (Port 8554 listening)" if is_open else "❌ CLOSED / UNREACHABLE"
        print(f"  * {name} ({ip}:8554): {status}")
        if is_open:
            open_rtsp_ips.append(ip)

    # 4. Fast OpenCV RTSP Stream Grab (Only if RTSP port is open)
    print("\n[4/4] Testing OpenCV Video Frame Capture over TCP RTSP...")
    if not open_rtsp_ips:
        print("  ⚠️ Skipping OpenCV RTSP capture because Port 8554 is unreachable on 192.168.144.x.")
        print("  ℹ️ Web GCS has automatically engaged zero-lag Synthetic HUD mode.")
        success = False
    else:
        import cv2
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|max_delay;500000"
        
        rtsp_urls = [
            f"rtsp://{open_rtsp_ips[0]}:8554/main.264",
            f"rtsp://{open_rtsp_ips[0]}:8554/stream1",
            f"rtsp://{open_rtsp_ips[0]}:8554/live/0"
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
        print("⚠️ DIAGNOSTIC SUMMARY: Physical camera hardware not detected over RTSP.")
        print("   Quick Hardware Checklist on Drone:")
        print("   1. Connect Ethernet cable from SIYI HM30 Air Unit LAN to Jetson Nano Ethernet port.")
        print("   2. Run: sudo ip addr add 192.168.144.167/24 dev eth0 (or enP8p1s0).")
        print("   3. Verify solid green LINK LED on SIYI HM30 Air Unit & Ground Unit.")
    print("=" * 70)

if __name__ == "__main__":
    main()
