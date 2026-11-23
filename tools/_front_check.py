#!/usr/bin/env python3
"""读车头中心 1500us 的超声距离，用于确认 15~18 cm 摆位。"""
import serial, time, os, sys

PORT = os.environ.get("M4_PORT", os.environ.get("M3_PORT", "COM3"))
s = serial.Serial(PORT, 115200, timeout=0.3)
try:
    s.reset_input_buffer()
    s.write(b"log 0\r\n")
    time.sleep(0.3)
    s.write(b"scan 100 50 120\r\n")
    end = time.time() + 3.0
    pts = []
    while time.time() < end:
        line = s.readline()
        if not line:
            continue
        t = line.decode("utf-8", "replace").strip()
        if t.startswith("SCAN_PT,"):
            parts = t.split(",")
            if len(parts) >= 4:
                pts.append((int(parts[1]), int(parts[2]), int(parts[3])))
finally:
    s.close()

center = [p for p in pts if p[0] == 1500]
if center:
    print(center[-1][1])
else:
    print(-1)
