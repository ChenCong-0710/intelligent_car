#!/usr/bin/env python3
"""M3 台架串口助手：check / side / leg / m3 / dirscan / raw。仅本机使用。"""
import serial, sys, time, datetime, os, re, math

PORT = os.environ.get("M3_PORT", "COM3")
BAUD = 115200

def open_port():
    s = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=1.0)
    time.sleep(0.15)
    s.reset_input_buffer()
    return s

def send(s, cmd, wait=0.35):
    s.write((cmd + "\r\n").encode("utf-8"))
    s.flush()
    time.sleep(wait)

def collect(s, seconds):
    end = time.time() + seconds
    buf = []
    while time.time() < end:
        line = s.readline()
        if line:
            buf.append(line.decode("utf-8", "replace").rstrip("\r\n"))
    return buf

def until(s, patterns, timeout):
    end = time.time() + timeout
    lines = []
    while time.time() < end:
        line = s.readline()
        if not line:
            continue
        text = line.decode("utf-8", "replace").rstrip("\r\n")
        lines.append(text)
        print(text)
        for p in patterns:
            if p in text:
                return text, lines
    return None, lines

def save(lines, tag):
    os.makedirs("data", exist_ok=True)
    fn = "data/m3_" + tag + "_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".log"
    with open(fn, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("SAVED", fn)

def check():
    s = open_port()
    send(s, "log 0", 0.3)
    send(s, "probe_log 1", 0.3)
    send(s, "status", 0.5)
    print("===== STATUS =====")
    print("\n".join(collect(s, 0.8)))
    send(s, "us", 0.5)
    print("===== US =====")
    print("\n".join(collect(s, 0.8)))
    send(s, "help", 0.5)
    print("===== HELP(LEG?) =====")
    h = collect(s, 0.8)
    for line in h:
        if "LEG" in line or "SIDE-BENCH" in line:
            print(line)
    s.close()

def run_until_stopped(s, timeout):
    end = time.time() + timeout
    lines = []
    while time.time() < end:
        line = s.readline()
        if not line:
            continue
        t = line.decode("utf-8", "replace").rstrip("\r\n")
        lines.append(t)
        if not t.startswith("PROBE,"):
            print(t)
        if ("EVENT" in t and "STOPPED" in t) or "DENIED" in t:
            return lines
    print("TIMEOUT waiting for STOPPED")
    return lines

def side():
    s = open_port()
    send(s, "log 0", 0.3)
    send(s, "probe_log 1", 0.3)
    for cmd in ["set probe_laser_ms 1000", "set probe_side_verify 0",
                "set probe_scan_mode 1", "set probe_side_left 1", "set probe_side_us 2400"]:
        send(s, cmd, 0.25)
    send(s, "status", 0.4)
    st = collect(s, 0.6)
    print("\n".join(st))
    if not any("STOPPED" in x for x in st):
        print("ABORT: not STOPPED")
        s.close(); return
    s.reset_input_buffer()
    send(s, "side", 0.2)
    lines = run_until_stopped(s, 55.0)
    save(lines, "side")
    for x in lines:
        if x.startswith(("PROBE_SIDE_LOCK", "PROBE_SIDE_MID", "PROBE_SIDE_FAIL", "PROBE_LEG", "EVENT")):
            print("KEY:", x)
    s.close()

def leg():
    s = open_port()
    send(s, "log 0", 0.3)
    send(s, "probe_log 1", 0.3)
    s.reset_input_buffer()
    send(s, "leg", 0.2)
    lines = run_until_stopped(s, 60.0)
    save(lines, "leg")
    for x in lines:
        if x.startswith(("LEG:", "PROBE_LEG", "PROBE_SIDE_LOCK", "PROBE_SIDE_MID", "PROBE_SIDE_FAIL", "EVENT")):
            print("KEY:", x)
    s.close()

def dirscan():
    """方向判定：side 定位 -> turn +20 -> 再 SIDE_LOCK 测 h。
    h 变小 = 车朝箱子（左）转；h 变大 = 背离箱子（右）转。"""
    s = open_port()
    send(s, "log 0", 0.3)
    send(s, "probe_log 1", 0.3)
    for cmd in ["set probe_laser_ms 1000", "set probe_side_verify 0",
                "set probe_scan_mode 1", "set probe_side_left 1", "set probe_side_us 2400"]:
        send(s, cmd, 0.25)
    send(s, "status", 0.4)
    st = collect(s, 0.6)
    print("\n".join(st))
    if not any("STOPPED" in x for x in st):
        print("ABORT: not STOPPED")
        s.close(); return
    s.reset_input_buffer()
    print("===== SIDE 1 (baseline) =====")
    send(s, "side", 0.2)
    lines = run_until_stopped(s, 55.0)
    save(lines, "dirscan_side1")
    h0 = None
    for x in lines:
        if x.startswith("PROBE_SIDE_LOCK"):
            m = re.search(r"h=([0-9.]+)", x)
            if m: h0 = float(m.group(1))
    print("H0", h0)
    if h0 is None:
        print("ABORT: no SIDE_LOCK h")
        s.close(); return
    print("===== TURN +20 =====")
    s.reset_input_buffer()
    send(s, "turn 20 60", 0.2)
    until(s, ["MOVE_DONE", "MOVE ERROR"], 6.0)
    time.sleep(0.3)
    print("===== SIDE 2 (after turn) =====")
    s.reset_input_buffer()
    send(s, "side", 0.2)
    line, l2 = until(s, ["PROBE_SIDE_LOCK"], 10.0)
    h1 = None
    if line:
        m = re.search(r"h=([0-9.]+)", line)
        if m: h1 = float(m.group(1))
    send(s, "stop", 0.2)
    print("\n".join(collect(s, 0.8)))
    print("H0=%.1f H1=%s" % (h0, h1))
    if h1 is not None:
        dh = h1 - h0
        # 纯 20° 转角本身会把侧距放大 1/cos20；朝箱子转再减 pan_x*sin20，背离则加。
        mid = h0 * (1.0 / math.cos(math.radians(20.0)) - 1.0)
        print("DH=%.1f MID_DH=%.1f" % (dh, mid))
        if dh < mid - 3.0:
            print("VERDICT: +turn 朝箱子（车左转）——对调正确")
        elif dh > mid + 3.0:
            print("VERDICT: +turn 背离箱子（车右转）——对调仍反")
        else:
            print("VERDICT: 不明确（贴近纯转角预期），需重试")
    s.close()

def m3():
    """一次跑完 side + leg：先侧扫定位，再绕到下一面，最后看下一面的 SIDE_LOCK。"""
    s = open_port()
    send(s, "log 0", 0.3)
    send(s, "probe_log 1", 0.3)
    # 2026-11-22 21:36：BOARD_SWAP_LR 改回 0，命令 +90 = 实车左转（CCW）；LEG 的 14° ALIGN 已永久删除。
    # 箱体在左用 side_left=1：DRIVE1 直行 → TURN +90 就是左转，不要再翻符号。
    for cmd in ["set probe_laser_ms 1000", "set probe_side_verify 0",
                "set probe_scan_mode 1", "set probe_side_left 1", "set probe_side_us 2400"]:
        send(s, cmd, 0.25)
    send(s, "status", 0.4)
    st = collect(s, 0.6)
    print("\n".join(st))
    if not any("STOPPED" in x for x in st):
        print("ABORT: not STOPPED")
        s.close(); return
    s.reset_input_buffer()
    print("===== SIDE =====")
    send(s, "side", 0.2)
    lines = run_until_stopped(s, 55.0)
    save(lines, "m3_side")
    if not any(x.startswith("PROBE_SIDE_MID") for x in lines):
        print("ABORT: no PROBE_SIDE_MID, skip leg")
        s.close(); return
    k = None
    for x in lines:
        if x.startswith("PROBE_SIDE_MID"):
            m = re.search(r"k=([-0-9.]+)", x)
            if m: k = float(m.group(1))
    if k is not None and abs(k) > 0.30:
        # ALIGN 已删除：k 只作诊断，不再中断；DRIVE1 保持当前航向直行
        print("WARN: |k|=%.3f > 0.30 (diagnostic only, LEG will NOT correct it)" % k)
    time.sleep(0.5)
    s.reset_input_buffer()
    print("===== LEG =====")
    send(s, "leg", 0.2)
    lines2 = run_until_stopped(s, 80.0)
    save(lines + lines2, "m3_leg")
    for x in lines2:
        if x.startswith(("LEG:", "PROBE_LEG", "PROBE_SIDE_LOCK", "PROBE_SIDE_MID",
                         "PROBE_SIDE_FAIL", "PROBE_TURN", "EVENT")):
            print("KEY:", x)
    s.close()

def raw():
    s = open_port()
    cmd = sys.argv[2]
    send(s, cmd, 0.2)
    print("\n".join(collect(s, 1.5)))
    s.close()

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    if mode == "check": check()
    elif mode == "side": side()
    elif mode == "leg": leg()
    elif mode == "m3": m3()
    elif mode == "dirscan": dirscan()
    elif mode == "raw": raw()
    else: print("usage: check|side|leg|m3|dirscan|raw <cmd>")
