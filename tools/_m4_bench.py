#!/usr/bin/env python3
"""M4 / Gate 0 台架串口助手：status / turnpar / faces2 / side / leg / m3 / raw。仅本机使用。

Gate 0 判据见根目录 M4计划_评审与修订.md §1.5：
  G0-2 TURN_PAR 实车：PROBE_TURN,tag=PAR 的 ok=1 且 |ach-cmd|<=8，随后 SIDE_LOCK ok=1
  G0-3 窄→宽 LEG：不重新 side，直接 leg，下一面 SIDE_LOCK ok=1、w>30、h∈[150,220]
  G0-4 faces=2 自动：面1 AIM→TURN_PAR→LOCK→SCAN→MOVE→LASER→LEG→面2 LOCK→…→COMPLETE

用法：
  python tools/_m4_bench.py status
  python tools/_m4_bench.py turnpar      # faces=1，从 15~18cm 触发 probe，抓 TURN_PAR
  python tools/_m4_bench.py faces2       # faces=2 自动全流程
  python tools/_m4_bench.py side         # 侧扫（人工把车摆到面外、箱在左）
  python tools/_m4_bench.py leg          # 不重新 side，只跑 LEG（窄→宽用）
  python tools/_m4_bench.py m3           # side + leg（宽→窄回归）
  python tools/_m4_bench.py raw "status"
"""
import serial, sys, time, datetime, os, re

PORT = os.environ.get("M4_PORT", os.environ.get("M3_PORT", "COM3"))
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


def wait_stopped(s, timeout):
    """读到 EVENT ... STOPPED 或 DENIED 为止；返回全部行。"""
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


def save(lines, tag):
    os.makedirs("data", exist_ok=True)
    fn = "data/m4_" + tag + "_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".log"
    with open(fn, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("SAVED", fn)
    return fn


def config(s):
    for cmd in ["log 0", "probe_log 1",
                "set probe_laser_ms 1000", "set probe_side_verify 0",
                "set probe_scan_mode 1", "set probe_side_left 1", "set probe_side_us 2400"]:
        send(s, cmd, 0.25)


def get_status(s):
    send(s, "status", 0.4)
    lines = collect(s, 0.7)
    for line in lines:
        if line.startswith("STATE="):
            print(line)
            return line
    return ""


def require_stopped(s):
    st = get_status(s)
    if "STOPPED" not in st:
        print("ABORT: not STOPPED")
        s.close()
        return False
    return True


def key_lines(lines, prefixes):
    for x in lines:
        if x.startswith(prefixes):
            print("KEY:", x)


def parse_turnpar(lines):
    turn = [x for x in lines if x.startswith("PROBE_TURN,tag=PAR")]
    lock = [x for x in lines if x.startswith("PROBE_SIDE_LOCK")]
    if not turn:
        print("RESULT: NO PROBE_TURN,tag=PAR (AIM 没成功或没进 TURN_PAR)")
        return False
    t = turn[-1]
    m_cmd = re.search(r"cmd=([-0-9.]+)", t)
    m_ach = re.search(r"ach=([-0-9.]+)", t)
    m_ok = re.search(r"ok=([01])", t)
    ok = bool(m_ok and m_ok.group(1) == "1")
    derr = None
    if m_cmd and m_ach:
        derr = abs(float(m_ach.group(1)) - float(m_cmd.group(1)))
        ok = ok and derr <= 8.0
    lock_ok = bool(lock and re.search(r"ok=1", lock[-1]))
    print("RESULT: TURN_PAR ok=%s derr=%.1f lock_ok=%s" % (ok, derr if derr is not None else -1, lock_ok))
    return ok and lock_ok


def status():
    s = open_port()
    send(s, "log 0", 0.3)
    send(s, "probe_log 1", 0.3)
    get_status(s)
    send(s, "us", 0.5)
    print("\n".join(collect(s, 0.8)))
    s.close()


def turnpar():
    s = open_port()
    config(s)
    send(s, "set probe_faces 1", 0.25)
    if not require_stopped(s):
        return
    print("===== PROBE (faces=1, TURN_PAR) =====")
    s.reset_input_buffer()
    send(s, "probe", 0.2)
    lines = wait_stopped(s, 50.0)
    save(lines, "gate0_turnpar")
    key_lines(lines, ("PROBE_FACE,", "PROBE_TURN", "PROBE_SIDE_LOCK", "PROBE_SIDE_MID", "PROBE_LEG", "EVENT"))
    parse_turnpar(lines)
    s.close()


def faces2():
    s = open_port()
    config(s)
    send(s, "set probe_faces 2", 0.25)
    if not require_stopped(s):
        return
    print("===== PROBE (faces=2 automatic) =====")
    s.reset_input_buffer()
    send(s, "probe", 0.2)
    lines = wait_stopped(s, 150.0)
    save(lines, "gate0_faces2")
    key_lines(lines, ("PROBE_FACE,", "PROBE_TURN", "PROBE_SIDE_LOCK", "PROBE_SIDE_MID",
                      "PROBE_LEG", "PROBE_EVENT", "EVENT"))
    print("RESULT: faces=%d stop=%s" % (
        len([x for x in lines if x.startswith("PROBE_SIDE_MID")]),
        any("EVENT" in x and "STOPPED" in x for x in lines)))
    s.close()


def side(push=False):
    s = open_port()
    config(s)
    if not require_stopped(s):
        return
    print("===== SIDE =====")
    s.reset_input_buffer()
    send(s, "side push" if push else "side", 0.2)
    lines = wait_stopped(s, 60.0)
    save(lines, "gate0_side")
    key_lines(lines, ("PROBE_SIDE_LOCK", "PROBE_SIDE_MID", "PROBE_SIDE_FAIL", "PROBE_LEG", "EVENT"))
    s.close()


def leg():
    s = open_port()
    send(s, "log 0", 0.25)
    send(s, "probe_log 1", 0.25)
    if not require_stopped(s):
        return
    print("===== LEG (no re-side, keep face lock) =====")
    s.reset_input_buffer()
    send(s, "leg", 0.2)
    lines = wait_stopped(s, 80.0)
    save(lines, "gate0_leg")
    key_lines(lines, ("LEG:", "PROBE_LEG", "PROBE_SIDE_LOCK", "PROBE_SIDE_MID", "PROBE_SIDE_FAIL", "PROBE_TURN", "EVENT"))
    s.close()


def m3():
    side()
    time.sleep(0.5)
    leg()


def raw():
    s = open_port()
    cmd = sys.argv[2]
    send(s, cmd, 0.2)
    print("\n".join(collect(s, 2.0)))
    s.close()


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "status"
    if mode == "status": status()
    elif mode == "turnpar": turnpar()
    elif mode == "faces2": faces2()
    elif mode == "side": side()
    elif mode == "sidepush": side(True)
    elif mode == "leg": leg()
    elif mode == "m3": m3()
    elif mode == "raw": raw()
    else: print("usage: status|turnpar|faces2|side|sidepush|leg|m3|raw <cmd>")
