
import serial, time, datetime, os, re
s = serial.Serial("COM3", 115200, timeout=0.2, write_timeout=1.0)
time.sleep(0.2)
s.reset_input_buffer()
def send(c, w=0.3):
    s.write((c+"\r\n").encode()); s.flush(); time.sleep(w)
def read_us():
    s.reset_input_buffer(); send("us", 0.0)
    end=time.time()+0.7
    while time.time()<end:
        l=s.readline()
        if not l: continue
        t=l.decode("utf-8","replace").rstrip()
        m=re.search(r"obs_mm=(\d+),obs_ok=(\d+)", t)
        if m: return int(m.group(1))
    return -1
def wait_line(prefix, timeout):
    end=time.time()+timeout
    while time.time()<end:
        l=s.readline()
        if not l: continue
        t=l.decode("utf-8","replace").rstrip()
        if t.startswith(prefix): return t
    return None
send("log 0")
# 1) deliberate 10 deg skew (nose away from the face = increase h when driving forward)
send("turn 10 50", 0.2)
print("TURN:", wait_line("MOVE_DONE", 8.0))
time.sleep(0.4)
# 2) config: face at 600us side, right-hand geometry
for cmd in ["set probe_laser_ms 1000","set probe_side_verify 0","set probe_scan_mode 1",
            "set probe_side_left 0","set probe_side_us 600"]:
    send(cmd, 0.25)
# 3) keep servo live at 600 straight into the probe
send("outtest servo 600 30000", 0.1)
time.sleep(1.0)
v1=read_us(); v2=read_us()
print("PRECHECK_600:", v1, v2)
if not (80 <= v1 <= 400):
    print("NOT IN VIEW -> abort"); send("outtest stop",0.1); s.close(); raise SystemExit
s.reset_input_buffer(); send("side", 0.1)
out=[]; end=time.time()+70
while time.time()<end:
    l=s.readline()
    if not l: continue
    t=l.decode("utf-8","replace").rstrip(); out.append(t)
    if not t.startswith("PROBE,"): print(t)
    if ("EVENT" in t and "STOPPED" in t) or "DENIED" in t: break
fn="data/m4_side_skew10_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+".log"
open(fn,"w",encoding="utf-8").write("\n".join(out)+"\n")
print("SAVED", fn)
for x in out:
    if x.startswith(("PROBE_SIDE_LOCK","PROBE_SIDE_MID","PROBE_SIDE_FAIL","PROBE_SIDE_EDGE","PROBE_TURN","EVENT")):
        print("KEY:", x)
s.close()
