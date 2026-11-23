
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
send("log 0")
send("probe_log 1", 0.2)
send("set probe_laser_ms 1000", 0.2)
send("set probe_side_verify 0", 0.2)
send("set probe_scan_mode 1", 0.2)
send("set probe_side_left 1", 0.2)
send("set probe_side_us 2400", 0.2)
send("outtest servo 2400 30000", 0.1)
time.sleep(0.9)
v1 = read_us(); v2 = read_us()
print("PRECHECK_LEFT:", v1, v2)
if not (80 <= v1 <= 400):
    print("NOT IN VIEW (need 80..350) -> abort, nothing moved")
    s.close(); raise SystemExit
send("outtest stop", 0.15)
# immediately start; no human gap
s.reset_input_buffer(); send("side", 0.15)
out=[]; end=time.time()+70
while time.time()<end:
    l=s.readline()
    if not l: continue
    t=l.decode("utf-8","replace").rstrip(); out.append(t)
    if not t.startswith("PROBE,"): print(t)
    if ("EVENT" in t and "STOPPED" in t) or "DENIED" in t: break
fn="data/m4_gate0_side_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+".log"
open(fn,"w",encoding="utf-8").write("\n".join(out)+"\n")
print("SAVED", fn)
for x in out:
    if x.startswith(("PROBE_SIDE_LOCK","PROBE_SIDE_MID","PROBE_SIDE_FAIL","PROBE_SIDE_EDGE","EVENT")):
        print("KEY:", x)
s.close()
