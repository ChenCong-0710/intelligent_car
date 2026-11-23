
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
for cmd in ["set probe_laser_ms 1000","set probe_side_verify 0","set probe_scan_mode 1"]:
    send(cmd, 0.25)
res={}
for us in (2400, 600):
    send("outtest servo %d 30000" % us, 0.1)
    time.sleep(0.9)
    res[us]=(read_us(), read_us())
    send("outtest stop", 0.1)
    time.sleep(0.4)
print("PRECHECK: 2400=%s  600=%s" % (res[2400], res[600]))
def ok(v): return 80 <= v <= 400
side = 2400 if ok(res[2400][0]) else (600 if ok(res[600][0]) else None)
if side is None:
    print("NO VALID SIDE IN 80..400 -> abort, car not moved")
    s.close(); raise SystemExit
left = 1 if side == 2400 else 0
print("CHOSEN side_us=%d left=%d" % (side, left))
send("set probe_side_left %d" % left, 0.25)
send("set probe_side_us %d" % side, 0.25)
send("status", 0.3)
s.reset_input_buffer(); send("side", 0.15)
out=[]; end=time.time()+70
while time.time()<end:
    l=s.readline()
    if not l: continue
    t=l.decode("utf-8","replace").rstrip(); out.append(t)
    if not t.startswith("PROBE,"): print(t)
    if ("EVENT" in t and "STOPPED" in t) or "DENIED" in t: break
fn="data/m4_side_auto_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+".log"
open(fn,"w",encoding="utf-8").write("\n".join(out)+"\n")
print("SAVED", fn)
for x in out:
    if x.startswith(("PROBE_SIDE_LOCK","PROBE_SIDE_MID","PROBE_SIDE_FAIL","PROBE_SIDE_EDGE","EVENT")):
        print("KEY:", x)
s.close()
