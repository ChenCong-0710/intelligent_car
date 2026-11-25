"""Check that V4 defaults preserve every serialized control field over 50,000 ticks."""
from pathlib import Path
import subprocess,hashlib
R=Path(__file__).resolve().parents[1]
B=Path((R/"work/portable_baseline_path.txt").read_text().strip())
OUT=R/"work/portable_validation";OUT.mkdir(exist_ok=True)
GCC=Path(r"E:/mingw/mingw64/bin/gcc.exe")
def run(args):subprocess.run([str(x) for x in args],check=True,cwd=R)
flags=["-std=c11","-O1","-Wall","-Wextra"]
for name,inc,sources in [("original",B/"Core/Inc",[B/"Core/Src/motion_pd.c"]),("portable",R/"Core/Inc",[R/"tests/v4_compat_fixture.c",R/"portable/dual_motor/dual_motor.c"])]:
 exe=OUT/(name+".exe")
 run([GCC,*flags,"-I",inc,R/"tests/trace_v4_portable.c",*sources,"-lm","-o",exe]);run([exe,OUT/(name+".trace")])
a=(OUT/"original.trace").read_bytes();b=(OUT/"portable.trace").read_bytes()
assert a==b,"Original and portable traces differ"
print("PASS: 50,000 ticks identical control traces, bytes=",len(a),"sha256=",hashlib.sha256(a).hexdigest())
exe=OUT/"api.exe";run([GCC,*flags,R/"tests/test_dual_motor_api.c",R/"portable/dual_motor/dual_motor.c","-lm","-o",exe]);run([exe])
exe=OUT/"v4_regression.exe";run([GCC,*flags,"-I",R/"Core/Inc",R/"tests/test_a50_sync_v4.c",R/"tests/v4_compat_fixture.c",R/"portable/dual_motor/dual_motor.c","-lm","-o",exe]);run([exe])
