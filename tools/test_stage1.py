from pathlib import Path
import subprocess
R=Path(__file__).resolve().parents[1]
G=Path(r"E:/mingw/mingw64/bin/gcc.exe");O=R/"work/tests";O.mkdir(parents=True,exist_ok=True)
for name in ["test_initial_align","test_car_motion","test_approach_align","test_box_center","test_low_speed_assist","test_four_faces","test_side_scan"]:
 exe=O/(name+".exe")
 subprocess.run([str(x) for x in [G,"-std=c11","-O1","-Wall","-Wextra","-I",R/"Core/Inc",R/f"tests/{name}.c",R/"Core/Src/four_faces.c",R/"Core/Src/box_center.c",R/"Core/Src/initial_align.c",R/"Core/Src/car_motion.c",R/"portable/dual_motor/dual_motor.c","-lm","-o",exe]],check=True)
 subprocess.run([str(exe)],check=True)
