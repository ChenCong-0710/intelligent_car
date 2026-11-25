"""Build isolated TEST calibration firmware; never flashes hardware."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
BIN=next(Path(r"E:/stm32cubeIDE").rglob("arm-none-eabi-gcc.exe")).parent
OUT=ROOT/"Debug/a34_pivot_v2"
OUT.mkdir(exist_ok=True)
def run(args):
 subprocess.run([str(x) for x in args],cwd=ROOT/"Debug",check=True)
flags=["-mcpu=cortex-m4","-std=gnu11","-g3","-DUSE_HAL_DRIVER","-DSTM32G474xx","-O1","-ffunction-sections","-fdata-sections","-Wall","-Wextra","-mfpu=fpv4-sp-d16","-mfloat-abi=hard","-mthumb","--specs=nano.specs"]
for inc in ["Core/Inc","Drivers/STM32G4xx_HAL_Driver/Inc","Drivers/STM32G4xx_HAL_Driver/Inc/Legacy","Drivers/CMSIS/Device/ST/STM32G4xx/Include","Drivers/CMSIS/Include"]:
 flags.extend(["-I",str(ROOT/inc)])
for src in ["main","motion_pd"]:
 run([BIN/"arm-none-eabi-gcc.exe",*flags,"-c",ROOT/f"Core/Src/{src}.c","-o",OUT/f"{src}.o"])
run([BIN/"arm-none-eabi-gcc.exe",*flags,"-c",ROOT/"portable/dual_motor/dual_motor.c","-o",OUT/"dual_motor.o"])
objects=[]
for line in (ROOT/"Debug/objects.list").read_text().splitlines():
 name=line.strip().strip('"')
 if not name: continue
 if name.endswith("/dual_motor.o"):continue
 if name.endswith("/main.o") or name.endswith("/motion_pd.o") or name.endswith("/initial_align.o"):continue
 objects.append(name)
elf=OUT/"a34_pivot_r90_v2.elf"
run([BIN/"arm-none-eabi-gcc.exe",*flags,"-o",elf,*objects,OUT/"main.o",OUT/"motion_pd.o",OUT/"dual_motor.o","-T",ROOT/"STM32G474VETX_FLASH.ld","--specs=nosys.specs",f"-Wl,-Map={OUT/'a34_pivot_r90_v2.map'}","-Wl,--gc-sections","-static","-Wl,--start-group","-lc","-lm","-Wl,--end-group"])
for fmt,ext in [("ihex","hex"),("binary","bin")]:
 run([BIN/"arm-none-eabi-objcopy.exe","-O",fmt,elf,elf.with_suffix('.'+ext)])
run([BIN/"arm-none-eabi-size.exe",elf])
print("BUILT",elf,"NOT FLASHED")
