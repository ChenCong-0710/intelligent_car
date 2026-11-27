"""Build find_box_and_shoot_cc stage1 only. Never flashes hardware."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
BIN=next(Path(r"E:/stm32cubeIDE").rglob("arm-none-eabi-gcc.exe")).parent
OUT=ROOT/"Debug/stage1_pd";OUT.mkdir(exist_ok=True)
def run(args):subprocess.run([str(x) for x in args],cwd=ROOT/"Debug",check=True)
flags=["-mcpu=cortex-m4","-std=gnu11","-g3","-DUSE_HAL_DRIVER","-DSTM32G474xx","-O1","-ffunction-sections","-fdata-sections","-Wall","-Wextra","-mfpu=fpv4-sp-d16","-mfloat-abi=hard","-mthumb","--specs=nano.specs"]
for inc in ["Core/Inc","Drivers/STM32G4xx_HAL_Driver/Inc","Drivers/STM32G4xx_HAL_Driver/Inc/Legacy","Drivers/CMSIS/Device/ST/STM32G4xx/Include","Drivers/CMSIS/Include"]:flags.extend(["-I",ROOT/inc])
new=[]
for name in ["main","initial_align","car_motion","box_center","four_faces","stm32g4xx_it"]:
 obj=OUT/(name+".o");run([BIN/"arm-none-eabi-gcc.exe",*flags,"-c",ROOT/f"Core/Src/{name}.c","-o",obj]);new.append(obj)
obj=OUT/"dual_motor.o";run([BIN/"arm-none-eabi-gcc.exe",*flags,"-c",ROOT/"portable/dual_motor/dual_motor.c","-o",obj]);new.append(obj)
objects=[]
for line in (ROOT/"Debug/objects.list").read_text().splitlines():
 name=line.strip().strip('"')
 if not name:continue
 if any(name.endswith('/'+x.name) for x in new):continue
 objects.append(name)
elf=OUT/"box_stage1_pd.elf"
run([BIN/"arm-none-eabi-gcc.exe",*flags,"-o",elf,*objects,*new,"-T",ROOT/"STM32G474VETX_FLASH.ld","--specs=nosys.specs",f"-Wl,-Map={OUT/'box_stage1_pd.map'}","-Wl,--gc-sections","-static","-Wl,--start-group","-lc","-lm","-Wl,--end-group"])
for fmt,ext in [("ihex","hex"),("binary","bin")]:run([BIN/"arm-none-eabi-objcopy.exe","-O",fmt,elf,elf.with_suffix('.'+ext)])
run([BIN/"arm-none-eabi-size.exe",elf]);print("BUILT",elf,"NOT FLASHED")
