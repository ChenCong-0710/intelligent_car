# STM32G474 智能小车固件工程：快速循迹、电机死区自标定与全流程控制

本工程是智能小车项目的核心固件工作区，支持 **200 Hz 快速软件 PID 循迹、左右轮电机死区与转速增益多次自动标定（`mcal`）、`<15cm` 超声波主动电枢短接制动停车（`LF_OBSTACLE_STOP`）、停车后自动步进扫掠求障碍物平面法线（垂直位置，M1 两段式 AIM 面模型）、侧视横扫找障碍面左右边界与中点（M2：`SIDE_LOCK/SIDE_SCAN/SIDE_MOVE`，台架命令 `side`/`side push`）、3 分钟顺时针四面障碍物瞄准与 10s 激光打靶（`LF_PROBING`）、片内 Flash 掉电参数自动保存，以及辅助纯硬件模拟 PID 排查的蓝牙实时诊断（`diag`）**。

---

## 0. 项目里程碑与工作区核心文档导航（全组必读）

> **当前项目进度**：
> - ✅ **纯硬件模拟电路循迹**：已完成改板焊接与实车跑通（见 [docs/pure-analog-rework-and-calibration.md](docs/pure-analog-rework-and-calibration.md)）。
> - ✅ **软件闭环循迹与电机死区标定**：已完成全量程地面带载曲线标定（`data/latest_calibration.json`）与软件循迹闭环。
> - ✅ **停车后自动寻找障碍物“垂直位置”（平面法线方向）**：`<15cm` 停车 → 300 ms 内自动舵机步进扫掠 + 超声波 `1/r = a·cosθ+b·sinθ` 最小二乘拟合 → 锁定正对角度 → **LED 150 ms 周期闪烁**（`PROBE_LOCK` 蓝牙回传角度与距离）。
> - ✅ **M1 面模型（2026-11-22 已烧录）**：两段式 AIM（粗扫 ±30° + 细扫 ±12°）输出 `face.phi/rho/rms/hits` 与 `PROBE_FACE` 日志。
> - ✅ **M2 侧视横扫（2026-11-22 已烧录并完成 4 次台架全流程验证）**：
>   `SIDE_LOCK`（舵机掰 90° 测站立距离 h 并自检）→ `SIDE_SCAN`（沿面平移，回波跳变找左右边界，位置域中点）→
>   `SIDE_MOVE`（沿切线开到中点，含发射前复测）→ 侧向 10 s 激光。台架入口：**`side`（自走）/ `side push`（人工推车）**。
>   详细落地记录见《手册》§3.9、《方案》§13.4、《问题排查手册》§17.14。
> - ✅ **M3 转平行 + 绕行（2026-11-22 已烧录）**：`TURN_PAR` 接在 AIM 后；`LEG` 用 DRIVE1→TURN→DRIVE2 绕到下一面，**宽面→窄面单向台架通过**（下一面 `SIDE_LOCK h=215 ok=1`、`w=43.6 NARROW`、`TURN 90/91` 实车左转）。**M3 还没收口**：TURN_PAR 实车、窄→宽、faces=2 自动是 Gate 0。
>   关键修正：`BOARD_SWAP_LR=0`（否则日志左转、实车右转）、`track_width_mm=112.5`（90° 标度）、**LEG 不做 14° ALIGN**。
> - ✅ **M4 Gate 0 实车（2026-11-23，已烧录）**：**G0-2 TURN_PAR 通过**（`cmd=-90.4 / ach=-91.0`，误差 0.6°，`SIDE_LOCK h=192 ok=1`）；**G0-4 faces=2 自动通过**（面1 `w=182.0 WIDE` → LEG `DRIVE1 302.8 / TURN 90.7 / DRIVE2 172.8` → 面2 `w=65.9 NARROW` → COMPLETE，无 FAIL）；**G0-3 窄→宽通过**（不重新 `side`，窄面 `w=71.2 NARROW` → 下一面 `w=170.8 WIDE h=160 ok=1`）。Gate 0 三项齐。
>   这一轮新增：LEG 侧距连续采样 `PROBE_LEG,seg=PROX/PROX2`、`SIDE_LOCK` 沿面 60 mm 平移测斜、`seg=BACKUP` 收尾倒回 40 mm、宽窄判据 130→100、锁定容差 40→70 mm。记录见 **[docs/M4-gate0-20261123.md](docs/M4-gate0-20261123.md)**。
> - ✅ **M4e 残余偏航一次性修正（2026-11-23 已实现 + 实车验证，M4e 判据 PASS）**：删掉 `SIDE_MOVE` 到位后的原地转（**扫描开始到 LASER 结束之间不许转身**）、`h_leg` 去掉 `side_k`、测斜死区 `1.5°→7°`、窄面基线 30 mm、每面最多转 1 次；`PROBE_SIDE_LOCK` 只追加 `eps=/shift=`。提交 `2e00745`/`8575ecd`/`a6ee2fe`/`a75e1b0`；主机单测 **37/37**、Debug 112312 B / Release 105200 B、零告警。**2026-11-23 18:04 烧录、18:12~18:21 跑 4 次 4 面全流程**：face0 `eps=-2.9°` 死区不转、face2 `eps=+14.9°` 只转一次、窄面 `shift=30`（`warn=8` 按设计跳过）、`SIDE_MID→LASER` 无 `PROBE_TURN` 且单拍航向跳变 ≤0.5° —— **三条判据 PASS**；有一次跑到 3 面激光（87.5 s）。**全 4 面未跑通不是代码回归**：摆位超 AIM 包络（`phi=25.9/31.7°`）、烧录后 RAM 电机标定丢失（`MCAL=0` 时侧扫航向漂 10.3°）、场地中间撞到无关物体；逐条见 `docs/validation.json` 的 `m4e.real_car.open_issues`。A/B/C 受控摆位组仍欠一次专测。
> - 🚀 **下一阶段 M4**：四面循环、回 AIM 重试、`h_pred`/`pred` 跨面、面宽分类、`probetour`、参数/Flash 8；计划见根目录 **[M4计划_评审与修订.md](../M4计划_评审与修订.md)**。
>   接手先读 **Gate 0**：[docs/M3-task-handover.md](docs/M3-task-handover.md)、[docs/teammate-targeting-handover.md](docs/teammate-targeting-handover.md)。

为了让每位组员无需翻阅底层寄存器即可快速掌握整个项目，所有软件与硬件对接文档均整理在本工作区 `docs/` 目录下：

1. **[docs/teammate-targeting-handover.md](docs/teammate-targeting-handover.md)**：**《队友交接指南：全软件任务架构与四面激光打靶（蓝牙标定/录数）开发手册》**（★ 接手打靶任务首读！）
2. **[docs/pure-analog-rework-and-calibration.md](docs/pure-analog-rework-and-calibration.md)**：**《纯硬件模拟电路改板焊接与电位器标定归档手册》** —— 已实车跑通的纯模拟改板 BOM、Slow Decay 飞线与 3296W 电位器圈数/电压标定记录。
3. **[docs/codebase-guide.md](docs/codebase-guide.md)**：**《软件工作区代码全景说明书》** —— 详细介绍工作区每个文件夹、每个 `.c/.h` 文件、核心结构体以及算法模块的通俗原理解析。
4. **[docs/hardware-interface-manual.md](docs/hardware-interface-manual.md)**：**《STM32G474 核心板与主扩展板硬件接口逐脚对应手册》** —— 详细到每一个 MCU 引脚、核心板排针、主扩展板排母、逻辑门芯片引脚与外部传感器/电机的电气对接关系。
5. **[docs/software-architecture-and-review.md](docs/software-architecture-and-review.md)**：**《固件审查与软件总体架构设计》** —— 记录硬件复用修复、定时器零冲突分配矩阵与快速循迹专项优化设计。
6. **[docs/board-pin-map.csv](docs/board-pin-map.csv)**：全车 27 项 MCU 引脚、网络名与外设复用速查表格。
7. **[docs/validation.json](docs/validation.json)**：每次代码修改后的 Debug/Release 编译、静态检查、**37 项**自动化单元测试与**台架验证**记录；M2 台架遥测在 [data/m2_side_bench_20261122.csv](data/m2_side_bench_20261122.csv)。

---

## 1. 在 VS Code 中编译、下载与在线调试（ST-LINK 完整操作指南）

### 1.1 ST-LINK 硬件接线注意事项（重要！）

- **请将 ST-LINK 插在主扩展板边缘的 5Pin 专用受保护调试座 `J_SWD` 上**：
  - `J_SWD Pin 1` = `DBG_5V_IN`（经板上 `TPS2553` 限流保护与 `TPS2117` 自动电源切换芯片）
  - `J_SWD Pin 2` = `SWCLK (PA14)`
  - `J_SWD Pin 3` = `GND`
  - `J_SWD Pin 4` = `SWDIO (PA13)`
  - `J_SWD Pin 5` = `NRST`
- **安全警告**：
  1. **严禁将 ST-LINK 的 `5V` 电源线直接插在核心板顶部的 `J4-4` 排针上**（否则会绕过扩展板的防倒灌与限流保护电路）。
  2. 当小车装有 4 节 AA 电池供电时，建议只接 `J_SWD` 的 `SWCLK / GND / SWDIO / NRST` 四根线（不接 ST-LINK 的 `5V`），且不要将电脑 USB 线直接插在核心板自带的 Type-C 口上。

---

### 1.2 只编译工程（不连接开发板、不下载）

- **快捷键方式**：在 VS Code 中直接按 **`Ctrl + Shift + B`**（默认执行 `Build Debug` 任务）。
- **命令面板方式**：按 `Ctrl + Shift + P` → 输入并选择 **`Tasks: Run Task`** → 选择：
  - **`Build Debug`**：编译带调试符号的 Debug 固件（输出至 `build/Debug/line-following.elf/.hex/.bin`）。
  - **`Build Release`**：编译体积与速度优化的 Release 固件（输出至 `build/Release/line-following.elf/.hex/.bin`）。
  - **`Test Controller`**：在电脑本机运行全部 **37 项控制算法自动化单元测试**（无需连接开发板）。

---

### 1.3 只下载程序到开发板、不进入调试模式（最常用！）

当你修改完代码或参数，只想把程序快速烧录进单片机并让小车直接脱机运行（例如准备用蓝牙在赛道上跑圈测试）时：

1. 用 ST-LINK 连接好开发板 `J_SWD` 接口并给开发板上电。
2. 在 VS Code 中按 **`Ctrl + Shift + P`** 打开命令面板。
3. 输入并点击 **`Tasks: Run Task`（任务：运行任务）**。
4. 在弹出的任务列表中选择：
   - **`Flash Debug (ST-LINK)`**：自动完成最新代码编译 + 通过 `STM32_Programmer_CLI` 擦写校验 + 硬件复位直接运行。
   - 或 **`Flash Release (ST-LINK)`**：编译并下载 Release 优化版固件并直接运行。
5. 底部终端显示 `Download verified successfully` 即表示烧录成功，此时可直接拔下 ST-LINK 排线去赛道试车！

**等价终端命令**（也可在 VS Code 终端中直接粘贴执行）：
```powershell
# 仅编译 Debug / Release / 运行 37 项主机测试：
powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1 -Configuration Debug
powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1 -Configuration Release
powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1 -Test

# 编译并一键通过 ST-LINK 下载到开发板（不进入调试器）：
powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1 -Configuration Debug -Flash
```

---

### 1.4 连接开发板进行在线调试与实时变量观测（Live Watch）

当你需要连接 ST-LINK 在线观察传感器数值、电机转速或触发校准时：

1. 连接 ST-LINK 后，直接按 **`F5`**（或点击左侧活动栏“运行和调试”图标，顶部选择 **`STM32Cube: Debug line-following (ST-LINK)`** 并点击绿色开始按钮）。
2. VS Code 会自动先执行 `Build Debug`，随后自动烧录固件并停在 `main()` 函数第一行。
3. 点击顶部调试栏的 **“继续 (Continue / `F5`)”** 让程序全速运行。
4. **使用 `ST-LINK Live Watch` 实时无阻塞看变量（无需暂停 CPU！）**：
   - 在调试侧边栏底部展开 **`ST-LINK Live Watch`** 面板，点击 `+` 号添加全局变量 **`g_debug`**。
   - 程序全速运行时，`g_debug` 会以每秒 10 次的速率实时刷新以下全部关键数据：
     - `g_debug.raw[0..3]`：四路红外 ADC 原始值（`0 ~ 4095`）
     - `g_debug.darkness[0..3]`：四路归一化见黑强度（`0.0 ~ 1.0`）
     - `g_debug.position`：当前黑线横向位置（`-1.0 ~ +1.0`，右正左负）
     - `g_debug.speed[0..1]` / `g_debug.target[0..1]`：左右轮实测线速度与目标线速度（`mm/s`）
     - `g_debug.duty[0..1]`：左右轮当前输出的 PWM 占空比
     - `g_debug.wheel_deadband[0..1]` / `g_debug.wheel_free_speed[0..1]`：左右轮死区与满速标定值
     - `g_debug.obstacle_mm`：超声波实时障碍物距离（`mm`）
5. **在调试器中直接远程触发动作**：
   - 将变量 **`g_debug_request`** 添加到 `Live Watch` 或 `Watch` 窗口，双击修改其数值即可触发对应动作（主循环读取后会自动清零）：
     - 写入 **`1`**：立即启动 8 秒红外传感器黑白校准
     - 写入 **`2`**：立即启动 2 秒倒计时并发车循迹
     - 写入 **`3`**：立即紧急刹车停止并清除故障
     - 写入 **`4`**：立即启动四面顺时针目标探测
     - 写入 **`5`**：立即启动 3 轮左右电机死区与转速自动标定
6. **关于独立看门狗（IWDG）的调试提示**：
   - 停在 `main()` 入口时看门狗尚未启动，可任意查看；一旦点击继续进入主循环，硬件独立看门狗（超时约 `0.5 s`）即已开启。在主循环内打断点暂停超过 0.5 秒会导致单片机看门狗安全复位（复位后电机自动禁能，防止挂死飞车）。因此**推荐全程使用 `Live Watch` 加上面的 `g_debug_request` 或蓝牙串口命令进行不暂停在线调试**。

---

## 2. 使用 VS Code 内置 `Serial Monitor` 插件进行蓝牙/串口无线闭环调试

本工程已在 `.vscode/extensions.json` 中配置好微软官方串口助手插件 **`Serial Monitor` (`ms-vscode.vscode-serial-monitor`)**，无需切换外部串口软件，直接在 VS Code 底部面板即可通过 **HC-04 蓝牙模块**（或 USB-TTL）收发数据。

### 2.1 硬件连接方式（蓝牙无线 vs USB-TTL 有线）

- **方式 A：使用 HC-04 蓝牙模块无线连接（推荐实车跑圈与架空标定使用！）**
  1. 将 `HC-04` 蓝牙模块插在主扩展板 `J_BT`（6Pin 座：Pin1=`STATUS`、Pin2=`RXD`、Pin3=`TXD`、Pin4=`GND`、Pin5=`5V`、Pin6=`KEY`）上（**注意此时不要再把外部 USB-TTL 接到 `PA9/PA10`，避免发送端冲突**）。
  2. **波特率自动适配说明**：HC-04 出厂默认波特率为 `9600 8N1`，而固件目标速率为 `115200 8N1`。本固件上电会在开启看门狗前等待 `>=400 ms`，先于 `9600` 无乱码探测并用 `AT+BAUD=115200` 永久升级（切换波特率间强制静默 `500 ms` 防止缓冲区污染，支持 `BT_KEY` 低/高双重尝试）。
  3. 小车上电后，打开 Windows“蓝牙和其他设备”添加设备，选择 **SPP 经典蓝牙设备名称 `HC-04`**（注意：**不要连 BLE 名称 `HC-04LE`**，PC 串口通道使用经典 SPP！），输入默认配对密码 **`1234`**。
  4. **Windows 长连接关键防断连规则（★ 必看！）**：Windows 蓝牙设置里显示“已连接”几秒后掉回“已配对”属于 **SPP 串口的正常协议行为**——只要电脑端没有串口软件打开对应 **传出 (Outgoing) COM 口（本机为 `COM10`）**，Windows 就会自动释放链路！请直接在 VS Code `Serial Monitor` 或上位机 `tools/host_station.py` 中**打开并长期占用传出串口（如 `COM10`，切勿误选全零传入端口 `COM5`/`COM11`）**，即可保持模块 LED 常亮与持久稳定收发。若当前保存端口不对，运行 `python tools/host_station.py set-port COM10` 绑定。
- **方式 B：使用 3.3V USB-TTL 有线连接**
  1. 先拔下扩展板上的 `HC-04` 蓝牙模块。
  2. 将 3.3V USB-TTL 的 `RXD` 接核心板 `PA9`、`TXD` 接 `PA10`、`GND` 接 `GND`。

### 2.2 在 VS Code 中打开与配置 `Serial Monitor`

1. 在 VS Code 底部面板（与“终端 Terminal”、“输出 Output”同一排标签栏）点击 **`Serial Monitor`（串口监视器）** 标签；如果没看到，按 `Ctrl + Shift + P` 输入 **`Serial Monitor: Focus on Serial Monitor View`** 回车即可打开。
2. 在 `Serial Monitor` 面板中按如下参数配置：
   - **Monitor Mode（监视模式）**：`Serial`
   - **Port（端口）**：选择你的蓝牙串口或 USB-TTL 对应的 `COMx`
   - **Baud rate（波特率）**：**`115200`**
   - **Line ending（行尾结束符）**：**必须选择 `LF` 或 `CRLF`**（不要选 `None`，因为单片机靠回车换行符判断一条命令输入完毕！）
   - 如需调整数据位/校验位，点击齿轮展开高级设置：数据位 `8`、停止位 `1`、校验位 `None`。
3. 点击蓝色的 **`Start Monitoring`（开始监视）** 按钮，即可看到小车实时发回的 `CSV` 遥测数据流！
4. **发送控制命令的小技巧**：
   - 在开始发送调参命令前，如果觉得每秒 10 行的 `CSV` 刷屏太快，可在下方输入框输入 **`log 0`** 回车，即可暂时关闭周期 CSV 刷屏，只显示命令回复和状态事件；调完参数想录数据时再输入 **`log 1`** 恢复！

---

## 3. 蓝牙上位机闭环标定与测试流程（推荐：`tools/host_station.py`）

为了高效完成**空载 5 轮标定 (`mcal air 5`)、地面带载自动往返 5 轮标定 (`mcal ground 5`)、前馈等速验证 (`vtest`)、赛道数据批量归档与纯硬件模拟 PID 反哺寻优**，工作区专门配套了 Python 蓝牙上位机软件 [tools/host_station.py](tools/host_station.py)。

### 3.0 启动上位机（支持 Agent 自动化 CLI 与 人工可视化 GUI）

- **人工可视化界面**：在 VS Code 按 `Ctrl + Shift + P` → `Tasks: Run Task` → **`Launch Bluetooth Host Station`**，或终端运行：
  ```powershell
  python tools/host_station.py
  ```
- **Agent / 脚本自动化蓝牙 CLI（输出结构化 JSON 并自动保存至 `data/`）**：
  ```powershell
  python tools/host_station.py ports                                     # 自动扫描蓝牙/串口
  python tools/host_station.py set-port COM5                             # 绑定默认串口至 data/port_config.json
  python tools/host_station.py mcal --port AUTO --mode air --passes 5    # 架空空载 5 轮 × 15 档标定
  python tools/host_station.py mcal --port AUTO --mode ground --passes 5 # 地面带载 5 轮 × 15 档自动往返标定
  python tools/host_station.py vtest --port AUTO --speeds 80,120,160,220,300 --ms 1000
  python tools/host_station.py capture --port AUTO --seconds 20 --cmd start --stop-after
  python tools/host_station.py analyze --file data/track_telemetry_xxx.csv
  ```
- 所有数据自动保存至工作区 [data/](data/README.md) 目录（`JSON` + `CSV`），**默认不频繁擦写片内 Flash**（仅在最终赛前需要脱机断电固化时才手动输入 `save`）。

> **15 档全量程标定阶梯 (`0.06 ~ 0.95`)**
> 现已将 `max_duty` 默认设为 **`0.95`**，标定阶梯固定为 **15 档：`0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24, 0.30, 0.38, 0.46, 0.55, 0.65, 0.75, 0.85, 0.95`**（`0.06 ~ 0.24` 采用 3% 细步长精确测定静摩擦起转死区，`0.30 ~ 0.95` 覆盖 0.50 以上直至 95% 全量程速度区）。

### 第一步：架空车轮，验证左右轮转向与编码器极性

将小车驱动轮架空（离地），在上位机或 `Serial Monitor` 中依次发送：
```text
log 0
bench 0.22 0 800
bench 0 0.22 800
```
- 若某侧车轮实际**向后转**：输入 `set motor_l -1` 或 `set motor_r -1` 修正方向。
- 若车轮向前转但报告的轮速 `vL` 或 `vR` **为负数**：输入 `set enc_l -1` 或 `set enc_r -1` 修正编码器极性。

### 第二步：地面带载 5 轮往返标定（主用）与空载 5 轮体检参考 (`mcal`)

> **工程使用原则**：最终软件循迹**始终以地面带载 (`mcal ground 5`) 曲线作为主前馈模型**；架空空载 (`mcal air 5`) 仅用于检查机械阻力与作为对比参考（若先完成了地面标定，之后再跑空载对比也不会覆盖地面主曲线）。

1. **架空空载 5 轮参考（机械体检）**：保持驱动轮离地，在上位机点击 **“① 架空空载 5轮标定”**（或发送 `mcal air 5`）：
   - 自动重复扫描 **5 轮** × 15 档占空比（`0.06 ~ 0.95`），检测电机本身空载死区与左右轮一致性，作为机械参考存入电脑 `data/`。
2. **地面带载 5 轮自动往返（★ 实车循迹主模型）**：将小车放到真实赛道白底/地板上（确保前方留有约 80 cm 直行空间），点击 **“② 地面带载 5轮往返标定”**（或发送 `mcal ground 5`）：
   - 自动重复执行 **5 轮** × 15 档占空比（`0.06 ~ 0.95`），每档采样后主动刹车 `80 ms` 控制单步位移，**每轮结束后利用编码器里程计（带航向纠偏）自动倒车回到起始原点**再开始下一轮；若超声波检测到前方 `<18cm` 障碍物会自动停机保护。
   - 地面测试能测出**含轮胎滚动阻力、万向轮阻力及整车转动惯量的真实地面死区与速度曲线**，拟合后自动激活为循迹主前馈模型（随时可用 `use_mcal ground/air` 切换比对）。

### 第三步：前馈补偿等速闭环验证（想转多快就转多快：`vtest`）

标定完成后（或随时从 `data/` 加载历史标定同步至小车 RAM），在上位机执行 `vtest` 验证（或串口发送 `vtest 160 160 1000`）：
- 单片机在**不开启速度 PI 闭环的纯前馈状态下**，直接按 15 点实测分段插值曲线 + 左右轮独立死区分别计算左、右轮补偿占空比 `uL` 与 `uR` 并驱动 1 秒；
- 结束时通过蓝牙回传实测编码器平均转速 `VTEST_DONE,tL=160.0,tR=160.0,uL=...,uR=...,vL=...,vR=...,errL=...,errR=...`，直观验证**左右轮无需 PI 也能以不同占空比跑出几乎一致的目标速度**，随后再开启 `speed_loop = 1` 获得更强的抗扰能力！

### 第四步：八秒红外黑白校准 (`cal`)、高速循迹、`<15cm` 避障与四面打靶

1. **八秒红外校准**：将车放回赛道，按住探测键或核心板按键 1 秒，或点击上位机 **“③ 8s 红外校准 (`cal`)”**，8 秒内让四路探头各自多次经过黑带与白底。校准值保存在 RAM 中（若最终赛前需要断电永久保持，可手动输入一次 `save` 写入 Flash）。
2. **开始高速循迹**：放回黑线上，单击主板**循迹键**（或发送 `start`），等待 2 秒后发车。
3. **`<15cm` 自动制动停车**：超声波测得前方 `<150 mm` 障碍物时，立即进入 `OBSTACLE_STOP`（`1111` 主动电磁短接制动保持、停车灯常亮、蜂鸣器长鸣 1s）。*桌面纯调循迹可先 `set obstacle 0`。* 行驶期间云台舵机被锁定并保持供电在机械中位（`TRACK_SERVO_CENTER=1`），保证测距方向始终是车头正前方。
4. **停车后自动寻找障碍物垂直位置（法线方向，M1）**：停稳约 300 ms（`AUTO_PROBE_SETTLE_MS`）后**无需按键**自动进入 `PROBING/AIM`，舵机做**两段式**步进扫掠（粗扫 ±300 µs / 10 点，再以粗测法线为中心细扫 ±120 µs / 10 点，每点驻留 ≥180 ms、开头丢弃 120 ms），对回波做 `1/r = a·cosθ + b·sinθ` 最小二乘拟合，输出**面模型** `PROBE_FACE,phi,rho,rms,hits,valid`；`probe_scan_mode=0` 时随后锁定该角度进入 10 s 激光+蜂鸣器阶段，**核心板 LED 以 150 ms 周期闪烁表示“已找到垂直位置”**。
   - 也可随时手动发送 `probe`（或在 `OBSTACLE_STOP` 下按探测键）重新开始一轮寻找；`set probe_faces 4` + `PROBE_TEST_ONE_FACE 0` 可恢复四面打靶流程。
5. **M2 侧视横扫（`probe_scan_mode=1`，默认）**：车头平行于目标面、舵机掰到 `probe_side_us`（默认 2400 µs = **+90° = 向左，箱体在左**）后，**沿面的切线方向平移扫掠**，用 8 点最小二乘 `d = h0 + k·s` 拟合面内距离（k 吃掉车不平行造成的线性漂移），靠回波跳变找左右两个边界；中点 `s_m=(s_L+s_R)/2` 是**位置域**量，不受 `pan_x`、侧角是否正好 90° 影响（扫描与发射同轴同角 → 仿射抵消）；宽度按波束半角 γ 补偿 `w=(s_R-s_L)-2·h·tanγ` 后判窄/宽面；随后沿切线开到 `s_m` 停稳、可选复测、保持同侧角发射 10 s 激光。
   - **台架入口**：`side` = 自走扫描；`side push` = 电机断电、人工沿面推车扫描（到边界后按 `PROBE_SIDE_EDGE` 日志反向推）；`leg` = 用上一次 `side` 的 w/h 绕到下一面（M3）；`m3` = `side`+`leg`。自动流程从 M3 起是 `AIM→TURN_PAR→SIDE_LOCK→…→LASER→LEG`，不再需要手动先转平行。
   - 日志：`PROBE_SIDE_LOCK`（h/自检）、`PROBE_SIDE_EDGE`（两个边界）、`PROBE_SIDE_MID`（中点/宽度/宽窄/retry）、`PROBE_SIDE_FAIL`（失败原因）；参数见下方 `probe_side_*` 表。

---

## 4. 整圈实测反哺【纯硬件模拟 PID】（1:1 硬件数字孪生仿真 `hwemu` + `hwmap`）

> **`cal`（8秒黑白校准）≠ 循迹测试**：`cal` 只是发车前花 8 秒让四路红外记下当前光线的白底/黑线上下限；真正反哺硬件 PID 需要**让小车在完整赛道跑多圈**并录制全过程波形。

1. **快速软件循迹基线**：按第三步完成地面带载 5 轮标定后，通过 `capture --port AUTO --seconds 25 --cmd start --stop-after` 跑完多圈完整赛道并自动保存至 `data/`，获得理想速度、过弯表现与四路电压波形。
2. **开启 `1:1` 硬件模拟 PID 仿真模式（`set hw_emu 1` / `host_station.py hwemu`）**：
   - 在此模式下，软件完全停用非线性三次外推、失线记忆与编码器速度闭环，严格按主板当前硬件电路（`U6A -> U6B/C/D -> U7A -> U7B/C -> U10 -> DRV8833`）真实传输函数运行；
   - 软件中可调的全部参数 **1:1 一一对应硬件上可调/可改焊的元器件**（`hw_w_out`↔`R39/R40`、`hw_w_in`↔`R41/R42`、`hw_kp`↔`RV_P`、`hw_r60`↔`R60`、`hw_kd_ms`↔`C50/RV_D`、`hw_r61`↔`R61`、`hw_v_base`↔`RV_V/JP_V`、`hw_i`↔`JP_I`、`hw_coast`↔`DRV8833` 滑行/制动）。
   - 在整圈赛道上反复调整这些硬件等效参数跑多轮，直至硬件仿真模式也能稳快过全部连续弯道；随时运行：
     ```powershell
     python tools/host_station.py hwemu --port AUTO --w-out 0.333 --kp 2.2 --kd-ms 18 --v-base 2.30 --run --seconds 25 --stop-after
     ```
   - 或串口发送 **`hwmap`**，直接把仿真最优参数 1:1 翻译为实板需要旋转的电位器阻值与建议改焊的贴片阻容/飞线方案（并在板载 1.8 寸 LCD 上实时观测四路 IR 电压、`PID_E` 与 `vL/vR`）。完整现板改焊方案、下一版原理图/PCB 演进规划及按键/输出硬件故障排查手册见 [docs/hardware-optimization-and-next-pcb-plan.md](docs/hardware-optimization-and-next-pcb-plan.md)。
3. **关键物理硬约束提醒**：据实板网表，`JP_V` 插在 `1-2` 会把 `V_BASE` 锁死在 `≥2.50 V`（最小占空比 `≥48.5%`）；若仿真验证需要降低基速（`D0 < 48.5%`），必须将 **`JP_V` 改插 `2-3`（接 `R68 = 6.2 kΩ` 到地）**。

---

## 5. 蓝牙/串口命令与可调参数完整速查表

所有参数修改命令必须在停止状态（`STOPPED` / `OBSTACLE_STOP` / `FAULT`）下执行；默认保存在 RAM 中不磨损 Flash（若确需断电固化可手动执行 `save`）：

| 命令格式 | 功能说明 |
| --- | --- |
| `help` | 打印全部支持的命令与参数名列表 |
| `status` | 打印当前状态、激活电机模型(`GROUND`/`AIR_REF`)、软件/硬件仿真参数、蓝牙状态与红外标定 |
| `diag` | 打印四路红外电压 (V)、`PID_E`/`PID_C` 等效电压与左右电机不对称度报告 |
| `io` | **硬件 I/O 自检**：实时输出三路按键 (`PD0/PD1/PD15`) 原始高低电平与按下判定、`PC4/PC10/PC6`、编码器 `TIM4/TIM3` 与 4 路 `ADC` |
| `lcdflip 0\|1` | **切换 1.8 寸竖屏方向**：`0`=默认正竖屏 (`0xC0`，含上下左右同步 180° 正向)，`1`=旋转 180° 倒竖屏 (`0x00`) |
| `outtest <目标> <值> [ms]` | **单路输出台架点亮测试**：逐项测试 `lamp`/`buz`/`laser`/`led` (`0`/`1`) 与 `servo` (`600..2400`，舵机最长可保持 `30000 ms`；其余 `3000 ms` 自动关闭；电机请用 `bench`) |
| `servosweep [period_ms]` | **舵机连续往返台架测试**：以 `900..2100 us` 三角波持续扫掠，`period_ms` 默认 `2000`（范围 `500..10000`）；发送 `stop` 或 `servosweep stop` 立即停止。也可用 `python tools/servo_sweep.py --port COM3` |
| `hwmap` | 将当前 `HW-EMU` 硬件仿真参数 1:1 翻译为物理电阻/电容/电位器阻值与跳线建议 |
| `mcal [ground\|air] [passes=5]` | 执行 `passes` 轮（默认 `ground` 地面往返 5 轮）15 档 (`0.06~0.95`) 电机标定 |
| `use_mcal [ground\|air]` | 切换当前循迹激活的前馈数据集（建议始终使用 `ground`，`air` 仅作对比） |
| `vtest VL VR MS` | **纯前馈等速验证**：按实测 15 点曲线计算前馈占空比驱动 `MS` 毫秒并回传实测转速与误差 |
| `mcurve STEP VL VR` | 由上位机向单片机 RAM 写入第 `STEP` (0..14) 档占空比对应的实测左右轮转速 |
| `cal` | 启动 8 秒四路红外传感器黑白扫描校准 |
| `calset CH BLACK WHITE` | 手动写入第 `CH` (0..3) 路的黑、白 ADC 标定值至 RAM |
| `start` | 启动 2 秒倒计时并发车循迹 |
| `probe` | 手动启动探测/瞄准任务（`<15cm` 停车后已默认自动启动，无需按键） |
| `side` / `side push` | **M2 台架侧扫**：跳过 AIM 直接 `SIDE_LOCK→SIDE_SCAN→SIDE_MOVE→LASER`；`push` = 电机断电、人工推车 |
| `leg` | **M3 台架绕行**：用上一次 `side` 的 w/h 跑 DRIVE1→TURN→DRIVE2 到下一面；**不要再先 `side`**，否则面序锁存重置。`m3` 是上位机脚本 `tools/_m3_bench.py` 的组合命令，不是固件命令 |
| `stop` | 立即刹车停止并清除当前故障状态 |
| `save` / `load` / `defaults` | 手动将当前 RAM 参数写入 Flash / 从 Flash 加载 / 恢复编译期默认参数 |
| `log 0` / `log 1` | 关闭 / 开启每 100 ms (10 Hz) 的 CSV 实时遥测数据流输出 |
| `bench L_DUTY R_DUTY MS` | 手动占空比测试（占空比 `-0.95 ~ +0.95`，时长 `1 ~ 1500 ms`；`>0.30` 请先架空或确保赛道畅通） |
| `order C0 C1 C2 C3` | 重排四路红外物理通道顺序（如 `order 3 2 1 0`） |

### `set <参数名> <数值>` 可调参数列表

| 参数名 | 默认值 | 取值范围 | 物理意义与调参指南 |
| --- | :---: | :---: | --- |
| `kp` | `115` | `0 ~ 400` | 位置环比例增益 (mm/s)；纠偏偏软时增大，直线摆头时减小 |
| `kd` | `1.8` | `0 ~ 10` | 位置环微分阻尼系数 (mm)；抑制入弯过冲与连续 S 弯甩尾 |
| `k3` | `0.35` | `0 ~ 3.0` | 三次方渐进非线性比例系数：`pos_eff = pos*(1 + k3*pos²)`，强化外侧急弯拉力 |
| `ki` | `0` | `0 ~ 100` | 位置环积分增益 (mm/s²)，建议保持为 `0` |
| `d_tau` | `0.016` | `0.003 ~ 0.2` | 位置微分低通滤波时间常数 (s) |
| `speed` | `120` | `20 ~ 500` | 直线巡航基速 (mm/s)；首轮跑通后可逐步提至 `160 ~ 240` 冲击竞速满分 |
| `curve_min` | `55` | `10 ~ 300` | 急弯自动减速的最低基速下限 (mm/s)，不可高于 `speed` |
| `max_wheel` / `max_turn` | `280` / `190` | `50~700` / `20~500` | 单轮最大目标速度与最大转向差速上限 (mm/s) |
| `max_duty` | `0.95` | `0.1 ~ 0.95` | PWM 占空比上限（覆盖至 `95%`） |
| `accel` / `slew` | `1200` / `5.0` | `50~5000` / `0.5~20` | 车轮加速斜率 (mm/s²) 与占空比变化率 (1/s)；内轮减速时自动按 3.5 倍放宽 |
| `db_l` / `db_r` | `0.14` / `0.14` | `0 ~ 0.45` | 左轮 / 右轮独立起转死区占空比（由 `mcal` 自动测得，也可手动微调） |
| `fs_l` / `fs_r` | `450` / `450` | `100 ~ 1500` | 左轮 / 右轮独立满占空比等效转速 (mm/s，由 `mcal` 自动测得) |
| `deadband` / `free_speed` | `0.14` / `450` | `0~0.4` / `100~1500` | 同时将左右两轮死区或满速设为同一数值 |
| `speed_loop` | `1` | `0` 或 `1` | `1`=启用分段前馈+编码器速度 PI 闭环；`0`=仅实测分段前馈开环控制 |
| `speed_kp` / `speed_ki` | `0.0012` / `0.003` | `0~0.01` / `0~0.03` | 编码器速度环 PI 系数 |
| `reverse` | `0` | `0` 或 `1` | `0`=禁止内轮反转；`1`=允许急弯内轮反转以突破 `b/2=65mm` 单向转弯半径极限 |
| `brake_pwm` | `1` | `0` 或 `1` | `1`=DRV8833 驱动/制动（慢衰减，推荐）；`0`=驱动/滑行（快衰减） |
| `lost_brake_ms` / `search_ms` | `60` / `350` | `0~500` / `0~1000` | 丢线后前置制动时间与旋回找线最大时间 (ms)；竞速时可设 `lost_brake_ms 0` 立即旋回 |
| `cross_ms` | `80` | `20 ~ 1000` | 四路全黑（起终点横线/十字交叉口）平顺保持最大允许时长 (ms) |
| `obstacle` / `obs_mm` | `1` / `150` | `0/1` / `50~500` | 超声波自动避障停车开关与停车距离阈值 (mm，默认 `150 mm = 15 cm`) |
| `servo_us` | `1500` | `900 ~ 2100` | 水平云台舵机正对前方的中位脉宽 (µs) |
| `probe_aim_ms` | `2000` | `200 ~ 5000` | **每段** AIM 扫掠时长 (ms)；粗扫+细扫合计约 4 s（前 150 ms 归位稳定，其余均分为离散扫描点） |
| `probe_sweep_us` | `300` | `20 ~ 600` | 粗扫相对中位的单侧扫掠半幅 (µs)；默认 ±300 µs = ±30°（单面测试编译开关会覆盖成 600） |
| `probe_step_us` | `60` | `20 ~ 200` | 粗扫步进扫掠的脉宽间隔 (µs)；细扫固定 20 µs（2°） |
| `probe_fit_rms_mm` | `15` | `2 ~ 20` | M2：面拟合残差上限 (mm) |
| `probe_scan_mode` | `1` | `0` 或 `1` | M2：`1`=侧视横扫（主方案）；`0`=M1 过渡流程（AIM→正面发射→方阵巡边） |
| `probe_side_left` | `1` | `0` 或 `1` | M2：`1`=箱体在左（逆时针巡边），M3 起决定转弯方向 |
| `probe_side_us` | `2400` | `600 ~ 2400` | M2：侧视舵机脉宽 (µs)；2400=**+90°=向左（箱体在左）**，顶死就降到 2200~2350 |
| `probe_side_spd_mm_s` | `50` | `20 ~ 100` | M2：侧扫/开位速度 (mm/s)；50 对应 60 ms 走 3 mm 一个样本 |
| `probe_side_reach_mm` | `260` | `80 ~ 400` | M2：单方向最大扫描行程 (mm) |
| `probe_side_band_mm` | `15` | `4 ~ 30` | M2：面内判据带宽 (mm，车动时实测噪声 ±15 mm) |
| `probe_side_jump_mm` | `40` | `10 ~ 80` | M2：跳变阈值 (mm)，必须大于 band；背景 500~900 mm 下 40 仍安全 |
| `probe_side_verify` | `1` | `0` 或 `1` | M2：发射前以 `s_m` 为中心复测一次（关掉可省 2~3 s/面） |
| `probe_beam_half_deg` | `7.5` | `2 ~ 15` | M2：超声波束半角 γ（只影响宽度，不影响中点；不标定） |
| `probe_pan_x_mm` | `30` | `0 ~ 120` | M2：云台轴相对轮轴前移量，只用于 `SIDE_LOCK` 自检 |
| `probe_aim_min` / `probe_aim_max` | `40` / `400` | `20~500` / `100~2000` | 参与法线拟合的有效回波距离窗口 (mm)，滤除地面与远端杂波 |
| `probe_laser_ms` | `10000` | `200 ~ 30000` | 找到法线后激光+蜂鸣器保持时长 (ms) |
| `enc_l` / `enc_r` / `motor_l` / `motor_r` | `1` | `-1` 或 `1` | 左右编码器与左右电机的方向极性修正系数 |
