# M3 任务交接：TURN_PAR + LEG（2026-11-22）

> **交接对象**：接手 M4（四面循环）的队友/代理。
> **当前固件**：Debug `110816 B`（RAM 11792 B），已烧录并校验通过；源码见本仓当前 commit（`git log -1`）。
> **一句话状态**：M3 代码全部落地；**宽面→窄面单向 LEG 已在台架端到端通过**；faces=2 自动全流程（AIM→TURN_PAR→SIDE_LOCK→…→LEG）尚未实车跑过。

---

## 1. 已完成 / 已验证 / 未验证

| 项 | 状态 | 说明 |
| --- | --- | --- |
| TURN_PAR | ✅ 代码 + 单测 | AIM 成功后按 φ 原地转 `side_left ? φ−90 : φ+90`；3 s 超时、8° 容差、失败即停；台架方向未单独跑 |
| LEG 三段 | ✅ 代码 + 单测 + 台架 | DRIVE1→TURN→DRIVE2；无 ALIGN；宽→窄下一面 `h=215 ok=1` |
| LEG 事件/日志 | ✅ | `PROBE_LEG,seg=DRIVE1/TURN/DRIVE2/FAIL` |
| 面序宽窄锁存 | ✅ 代码 + 单测 | 首个 `w>=130` 锁存为宽，之后按面序奇偶交替；不重新 `side` 才保留锁存 |
| 自动流程接入 | ✅ 代码 | AIM→TURN_PAR→SIDE_LOCK；LASER→LEG→下一面 SIDE_LOCK；`probe_face` 在 LASER 出口/LEG 入口递增 |
| faces=2 自动全流程 | ⏳ **待验** | 需要在实车跑一次，确认 AIM/TURN_PAR/LEG 衔接与面 2 LASER 后 COMPLETE |
| 窄→宽第二段 LEG | ⏳ 待验 | 不重新 `side`，直接 `leg`，看面序锁存与 h 收敛 |
| `h_pred` | ⚠️ 偏差 39 mm | 21:49 `h_pred=176`、实测 215；公式未计转后云台轴/pan_x 偏置，M4 修 |

---

## 2. 硬件与环境

- 主控：STM32G474VET6；固件仓根目录 `line-following-car/`。
- 烧录：ST-Link 插扩展板 `J_SWD`；**开车测试时必须拔掉**（线会拖车）。
  ```powershell
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Configuration Debug -Flash
  ```
- 蓝牙：HC-04，**COM3，115200**，行尾 CRLF；`log 0` 关 CSV、`probe_log 1` 开 PROBE 日志。
- 供电：4 节 AA；电压 3.29 V 左右时动作正常；掉压会导致两轮速度差变大。
- 台架脚本：`tools/_m3_bench.py`（`check|side|leg|m3|dirscan|raw`），日志落在 `data/m3_*.log`。

---

## 3. 本次三个关键修正（务必先读）

### 3.1 `BOARD_SWAP_LR` 必须为 0

- 21:01/21:34 两次「该左转却右拐」：日志 `TURN cmd=+90.0, ach=+90.9, dir=L` 完全正常，实车右转。
- 根因：`BOARD_SWAP_LR=1` 把**电机和编码器同时**镜像；里程计仍自洽，但 `+90` 的物理方向反了。
- 已改 0（[Board/board.h](../Board/board.h)）：`+` = 实车左转（CCW），与 `probe_turn_to`、TURN_PAR、单测、LEG 约定一致。
- **门禁**：上电 `PROBE_BUILD,swaplr=0`；空载 `turn 20 60` 目视必须左转。若日志 `dir=L` 但实车右转，先查这里，**不要翻 LEG 符号**。

### 3.2 等效轮距 `track_width_mm = 112.5`

- 21:43（track=130）：`TURN cmd=90/ach=90.9`，但下一面 `k=-0.211`，车头多转约 12~14°。
- 标定：`130 × 90/104 ≈ 112.5`（物理轮距仍约 130；这里改的是里程计偏航标度）。
- 21:49（track=112.5）：`TURN 90/91`，下一面 `k=+0.203`——从「朝里 14°」翻成「朝外 14°」，说明 90° 本身已基本无过冲。
- 以后用 `turn 90 60` 复测，按 `原值 × 90/实测` 微调。

### 3.3 LEG 的 14° ALIGN 已永久删除

- `k=-0.25` 在车目视平行时也稳定出现，是侧扫弧段/电机死区的**测量偏置**，不是静态歪斜。
- 按它修 14° 会把车头顶向箱面（h 143→约 100），转角时前角顶住箱角，反而制造「该左转却右拐」。
- 删除后 DRIVE1 保持 h，21:49 转向正常。`PROBE_SIDE_MID.k` 只作诊断，**不要再加回控制**。

---

## 4. 关键参数与几何

| 参数 | 值 | 位置 |
| --- | --- | --- |
| `BOARD_SWAP_LR` | 0 | `Board/board.h` |
| `track_width_mm` | 112.5（等效） | `Control/line_follower.c` 默认配置 |
| `LF_LEG_SPEED_MM_S` / `LF_LEG_TURN_SPEED_MM_S` | 100 / 60 | `Control/line_follower.h` |
| `LF_LEG_MARGIN_MM` | 20 | 同上 |
| `LF_LEG_H_MIN_MM` | 120 | 同上 |
| `LF_TURN_TOL_DEG` / `LF_TURN_SETTLE_MS` | 8 / 150 | 同上 |
| `LF_BOX_LONG_MM` / `LF_BOX_SHORT_MM` | 150 / 50 | 同上 |
| `LF_SIDE_KIND_SPLIT_MM` | 130 | 宽窄分类（窄面实测 109→判窄） |

LEG 几何（从轮轴起算，入口锁存）：

```c
w_nom = (leg_kind == 0) ? 50 : 150;
d_B   = (leg_kind == 0) ? 150 : 50;
h_leg = side_h + side_k * face.s_m;          // k 只诊断
L1 = 0.5*w_nom + pan_x_mm + h_leg + 20;      // pan_x=30
L2 = h_leg + 0.5*d_B - pan_x_mm;
```

- 转角：`side_left ? +90 : -90`；箱体在左 → `+90` = 实车左转。
- `leg_kind`：首个实测 `w>=130` 的面锁存为宽，之后按面序奇偶交替；**两次 LEG 之间不要重新 `side`**，否则锁存重置。

---

## 5. 台架实测（2026-11-22）

### 21:49 通过（track=112.5，用户确认「效果非常好」）

| 阶段 | 日志 |
| --- | --- |
| SIDE | `h=150`，`s_m=-25.8`，`w=136.8 WIDE`，`k=-0.250` |
| DRIVE1 | `len=281.5, ach=281.7, hdg=-0.25` |
| TURN | `cmd=+90.0, ach=+91.0, ms=1365, dir=L, h_pred=176` |
| DRIVE2 | `len=151.5, ach=151.8, hdg=+0.74` |
| 下一面 | `SIDE_LOCK h=215.0, ok=1`；`SCAN w=43.6 NARROW, k=+0.203`；`SIDE_MOVE/LASER` 正常 |

### 21:43 对照（track=130）

`TURN 90/90.9`，下一面 `h=163`、`w=79.7 NARROW`、`k=-0.211`；方向已对，但转弯多约 12°。

原始日志：`data/m3_m3_side_20261122_214914.log`、`data/m3_m3_leg_20261122_214931.log`；结构化记录见 [validation.json](validation.json) 的 `m3_bench`。

---

## 6. 编译 / 烧录 / 测试命令

```powershell
cd line-following-car
.\tools\build.ps1 -Test                                  # 33/33
.\tools\build.ps1 -Configuration Debug                    # 0 warnings
.\tools\build.ps1 -Configuration Debug -Flash             # ST-Link 必须插好
```

蓝牙台架（拔掉 ST-Link，车放好）：

```powershell
python .\tools\_m3_bench.py check     # 确认 STOPPED、COM3
python .\tools\_m3_bench.py side      # 宽面侧扫到中点
python .\tools\_m3_bench.py leg       # 不重新 side，保留面序锁存
# 或一次跑 side+leg：
python .\tools\_m3_bench.py m3
```

方向门禁（先做，别直接跑 LEG）：

```
turn 20 60     # 必须左转（逆时针）
```

---

## 7. 已知问题 / M4 待办（按优先级）

1. **faces=2 自动全流程未跑**：`set probe_faces 2`、`set probe_laser_ms 1000`、`set probe_side_verify 0`，从 15 cm 停车自动跑；确认面 1 AIM→TURN_PAR→SIDE_LOCK→…→LASER→LEG→面 2 SIDE_LOCK→SCAN→MOVE→LASER→COMPLETE。这是 M3c 的 DoD。
2. **窄→宽第二段 LEG**：在 21:49 的窄面位置直接 `leg`（不要再 `side`），看 `leg_kind` 是否按面序取 0→1、h 是否继续外扩。
3. **`h_pred` 修公式**：21:49 预测 176、实测 215（+39 mm）；当前 `h_pred = h_leg + margin`，未计转后云台轴在横向的 pan_x/波束偏置。M4 连续四面必须修，否则站位会越走越远。
4. **面宽分类**：21:49 起始 `w=136.8` 刚过 130 判 WIDE；窄面测得 43.6~109.5，阈值 130 可用但不宽裕。M4 若出现 120~140 的实测要复核。
5. **电机死区未标定**：`db_L=db_R=0.140`、`MCAL=0(DEFAULT)`，实际约 22%/32%。连续四面直线/转角会有系统性偏差；建议 `mcal ground 5` + `use_mcal`。
6. **TURN_PAR 未单独台架验收**：单测覆盖 φ/符号/堵转，但实车 AIM→TURN_PAR 的落点与残差还没量。
7. **文档**：根目录《M3计划》《手册》《问题排查手册》已按 21:49 数据更新；本文件随代码仓提交。

---

## 8. 文件地图

| 文件 | 作用 |
| --- | --- |
| `Control/line_follower.c` | `probe_turn_to`、`probe_leg_tick`、TURN_PAR/LEG 状态、`track_width_mm=112.5` 默认值 |
| `Control/line_follower.h` | `LfLegStage`（0/1/2/3）、`LF_LEG_EVENT_*`、LEG 宏、`leg_kind` 等字段 |
| `Core/Src/main.c` | `PROBE_LEG`/`PROBE_TURN` 打印、`leg` 台架命令、`PROBE_BUILD,swaplr=` |
| `Board/board.h` | `BOARD_SWAP_LR=0`（关键） |
| `tests/test_controller.c` | `probepar`、`probeleg` 真值测试 |
| `tools/_m3_bench.py` | 台架 `check/side/leg/m3/dirscan/raw` |
| `docs/validation.json` | `m3_bench` 两轮记录、源码哈希、33/33 |
| `data/m3_m3_*.log` | 台架原始日志 |

日志关键行：

```
PROBE_BUILD,swaplr=0
PROBE_SIDE_LOCK,h=...,ok=1
PROBE_SIDE_MID,s_m=...,w=...,kind=WIDE/NARROW,retry=...,k=...
PROBE_LEG,seg=DRIVE1,len=...,ach=...,hdg=...
PROBE_LEG,seg=TURN,cmd=90.0,ach=...,ms=...,dir=L,h_pred=...
PROBE_LEG,seg=DRIVE2,len=...,ach=...,hdg=...
PROBE_LEG,seg=FAIL,stage=...,why=...,h=...
```

---

## 9. 交接检查清单

- [ ] 上电看到 `PROBE_BUILD,swaplr=0`；
- [ ] `turn 20 60` 目视左转；
- [ ] `python tools/_m3_bench.py check` 能连 COM3、状态 STOPPED；
- [ ] 跑一次 `m3`，确认下一面 `SIDE_LOCK ok=1`、`w>30`、`h∈[150,220]`；
- [ ] 跑 faces=2 自动全流程，记录面 2 的 `SIDE_LOCK/SCAN/MOVE/LASER`；
- [ ] 在 `docs/validation.json` 追加本次数据（编译字节数、单测数、台架结果）。
