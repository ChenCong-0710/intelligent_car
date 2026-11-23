# 台架功能测试程序（tests/bench）

**定位：测试程序只通过固件对外的功能命令驱动小车，用来验收功能；
固件功能代码（Control/、Core/、Board/）一行都不为测试而改。**

> 上一轮的教训：临时脚本里出现过 `h1==h2` 被当成"车是平行的"、脚本先测 600us 再测
> 2400us 让舵机走 180° 而固件只等 400 ms、`outtest servo` 直接戳舵机导致读数假距离。
> 本程序把这些坑固化成流程和判据，并在 `selftest` 里用历史日志做了正反用例。

## 快速开始

    cd line-following-car
    python tests/bench/run.py selftest      # 离线自检（不需要小车/串口）
    python tests/bench/run.py status        # 只读自检（不动机器）
    python tests/bench/run.py turn          # turn +10°/-10° 功能验收
    python tests/bench/run.py side-tilt     # 已知 +10° 斜置，再 side（核心用例）
    python tests/bench/run.py side-parallel # 平行对照组（0°）
    python tests/bench/run.py faces2        # M4 Gate0：faces=2 自动全流程

常用参数：

    --port COM3        串口（默认环境变量 BENCH_PORT / M4_PORT / M3_PORT，再默认 COM3）
    --side left        箱子在车体哪一侧（默认 left；本车实测 2400us = 左侧，600us = 右侧）
    --side-us 2400     朝该侧的舵机脉宽（默认左侧 2400、右侧 600）
    --skew 10          side-tilt 的已知摆角（+ = CCW 左转）
    --prompt           开跑前暂停，等人工摆位后回车（现场手摆时用）
    --data-dir PATH    报告目录（默认 data/bench）
    --no-save          不落盘

退出码：0 = 全部判据通过；1 = 有用例失败；2 = 串口打不开；3 = 用例执行中断。

## 用例与判据

| 用例 | 动作 | 判据 |
| --- | --- | --- |
| `status` | `status` + `us`，不动机器 | STOPPED/OBSTACLE_STOP、FAULT=OK、超声有效、舵机中位 1500 |
| `turn` | `turn +10 50`、`turn -10 50` | MOVE_DONE、timeout=0、到位误差 ≤3°、净转角 ≈0 |
| `side-tilt` | 配 side 参数 → `turn +10 50` 摆已知斜角 → `side` | SIDE_LOCK ok=1、warn=0、h1/h2/h3 ∈ [80,450]、ds≈60、`h2≠h1`、Δh≈−10.6 mm、corr≈−10.4°、SIDE_SCAN 出 w>30、无 SIDE_FAIL |
| `side-parallel` | 同上但 skew=0 | abs(h2−h1) ≤5 mm、abs(corr) ≤3° |
| `faces2` | 配 faces=2 → `probe` | 2 面 SIDE_LOCK ok=1 且 warn=0、2 次侧扫 w>30、TURN_PAR ok 且 abs(ach−cmd)≤8°、LEG 有 TURN/DRIVE、结束 STOPPED/OK、总时间 <150 s |

### 测斜判据的物理推导（写成代码也写成注释）

车轴与箱面成 θ 角时，沿车轴前进 s：

* 箱面在左侧：h 的变化 Δh = −s·tanθ；箱面在右侧：Δh = +s·tanθ；
* 固件用 `corr = atan2(Δh, 60 mm)`，右面取反，所以**左右两侧的期望 corr 都 = −θ**；
* 台架摆位：`turn +θ`（CCW 左转）= 车头向左侧箱面偏 → 前进时 h 变小 → corr 为负（右转修正）。

`60 mm` 是 `LF_SIDE_TILT_SHIFT_MM`：θ=10° 时 Δh=−10.6 mm，是 h 噪声（±1 mm）的 10 倍。

## 现场操作规程（按顺序）

1. 上电、插 USB-TTL（默认 COM3），先跑 `status`，确认 `STATE=STOPPED FAULT=OK`。
2. **固定用左侧 2400 us**（`--side left --side-us 2400`）。不要在同一次试验里来回切左右：
   600→2400 是 180° 行程，固件 SIDE_LOCK 只等 400 ms，会读到"假距离"。
   上机前可先用 `outtest servo` 扫一遍确认盒子在哪侧（本机实测：600→2400 是从右向左，
   2400 侧读到 167~170 mm 的盒子，600 侧是 431 mm 的远墙）。
3. 摆位：箱子在车体左侧，云台轴距面 180~400 mm；地面贴一条与箱面平行的基准线。
4. 跑 `side-tilt`：程序会先用 `turn` 摆出**已知的 10°**（这一步本身有精度判据），
   再用 `side` 跑 SIDE_LOCK→SIDE_SCAN→SIDE_MOVE，最后按上面判据给出 PASS/FAIL。
5. 看落盘报告：`data/bench/side-tilt_YYYYMMDD_HHMMSS.log`（原始串口日志）和同名 `.json`
   （每条判据 + 指标，便于写进 validation.json）。
6. 换摆位重跑前，先让车停稳；`side` 会移动小车，重复试验必须重新摆位。

## 与旧脚本的关系

* `tools/_m3_bench.py`、`tools/_m4_bench.py`、`tools/_side_*.py` 是上一轮的一次性脚本，
  保留作历史参考，不再扩展；新测试一律在本程序里加用例（`cases.py` + `protocol.py` 判据
  + `selftest.py` 正反用例）。
* `tools/host_station.py` 是人工可视化上位机，定位不同：它给人看曲线/手动发命令，
  本程序给自动化验收（可重复、可判 PASS/FAIL、可落盘）。
* `tests/test_controller.c` 是主机端 C 单元测试（不起硬件），与本程序互补：
  单元测试证明算法在合成世界里正确，本程序证明它在实车链路上跑通。

## 代码约定

* `protocol.py` 只写纯函数（解析/判据），任何新判据都要在 `selftest.py` 里补正反用例。
* `device.py` 只允许调用固件功能命令；不许 `outtest/vtest/mcal/servosweep` 这类台架自检命令。
* 任何会动机器的用例都必须由 `cli.py` 的 `finally` 兜底发 `stop`。
* 判据里的常数（80/450/60/1.5/15）与 `Control/line_follower.h` 保持一致；
  固件改了上限/基线，这里同步改并跑 `selftest`。
