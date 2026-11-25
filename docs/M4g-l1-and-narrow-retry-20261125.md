# M4g：首面后行程 L1×0.85 与第二面窄面重扫修复

> 日期：2026-11-25  
> 状态：代码完成、Host 单测 38/38、Debug/Release 构建通过、已烧录；用户现场反馈“这次相当好”。  
> 量化日志尚未归档到 `data/bench/`，下一次自动录数后应补 `h / warn / 光斑偏距`。

---

## 0. 一句话

上一轮第二面失败不是“锁错面”，而是两个问题叠加：

1. 首面扫描后 `DRIVE1` 越角行程偏大，绕到第二面时 `h=205 mm`（期望 153 mm）；
2. 窄面把“原始边界跨度 66.3 mm”按宽面足印公式扣掉 54 mm，得到假宽度 `w=12.3 mm`，旧逻辑因此回到旧扫描原点 0 重扫；该点离初扫中点 29.5 mm，重扫在面外/边缘无法积累面内证据，最终 `edges=0` 等到 13.4 s 超时。

本轮把 `L1` 缩到 **85%**，并把“宽度公式失配”和“边界/中点无效”拆开处理。

---

## 1. 失败证据链（2026-11-25 12:30 日志）

原始日志：[`data/bench/second_face_bt_20261125_123050.log`](../data/bench/second_face_bt_20261125_123050.log)

| 项 | 数值 |
| --- | --- |
| 首面复测 | `w=153.5 mm`，随后 `LEG` |
| `DRIVE1` | `len=258.0 mm`（`L1 = 150/2 + 30 + h_ref(153)`） |
| 绕角 | `TURN cmd=90.0 / ach=90.4`，`DRIVE2 stop=E` |
| 第二面 `SIDE_LOCK` | `h=205.0 mm`，`pred=153.0 mm` |
| 第二面初扫 | 原始边 `+62.6 / −3.7 mm`，跨度 `66.3 mm` |
| 足印补偿 | `2·h·tan(7.5°) = 53.98 mm` |
| 旧宽高结果 | `w=66.3−54=12.3 mm`，`s_m=29.5 mm` |
| 旧重扫 | `side_target=0`；约 13.4 s 后 `PROBE_SIDE_FAIL,edges=0` |

`13.4 s` 正好等于旧扫描超时：

```text
((2 × 260 + 300) / 75) × 1000 + 2500 ≈ 13433 ms
```

说明车不是在正常扫第二条边，而是在第一次候选外点后停住等确认。

---

## 2. 根因

### 2.1 `w=12.3 mm` 是补偿模型的产物，不是真实面宽

窄面真值 **50 mm**，`h=205 mm` 时波束直径约：

```text
2 × h × tan(7.5°) ≈ 54 mm
```

波束比整个窄面还宽。此时仍按宽面模型“两侧各扣一个完整足印”会把 50 mm 扣成 12.3 mm。中点 `s_m=(lo+hi)/2` 不受左右对称补偿影响，仍然是可用的定位量。

### 2.2 旧重扫从 `s=0` 开始

`side_target=0` 是旧扫描原点，与初扫中点相距 29.5 mm；对 50 mm 窄面来说已经接近/落到面外。重扫时：

- 回波只剩部分波束擦面，面内样本不足；
- `LF_SIDE_EDGE_MIN_INFACE=10` 无法满足；
- 第一次外点又把 `side_edge_pending` 置 1 并刹车；
- 车停住后 `side_inface` 不会再增长，只能等整段超时。

---

## 3. 本轮改动

### 3.1 `L1` 统一乘 0.85

`Control/line_follower.h`：

```c
#define LF_LEG_L1_SCALE 0.85f
```

`Control/line_follower.c`：

```c
leg_l1_raw_mm = 0.5f*w_nom + pan_x + h_ref;
leg_l1_mm     = LF_LEG_L1_SCALE * leg_l1_raw_mm;
```

只缩 `L1`，不缩 `L2`：

- `L1` = 沿当前面越过拐角的行程，影响转完后与下一面的站位；
- `L2` = 沿下一面的展开行程，只影响面内落点。

日志新增：

```text
PROBE_LEG,seg=DRIVE1,len=219.3,ach=...,hdg=...,raw=258.0,k=0.85
```

### 3.2 宽度公式失配不再直接判整遍扫描无效

新增 `probe_side_pair_plausible()`：检查两条原始边界跨度相对名义面宽、`s_m` 与扫描行程是否仍合理。合理时：

- `side_warn=14`；
- `face.valid=1`；
- 按首面锁存 + 面序交替恢复名义宽窄；
- 照常开到 `s_m` 做 `VERIFY`，复测也允许“中点可信、宽度公式不可信”。

### 3.3 异常重扫回中点，不回旧原点

新增 `probe_side_retry_target()`：若已有两个边界且 `s_m` 在可用行程内，重扫目标取 `face.s_m`；只有中点本身不可用时才回 0。

### 3.4 面内证据不足时不提前刹车

`probe_side_feed()` 在 `side_inface < LF_SIDE_EDGE_MIN_INFACE` 时不再因候选外点置 `side_edge_pending`。这样不会出现“刹车后证据永远长不出来”的死锁。

### 3.5 同期带入的现场调参

- `Board/board.c`：超声触发周期统一用 `LF_SIDE_PING_MS=40 ms`；每次触发发布唯一 `us_result_seq`，无回波在下一触发前结算为 `0`，侧扫的 `probe_side_ping()` 才能把“旧回波/丢帧/跨周期残留”分清。
- 默认侧扫速度 `50→75 mm/s`；因帧周期同为 3 mm（75×0.04），不牺牲采样间距。
- `update_odometry` 用 `θ+Δθ/2` 中点积分，减少侧扫/开位期间转弯积分误差。
- `probe_drive_signed(..., ref_rad, ...)` 显式传入 `side_origin_rad`，避免把 SIDE_MOVE 相位起点当成扫描轴基准。

---

## 4. 验证

| 项 | 结果 |
| --- | --- |
| Host 单测 | **38/38 PASS** |
| `probeside` 新增回归 | `h=205、原始边 +62.6/−3.7、w=12.3、s_m=29.5 → 直接去 s_m 复测` |
| 异常重扫回归 | 即使原始跨度也不可信，有两个边时优先回 `s_m` 而非 0 |
| 死锁回归 | `side_inface=5` 时出现远回波，不置 pending、不刹车 |
| LEG 回归 | `L1_raw` 与 `0.85×raw` 断言通过 |
| Debug 固件 | FLASH **119480 B**，0 告警 |
| Release 固件 | FLASH **111696 B**，0 告警 |
| 主机自检 | `python tools/host_station.py --selftest` PASS |
| 实车 | 已烧录；用户反馈“这次相当好”，量化日志待补 |

---

## 5. 下次录数重点

1. `PROBE_LEG,seg=DRIVE1,...,raw=258.0,k=0.85`，期望 `len≈219.3 mm`。
2. 第二面 `PROBE_SIDE_LOCK,h=`，期望从 205 mm 降到 **160~175 mm**；若 `<100 mm` 或转弯擦箱，回调 `LF_LEG_L1_SCALE`。
3. 第二面应出现 `PROBE_SIDE_MID,...,warn=14`，随后 `SIDE_MOVE → VERIFY → LASER`，不再出现 `edges=0` 的 13.4 s 超时。
4. 量首次光斑到窄面中心线的水平距离；窄面只有 50 mm，最终仍需按光斑偏距微调。
5. 把本轮日志补到 `data/bench/`，并在 `docs/validation.json` 增加 `m4g_l1_85_narrow_retry_20261125` 的量化字段。

---

## 6. 复现/烧录

```powershell
# 构建
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Configuration Debug
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Test

# 烧录（用户已执行）
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Configuration Debug -Flash
python tools\host_station.py push --port COM3 --mode GROUND
```

---

## 7. 相关文档

- [队友交接指南](teammate-targeting-handover.md)
- [代码全景说明](codebase-guide.md)
- [M4f AIM 边界法与航向补偿](M4f-aim-boundary-20261124.md)
- [M4e 四面全流程通过记录](M4e-full4-20261123.md)
