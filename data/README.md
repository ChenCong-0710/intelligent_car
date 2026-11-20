# 测试与标定数据归档目录 (`data/`)

本目录由蓝牙上位机软件 `tools/host_station.py` 自动创建并写入测试数据（默认保存在电脑本机，不频繁擦写单片机片内 Flash）：

- `latest_calibration.json`：最近一次标定的完整数据包（含 5 轮空载 `AIR` 与 5 轮地面带载 `GROUND` 15 档原始散点、平均值、拟合死区/斜率）。
- `calibration_session_<时间戳>.json`：每次标定的历史快照归档。
- `mcal_<mode>_<时间戳>.csv`：15 档占空比 (`0.06 ~ 0.95`) × 5 轮左右轮实测轮速明细表。
- `track_telemetry_<时间戳>.csv`：赛道跑圈 10 Hz 全量遥测流（四路红外 `raw0..3`、`q0..3`、`pos`、`vL/vR`、`tL/tR`、`uL/uR`），用于软件调参及实测反哺纯硬件模拟 PID。
- `port_config.json`（可选）：记录默认蓝牙串口 `COMx`，方便上位机及 Agent 一键直连。
