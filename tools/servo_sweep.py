#!/usr/bin/env python3
"""舵机连续往返台架测试工具。

用法:
    python tools/servo_sweep.py                 # 默认 COM 口，2 s 一个往返周期，Ctrl+C 停止
    python tools/servo_sweep.py --port COM3 --period 1500
    python tools/servo_sweep.py --seconds 10    # 运行 10 秒后自动停止

固件侧命令:
    servosweep [period_ms]   启动连续 900us -> 2100us -> 900us 三角波扫掠
    servosweep stop          停止扫掠
    stop                     立即停止全部输出

注意: 运行前请确认舵机周围没有障碍物，小车处于架空/安全状态。
"""

from __future__ import annotations

import argparse
import time

from host_station import SerialClient


def main() -> int:
    parser = argparse.ArgumentParser(description="连续扫掠云台舵机 (900..2100us)")
    parser.add_argument("--port", default="AUTO", help="串口，如 COM3 或 AUTO")
    parser.add_argument("--period", type=int, default=2000, help="完整往返周期 ms (500..10000)")
    parser.add_argument("--seconds", type=float, default=0.0, help="运行时长 s；0 表示直到 Ctrl+C")
    args = parser.parse_args()

    if not 500 <= args.period <= 10000:
        parser.error("--period 必须在 500..10000 ms 之间")

    client = SerialClient(args.port)
    try:
        client.send("stop")
        time.sleep(0.2)
        client.read_lines(0.2)
        client.send(f"servosweep {args.period}")
        print(f"[servo] 已连接 {client.port}，启动连续扫掠，周期 {args.period} ms")
        print("[servo] 停止: Ctrl+C，或在另一个终端发送: "
              f"python tools/host_station.py cmd --port {client.port} \"stop\"")
        started = time.monotonic()
        while True:
            for line in client.read_lines(0.2):
                if line.startswith(("SERVOSWEEP", "OUTTEST", "EVENT", "DENIED")):
                    print(f"[mcu] {line}")
            if args.seconds > 0 and time.monotonic() - started >= args.seconds:
                break
    except KeyboardInterrupt:
        print("\n[servo] 收到 Ctrl+C，停止扫掠")
    finally:
        try:
            client.send("stop")
            time.sleep(0.1)
        finally:
            client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
