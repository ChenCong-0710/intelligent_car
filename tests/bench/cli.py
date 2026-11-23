# -*- coding: utf-8 -*-
"""台架功能测试程序入口。

    python tests/bench/run.py status            # 只读自检（不动机器）
    python tests/bench/run.py turn              # turn +10/-10 功能验收
    python tests/bench/run.py side-tilt         # 已知 10° 斜置 + side（核心用例）
    python tests/bench/run.py side-parallel     # 平行对照组
    python tests/bench/run.py faces2            # M4 Gate0 faces=2 自动全流程
    python tests/bench/run.py selftest          # 离线自检（不需要小车）

常用参数：
    --port COM3          串口（默认环境变量 BENCH_PORT/M4_PORT/M3_PORT 或 COM3）
    --side left|right    箱子在车体哪一侧（默认 left）
    --side-us 2400       朝该侧的舵机脉宽（左侧默认 2400，右侧默认 600）
    --skew 10            已知摆角（+ = CCW 左转）
    --prompt             开跑前暂停，等人工摆位后回车
    --data-dir PATH      报告目录（默认 line-following-car/data/bench）
    --no-save            不落盘
"""
from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

from . import cases, report
from .device import BenchError, Car
from .link import LinkError, SerialLink, default_port


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="bench",
        description="小车台架功能测试程序（只通过功能命令驱动，不动固件功能代码）")
    parser.add_argument("case", choices=sorted(cases.CASE_TABLE) + ["selftest"],
                        help="要跑的用例")
    parser.add_argument("--port", default=default_port(), help="串口，默认 %(default)s")
    parser.add_argument("--side", choices=("left", "right"), default="left",
                        help="箱子在车体哪一侧，默认 left")
    parser.add_argument("--side-us", type=int, default=None,
                        help="朝该侧的舵机脉宽；默认左侧 2400 / 右侧 600")
    parser.add_argument("--skew", type=float, default=10.0,
                        help="side-tilt 的已知摆角（度，+ = CCW 左转），默认 10")
    parser.add_argument("--angles", default="10,-10",
                        help="turn 用例的角度序列，默认 10,-10")
    parser.add_argument("--speed", type=float, default=50.0, help="turn 速度 mm/s，默认 50")
    parser.add_argument("--tol", type=float, default=3.0, help="turn 到位容差（度），默认 3")
    parser.add_argument("--timeout", type=float, default=None,
                        help="用例动作超时（side 默认 70s，faces2 默认 170s）")
    parser.add_argument("--prompt", action="store_true", help="开跑前暂停，等人工摆位")
    parser.add_argument("--data-dir", default=str(report.DEFAULT_DATA_DIR), help="报告目录")
    parser.add_argument("--no-save", action="store_true", help="不落盘")
    return parser


def run_cli(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.case == "selftest":
        from . import selftest
        return selftest.main()

    side_left = args.side == "left"
    side_us = args.side_us
    if side_us is None:
        side_us = 2400 if side_left else 600
    data_dir = Path(args.data_dir)

    link = SerialLink(args.port)
    car = Car(link, side_left=side_left, side_us=side_us)
    result = None
    try:
        link.open()
        car.init()
        print("PORT=%s  side=%s us=%d" % (args.port, args.side, side_us))
        st = car.status()
        print("STATE=%s FAULT=%s CAL=%s" % (st.s("state"), st.s("fault"), st.s("cal")))

        if args.case == "status":
            result = cases.case_status(car)
        elif args.case == "turn":
            angles = tuple(float(x) for x in args.angles.split(",") if x.strip())
            result = cases.case_turn(car, angles=angles, speed=args.speed, tol_deg=args.tol)
        elif args.case == "side-tilt":
            timeout = args.timeout or 70.0
            result = cases.case_side_tilt(car, skew_deg=args.skew, prompt=args.prompt, timeout=timeout)
        elif args.case == "side-parallel":
            timeout = args.timeout or 70.0
            result = cases.case_side_parallel(car, prompt=args.prompt, timeout=timeout)
        elif args.case == "faces2":
            timeout = args.timeout or 170.0
            result = cases.case_faces2(car, prompt=args.prompt, timeout=timeout)
        else:  # pragma: no cover - argparse 已经限制
            raise BenchError("未知用例：%s" % args.case)
    except LinkError as exc:
        print("SERIAL ERROR: %s" % exc, file=sys.stderr)
        return 2
    except BenchError as exc:
        print("BENCH ERROR: %s" % exc, file=sys.stderr)
        result = None
    except KeyboardInterrupt:
        print("\n中断：正在停车…")
        car.stop()
        return 130
    except Exception:  # noqa: BLE001 - 未预期异常也要先停车
        traceback.print_exc()
        result = None
    finally:
        # 任何路径都保证：先停车，再关串口
        try:
            if link is not None:
                car.stop()
        finally:
            link.close()

    if result is not None:
        report.print_result(result)
        if not args.no_save:
            log_path, json_path = report.save_case(result, result.lines, data_dir, args.port)
            print("  saved: %s" % log_path)
            print("  saved: %s" % json_path)
        return 0 if result.ok else 1
    return 3


def main(argv=None) -> int:
    return run_cli(argv)
