# -*- coding: utf-8 -*-
"""功能测试用例：一个用例 = 一段可复现的操作 + 一组判据。

每个用例都返回 protocol.CaseResult；runner 负责落盘/打印。
用例之间不共享状态，也不假设上一条用例小车停在哪。
"""
from __future__ import annotations

import time
from typing import Optional

from . import protocol
from .device import BenchError, Car


def _final_state_check(done: Optional[protocol.LogLine]) -> protocol.Check:
    if done is None:
        return protocol.Check("最终回到停机状态", False, "没有结束 EVENT")
    ok = done.s("state") in ("STOPPED", "OBSTACLE_STOP") and done.s("fault") == "OK"
    return protocol.Check("最终回到停机状态且无故障", ok, done.raw)


def case_status(car: Car) -> protocol.CaseResult:
    """只读自检：串口通、停机、无故障、超声/舵机可读。不动机器。"""
    import time as _time
    t0 = _time.monotonic()
    res = protocol.CaseResult(case="status")
    car.link.clear()
    try:
        st = car.status()
        snap = car.us()
        checks, metrics = protocol.criteria_status(st, snap["us_odom"])
        res.checks.extend(checks)
        res.metrics.update(metrics)
        if snap["side_cfg"] is not None:
            res.metrics.update({("cfg_" + k): v for k, v in snap["side_cfg"].fields.items()})
    except BenchError as exc:
        res.checks.append(protocol.Check("只读自检成功", False, str(exc)))
    res.elapsed_ms = (_time.monotonic() - t0) * 1000.0
    res.lines = list(car.link.lines)
    return res


def case_turn(car: Car, angles: tuple[float, ...] = (10.0, -10.0),
              speed: float = 50.0, tol_deg: float = 3.0) -> protocol.CaseResult:
    """turn 功能验收：按命令原地转，编码器到位精度。默认 +10/-10 回到原航向。"""
    import time as _time
    t0 = _time.monotonic()
    res = protocol.CaseResult(case="turn")
    car.link.clear()
    res.metrics["angles"] = list(angles)
    res.metrics["speed"] = speed
    try:
        car.require_ready()
        before = car.us()["us_odom"]
        res.metrics["hdg_before"] = before.f("hdg_deg") if before else None
        for angle in angles:
            try:
                done = car.turn(angle, speed)
            except BenchError as exc:
                res.checks.append(protocol.Check("turn %+.0f° 执行" % angle, False, str(exc)))
                continue
            checks, metrics = protocol.criteria_turn(done, angle, tol_deg)
            for c in checks:
                c.name = "turn %+.0f°：%s" % (angle, c.name)
            res.checks.extend(checks)
            res.metrics["turn_%+.0f" % angle] = metrics
        after = car.us()["us_odom"]
        res.metrics["hdg_after"] = after.f("hdg_deg") if after else None
        drifts = res.metrics.get("hdg_before"), res.metrics.get("hdg_after")
        if None not in drifts:
            net = res.metrics["hdg_after"] - res.metrics["hdg_before"]
            res.metrics["hdg_net_deg"] = net
            # 对称角度序列净转角应接近 0
            expect_net = 0.0 if abs(sum(angles)) < 0.01 else sum(angles)
            res.checks.append(protocol.Check(
                "净转角符合预期（%+.1f° ±6°）" % expect_net, abs(net - expect_net) <= 6.0,
                "hdg %.2f -> %.2f，净 %+.2f°" % (res.metrics["hdg_before"], res.metrics["hdg_after"], net)))
    except BenchError as exc:
        res.checks.append(protocol.Check("turn 用例前置条件", False, str(exc)))
    res.elapsed_ms = (_time.monotonic() - t0) * 1000.0
    res.lines = list(car.link.lines)
    return res


def case_side_tilt(car: Car, skew_deg: float = 10.0, prompt: bool = False,
                   timeout: float = 70.0) -> protocol.CaseResult:
    """核心用例：用 turn 摆出**已知角度**，再跑 side，验收 SIDE_LOCK 测斜。

    流程（就是《台架测量操作单》里那条有效试验）：
      1. 配置功能参数：侧扫模式 B、side_left=1、side_us=2400（本机 2400us = 左侧）；
      2. 用 turn 命令摆出已知 skew（+ = CCW 左转，默认 +10°）——这一步本身就是功能验收；
      3. 发 side，等固件自己跑完 SIDE_LOCK -> SIDE_SCAN -> SIDE_MOVE；
      4. 判据：h2 必须与 h1 独立、Δh ≈ -60·tan(10°) = -10.6 mm、corr ≈ -10.4°、warn=0。

    skew=0 时是平行对照组（|Δh|<=5、|corr|<=3）。
    """
    res = protocol.CaseResult(case="side-tilt", elapsed_ms=None)
    car.link.clear()
    res.notes.append("摆位：箱子在车体左侧，云台轴距面 180~400 mm；车轴与面夹角 = skew=%+.1f°" % skew_deg)
    try:
        car.require_ready()
        snap = car.configure_side()
        res.checks.extend(protocol.criteria_side_cfg(snap["side_cfg"], car.side_left, car.side_us))

        if prompt:
            input("把车摆到面外、车轴与箱面约 %+.0f°，然后回车开始 side…" % skew_deg)

        if abs(skew_deg) >= protocol.SIDE_TILT_MIN_DEG:
            try:
                done = car.turn(skew_deg, 50.0)
            except BenchError as exc:
                res.checks.append(protocol.Check("前置摆角：turn %+.0f° 执行" % skew_deg, False, str(exc)))
                done = None
            if done is not None:
                checks, metrics = protocol.criteria_turn(done, skew_deg, 2.0)
                for c in checks:
                    c.name = "前置摆角：" + c.name
                res.checks.extend(checks)
                res.metrics["pre_skew"] = metrics
        time.sleep(0.3)

        lines, done_event, elapsed = car.side(timeout)
        res.lines = list(lines)
        res.elapsed_ms = elapsed
        checks, metrics = protocol.criteria_side_lock(
            lines, skew_deg, left=car.side_left, side_us=car.side_us)
        res.checks.extend(checks)
        res.metrics.update(metrics)
        res.checks.append(_final_state_check(done_event))
    except BenchError as exc:
        res.checks.append(protocol.Check("side-tilt 用例执行", False, str(exc)))
    return res


def case_side_parallel(car: Car, prompt: bool = False, timeout: float = 70.0) -> protocol.CaseResult:
    """平行对照组：车轴与面平行（skew=0），期望 corr≈0、|Δh|<=5。"""
    res = case_side_tilt(car, skew_deg=0.0, prompt=prompt, timeout=timeout)
    res.case = "side-parallel"
    res.notes = ["摆位：箱子在车体左侧，车轴尽量与面平行（0°）"]
    return res


def case_faces2(car: Car, prompt: bool = False, timeout: float = 170.0) -> protocol.CaseResult:
    """M4 Gate 0：faces=2 自动全流程（AIM->TURN_PAR->LOCK->SCAN->MOVE->LASER->LEG->面2->COMPLETE）。"""
    res = protocol.CaseResult(case="faces2")
    car.link.clear()
    res.notes.append("摆位：车停在盒面正前方约 15 cm、车头基本朝面；AIM 会自己找中心")
    try:
        car.require_ready()
        snap = car.configure_probe(faces=2, laser_ms=1000)
        res.checks.extend(protocol.criteria_side_cfg(snap["side_cfg"], car.side_left, car.side_us))
        if prompt:
            input("把车摆到盒面正前方约 15 cm，然后回车开始 probe（faces=2）…")
        lines, done_event, elapsed = car.probe(timeout)
        res.lines = list(lines)
        res.elapsed_ms = elapsed
        checks, metrics = protocol.criteria_faces2(lines, elapsed)
        res.checks.extend(checks)
        res.metrics.update(metrics)
        res.checks.append(_final_state_check(done_event))
    except BenchError as exc:
        res.checks.append(protocol.Check("faces2 用例执行", False, str(exc)))
    return res


CASE_TABLE = {
    "status": case_status,
    "turn": case_turn,
    "side-tilt": case_side_tilt,
    "side-parallel": case_side_parallel,
    "faces2": case_faces2,
}
