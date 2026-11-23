# -*- coding: utf-8 -*-
"""功能层：只通过固件的**功能命令**驱动小车（status/us/set/turn/side/leg/probe/stop）。

铁律（测试程序和功能分开）：
  * 这里不出现 outtest / vtest / mcal / servosweep 这类台架自检命令；
  * 不直接改控制器内部状态，只用固件对外暴露的命令行；
  * 任何异常路径都要保证最后发一次 stop，不能把小车轮子留在使能状态。

用法示例：

    with SerialLink("COM3") as link:
        car = Car(link, side_left=True, side_us=2400)
        car.init()
        car.require_ready()
        done = car.turn(10, 50)
        lines, elapsed = car.side()
"""
from __future__ import annotations

import re
import time
from typing import Optional

from . import protocol
from .link import LinkError, SerialLink


class BenchError(RuntimeError):
    """前置条件不满足 / 命令被固件拒绝 / 超时。"""


EVENT_DONE_RE = re.compile(r"^EVENT\s+\d+\s+(STOPPED|OBSTACLE_STOP|FAULT)\b")
DENIED_RE = re.compile(r"DENIED|ERROR|Unknown command")


class Car:
    """按功能命令操作小车；所有方法失败都抛 BenchError（带上下文）。"""

    def __init__(self, link: SerialLink, side_left: bool = True, side_us: int = protocol.SIDE_US_LEFT_DEFAULT):
        self.link = link
        self.side_left = side_left
        self.side_us = side_us
        self.last_status: Optional[protocol.LogLine] = None

    # -- 基础 -------------------------------------------------------------
    def init(self) -> "Car":
        """关掉 CSV 遥测（避免刷屏把事件行淹掉），清掉开机残留。"""
        self.link.send("log 0", 0.15)
        self.link.send("probe_log 0", 0.15)
        self.link.drain(0.15)
        self.link.clear()
        return self

    def status(self) -> protocol.LogLine:
        self.link.send("status", 0.15)
        line, _ = self.link.wait_prefix(["STATE="], 1.5)
        if line is None:
            raise BenchError("status 无响应（串口没接上，或固件在长时间阻塞动作中）")
        self.last_status = protocol.parse_line(line)
        return self.last_status

    def us(self) -> dict[str, Optional[protocol.LogLine]]:
        """读一次 US_ODOM / PROBE_CFG / PROBE_SIDE_CFG（固件 us 命令一次打三行）。"""
        self.link.send("us", 0.0)
        result: dict[str, Optional[protocol.LogLine]] = {
            "us_odom": None, "probe_cfg": None, "side_cfg": None}
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            line, _ = self.link.wait_prefix(
                ["US_ODOM,", "PROBE_CFG,", "PROBE_SIDE_CFG,"], max(0.05, deadline - time.monotonic()))
            if line is None:
                break
            if line.startswith("US_ODOM,"):
                result["us_odom"] = protocol.parse_line(line)
            elif line.startswith("PROBE_CFG,"):
                result["probe_cfg"] = protocol.parse_line(line)
            elif line.startswith("PROBE_SIDE_CFG,"):
                result["side_cfg"] = protocol.parse_line(line)
                break
        # 等剩下两行到齐（一般已经在缓冲里）
        self.link.drain(0.1)
        return result

    def require_ready(self) -> protocol.LogLine:
        st = self.status()
        state = st.s("state")
        fault = st.s("fault")
        if state not in ("STOPPED", "OBSTACLE_STOP"):
            raise BenchError("车不在可测试状态：STATE=%s（先发 stop，等它停下）" % state)
        if fault != "OK":
            raise BenchError("固件有故障：FAULT=%s（先 stop 清故障；故障清不掉就重上电）" % fault)
        return st

    def set_param(self, name: str, value) -> None:
        self.link.send("set %s %s" % (name, value), 0.08)
        line, got = self.link.wait_for(
            lambda x: x.startswith("SET OK") or x.startswith("SET ERROR"), 1.5)
        if line is None:
            raise BenchError("set %s %s 无回应（收到 %d 行）" % (name, value, len(got)))
        if not line.startswith("SET OK"):
            raise BenchError("固件拒绝 set %s %s：%s" % (name, value, line))

    def stop(self) -> None:
        """尽力停机；本身不抛异常（用于 finally）。"""
        try:
            self.link.send("stop", 0.1)
            self.link.wait_for(lambda x: x.startswith("STOP") or EVENT_DONE_RE.search(x), 2.0)
        except (LinkError, Exception):  # noqa: BLE001 - finally 里必须吞掉
            pass

    # -- 配置（功能参数，不是台架注入）-----------------------------------
    def configure_side(self, verify: bool = False) -> dict:
        self.require_ready()
        for key, value in (
            ("probe_laser_ms", 1000),
            ("probe_side_verify", 1 if verify else 0),
            ("probe_scan_mode", 1),
            ("probe_side_left", 1 if self.side_left else 0),
            ("probe_side_us", self.side_us),
        ):
            self.set_param(key, value)
        snap = self.us()
        return snap

    def configure_probe(self, faces: int = 2, laser_ms: int = 1000) -> dict:
        self.require_ready()
        for key, value in (
            ("probe_laser_ms", laser_ms),
            ("probe_side_verify", 0),
            ("probe_scan_mode", 1),
            ("probe_side_left", 1 if self.side_left else 0),
            ("probe_side_us", self.side_us),
            ("probe_faces", faces),
        ):
            self.set_param(key, value)
        return self.us()

    # -- 功能动作 ---------------------------------------------------------
    def _run_command(self, command: str, timeout: float) -> tuple[list[str], Optional[protocol.LogLine], float]:
        """发一条会自己跑完的功能命令，等它回到 STOPPED/OBSTACLE_STOP/FAULT。

        返回（期间收到的行，结束 EVENT，耗时 ms）。
        """
        self.link.clear()
        t0 = time.monotonic()
        self.link.send(command, 0.0)
        deadline = t0 + timeout
        got: list[str] = []
        done = None
        while time.monotonic() < deadline:
            line, chunk = self.link.wait_for(
                lambda x: bool(EVENT_DONE_RE.search(x)) or bool(DENIED_RE.search(x)),
                max(0.2, min(2.0, deadline - time.monotonic())))
            got.extend(chunk)
            if line is None:
                continue
            if DENIED_RE.search(line) and not line.startswith("EVENT"):
                raise BenchError("固件拒绝命令 %r：%s" % (command, line))
            done = protocol.parse_line(line)
            break
        elapsed_ms = (time.monotonic() - t0) * 1000.0
        if done is None:
            raise BenchError("命令 %r 在 %.0f s 内没有回到停机状态（最后 5 行：%r）"
                             % (command, timeout, got[-5:]))
        if done.s("fault") not in ("OK", None):
            # 回到 FAULT 也是「跑完了」，但结果肯定不通过，交给判据；这里不抛
            pass
        return got, done, elapsed_ms

    def turn(self, deg: float, speed: float = 50.0, timeout: float = 10.0) -> Optional[protocol.LogLine]:
        """原地转 deg 度（+ = 左转 CCW），等 MOVE_DONE。"""
        self.require_ready()
        self.link.clear()
        self.link.send("turn %g %g" % (deg, speed), 0.0)
        line, got = self.link.wait_prefix(["MOVE_DONE", "MOVE ERROR"], timeout)
        if line is None:
            raise BenchError("turn %g 在 %.0f s 内没有 MOVE_DONE（最后 5 行：%r）" % (deg, timeout, got[-5:]))
        if line.startswith("MOVE ERROR"):
            raise BenchError("turn 命令被拒绝：%s" % line)
        time.sleep(0.25)  # 等刹车停稳，避免下一条命令和它抢
        self.link.drain(0.15)
        return protocol.parse_line(line)

    def side(self, timeout: float = 70.0) -> tuple[list[str], Optional[protocol.LogLine], float]:
        return self._run_command("side", timeout)

    def leg(self, timeout: float = 90.0) -> tuple[list[str], Optional[protocol.LogLine], float]:
        return self._run_command("leg", timeout)

    def probe(self, timeout: float = 170.0) -> tuple[list[str], Optional[protocol.LogLine], float]:
        return self._run_command("probe", timeout)
