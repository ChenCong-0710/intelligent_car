# -*- coding: utf-8 -*-
"""串口协议层：把固件日志解析成结构化数据，并提供功能判据（纯函数）。

这一层不碰串口，所以可以在没有小车的情况下，用录制的日志离线自检
（见 selftest.py）。所有判据的物理含义都写在注释里；现场出问题先看这里，
再看固件日志。

固件日志格式（Core/Src/main.c，只列本程序用到的）：

    PROBE_SIDE_LOCK,h=..,pred=..,us=..,ok=..,h1=..,h2=..,h3=..,corr=..,warn=..,ds=..
    PROBE_SIDE_MID,s_m=..,w=..,kind=..,retry=..,k=..
    PROBE_SIDE_FAIL,lock_stage=..,edges=..,retry=..,h=..
    PROBE_SIDE_YAW,dh=..,ds=..,corr=..
    PROBE_TURN,tag=PAR,cmd=..,ach=..,ms=..,ok=..,dir=..
    PROBE_LEG,seg=DRIVE1|TURN|DRIVE2|PROX|PROX2|BACKUP|FAIL,...
    MOVE_DONE,mode=TURN,target=..,actual_mm=..,actual_deg=..,ms=..,timeout=..
    US_ODOM,obs_mm=..,obs_ok=..,servo_center=..,dist_mm=..,hdg_deg=..,x_mm=..,y_mm=..
    PROBE_SIDE_CFG,scan=..,left=..,us=..,spd=..,reach=..,...
    PROBE_FACE,phi=..,rho=..,rms=..,hits=..,valid=..
    EVENT <ms> <state> <fault>
    STATE=<state> FAULT=<fault> CAL=.. MCAL=..

约定：
  * left=True 表示箱子在车体左侧（固件 probe_side_left=1）；
  * 本车实测：2400 us 指向车体左侧（600 us 是右侧）。箱子在左侧时用 side_us=2400，
    与固件默认一致；2026-11-23 13:40 的舵机扫角实测确认：600→2400 是从右向左扫，
    最后（2400）读到的 167~170 mm 就是盒子。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

# ---------------------------------------------------------------------------
# 与固件保持一致的常数（Control/line_follower.h）
# ---------------------------------------------------------------------------
SIDE_H_MIN_MM = 80.0        # LF_SIDE_H_MIN_MM
SIDE_H_MAX_MM = 450.0       # LF_SIDE_H_MAX_MM（2026-11-23 从 350 放到 450）
SIDE_TILT_SHIFT_MM = 60.0   # LF_SIDE_TILT_SHIFT_MM：测斜两点法基线
SIDE_TILT_MIN_DEG = 1.5     # LF_SIDE_TILT_MIN_DEG：死区
SIDE_TILT_MAX_DEG = 15.0    # LF_SIDE_TILT_MAX_DEG：钳位
SIDE_US_LEFT_DEFAULT = 2400  # 本车实测：2400 us 朝车体左侧（600 us = 右侧）
SIDE_US_RIGHT_DEFAULT = 600


# ---------------------------------------------------------------------------
# 日志行
# ---------------------------------------------------------------------------
class LogLine:
    """一行固件日志：tag + 原始文本 + 解析后的字段。"""

    __slots__ = ("tag", "raw", "fields")

    def __init__(self, tag: str, raw: str, fields: dict[str, str]):
        self.tag = tag
        self.raw = raw
        self.fields = fields

    # -- 取字段 ---------------------------------------------------------
    def s(self, key: str, default: Optional[str] = None) -> Optional[str]:
        return self.fields.get(key, default)

    def f(self, key: str, default: Optional[float] = None) -> Optional[float]:
        try:
            return float(self.fields[key])
        except (KeyError, ValueError):
            return default

    def i(self, key: str, default: Optional[int] = None) -> Optional[int]:
        v = self.f(key)
        return default if v is None else int(v)

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        return "<LogLine %s %r>" % (self.tag, self.raw)


def parse_line(line: str) -> Optional[LogLine]:
    """把一行日志解析成 LogLine；空行/非协议行返回 None。"""
    line = line.strip()
    if not line:
        return None
    if line.startswith("EVENT "):
        parts = line.split()
        if len(parts) >= 4:
            return LogLine("EVENT", line, {"ms": parts[1], "state": parts[2], "fault": parts[3]})
        return LogLine("EVENT", line, {})
    if "=" in line.split(",")[0]:
        # 形如 "STATE=STOPPED FAULT=OK CAL=0 ..."（空格分隔）
        fields = {}
        for tok in line.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                fields[k.lower()] = v
        return LogLine(line.split("=", 1)[0], line, fields)
    tag, sep, rest = line.partition(",")
    if not sep:
        return None
    fields = {}
    for tok in rest.split(","):
        if "=" in tok:
            k, v = tok.split("=", 1)
            fields[k.lower()] = v
    return LogLine(tag, line, fields)


def parse_lines(lines: Iterable[str], tag: Optional[str] = None) -> list[LogLine]:
    out = []
    for raw in lines:
        ln = parse_line(raw)
        if ln is None:
            continue
        if tag is None or ln.tag == tag:
            out.append(ln)
    return out


def last_line(lines: Iterable[str], tag: str) -> Optional[LogLine]:
    best = None
    for raw in lines:
        ln = parse_line(raw)
        if ln is not None and ln.tag == tag:
            best = ln
    return best


def find_text(lines: Iterable[str], needle: str) -> Optional[str]:
    for raw in lines:
        if needle in raw:
            return raw
    return None


def wait_done_event(lines: Iterable[str]) -> Optional[LogLine]:
    """取最后一条运行结束 EVENT：STOPPED / OBSTACLE_STOP / FAULT。"""
    done = None
    for raw in lines:
        ln = parse_line(raw)
        if ln is None or ln.tag != "EVENT":
            continue
        if ln.s("state") in ("STOPPED", "OBSTACLE_STOP", "FAULT"):
            done = ln
    return done


# ---------------------------------------------------------------------------
# 判定结果
# ---------------------------------------------------------------------------
@dataclass
class Check:
    name: str
    ok: bool
    detail: str = ""

    def __str__(self) -> str:
        mark = "PASS" if self.ok else "FAIL"
        return "[%s] %s%s" % (mark, self.name, ("  (" + self.detail + ")") if self.detail else "")


@dataclass
class CaseResult:
    case: str
    checks: list[Check] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    elapsed_ms: Optional[int] = None
    lines: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.checks) and all(c.ok for c in self.checks)

    def fail_count(self) -> int:
        return sum(0 if c.ok else 1 for c in self.checks)

    def to_dict(self) -> dict:
        return {
            "case": self.case,
            "ok": self.ok,
            "checks": [{"name": c.name, "ok": c.ok, "detail": c.detail} for c in self.checks],
            "metrics": self.metrics,
            "notes": self.notes,
            "elapsed_ms": self.elapsed_ms,
        }


# ---------------------------------------------------------------------------
# 判据：本车功能验收
# ---------------------------------------------------------------------------
def _range_check(name: str, value: Optional[float], lo: float, hi: float) -> Check:
    if value is None:
        return Check(name, False, "字段缺失")
    return Check(name, lo <= value <= hi, "%.1f in [%.0f, %.0f]" % (value, lo, hi))


def criteria_status(status: Optional[LogLine], us: Optional[LogLine]) -> tuple[list[Check], dict]:
    """只读自检：停机、无故障、超声/舵机配置可读。"""
    checks: list[Check] = []
    metrics: dict = {}
    if status is None:
        return [Check("读到 STATE 行", False)], metrics
    state = status.s("state")
    fault = status.s("fault")
    metrics["state"] = state
    metrics["fault"] = fault
    checks.append(Check("状态为 STOPPED/OBSTACLE_STOP", state in ("STOPPED", "OBSTACLE_STOP"), str(state)))
    checks.append(Check("无故障 FAULT=OK", fault == "OK", str(fault)))
    if us is not None:
        metrics["obs_mm"] = us.i("obs_mm")
        metrics["obs_ok"] = us.i("obs_ok")
        metrics["servo_center"] = us.i("servo_center")
        checks.append(Check("超声读数有效（obs_ok==1）", us.i("obs_ok") == 1, "obs_mm=%s" % us.i("obs_mm")))
        checks.append(_range_check("超声距离在声呐量程 [20, 3000] mm", us.f("obs_mm"), 20.0, 3000.0))
        checks.append(Check("舵机中位为 1500 us", us.i("servo_center") == 1500, str(us.i("servo_center"))))
    return checks, metrics


def criteria_side_cfg(cfg: Optional[LogLine], left: bool, side_us: int) -> list[Check]:
    """确认功能配置真的生效（不是脚本以为改了、固件没改）。"""
    if cfg is None:
        return [Check("读到 PROBE_SIDE_CFG", False)]
    return [
        Check("scan_mode=1（侧扫模式 B）", cfg.i("scan") == 1, "scan=%s" % cfg.s("scan")),
        Check("side_left=%d" % (1 if left else 0), cfg.i("left") == (1 if left else 0), "left=%s" % cfg.s("left")),
        Check("side_us=%d" % side_us, cfg.i("us") == side_us, "us=%s" % cfg.s("us")),
    ]


def criteria_turn(done: Optional[LogLine], cmd_deg: float, tol_deg: float = 3.0) -> tuple[list[Check], dict]:
    """turn 功能验收：编码器原地转的到位精度（台架实测 turn 10 50 -> +10.16）。"""
    checks: list[Check] = []
    metrics: dict = {"cmd_deg": cmd_deg}
    if done is None:
        return [Check("收到 MOVE_DONE", False, "超时未收到")], metrics
    target = done.f("target")
    actual = done.f("actual_deg")
    metrics.update({"target": target, "actual_deg": actual,
                    "ms": done.i("ms"), "timeout": done.i("timeout")})
    checks.append(Check("MOVE_DONE.mode=TURN", done.s("mode") == "TURN", str(done.s("mode"))))
    checks.append(Check("目标角与命令一致", target is not None and abs(target - cmd_deg) < 0.01,
                        "target=%s cmd=%.1f" % (target, cmd_deg)))
    if actual is not None:
        err = actual - cmd_deg
        metrics["err_deg"] = err
        checks.append(Check("到位误差 <= %.1f°" % tol_deg, abs(err) <= tol_deg,
                            "actual=%.2f err=%+.2f" % (actual, err)))
    else:
        checks.append(Check("actual_deg 可读", False))
    checks.append(Check("未超时", done.i("timeout") == 0, "timeout=%s" % done.s("timeout")))
    return checks, metrics


def criteria_side_lock(lines: Iterable[str], skew_deg: float, left: bool = True,
                       side_us: int = SIDE_US_LEFT_DEFAULT,
                       shift_mm: float = SIDE_TILT_SHIFT_MM) -> tuple[list[Check], dict]:
    """SIDE_LOCK 功能验收（M2 + 11-23 测斜修复）。

    skew_deg = 运行 side 之前用 turn 命令刻意摆出的角度（+ = CCW 左转）：
      * 左面（箱在左）：turn +θ 让车头更靠近箱面；
      * 推导：沿车轴前进 s，h 的变化 = -s*tanθ（左面）/ +s*tanθ（右面）；
      * 修正角期望 corr = -θ（左、右都成立，因为固件对右面取反）。
    """
    checks: list[Check] = []
    metrics: dict = {"skew_deg": skew_deg, "left": left, "side_us": side_us}
    lock = last_line(lines, "PROBE_SIDE_LOCK")
    if lock is None:
        checks.append(Check("收到 PROBE_SIDE_LOCK", False, "SIDE_LOCK 没完成（超时或前置失败）"))
        return checks, metrics

    h = lock.f("h");  h1 = lock.f("h1"); h2 = lock.f("h2"); h3 = lock.f("h3")
    corr = lock.f("corr"); warn = lock.i("warn"); ds = lock.f("ds")
    metrics.update({"h": h, "h1": h1, "h2": h2, "h3": h3,
                    "corr_deg": corr, "warn": warn, "ds_mm": ds, "pred": lock.f("pred")})

    checks.append(Check("SIDE_LOCK ok=1（锁面成功）", lock.i("ok") == 1, "ok=%s" % lock.s("ok")))
    checks.append(Check("舵机用的就是配置的 side_us", lock.i("us") == side_us, "us=%s" % lock.s("us")))
    for name, v in (("h1", h1), ("h2", h2), ("h3", h3)):
        checks.append(_range_check("%s 在物理有效区间 [%.0f, %.0f] mm" % (name, SIDE_H_MIN_MM, SIDE_H_MAX_MM),
                                   v, SIDE_H_MIN_MM, SIDE_H_MAX_MM))
    checks.append(Check("warn=0（测斜无告警）", warn == 0,
                        "warn=%s%s" % (warn, "：h2 重采不足，60 mm 平移可能把光束带出面" if warn == 8 else "")))
    # ds = 平移段实际走到的最大里程：只排除「编码器完全没动」，排除不了打滑
    checks.append(Check("平移段真的走了约 %.0f mm（ds >= %.0f）" % (shift_mm, shift_mm - 10.0),
                        ds is not None and ds >= shift_mm - 10.0, "ds=%.0f" % (ds if ds is not None else -1)))

    if abs(skew_deg) >= SIDE_TILT_MIN_DEG:
        sign = -1.0 if left else 1.0
        expected_dh = sign * shift_mm * math.tan(math.radians(skew_deg))
        expected_corr = -skew_deg
        metrics["expect_dh_mm"] = expected_dh
        metrics["expect_corr_deg"] = expected_corr
        if h1 is not None and h2 is not None:
            dh = h2 - h1
            metrics["dh_mm"] = dh
            checks.append(Check("h2 与 h1 独立且等于预期 Δh=%.1f mm" % expected_dh,
                                abs(dh - expected_dh) <= 4.0,
                                "h2-h1=%+.1f" % dh))
            checks.append(Check("h2 确实被重新采样（h2 不等于 h1）", abs(dh) >= 3.0,
                                "h1=%.1f h2=%.1f" % (h1, h2)))
        if corr is not None:
            checks.append(Check("corr 符号/幅度正确（期望 %+.1f ±4°）" % expected_corr,
                                abs(corr - expected_corr) <= 4.0, "corr=%+.1f°" % corr))
    else:
        if h1 is not None and h2 is not None:
            metrics["dh_mm"] = h2 - h1
            checks.append(Check("平行摆位：|h2-h1| <= 5 mm", abs(h2 - h1) <= 5.0, "h2-h1=%+.1f" % (h2 - h1)))
        if corr is not None:
            checks.append(Check("平行摆位：|corr| <= 3°", abs(corr) <= 3.0, "corr=%+.1f°" % corr))

    # 锁面后必须真的进入 SIDE_SCAN 并测出宽度
    mid = last_line(lines, "PROBE_SIDE_MID")
    fail = last_line(lines, "PROBE_SIDE_FAIL")
    if mid is not None:
        metrics.update({"w_mm": mid.f("w"), "s_m": mid.f("s_m"), "kind": mid.s("kind"), "k": mid.f("k")})
        checks.append(Check("SIDE_SCAN 测出宽度 w > 30 mm", (mid.f("w") or 0) > 30.0,
                            "w=%.1f kind=%s" % (mid.f("w") or -1, mid.s("kind"))))
    else:
        checks.append(Check("收到 PROBE_SIDE_MID", False, "侧扫没跑完"))
    checks.append(Check("没有 PROBE_SIDE_FAIL", fail is None, fail.raw if fail else ""))
    return checks, metrics


def criteria_faces2(lines: Iterable[str], elapsed_ms: Optional[float] = None) -> tuple[list[Check], dict]:
    """M4 Gate 0 自动 faces=2 验收（AIM->TURN_PAR->LOCK->SCAN->MOVE->LASER->LEG->面2->COMPLETE）。"""
    checks: list[Check] = []
    metrics: dict = {}
    lock_all = parse_lines(lines, "PROBE_SIDE_LOCK")
    lock_ok = [ln for ln in lock_all if ln.i("ok") == 1]
    mids = parse_lines(lines, "PROBE_SIDE_MID")
    turns = parse_lines(lines, "PROBE_TURN")
    legs = parse_lines(lines, "PROBE_LEG")
    done = wait_done_event(lines)

    metrics["side_locks"] = len(lock_all)
    metrics["side_locks_ok"] = len(lock_ok)
    metrics["mids"] = len(mids)
    metrics["turn_par"] = len(turns)
    metrics["leg_segments"] = len(legs)
    if done is not None:
        metrics["final_state"] = done.s("state")
        metrics["final_fault"] = done.s("fault")

    checks.append(Check("自动流程结束于 STOPPED/OBSTACLE_STOP 且无故障",
                        done is not None and done.s("state") in ("STOPPED", "OBSTACLE_STOP")
                        and done.s("fault") == "OK",
                        (done.raw if done else "没有结束 EVENT")))
    checks.append(Check("至少 2 面 SIDE_LOCK 成功（faces=2）", len(lock_ok) >= 2,
                        "%d/%d" % (len(lock_ok), len(lock_all))))
    for idx, ln in enumerate(lock_ok):
        checks.append(Check("面 %d SIDE_LOCK warn=0" % idx, ln.i("warn") == 0,
                            "h=%.1f warn=%s corr=%+.1f ds=%s" % (ln.f("h") or -1, ln.s("warn"),
                                                                  ln.f("corr") or 0, ln.s("ds"))))
        checks.append(_range_check("面 %d 锁面 h 合理" % idx, ln.f("h"), SIDE_H_MIN_MM, SIDE_H_MAX_MM))
    if mids:
        for idx, ln in enumerate(mids):
            checks.append(Check("面 %d 侧扫 w > 30 mm" % idx, (ln.f("w") or 0) > 30.0,
                                "w=%.1f kind=%s" % (ln.f("w") or -1, ln.s("kind"))))
    else:
        checks.append(Check("收到 PROBE_SIDE_MID", False))
    par = [ln for ln in turns if ln.s("tag") == "PAR"]
    par_ok = [ln for ln in par if ln.i("ok") == 1 and abs((ln.f("ach") or 0) - (ln.f("cmd") or 0)) <= 8.0]
    checks.append(Check("TURN_PAR ok=1 且 |ach-cmd| <= 8°", bool(par_ok),
                        par[-1].raw if par else "没有 PROBE_TURN,tag=PAR"))
    checks.append(Check("LEG 有 TURN/DRIVE 段",
                        any(ln.s("seg") in ("TURN", "DRIVE1", "DRIVE2") for ln in legs),
                        "segments=%d" % len(legs)))
    checks.append(Check("总时间 < 150 s", elapsed_ms is None or elapsed_ms < 150000.0,
                        "%.1f s" % ((elapsed_ms or 0) / 1000.0)))
    return checks, metrics
