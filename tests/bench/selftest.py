# -*- coding: utf-8 -*-
"""测试程序的离线自检：不需要小车，验证「判据」真的能分辨好坏日志。

为什么需要它：2026-11-23 之前测斜失效，是因为脚本把「h1==h2」当成「车是平行的」。
所以判据本身必须先被测试 —— 这里用历史真实日志做正反用例：

  * 好日志（独立重采、Δh 正确、corr 符号正确）必须 PASS；
  * 历史坏日志（h1==h2、corr=0）必须 FAIL；
  * warn=8（h2 重采不足）、corr 符号反了、turn 超时……都必须 FAIL。

跑法：python tests/bench/run.py selftest
"""
from __future__ import annotations

import math

from . import protocol

# ---------------------------------------------------------------------------
# 真实格式的样本行（字段顺序与固件 main.c 一致）
# ---------------------------------------------------------------------------
GOOD_SIDE_LOCK = ("PROBE_SIDE_LOCK,h=383.4,pred=0.0,us=2400,ok=1,h1=394.0,h2=383.4,"
                  "h3=390.0,corr=-10.0,warn=0,ds=60")
BUGGY_SIDE_LOCK_H1EQH2 = ("PROBE_SIDE_LOCK,h=167.0,pred=0.0,us=2400,ok=1,h1=167.0,h2=167.0,"
                          "h3=176.0,corr=0.0,warn=0,ds=60")
WARN8_SIDE_LOCK = ("PROBE_SIDE_LOCK,h=394.0,pred=0.0,us=2400,ok=1,h1=394.0,h2=394.0,"
                   "h3=393.0,corr=0.0,warn=8,ds=60")
WRONG_SIGN_SIDE_LOCK = ("PROBE_SIDE_LOCK,h=383.4,pred=0.0,us=2400,ok=1,h1=394.0,h2=383.4,"
                        "h3=390.0,corr=10.0,warn=0,ds=60")
PARALLEL_SIDE_LOCK = ("PROBE_SIDE_LOCK,h=390.2,pred=0.0,us=2400,ok=1,h1=390.0,h2=390.5,"
                      "h3=390.0,corr=0.0,warn=0,ds=60")
PARALLEL_BAD_SIDE_LOCK = ("PROBE_SIDE_LOCK,h=390.2,pred=0.0,us=2400,ok=1,h1=390.0,h2=400.0,"
                          "h3=390.0,corr=0.0,warn=0,ds=60")
RIGHT_SIDE_LOCK = ("PROBE_SIDE_LOCK,h=400.6,pred=0.0,us=600,ok=1,h1=390.0,h2=400.6,"
                   "h3=395.0,corr=-10.0,warn=0,ds=60")
SIDE_MID = "PROBE_SIDE_MID,s_m=13.0,w=150.0,kind=wide,retry=0,k=-0.250"
SIDE_MID_NARROW = "PROBE_SIDE_MID,s_m=12.0,w=50.0,kind=narrow,retry=0,k=-0.250"
SIDE_CFG_OK = ("PROBE_SIDE_CFG,scan=1,left=1,us=2400,spd=75,reach=260,band=15.0,"
               "jump=40.0,verify=0,gamma=7.5,pan_x=30,fit_rms=5.0")
SIDE_CFG_WRONG = SIDE_CFG_OK.replace("us=2400", "us=600")
US_ODOM = ("US_ODOM,obs_mm=394,obs_ok=1,servo_center=1500,dist_mm=0.0,hdg_deg=0.00,"
           "x_mm=0.0,y_mm=0.0,vL=0.0,vR=0.0")
STATUS_OK = "STATE=STOPPED FAULT=OK CAL=0 MCAL=0(DEFAULT gnd=0 air=0) speed=120.0"
TURN_DONE_OK = "MOVE_DONE,mode=TURN,target=10.0,actual_mm=0.0,actual_deg=10.16,ms=148,timeout=0"
TURN_DONE_TIMEOUT = "MOVE_DONE,mode=TURN,target=10.0,actual_mm=0.0,actual_deg=5.00,ms=6000,timeout=1"

FACES2_GOOD = [
    "PROBE: starting 2-face CCW-side target probing (scan=B, laser=1000ms, aim=1500ms, edge=120/150mm, turn=90.0deg).",
    "PROBE_MODE,scan=B,side=L,us=2400",
    "PROBE_FACE,phi=1.20,rho=180.0,rms=3.0,hits=5,valid=1",
    "PROBE_TURN,tag=PAR,cmd=-88.0,ach=-89.1,ms=900,ok=1,dir=R",
    "PROBE_SIDE_LOCK,h=180.0,pred=210.0,us=2400,ok=1,h1=178.0,h2=177.4,h3=180.0,corr=0.0,warn=0,ds=60",
    SIDE_MID,
    "PROBE_LEG,seg=DRIVE1,len=60.0,ach=60.2,hdg=0.00",
    "PROBE_LEG,seg=TURN,cmd=-90.0,ach=-89.5,ms=880,dir=R,h_pred=180",
    "PROBE_LEG,seg=DRIVE2,len=50.0,ach=50.1,hdg=0.00",
    "PROBE_SIDE_LOCK,h=175.0,pred=180.0,us=2400,ok=1,h1=176.0,h2=175.2,h3=175.0,corr=-0.5,warn=0,ds=60",
    SIDE_MID_NARROW,
    "EVENT 120000 STOPPED OK",
]


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
def _find(checks, needle):
    return [c for c in checks if needle in c.name]


def _all_ok(checks):
    return all(c.ok for c in checks)


def _line(text):
    return protocol.parse_line(text)


# ---------------------------------------------------------------------------
# 用例
# ---------------------------------------------------------------------------
def test_parse_basic():
    ln = _line(GOOD_SIDE_LOCK)
    assert ln is not None and ln.tag == "PROBE_SIDE_LOCK"
    assert ln.i("ok") == 1 and abs(ln.f("h2") - 383.4) < 1e-6
    assert ln.s("us") == "2400"
    ev = _line("EVENT 123456 STOPPED OK")
    assert ev is not None and ev.s("state") == "STOPPED" and ev.s("fault") == "OK"
    st = _line(STATUS_OK)
    assert st is not None and st.s("state") == "STOPPED" and st.s("fault") == "OK"
    assert st.s("mcal") == "0(DEFAULT"


def test_status_ok():
    checks, metrics = protocol.criteria_status(_line(STATUS_OK), _line(US_ODOM))
    assert _all_ok(checks), [str(c) for c in checks if not c.ok]
    assert metrics["obs_mm"] == 394


def test_side_lock_good_skew_left():
    lines = [GOOD_SIDE_LOCK, SIDE_MID]
    checks, metrics = protocol.criteria_side_lock(lines, skew_deg=10.0, left=True, side_us=2400)
    assert _all_ok(checks), [str(c) for c in checks if not c.ok]
    assert abs(metrics["dh_mm"] - (-10.58)) < 0.1


def test_side_lock_h1eqh2_must_fail():
    """2026-11-23 的历史坏日志：h1==h2、corr=0。判据必须判 FAIL。"""
    lines = [BUGGY_SIDE_LOCK_H1EQH2, SIDE_MID]
    checks, _ = protocol.criteria_side_lock(lines, skew_deg=10.0, left=True, side_us=2400)
    assert not _all_ok(checks)
    bad = _find(checks, "h2 确实被重新采样")
    assert bad and not bad[0].ok, "h1==h2 必须被 'h2 确实被重新采样' 判据抓住"
    bad2 = _find(checks, "h2 与 h1 独立")
    assert bad2 and not bad2[0].ok


def test_side_lock_warn8_must_fail():
    lines = [WARN8_SIDE_LOCK, SIDE_MID]
    checks, _ = protocol.criteria_side_lock(lines, skew_deg=10.0, left=True, side_us=2400)
    assert not _all_ok(checks)
    bad = _find(checks, "warn=0")
    assert bad and not bad[0].ok


def test_side_lock_wrong_corr_sign_must_fail():
    lines = [WRONG_SIGN_SIDE_LOCK, SIDE_MID]
    checks, _ = protocol.criteria_side_lock(lines, skew_deg=10.0, left=True, side_us=2400)
    assert not _all_ok(checks)
    bad = _find(checks, "corr 符号")
    assert bad and not bad[0].ok


def test_side_lock_parallel():
    checks, _ = protocol.criteria_side_lock([PARALLEL_SIDE_LOCK, SIDE_MID], skew_deg=0.0)
    assert _all_ok(checks), [str(c) for c in checks if not c.ok]
    checks, _ = protocol.criteria_side_lock([PARALLEL_BAD_SIDE_LOCK, SIDE_MID], skew_deg=0.0)
    assert not _all_ok(checks), "平行摆位下 h2-h1=10 mm 必须判 FAIL"


def test_side_lock_right_geometry():
    """右面（side_left=0）：同样的 h2>h1 期望 corr=-θ。"""
    checks, metrics = protocol.criteria_side_lock([RIGHT_SIDE_LOCK, SIDE_MID], skew_deg=10.0,
                                                  left=False, side_us=600)
    assert _all_ok(checks), [str(c) for c in checks if not c.ok]
    assert abs(metrics["dh_mm"] - 10.58) < 0.1
    # 如果拿左面判据去套右面日志，Δh 符号必须判错
    checks, _ = protocol.criteria_side_lock([RIGHT_SIDE_LOCK, SIDE_MID], skew_deg=10.0,
                                            left=True, side_us=2400)
    assert not _all_ok(checks)


def test_side_cfg():
    assert _all_ok(protocol.criteria_side_cfg(_line(SIDE_CFG_OK), left=True, side_us=2400))
    assert not _all_ok(protocol.criteria_side_cfg(_line(SIDE_CFG_WRONG), left=True, side_us=2400))


def test_turn():
    checks, metrics = protocol.criteria_turn(_line(TURN_DONE_OK), 10.0, 3.0)
    assert _all_ok(checks), [str(c) for c in checks if not c.ok]
    assert abs(metrics["err_deg"] - 0.16) < 0.01
    checks, _ = protocol.criteria_turn(_line(TURN_DONE_TIMEOUT), 10.0, 3.0)
    assert not _all_ok(checks)
    checks, _ = protocol.criteria_turn(None, 10.0, 3.0)
    assert not _all_ok(checks)


def test_faces2():
    checks, metrics = protocol.criteria_faces2(FACES2_GOOD, elapsed_ms=120000)
    assert _all_ok(checks), [str(c) for c in checks if not c.ok]
    assert metrics["side_locks_ok"] == 2 and metrics["mids"] == 2
    # 少一面锁面失败
    checks, _ = protocol.criteria_faces2(FACES2_GOOD[:4] + FACES2_GOOD[5:], elapsed_ms=120000)
    assert not _all_ok(checks)


def test_done_event():
    ev = protocol.wait_done_event(["EVENT 1 AIM OK", "EVENT 2 STOPPED OK"])
    assert ev is not None and ev.s("state") == "STOPPED"


TESTS = [
    ("parse_basic", test_parse_basic),
    ("status_ok", test_status_ok),
    ("side_lock_good_skew_left", test_side_lock_good_skew_left),
    ("side_lock_h1eqh2_must_fail", test_side_lock_h1eqh2_must_fail),
    ("side_lock_warn8_must_fail", test_side_lock_warn8_must_fail),
    ("side_lock_wrong_corr_sign_must_fail", test_side_lock_wrong_corr_sign_must_fail),
    ("side_lock_parallel", test_side_lock_parallel),
    ("side_lock_right_geometry", test_side_lock_right_geometry),
    ("side_cfg", test_side_cfg),
    ("turn", test_turn),
    ("faces2", test_faces2),
    ("done_event", test_done_event),
]


def main() -> int:
    print("== 测试程序离线自检（不需要小车） ==")
    failed = 0
    for name, fn in TESTS:
        try:
            fn()
            print("  PASS %s" % name)
        except AssertionError as exc:
            failed += 1
            print("  FAIL %s : %s" % (name, exc))
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print("  FAIL %s : 未预期异常 %r" % (name, exc))
    total = len(TESTS)
    print("\nSELFTEST: %d/%d passed" % (total - failed, total))
    return 0 if failed == 0 else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
