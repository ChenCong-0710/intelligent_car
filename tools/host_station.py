#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32G474 智能小车蓝牙闭环上位机软件 (Agent 自动调度 CLI + 人工可视化 GUI 双模式)
================================================================================
设计目标：
  1. 【以 Agent 自动高频收发与数据分析为主】：提供全套无界面 CLI 子命令，输出结构化 JSON，
     让 Agent 可直接在终端完成端口识别、5轮电机标定、前馈等速批量验证、赛道红外遥测录制、
     CSV 数据归档以及基于实测数据的纯硬件模拟 PID 优化分析。
  2. 【兼顾人工直观测试】：无参数直接运行 `python tools/host_station.py` 启动简洁 GUI。

常用 Agent / 终端自动化命令：
  python tools/host_station.py ports
  python tools/host_station.py set-port COM5
  python tools/host_station.py cmd --port AUTO "status" "diag"
  python tools/host_station.py mcal --port AUTO --mode air --passes 5
  python tools/host_station.py mcal --port AUTO --mode ground --passes 5
  python tools/host_station.py push --port AUTO --mode GROUND
  python tools/host_station.py vtest --port AUTO --speeds 80,120,160,220,300 --ms 1000
  python tools/host_station.py capture --port AUTO --seconds 20 --cmd start
  python tools/host_station.py analyze --file data/track_telemetry_xxx.csv
"""

import argparse
import csv
import datetime
import json
import math
import os
import queue
import sys
import threading
import time

import serial
import serial.tools.list_ports

# 15 档电机标定占空比：0.06~0.24 以 3% 细步长测起转死区，0.30~0.95 覆盖中高占空比全量程
MCAL_DUTIES = [
    0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24,
    0.30, 0.38, 0.46, 0.55, 0.65, 0.75, 0.85, 0.95
]

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(WORKSPACE_DIR, "data")
PORT_CONFIG_PATH = os.path.join(DATA_DIR, "port_config.json")
LATEST_CAL_PATH = os.path.join(DATA_DIR, "latest_calibration.json")


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def ensure_data_dir():
    os.makedirs(DATA_DIR, exist_ok=True)


# =============================================================================
# 1. 核心算法模块：电机 15 点拟合、分段插值前馈与基于实测数据的硬件 PID 分析
# =============================================================================

def fit_motor_curve(duties: list[float], speeds_l: list[float], speeds_r: list[float]) -> dict:
    """根据 5 轮平均后的 15 档 (占空比 -> 实测轮速) 数据，拟合左右轮独立死区与满速斜率。"""
    result = {}
    for side, speeds in [("L", speeds_l), ("R", speeds_r)]:
        pts = [(u, v) for u, v in zip(duties, speeds) if v > 25.0]
        first_idx = next((i for i, v in enumerate(speeds) if v > 15.0), -1)
        if len(pts) < 3 or first_idx < 0:
            result[f"db_{side}"] = 0.14
            result[f"fs_{side}"] = 450.0
            result[f"slope_{side}"] = 520.0
            continue
        n = float(len(pts))
        sum_u = sum(p[0] for p in pts)
        sum_v = sum(p[1] for p in pts)
        sum_uu = sum(p[0] * p[0] for p in pts)
        sum_uv = sum(p[0] * p[1] for p in pts)
        denom = n * sum_uu - sum_u * sum_u
        slope = (n * sum_uv - sum_u * sum_v) / denom if denom > 1e-6 else 500.0
        slope = max(100.0, slope)
        intercept = (sum_v - slope * sum_u) / n
        reg_db = clamp(-intercept / slope, 0.02, 0.35)
        break_db = (
            0.5 * (duties[first_idx - 1] + duties[first_idx])
            if first_idx > 0 else duties[0]
        )
        db = clamp(0.65 * reg_db + 0.35 * break_db, 0.02, 0.35)
        fs = clamp(slope * (1.0 - db), 150.0, 1500.0)
        result[f"db_{side}"] = round(db, 4)
        result[f"fs_{side}"] = round(fs, 1)
        result[f"slope_{side}"] = round(slope, 1)
    return result


def interpolate_duty_for_speed(
    target_mm_s: float,
    duties: list[float],
    curve_speeds: list[float],
    db: float,
    fs: float,
) -> float:
    """利用 15 点实测分段线性插值 + 死区斜率模型，换算目标轮速所需的精确前馈占空比。"""
    abs_t = abs(target_mm_s)
    if abs_t < 1.0:
        return 0.0
    u_mag = db + abs_t / fs * (1.0 - db)
    valid_pairs = [(u, v) for u, v in zip(duties, curve_speeds) if v > 8.0]
    if len(valid_pairs) >= 3:
        u_first, v_first = valid_pairs[0]
        if abs_t <= v_first:
            u0 = min(db, u_first * 0.85)
            u_mag = u0 + (abs_t / v_first) * (u_first - u0)
        else:
            found = False
            for i in range(len(valid_pairs) - 1):
                u0, v0 = valid_pairs[i]
                u1, v1 = valid_pairs[i + 1]
                if v1 > v0 + 1.0 and v0 <= abs_t <= v1:
                    u_mag = u0 + (abs_t - v0) / (v1 - v0) * (u1 - u0)
                    found = True
                    break
            if not found:
                u_last, v_last = valid_pairs[-1]
                if abs_t > v_last and fs > v_last + 10.0:
                    u_mag = u_last + (abs_t - v_last) / (fs - v_last) * (1.0 - u_last)
    u_mag = clamp(u_mag, 0.0, 0.95)
    return u_mag if target_mm_s >= 0 else -u_mag


def evaluate_analog_pid_from_data(
    csv_rows: list[dict],
    motor_model: dict,
    cruise_mm_s: float = 140.0,
    line_kp: float = 115.0,
    line_kd: float = 1.8,
) -> dict:
    """
    严格基于实测数据与当前 PCB 网表（R39/R40=150k, R41/R42=300k, R43..R46=100k, RV_V, JP_V, R60, RV_P, RV_D）
    评估纯硬件模拟 PID 性能：
      - 若实测样本 < 20 帧：明确标注尚未采集实地红外数据，仅给出当前电路工作点核算，不做无依据的改电阻断言；
      - 若实测样本 >= 20 帧：利用真实采到的四路电压 V0..V3 检验当前 2:1 加权比是否实际发生边缘回落或运放削顶，
        并定量对比候选阻值在真实波形下的相关度、单调性与线性动态范围。
    """
    db_l = motor_model.get("db_L", 0.14)
    db_r = motor_model.get("db_R", 0.14)
    fs_l = motor_model.get("fs_L", 450.0)
    fs_r = motor_model.get("fs_R", 450.0)
    db_avg = 0.5 * (db_l + db_r)
    fs_avg = 0.5 * (fs_l + fs_r)

    target_d0 = clamp(db_avg + (cruise_mm_s / fs_avg) * (1.0 - db_avg), 0.22, 0.60)
    ana_speed_0 = 0.3507 + 2.50 * target_d0
    v_base_target = clamp(0.5 * (ana_speed_0 / 0.625 + 2.50), 1.60, 2.85)
    jp_v_note = (
        "JP_V 须插 2-3 (接 R68=6.2kΩ 到地)，否则 V_BASE 最低被锁死在 2.50V (D0>=48.5%) 无法降速"
        if v_base_target < 2.50 else "JP_V 插 2-3 (全范围 1.40~3.60V 可调，推荐)"
    )

    vl_at_d0 = (target_d0 - db_l) / (1.0 - db_l) * fs_l if target_d0 > db_l else 0.0
    vr_at_d0 = (target_d0 - db_r) / (1.0 - db_r) * fs_r if target_d0 > db_r else 0.0

    valid_rows = [r for r in csv_rows if r.get("confidence", 0.0) >= 0.25]
    if len(valid_rows) < 20:
        return {
            "empirical_ready": False,
            "valid_frames": len(valid_rows),
            "note": "尚未采集到足够实车跑圈/横扫红外数据 (<20 帧)，暂不对四路加权电阻 R39/R40 下修改结论，请先实测录波！",
            "target_d0_pct": round(target_d0 * 100.0, 1),
            "v_base_target_v": round(v_base_target, 3),
            "jp_v_note": jp_v_note,
            "vl_at_d0_mm_s": round(vl_at_d0, 1),
            "vr_at_d0_mm_s": round(vr_at_d0, 1),
            "speed_mismatch_mm_s": round(vl_at_d0 - vr_at_d0, 1),
            "duty_mismatch_pct": round((db_l - db_r) * 100.0, 2),
        }

    # 从实车数据提取真实电压统计
    v_min = [3.3] * 4
    v_max = [0.0] * 4
    samples = []
    for r in valid_rows:
        vs = [r[f"raw{i}"] * 3.300 / 4095.0 for i in range(4)]
        for i in range(4):
            v_min[i] = min(v_min[i], vs[i])
            v_max[i] = max(v_max[i], vs[i])
        samples.append((vs, r["pos"], r.get("q0", 0.0), r.get("q1", 0.0), r.get("q2", 0.0), r.get("q3", 0.0)))

    spans_v = [round(max(0.05, v_max[i] - v_min[i]), 3) for i in range(4)]
    avg_span_v = sum(spans_v) / 4.0

    # 检验当前 2:1 电路 (R39/R40=150k, R41/R42=300k, R_f=50k) 在实车数据下的真实表现
    pair_e, outer_e = [], []
    for vs, pos, q0, q1, q2, q3 in samples:
        e_2to1 = abs((vs[0] - vs[3]) / 3.0 + (vs[1] - vs[2]) / 6.0)
        if (q2 > 0.45 and q3 > 0.45 and q0 < 0.15 and q1 < 0.15) or (q0 > 0.45 and q1 > 0.45 and q2 < 0.15 and q3 < 0.15):
            pair_e.append(e_2to1)
        elif (q3 > 0.35 and q2 < 0.15 and q1 < 0.10 and q0 < 0.10) or (q0 > 0.35 and q1 < 0.15 and q2 < 0.10 and q3 < 0.10):
            outer_e.append(e_2to1)

    pair_avg_e = sum(pair_e) / len(pair_e) if pair_e else None
    outer_avg_e = sum(outer_e) / len(outer_e) if outer_e else None
    edge_drop_observed = (
        pair_avg_e is not None and outer_avg_e is not None and outer_avg_e < pair_avg_e * 0.92
    )

    # 对比不同标准阻值 R39/R40 下的真实数据线性度 (与实车软件位置相关性) 与最高输出是否削顶
    candidates = [150, 120, 100, 82, 68]
    cand_eval = []
    ys = [s[1] for s in samples]
    my = sum(ys) / len(ys)
    var_y = sum((y - my) ** 2 for y in ys)
    for r_out in candidates:
        w_out = 50.0 / r_out
        w_in = 50.0 / 300.0
        es = [w_out * (s[0][0] - s[0][3]) + w_in * (s[0][1] - s[0][2]) for s in samples]
        mx = sum(es) / len(es)
        num = sum((x - mx) * (y - my) for x, y in zip(es, ys))
        var_x = sum((x - mx) ** 2 for x in es)
        corr = num / math.sqrt(var_x * var_y) if (var_x > 1e-9 and var_y > 1e-9) else 0.0
        max_abs_e = max(abs(x) for x in es)
        # 检验 U6A (供电 5V, 虚地 2.5V) 在当前增益下是否接近 0.1V / 4.9V 电源轨
        rail_margin = 2.40 - max_abs_e
        cand_eval.append({
            "R39_R40_kohm": r_out,
            "weight_ratio": round(300.0 / r_out, 2),
            "corr_with_sw_pos": round(corr, 4),
            "max_abs_e_V": round(max_abs_e, 3),
            "u6a_rail_margin_V": round(rail_margin, 3),
        })

    # 仅在实车数据确实观测到边缘回落时才建议改小 R39/R40，否则优先保留现板并调 Kp/V_BASE
    feasible = [c for c in cand_eval if c["u6a_rail_margin_V"] > 0.25]
    best_cand = max(feasible or cand_eval, key=lambda c: c["corr_with_sw_pos"])
    if edge_drop_observed and best_cand["R39_R40_kohm"] < 150:
        weight_advice = (
            f"实测确认边缘回落（双路均值 |e|={pair_avg_e:.3f}V -> 单外侧降至 {outer_avg_e:.3f}V）；"
            f"实测拟合最佳方案为保持 R41/R42=300kΩ，将外侧 R39/R40 由 150kΩ 改用 {best_cand['R39_R40_kohm']}kΩ "
            f"(权重 {best_cand['weight_ratio']}:1，相关性 {best_cand['corr_with_sw_pos']:.4f})"
        )
        chosen_w_out = 50.0 / best_cand["R39_R40_kohm"]
    else:
        weight_advice = (
            f"当前 2:1 (R39/R40=150kΩ, R41/R42=300kΩ) 在本次实测中相关性 {cand_eval[0]['corr_with_sw_pos']:.4f}，"
            f"{'未见明显边缘回落' if not edge_drop_observed else '需先确认边缘单路样本覆盖'}；可先保留原电阻对比基准，若需加大外侧权重候选为 {best_cand['R39_R40_kohm']}kΩ"
        )
        chosen_w_out = 50.0 / 150.0

    e_outer_real = max(0.20, chosen_w_out * avg_span_v)
    desired_delta_d = max(0.20, min(target_d0, (line_kp / fs_avg) * (1.0 - db_avg)))
    kp_total = clamp(desired_delta_d / (0.5 * e_outer_real), 0.45, 8.0)
    r60_kohm = 47.0 if kp_total < 1.05 else 20.0
    kp_stage1 = max(1.0, kp_total * (r60_kohm / 20.0))
    rv_p_kohm = clamp(10.0 * (kp_stage1 - 1.0), 0.0, 100.0)
    td_eff = line_kd / max(20.0, line_kp)
    rf_d_kohm = clamp((td_eff * kp_total * 1e6) / 100.0, 4.7, 104.7)
    rv_d_kohm = clamp(rf_d_kohm - 4.7, 0.0, 100.0)

    return {
        "empirical_ready": True,
        "valid_frames": len(valid_rows),
        "ir_spans_v": spans_v,
        "avg_span_v": round(avg_span_v, 3),
        "edge_drop_observed": edge_drop_observed,
        "pair_avg_e_V": round(pair_avg_e, 3) if pair_avg_e is not None else None,
        "outer_avg_e_V": round(outer_avg_e, 3) if outer_avg_e is not None else None,
        "weight_advice": weight_advice,
        "candidate_resistors": cand_eval,
        "target_d0_pct": round(target_d0 * 100.0, 1),
        "v_base_target_v": round(v_base_target, 3),
        "jp_v_note": jp_v_note,
        "kp_total_recommended": round(kp_total, 2),
        "r60_recommended_kohm": int(r60_kohm),
        "rv_p_kohm": round(rv_p_kohm, 1),
        "rv_d_kohm": round(rv_d_kohm, 1),
        "vl_at_d0_mm_s": round(vl_at_d0, 1),
        "vr_at_d0_mm_s": round(vr_at_d0, 1),
        "speed_mismatch_mm_s": round(vl_at_d0 - vr_at_d0, 1),
        "duty_mismatch_pct": round((db_l - db_r) * 100.0, 2),
    }


def translate_hwemu_to_bom(
    w_out: float = 1.0 / 3.0,
    w_in: float = 1.0 / 6.0,
    kp_stage: float = 2.5,
    r60_kohm: float = 20.0,
    kd_stage_ms: float = 5.5,
    r61_kohm: float = 20.0,
    d_tau1_ms: float = 2.2,
    v_base: float = 2.50,
    i_enable: bool = False,
    coast_mode: bool = True,
    *,
    netlist_mode: bool = False,
    r39_kohm: float = 150.0,
    r41_kohm: float = 300.0,
    r40_kohm: float = 150.0,
    r42_kohm: float = 300.0,
    r43_kohm: float = 100.0,
    r45_kohm: float = 100.0,
    r45_to_5v: bool = False,
    v_5v: float = 5.000,
    v_mid: float = 2.500,
    tri_min_v: float = 0.351,
    tri_max_v: float = 2.851,
    tri_freq_hz: float = 20000.0,
    ana_atten: float = 0.625,
    c50_nf: float = 100.0,
) -> dict:
    """将软件 1:1 硬件仿真模式下调优好的参数翻译为物理扩展板元器件实值与改造建议。"""
    r39_r40 = round(50.0 / max(1e-3, w_out), 1)
    r41_r42 = round(50.0 / max(1e-3, w_in), 1)
    kp_tot = (20.0 / r60_kohm) * kp_stage
    rv_p = (kp_stage - 1.0) * 10.0
    rv_d_stock = kd_stage_ms / 0.1 - 4.7
    rv_d_c50 = kd_stage_ms / max(0.01, c50_nf * 0.001) - 4.7
    r54_at_c50 = d_tau1_ms / max(0.01, c50_nf * 0.001)
    raw0 = v_mid + 2.0 * (v_base - v_mid)
    d0 = clamp((ana_atten * raw0 - tri_min_v) / max(0.1, tri_max_v - tri_min_v), 0.0, 1.0)
    v_bot = v_5v * (6.2 / 22.4)
    v_top = v_5v * (16.2 / 22.4)
    rv_v_23 = clamp((v_base - v_bot) / max(0.1, v_top - v_bot), 0.0, 1.0) * 100.0

    solder_mods = []
    bodge_mods = []
    if netlist_mode:
        if r45_to_5v:
            bodge_mods.append(
                f"U6A 按实焊阻值组对齐：R39={r39_kohm}k, R41={r41_kohm}k, R40={r40_kohm}k, "
                f"R42={r42_kohm}k, R43={r43_kohm}k, R45={r45_kohm}k (PWM_MID 端立焊翘起飞线至 PWM_5V)"
            )
    elif abs(r39_r40 - 150.0) > 12.0 or abs(r41_r42 - 300.0) > 25.0:
        solder_mods.append(f"更换加权电阻：R39/R40 → {r39_r40}kΩ（原150k），R41/R42 → {r41_r42}kΩ（原300k），保持 R43||R45=R44||R46=50kΩ")
    if kp_tot < 1.0:
        solder_mods.append(f"总比例 <1.0：更换 R60 → {max(33.0, round(20.0 / kp_tot * kp_stage, 1))}kΩ（推荐 47kΩ 使 Kp 涵盖 0.43~4.68）")
    if rv_d_stock > 100.0:
        c50_uf = 1.0 if rv_d_stock > 470.0 else (c50_nf / 1000.0 if c50_nf >= 220.0 else 0.47)
        r54_new = round(d_tau1_ms / c50_uf, 2)
        solder_mods.append(
            f"增强硬件 D 阻尼（原 C50=100nF 最大仅 10.47ms）：将 C50 换焊为 {c50_uf}uF、R54 换焊为 {r54_new}kΩ（RV_D≈{rv_d_c50:.1f}kΩ），"
            f"同时可按需调整 R61 (当前 {r61_kohm:.1f}kΩ)"
        )
    elif abs(r61_kohm - 20.0) > 2.0:
        solder_mods.append(f"调整 D 权重电阻：R61 → {r61_kohm:.1f}kΩ（原 20kΩ）")
    if not coast_mode:
        bodge_mods.append("需改 U4 驱动/制动（Slow Decay）或割线接入反相 PWM 到 IN2")

    return {
        "netlist_mode": netlist_mode,
        "R39_R40_kohm": [r39_kohm, r40_kohm] if netlist_mode else r39_r40,
        "R41_R42_kohm": [r41_kohm, r42_kohm] if netlist_mode else r41_r42,
        "R45_kohm": r45_kohm if netlist_mode else 100.0,
        "R45_to_5V": r45_to_5v,
        "kp_total": round(kp_tot, 2),
        "R60_kohm": round(r60_kohm, 1),
        "RV_P_kohm": round(clamp(rv_p, 0.0, 100.0), 1),
        "RV_P_note": "在 0~100kΩ 范围内可直调" if 0.0 <= rv_p <= 100.0 else "超出 RV_P 范围，需调整 R60",
        "kd_eff_at_R61_ms": round(kd_stage_ms * (20.0 / r61_kohm), 2),
        "R61_kohm": round(r61_kohm, 1),
        "C50_nf": round(c50_nf, 1),
        "R54_kohm": round(r54_at_c50, 2),
        "RV_D_at_C50_kohm": round(rv_d_c50, 1),
        "RV_D_stock_100n_kohm": round(rv_d_stock, 1),
        "D_feasible_on_stock_100n": rv_d_stock <= 100.0,
        "D_feasible_on_C50": 0.0 <= rv_d_c50 <= 100.0,
        "V_5V_V": round(v_5v, 3),
        "V_MID_V": round(v_mid, 3),
        "TRI_min_V": round(tri_min_v, 3),
        "TRI_max_V": round(tri_max_v, 3),
        "TRI_freq_Hz": round(tri_freq_hz, 0),
        "V_BASE_V": round(v_base, 3),
        "D0_pct": round(d0 * 100.0, 1),
        "JP_V": "2-3 (接 R68=6.2kΩ 到地，解锁 0~100% 调速)" if v_base < v_mid - 0.01 else "2-3 推荐 (或 1-2)",
        "RV_V_23_ratio_pct": round(rv_v_23, 1),
        "JP_I": "短接插上" if i_enable else "拔掉禁用 (仅 PD)",
        "DRV8833_mode": "原板滑行 (U4 IN2=GND)" if coast_mode else "需慢衰减制动 (Slow Decay)",
        "solder_only_mods": solder_mods or ["无需再换阻容，当前物理器件范围已可覆盖"],
        "minimal_bodge_mods": bodge_mods or ["无需额外飞线"],
    }


# =============================================================================
# 2. 通用数据读写与 Agent 自动化串口通信链路
# =============================================================================

def list_serial_ports() -> list[dict]:
    ports = []
    for p in serial.tools.list_ports.comports():
        desc = p.description or ""
        hwid = p.hwid or ""
        score = 0
        if "蓝牙" in desc or "Bluetooth" in desc or "BTHENUM" in hwid.upper():
            score += 5
            if "BTHENUM" in hwid.upper() and "000000000000" not in hwid.upper():
                score += 2  # Windows 传出蓝牙串口带有远端设备 MAC，优先于全零本地传入端口
        if any(k in desc.upper() for k in ("CH340", "CP210", "FT232", "USB-SERIAL", "SERIAL")):
            score += 3
        ports.append({"device": p.device, "description": desc, "hwid": hwid, "priority": score})
    ports.sort(key=lambda x: (-x["priority"], x["device"]))
    return ports


def resolve_port(port_arg: str) -> str:
    if port_arg and port_arg.upper() != "AUTO":
        return port_arg
    if os.path.exists(PORT_CONFIG_PATH):
        try:
            with open(PORT_CONFIG_PATH, "r", encoding="utf-8") as f:
                saved = json.load(f).get("port")
            available = {p["device"] for p in list_serial_ports()}
            if saved in available:
                return saved
        except Exception:
            pass
    ports = list_serial_ports()
    if not ports:
        raise RuntimeError("未检测到任何可用串口，请确认蓝牙已配对或 USB-TTL 已插入。")
    return ports[0]["device"]


def parse_csv_line(line: str) -> dict | None:
    parts = line.split(",")
    if parts[0] != "CSV" or len(parts) < 20:
        return None
    try:
        return {
            "ms": int(parts[1]), "state": int(parts[2]), "fault": int(parts[3]),
            "raw0": int(parts[4]), "raw1": int(parts[5]), "raw2": int(parts[6]), "raw3": int(parts[7]),
            "q0": float(parts[8]), "q1": float(parts[9]), "q2": float(parts[10]), "q3": float(parts[11]),
            "pos": float(parts[12]), "confidence": float(parts[13]),
            "vL": float(parts[14]), "vR": float(parts[15]),
            "tL": float(parts[16]), "tR": float(parts[17]),
            "uL": float(parts[18]), "uR": float(parts[19]),
        }
    except ValueError:
        return None


def load_latest_calibration() -> dict:
    if os.path.exists(LATEST_CAL_PATH):
        with open(LATEST_CAL_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    return {
        "duties": MCAL_DUTIES,
        "mcal_data": {
            "AIR": {"passes": {}, "curve_L": [0.0] * len(MCAL_DUTIES), "curve_R": [0.0] * len(MCAL_DUTIES), "fit": {}},
            "GROUND": {"passes": {}, "curve_L": [0.0] * len(MCAL_DUTIES), "curve_R": [0.0] * len(MCAL_DUTIES), "fit": {}},
        },
        "ir_cal": {"black": [500, 500, 500, 500], "white": [3000, 3000, 3000, 3000]},
    }


def save_calibration_files(mcal_data: dict, ir_cal: dict, csv_rows: list[dict], mode_tag: str = "") -> dict:
    ensure_data_dir()
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = os.path.join(DATA_DIR, f"calibration_session_{ts}.json")
    payload = {
        "timestamp": ts,
        "duties": MCAL_DUTIES,
        "mcal_data": mcal_data,
        "ir_cal": ir_cal,
        "csv_count": len(csv_rows),
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    with open(LATEST_CAL_PATH, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

    paths = {"session_json": json_path, "latest_json": LATEST_CAL_PATH}
    if mode_tag and mode_tag in mcal_data:
        m = mcal_data[mode_tag]
        mcal_csv = os.path.join(DATA_DIR, f"mcal_{mode_tag.lower()}_{ts}.csv")
        pass_keys = sorted(m["passes"].keys(), key=lambda x: int(x))
        with open(mcal_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            header = ["step", "duty", "avg_vL", "avg_vR"]
            for pk in pass_keys:
                header.extend([f"p{pk}_vL", f"p{pk}_vR"])
            w.writerow(header)
            for s, duty in enumerate(MCAL_DUTIES):
                row = [s, duty, round(m["curve_L"][s], 2), round(m["curve_R"][s], 2)]
                for pk in pass_keys:
                    st = m["passes"][pk].get(str(s)) or m["passes"][pk].get(s)
                    if st:
                        row.extend([round(st[1], 2), round(st[2], 2)])
                    else:
                        row.extend(["", ""])
                w.writerow(row)
        paths["mcal_csv"] = mcal_csv

    if csv_rows:
        csv_path = os.path.join(DATA_DIR, f"track_telemetry_{ts}.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
            writer.writeheader()
            writer.writerows(csv_rows)
        paths["track_csv"] = csv_path
    return paths


class SerialClient:
    """Windows/Bluetooth 串口闭环会话封装（支持超时写入保护与 Ctrl+C 安全急停）。"""

    def __init__(self, port: str, baud: int = 115200):
        self.port = resolve_port(port)
        self.ser = serial.Serial(self.port, baud, timeout=0.05, write_timeout=1.5)
        self._buf = b""
        time.sleep(0.15)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd: str):
        self.ser.write((cmd.strip() + "\r\n").encode("utf-8"))
        self.ser.flush()

    def safe_emergency_stop(self):
        """在脚本中断或异常退出前尽量下发 stop，避免小车继续带电运行。"""
        try:
            if self.ser and self.ser.is_open:
                self.send("stop")
                time.sleep(0.05)
        except Exception:
            pass

    def read_lines(self, duration_s: float, stop_predicate=None) -> list[str]:
        lines = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            chunk = self.ser.read(512)
            if chunk:
                self._buf += chunk
                while b"\n" in self._buf:
                    raw_line, self._buf = self._buf.split(b"\n", 1)
                    text = raw_line.decode("utf-8", errors="replace").strip()
                    if text:
                        lines.append(text)
                        if stop_predicate and stop_predicate(text):
                            return lines
            else:
                time.sleep(0.01)
        return lines


def push_model_to_mcu(client: SerialClient, m: dict):
    """将指定模式的死区/斜率及 15 点实测曲线逐行安全写入单片机 RAM（不写 Flash）。"""
    fit = m.get("fit") or fit_motor_curve(MCAL_DUTIES, m["curve_L"], m["curve_R"])
    client.send("stop")
    time.sleep(0.05)
    for c in [
        f"set db_l {fit['db_L']:.4f}",
        f"set db_r {fit['db_R']:.4f}",
        f"set fs_l {fit['fs_L']:.1f}",
        f"set fs_r {fit['fs_R']:.1f}",
    ]:
        client.send(c)
        time.sleep(0.02)
    for s_idx in range(len(MCAL_DUTIES)):
        vl, vr = m["curve_L"][s_idx], m["curve_R"][s_idx]
        if vl > 0 or vr > 0:
            client.send(f"mcurve {s_idx} {vl:.1f} {vr:.1f}")
            time.sleep(0.02)
    return client.read_lines(0.25)


# =============================================================================
# 3. Agent 自动化 CLI 子命令实现
# =============================================================================

def cli_ports(_args) -> int:
    ports = list_serial_ports()
    saved = None
    if os.path.exists(PORT_CONFIG_PATH):
        try:
            saved = json.load(open(PORT_CONFIG_PATH, "r", encoding="utf-8")).get("port")
        except Exception:
            pass
    print(json.dumps({"saved_default_port": saved, "ports": ports}, ensure_ascii=False, indent=2))
    return 0


def cli_set_port(args) -> int:
    ensure_data_dir()
    with open(PORT_CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump({"port": args.port}, f, indent=2)
    print(json.dumps({"ok": True, "default_port": args.port, "config_file": PORT_CONFIG_PATH}, ensure_ascii=False))
    return 0


def cli_cmd(args) -> int:
    c = SerialClient(args.port)
    try:
        for cmd_str in args.commands:
            c.send(cmd_str)
            time.sleep(0.03)
        lines = c.read_lines(args.wait)
        csv_rows = [r for l in lines if (r := parse_csv_line(l)) is not None]
        non_csv = [l for l in lines if not l.startswith("CSV,")]
        print(json.dumps({"port": c.port, "commands": args.commands, "messages": non_csv, "csv_count": len(csv_rows), "last_csv": csv_rows[-1] if csv_rows else None}, ensure_ascii=False, indent=2))
        return 0
    finally:
        c.close()


def cli_mcal(args) -> int:
    mode_str = "GROUND" if args.mode.lower() in ("ground", "gnd") else "AIR"
    c = SerialClient(args.port)
    pkg = load_latest_calibration()
    m = pkg["mcal_data"].setdefault(mode_str, {"passes": {}, "curve_L": [0.0] * len(MCAL_DUTIES), "curve_R": [0.0] * len(MCAL_DUTIES), "fit": {}})
    m["passes"] = {}
    m["curve_L"] = [0.0] * len(MCAL_DUTIES)
    m["curve_R"] = [0.0] * len(MCAL_DUTIES)
    try:
        c.send("stop")
        time.sleep(0.08)
        c.send("log 0")
        time.sleep(0.05)
        c.read_lines(0.1)
        c.send(f"mcal {mode_str.lower()} {args.passes}")
        timeout_s = max(45.0, args.passes * (26.0 if mode_str == "GROUND" else 11.0))
        lines = c.read_lines(
            timeout_s,
            stop_predicate=lambda l: l.startswith("MCAL_DONE,") or ("FAULT" in l and "EVENT" in l),
        )
        c.send("log 1")
        for l in lines:
            parts = l.split(",")
            if parts[0] == "MCAL_STEP" and len(parts) >= 8:
                p_idx = str(int(parts[2]))
                s_idx = int(parts[4])
                m["passes"].setdefault(p_idx, {})[str(s_idx)] = [float(parts[5]), float(parts[6]), float(parts[7])]
            elif parts[0] == "MCAL_CURVE" and len(parts) >= 5:
                s_idx = int(parts[1])
                if 0 <= s_idx < len(MCAL_DUTIES):
                    m["curve_L"][s_idx] = float(parts[3])
                    m["curve_R"][s_idx] = float(parts[4])
            elif parts[0] == "MCAL_DONE" and len(parts) >= 6:
                m["fit"] = {"db_L": float(parts[2]), "db_R": float(parts[3]), "fs_L": float(parts[4]), "fs_R": float(parts[5])}
        if not m["fit"]:
            m["fit"] = fit_motor_curve(MCAL_DUTIES, m["curve_L"], m["curve_R"])
        paths = save_calibration_files(pkg["mcal_data"], pkg.get("ir_cal", {}), [], mode_tag=mode_str)
        active_model = "GROUND" if pkg["mcal_data"].get("GROUND", {}).get("passes") else mode_str
        print(json.dumps({
            "port": c.port,
            "mode": mode_str,
            "active_tracking_model": active_model,
            "note": "地面带载 (GROUND) 优先作为实车循迹主前馈模型，空载 (AIR) 仅作机械阻尼体检参考",
            "passes_requested": args.passes,
            "passes_recorded": len(m["passes"]),
            "duties": MCAL_DUTIES,
            "curve_L": m["curve_L"],
            "curve_R": m["curve_R"],
            "fit": m["fit"],
            "saved_files": paths,
            "raw_messages": [l for l in lines if not l.startswith("CSV,")],
        }, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        c.safe_emergency_stop()
        raise
    finally:
        c.close()


def cli_push(args) -> int:
    pkg = load_latest_calibration()
    mode_key = args.mode.upper()
    m = pkg["mcal_data"].get(mode_key)
    if not m:
        raise RuntimeError(f"latest_calibration.json 中没有 {mode_key} 数据")
    c = SerialClient(args.port)
    try:
        replies = push_model_to_mcu(c, m)
        print(json.dumps({"port": c.port, "mode": mode_key, "fit": m.get("fit"), "replies": replies}, ensure_ascii=False, indent=2))
        return 0
    finally:
        c.close()


def cli_vtest(args) -> int:
    pkg = load_latest_calibration()
    mode_key = args.mode.upper()
    m = pkg["mcal_data"].get(mode_key)
    speeds = [float(x) for x in args.speeds.split(",") if x.strip()]
    c = SerialClient(args.port)
    results = []
    try:
        if m and any(v > 0 for v in m.get("curve_L", [])):
            push_model_to_mcu(c, m)
        c.send("log 0")
        time.sleep(0.05)
        for spd in speeds:
            c.send(f"vtest {spd:.1f} {spd:.1f} {args.ms}")
            lines = c.read_lines(args.ms / 1000.0 + 1.5, stop_predicate=lambda l: l.startswith("VTEST_DONE,"))
            done_line = next((l for l in lines if l.startswith("VTEST_DONE,")), None)
            if done_line:
                kv = dict(item.split("=", 1) for item in done_line.split(",")[1:] if "=" in item)
                results.append({k: float(v) for k, v in kv.items()})
            time.sleep(0.20)
        c.send("log 1")
        ensure_data_dir()
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(DATA_DIR, f"vtest_{mode_key.lower()}_{ts}.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump({"mode": mode_key, "results": results}, f, indent=2, ensure_ascii=False)
        print(json.dumps({"port": c.port, "mode": mode_key, "saved_file": out_path, "results": results}, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        c.safe_emergency_stop()
        raise
    finally:
        c.close()


class OffTrackWatchdog:
    """实时监控遥测帧，在软件/硬仿测试期间连续脱轨过远或冲出赛道板时触发主动急停。"""

    def __init__(self, max_offtrack_mm: float = 140.0, white_timeout_s: float = 0.70, dark_timeout_s: float = 0.45):
        self.max_offtrack_mm = max(0.0, float(max_offtrack_mm))
        self.white_timeout_s = white_timeout_s
        self.dark_timeout_s = dark_timeout_s
        self.seen_active = False
        self.prev_ms: int | None = None
        self.off_white_mm = 0.0
        self.off_white_s = 0.0
        self.all_dark_mm = 0.0
        self.all_dark_s = 0.0
        self.reason: str | None = None

    def feed_line(self, line: str) -> bool:
        r = parse_csv_line(line)
        if r is None:
            return False
        st = r["state"]
        if st in (3, 4, 5):
            self.seen_active = True
        elif self.seen_active and st in (0, 6, 7, 8):
            self.reason = f"mcu_stopped(state={st},fault={r['fault']})"
            return True

        ms = r["ms"]
        dt = clamp((ms - self.prev_ms) * 0.001, 0.0, 0.20) if (self.prev_ms is not None and ms >= self.prev_ms) else 0.025
        self.prev_ms = ms

        if self.max_offtrack_mm <= 0.0 or st not in (3, 4, 5):
            return False

        v_abs = 0.5 * (abs(r["vL"]) + abs(r["vR"]))
        if r["confidence"] < 0.20:
            self.off_white_s += dt
            self.off_white_mm += v_abs * dt
            if self.off_white_mm >= self.max_offtrack_mm or self.off_white_s >= self.white_timeout_s:
                self.reason = f"auto_stop_off_track_white(mm={self.off_white_mm:.1f},s={self.off_white_s:.2f})"
                return True
        else:
            self.off_white_s = 0.0
            self.off_white_mm = 0.0

        if r["q0"] >= 0.85 and r["q1"] >= 0.85 and r["q2"] >= 0.85 and r["q3"] >= 0.85:
            self.all_dark_s += dt
            self.all_dark_mm += v_abs * dt
            if self.all_dark_mm >= self.max_offtrack_mm or self.all_dark_s >= self.dark_timeout_s:
                self.reason = f"auto_stop_off_board_dark(mm={self.all_dark_mm:.1f},s={self.all_dark_s:.2f})"
                return True
        else:
            self.all_dark_s = 0.0
            self.all_dark_mm = 0.0

        return False


def _capture_with_watchdog(
    client: SerialClient,
    seconds: float,
    stop_after: bool,
    max_offtrack_mm: float = 140.0,
) -> tuple[list[str], str | None]:
    """按给定超时录制实时遥测，并在脱轨过远或下位机自停时主动发出 stop 并提前结束录制。"""
    wd = OffTrackWatchdog(max_offtrack_mm=max_offtrack_mm)
    lines = client.read_lines(seconds, stop_predicate=wd.feed_line)
    if (wd.reason and wd.reason.startswith("auto_stop_")) or stop_after:
        client.send("stop")
        lines.extend(client.read_lines(0.30))
    return lines, wd.reason


def cli_capture(args) -> int:
    ensure_data_dir()
    c = SerialClient(args.port)
    try:
        c.send("log 1")
        time.sleep(0.05)
        if args.cmd and args.cmd.lower() != "none":
            c.send(args.cmd)
        lines, stop_reason = _capture_with_watchdog(
            c, args.seconds, args.stop_after, getattr(args, "max_offtrack_mm", 140.0)
        )
        rows = [r for l in lines if (r := parse_csv_line(l)) is not None]
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(DATA_DIR, f"track_telemetry_{ts}.csv")
        if rows:
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
        pkg = load_latest_calibration()
        fit = pkg["mcal_data"].get("GROUND", {}).get("fit") or pkg["mcal_data"].get("AIR", {}).get("fit") or {}
        analysis = evaluate_analog_pid_from_data(rows, fit)
        print(json.dumps({
            "port": c.port,
            "captured_csv_frames": len(rows),
            "saved_csv": csv_path if rows else None,
            "auto_stop_reason": stop_reason,
            "events": [l for l in lines if not l.startswith("CSV,")],
            "hardware_pid_analysis": analysis,
        }, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        c.safe_emergency_stop()
        raise
    finally:
        c.close()


def cli_analyze(args) -> int:
    rows = []
    with open(args.file, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({k: (int(v) if k in ("ms", "state", "fault", "raw0", "raw1", "raw2", "raw3") else float(v)) for k, v in r.items()})
    pkg = json.load(open(args.motor, "r", encoding="utf-8")) if args.motor and os.path.exists(args.motor) else load_latest_calibration()
    fit = pkg["mcal_data"].get("GROUND", {}).get("fit") or pkg["mcal_data"].get("AIR", {}).get("fit") or {}
    analysis = evaluate_analog_pid_from_data(rows, fit)
    print(json.dumps({"telemetry_file": args.file, "motor_fit": fit, "analysis": analysis}, ensure_ascii=False, indent=2))
    return 0


def cli_hwemu(args) -> int:
    """下发 1:1 硬件模拟 PID 参数（支持实测三角波/电源/U6A网表）并在整圈跑车期间录数，生成物理 BOM/电位器映射。"""
    ensure_data_dir()
    # 支持从实测电位器 RV_P / RV_D / C50 自动推算等效 kp / kd / tau1 / tau2
    kp_stage = args.kp if args.rv_p is None else (1.0 + args.rv_p / 10.0)
    c50_nf = args.c50_nf
    r54_k = args.r54
    if args.rv_d is not None:
        kd_ms = (4.7 + args.rv_d) * (c50_nf * 0.001)
    else:
        kd_ms = args.kd_ms
    tau1_ms = (r54_k * c50_nf * 0.001) if args.tau1_ms is None else args.tau1_ms
    tau2_ms = (kd_ms / max(0.01, c50_nf * 0.001)) * (args.c60_nf * 0.001) if args.tau2_ms is None else args.tau2_ms

    use_group_b = (args.preset == "group_b")
    netlist_mode = bool(args.netlist or use_group_b)
    r39 = args.r39 if args.r39 is not None else (47.0 if use_group_b else 150.0)
    r41 = args.r41 if args.r41 is not None else (110.0 if use_group_b else 300.0)
    r40 = args.r40 if args.r40 is not None else (100.0 if use_group_b else 150.0)
    r42 = args.r42 if args.r42 is not None else (120.0 if use_group_b else 300.0)
    r43 = args.r43 if args.r43 is not None else 100.0
    r45 = args.r45 if args.r45 is not None else (120.0 if use_group_b else 100.0)
    r45_5v = args.r45_5v if args.r45_5v is not None else (1 if use_group_b else 0)
    r_ir = args.r_ir if args.r_ir is not None else (1.0 if netlist_mode else 0.0)

    bom = translate_hwemu_to_bom(
        w_out=args.w_out, w_in=args.w_in, kp_stage=kp_stage, r60_kohm=args.r60,
        kd_stage_ms=kd_ms, r61_kohm=args.r61, d_tau1_ms=tau1_ms,
        v_base=args.v_base, i_enable=bool(args.i), coast_mode=bool(args.coast),
        netlist_mode=netlist_mode, r39_kohm=r39, r41_kohm=r41, r40_kohm=r40,
        r42_kohm=r42, r43_kohm=r43, r45_kohm=r45, r45_to_5v=bool(r45_5v),
        v_5v=args.v5v, v_mid=args.vmid, tri_min_v=args.tri_min,
        tri_max_v=args.tri_max, tri_freq_hz=args.tri_hz,
        ana_atten=args.atten, c50_nf=c50_nf,
    )
    c = SerialClient(args.port)
    try:
        c.send("stop")
        time.sleep(0.06)
        cmds = [
            f"hw_preset {args.preset}",
            f"set hw_emu {args.enable}",
            f"set hw_netlist {1 if netlist_mode else 0}",
            f"set hw_w_out {args.w_out:.4f}",
            f"set hw_w_in {args.w_in:.4f}",
            f"set hw_v5v {args.v5v:.3f}",
            f"set hw_vmid {args.vmid:.3f}",
            f"set hw_e_trim {args.e_trim:.4f}",
            f"set hw_r39 {r39:.1f}",
            f"set hw_r41 {r41:.1f}",
            f"set hw_r40 {r40:.1f}",
            f"set hw_r42 {r42:.1f}",
            f"set hw_r43 {r43:.1f}",
            f"set hw_r45 {r45:.1f}",
            f"set hw_r45_5v {r45_5v}",
            f"set hw_r_ir {r_ir:.2f}",
            f"set hw_kp {kp_stage:.3f}",
            f"set hw_r60 {args.r60:.1f}",
            f"set hw_kd_ms {kd_ms:.2f}",
            f"set hw_r61 {args.r61:.1f}",
            f"set hw_tau1_ms {tau1_ms:.2f}",
            f"set hw_tau2_ms {clamp(tau2_ms, 0.01, 50.0):.2f}",
            f"set hw_v_base {args.v_base:.3f}",
            f"set hw_atten {args.atten:.4f}",
            f"set hw_spd_tau_ms {args.spd_tau_ms:.4f}",
            f"set hw_tri_min {args.tri_min:.3f}",
            f"set hw_tri_max {args.tri_max:.3f}",
            f"set hw_tri_hz {args.tri_hz:.0f}",
            f"set hw_i {args.i}",
            f"set hw_coast {args.coast}",
        ]
        if args.adc_vref is not None:
            cmds.append(f"set hw_adc_vref {args.adc_vref:.3f}")
        cmds.extend(["hwmap", "diag"])
        for s in cmds:
            c.send(s)
            time.sleep(0.02)
        hwmap_msgs = c.read_lines(0.55)

        rows = []
        csv_path = None
        stop_reason = None
        if args.run:
            c.send("log 1")
            time.sleep(0.05)
            c.send("start")
            lines, stop_reason = _capture_with_watchdog(
                c, args.seconds, args.stop_after, getattr(args, "max_offtrack_mm", 140.0)
            )
            rows = [r for l in lines if (r := parse_csv_line(l)) is not None]
            if rows:
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                csv_path = os.path.join(DATA_DIR, f"hwemu_track_{ts}.csv")
                with open(csv_path, "w", newline="", encoding="utf-8") as f:
                    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                    w.writeheader()
                    w.writerows(rows)

        pkg = load_latest_calibration()
        fit = pkg["mcal_data"].get("GROUND", {}).get("fit") or pkg["mcal_data"].get("AIR", {}).get("fit") or {}
        analysis = evaluate_analog_pid_from_data(rows, fit) if rows else None
        print(json.dumps({
            "port": c.port,
            "hwemu_enabled": bool(args.enable),
            "firmware_hwmap": hwmap_msgs,
            "bom_translation": bom,
            "captured_csv_frames": len(rows),
            "saved_csv": csv_path,
            "auto_stop_reason": stop_reason,
            "empirical_analysis": analysis,
        }, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        c.safe_emergency_stop()
        raise
    finally:
        c.close()


PROBE_FIELDS = [
    "ms", "state", "face", "phase", "obs_mm", "obs_ok",
    "servo_us", "best_us", "best_mm", "dist_mm", "hdg_deg", "laser", "buz",
]


def parse_probe_line(line: str) -> Optional[Dict[str, Any]]:
    if not line.startswith("PROBE,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 14:
        return None
    try:
        return {
            "ms": int(parts[1]),
            "state": int(parts[2]),
            "face": int(parts[3]),
            "phase": parts[4],
            "obs_mm": int(parts[5]),
            "obs_ok": int(parts[6]),
            "servo_us": int(parts[7]),
            "best_us": int(parts[8]),
            "best_mm": int(parts[9]),
            "dist_mm": float(parts[10]),
            "hdg_deg": float(parts[11]),
            "laser": int(parts[12]),
            "buz": int(parts[13]),
        }
    except ValueError:
        return None


def cli_probe_scan(args) -> int:
    ensure_data_dir()
    c = SerialClient(args.port)
    try:
        c.send("stop")
        time.sleep(0.08)
        if args.center is not None:
            c.send(f"set servo_us {int(args.center)}")
            time.sleep(0.05)
        c.read_lines(0.1)
        c.send(f"scan {int(args.span)} {int(args.step)} {int(args.dwell)}")
        est_steps = max(2, int((2 * args.span) // max(1, args.step)) + 2)
        timeout_s = max(5.0, est_steps * (args.dwell / 1000.0) + 3.0)
        lines = c.read_lines(
            timeout_s,
            stop_predicate=lambda l: l.startswith("SCAN_DONE,") or l.startswith("SCAN ERROR"),
        )
        points = []
        done_line = None
        for l in lines:
            if l.startswith("SCAN_PT,"):
                ps = l.split(",")
                if len(ps) >= 4:
                    points.append({"servo_us": int(ps[1]), "obs_mm": int(ps[2]), "obs_ok": int(ps[3])})
            elif l.startswith("SCAN_DONE,"):
                done_line = l
        ts = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(DATA_DIR, f"probe_scan_{ts}.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["servo_us", "obs_mm", "obs_ok"])
            w.writeheader()
            w.writerows(points)
        valid_pts = [p for p in points if p["obs_ok"] == 1 and 40 <= p["obs_mm"] <= 600]
        valley_center_us = None
        min_mm = None
        if valid_pts:
            min_mm = min(p["obs_mm"] for p in valid_pts)
            near_min = [p["servo_us"] for p in valid_pts if p["obs_mm"] <= min_mm + 15]
            valley_center_us = int(round(sum(near_min) / len(near_min)))
        print(json.dumps({
            "port": c.port,
            "points_count": len(points),
            "valid_points": len(valid_pts),
            "min_obs_mm": min_mm,
            "recommended_servo_center_us": valley_center_us,
            "firmware_scan_done": done_line,
            "saved_csv": csv_path,
        }, ensure_ascii=False, indent=2))
        return 0
    finally:
        c.close()


def cli_probe_odom(args) -> int:
    c = SerialClient(args.port)
    try:
        c.send("stop")
        time.sleep(0.08)
        if args.push:
            pkg = load_latest_calibration()
            m = pkg["mcal_data"].get("GROUND", {})
            fit = m.get("fit") or {}
            if fit:
                c.send(f"set db_l {fit['db_L']:.4f}")
                c.send(f"set db_r {fit['db_R']:.4f}")
                c.send(f"set fs_l {fit['fs_L']:.1f}")
                c.send(f"set fs_r {fit['fs_R']:.1f}")
                for idx, (vl, vr) in enumerate(zip(m.get("curve_L", []), m.get("curve_R", []))):
                    if vl > 0 or vr > 0:
                        c.send(f"mcurve {idx} {vl:.1f} {vr:.1f}")
                time.sleep(0.12)
        c.send("odom_reset")
        time.sleep(0.06)
        c.read_lines(0.1)
        cmd_str = f"{args.action} {args.target:.1f} {args.speed:.1f}"
        c.send(cmd_str)
        lines = c.read_lines(
            12.0,
            stop_predicate=lambda l: l.startswith("MOVE_DONE,") or l.startswith("MOVE ERROR") or ("FAULT" in l and "EVENT" in l),
        )
        done_line = next((l for l in lines if l.startswith("MOVE_DONE,")), None)
        print(json.dumps({
            "port": c.port,
            "command": cmd_str,
            "move_done": done_line,
            "calibration_tip": "若实测角度偏小，可增大 track_mm（或调大 probe_turn_deg）；若实测直行距离偏短，可减小 wheel_mm。",
            "messages": [l for l in lines if not l.startswith("CSV,") and not l.startswith("PROBE,")][-15:],
        }, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        c.safe_emergency_stop()
        raise
    finally:
        c.close()


def cli_probe_run(args) -> int:
    ensure_data_dir()
    c = SerialClient(args.port)
    try:
        c.send("stop")
        time.sleep(0.1)
        if args.push:
            pkg = load_latest_calibration()
            m = pkg["mcal_data"].get("GROUND", {})
            fit = m.get("fit") or {}
            if fit:
                c.send(f"set db_l {fit['db_L']:.4f}")
                c.send(f"set db_r {fit['db_R']:.4f}")
                c.send(f"set fs_l {fit['fs_L']:.1f}")
                c.send(f"set fs_r {fit['fs_R']:.1f}")
                for idx, (vl, vr) in enumerate(zip(m.get("curve_L", []), m.get("curve_R", []))):
                    if vl > 0 or vr > 0:
                        c.send(f"mcurve {idx} {vl:.1f} {vr:.1f}")
                time.sleep(0.12)
        c.send("set hw_emu 0")
        opt_map = [
            ("probe_cw", args.cw),
            ("probe_faces", args.faces),
            ("probe_aim_ms", args.aim_ms),
            ("probe_sweep_us", args.sweep_us),
            ("probe_laser_ms", args.laser_ms),
            ("probe_turn_deg", args.turn_deg),
            ("probe_edge1_mm", args.edge1),
            ("probe_edge2_mm", args.edge2),
            ("probe_approach_mm", args.approach),
            ("servo_us", args.servo_center),
        ]
        for k, v in opt_map:
            if v is not None:
                c.send(f"set {k} {v}")
                time.sleep(0.03)
        c.send("log 50")
        c.send("us")
        time.sleep(0.15)
        setup_msgs = [l for l in c.read_lines(0.2) if not l.startswith("CSV,")]
        c.send("probe")
        lines = c.read_lines(
            float(args.timeout),
            stop_predicate=lambda l: ("EVENT" in l and ("STOPPED" in l or "FAULT" in l)),
        )
        rows = [r for l in lines if (r := parse_probe_line(l)) is not None]
        events = [l for l in lines if l.startswith("PROBE_EVENT,") or l.startswith("EVENT ")]
        ts = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(DATA_DIR, f"probe_run_{ts}.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=PROBE_FIELDS)
            w.writeheader()
            w.writerows(rows)
        faces_summary: Dict[int, Dict[str, Any]] = {}
        for r in rows:
            fc = r["face"]
            if r["phase"] == "LASER":
                faces_summary[fc] = {"best_servo_us": r["best_us"], "best_obs_mm": r["best_mm"]}
        print(json.dumps({
            "port": c.port,
            "setup_messages": setup_msgs[-10:],
            "captured_probe_frames": len(rows),
            "events": events,
            "faces_aim_summary": faces_summary,
            "saved_csv": csv_path,
        }, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        c.safe_emergency_stop()
        raise
    finally:
        c.close()


# =============================================================================
# 4. 人工可视化简洁桌面软件 (Tkinter GUI)
# =============================================================================

class HostStationApp:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk, messagebox, filedialog

        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.filedialog = filedialog
        self.root = root
        self.root.title("STM32G474 智能小车蓝牙闭环标定与分析上位机 v2.3 (15档 0.06~0.95)")
        self.root.geometry("1240x830")

        self.ser = None
        self.rx_thread = None
        self.running = False
        self.rx_queue = queue.Queue()

        ensure_data_dir()
        pkg = load_latest_calibration()
        self.mcal_data = pkg["mcal_data"]
        self.ir_cal = pkg.get("ir_cal", {"black": [500] * 4, "white": [3000] * 4})
        self.active_mode = "GROUND"
        self.csv_rows = []
        self.recording_csv = True

        self._build_ui()
        self.refresh_ports()
        self.root.after(40, self._poll_rx_queue)

    def _build_ui(self):
        tk = self.tk
        ttk = self.ttk

        top = ttk.LabelFrame(self.root, text="1. 蓝牙/串口连接与一键闭环控制栏", padding=6)
        top.pack(fill=tk.X, padx=8, pady=4)

        ttk.Label(top, text="串口:").pack(side=tk.LEFT, padx=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28)
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="刷新端口", command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        self.conn_btn = ttk.Button(top, text="连接蓝牙串口 (115200)", command=self.toggle_connection)
        self.conn_btn.pack(side=tk.LEFT, padx=6)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(top, text="① 架空空载 5轮标定 (15档 0.06~0.95)", command=lambda: self.send_cmd("mcal air 5")).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="② 地面带载 5轮往返标定 (15档 0.06~0.95)", command=lambda: self.send_cmd("mcal ground 5")).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="③ 8s 红外校准 (cal)", command=lambda: self.send_cmd("cal")).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="④ 启动循迹 (start)", command=lambda: self.send_cmd("start")).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="■ 停止 (stop)", command=lambda: self.send_cmd("stop")).pack(side=tk.LEFT, padx=3)

        bar2 = ttk.Frame(self.root, padding=(8, 2))
        bar2.pack(fill=tk.X)
        ttk.Button(bar2, text="下发当前 15 点曲线与补偿到小车 RAM (免写Flash)", command=self.push_calibration_to_ram).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar2, text="归档保存至 data/ (JSON+CSV)", command=self.save_session_files).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar2, text="加载历史标定", command=self.load_session_file).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar2, text="查询状态与诊断 (status + diag)", command=lambda: (self.send_cmd("status"), self.send_cmd("diag"))).pack(side=tk.LEFT, padx=3)

        ttk.Label(bar2, text="  发送命令:").pack(side=tk.LEFT, padx=4)
        self.cmd_entry = ttk.Entry(bar2, width=26)
        self.cmd_entry.pack(side=tk.LEFT, padx=2)
        self.cmd_entry.bind("<Return>", lambda e: self.send_custom_cmd())
        ttk.Button(bar2, text="发送", command=self.send_custom_cmd).pack(side=tk.LEFT, padx=2)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        tab_motor = ttk.Frame(notebook, padding=6)
        notebook.add(tab_motor, text="电机死区与全量程 15 档转速曲线 (0.06 ~ 0.95)")

        left_pane = ttk.Frame(tab_motor)
        left_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(left_pane, bg="#1e1e1e", width=690, height=440)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        right_pane = ttk.Frame(tab_motor, width=490)
        right_pane.pack(side=tk.RIGHT, fill=tk.Y, padx=6)

        fit_box = ttk.LabelFrame(right_pane, text="5 轮标定拟合摘要（空载 AIR vs 地面 GROUND）", padding=6)
        fit_box.pack(fill=tk.X, pady=4)
        self.fit_text = tk.Text(fit_box, width=56, height=15, font=("Consolas", 10))
        self.fit_text.pack(fill=tk.X)

        verify_box = ttk.LabelFrame(right_pane, text="纯前馈等速验证 (vtest：想转多快就转在附近)", padding=6)
        verify_box.pack(fill=tk.X, pady=4)
        row_v = ttk.Frame(verify_box)
        row_v.pack(fill=tk.X, pady=2)
        ttk.Label(row_v, text="目标轮速 (mm/s):").pack(side=tk.LEFT)
        self.test_speed_var = tk.StringVar(value="160")
        ttk.Entry(row_v, textvariable=self.test_speed_var, width=8).pack(side=tk.LEFT, padx=4)
        self.use_mode_var = tk.StringVar(value="GROUND")
        ttk.Radiobutton(row_v, text="地面带载主用 (GROUND)", variable=self.use_mode_var, value="GROUND", command=self.redraw_motor_canvas).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(row_v, text="空载体检参考 (AIR)", variable=self.use_mode_var, value="AIR", command=self.redraw_motor_canvas).pack(side=tk.LEFT, padx=2)
        ttk.Button(verify_box, text="一键下发前馈曲线并执行 vtest 1.0s 实车验证", command=self.run_feedforward_verify).pack(fill=tk.X, pady=4)
        self.verify_label = ttk.Label(verify_box, text="待验证：将先同步 15 点曲线至 RAM，再执行纯前馈测试并回传实测误差")
        self.verify_label.pack(fill=tk.X)

        tab_hwpid = ttk.Frame(notebook, padding=6)
        notebook.add(tab_hwpid, text="实测赛道红外分析与【纯硬件模拟 PID】反哺评估")

        hw_top = ttk.Frame(tab_hwpid)
        hw_top.pack(fill=tk.X, pady=4)
        ttk.Button(hw_top, text="★ 基于当前实测数据 + 电机死区曲线，评估纯硬件模拟 PID 参数", command=self.run_hw_pid_optimizer).pack(side=tk.LEFT, padx=4)
        ttk.Button(hw_top, text="清空录制帧", command=self.clear_csv_rows).pack(side=tk.LEFT, padx=8)
        self.csv_count_label = ttk.Label(hw_top, text="已录制实车遥测样本: 0 帧")
        self.csv_count_label.pack(side=tk.LEFT, padx=12)

        self.hw_report_text = tk.Text(tab_hwpid, font=("Consolas", 10), height=26)
        self.hw_report_text.pack(fill=tk.BOTH, expand=True, pady=4)

        log_frame = ttk.LabelFrame(self.root, text="蓝牙串口实时收发日志 (USART1 115200)", padding=4)
        log_frame.pack(fill=tk.BOTH, padx=8, pady=4)
        self.log_text = tk.Text(log_frame, height=8, font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        self._update_fit_summary()
        self.redraw_motor_canvas()
        self.run_hw_pid_optimizer()

    def log(self, msg: str):
        self.log_text.insert(self.tk.END, msg + "\n")
        self.log_text.see(self.tk.END)

    def refresh_ports(self):
        ports = list_serial_ports()
        items = [f"{p['device']} - {p['description']}" for p in ports]
        self.port_combo["values"] = items
        if items and not self.port_var.get():
            self.port_var.set(items[0])

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.running = False
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.conn_btn.config(text="连接蓝牙串口 (115200)")
            self.log("[系统] 已断开串口连接。")
            return

        sel = self.port_var.get().strip()
        if not sel:
            self.messagebox.showwarning("提示", "请先选择蓝牙或 USB-TTL 对应的 COM 端口！")
            return
        port_name = sel.split(" ")[0]
        try:
            self.ser = serial.Serial(port_name, 115200, timeout=0.05)
            with open(PORT_CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump({"port": port_name}, f, indent=2)
            self.running = True
            self.rx_thread = threading.Thread(target=self._serial_rx_loop, daemon=True)
            self.rx_thread.start()
            self.conn_btn.config(text=f"断开 ({port_name})")
            self.log(f"[系统] 已成功连接 {port_name} (115200 8N1)，并设为默认端口。")
            self.send_cmd("status")
        except Exception as e:
            self.messagebox.showerror("连接失败", f"无法打开端口 {port_name}:\n{e}")

    def _serial_rx_loop(self):
        buf = b""
        while self.running and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(512)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", errors="replace").strip()
                        if text:
                            self.rx_queue.put(text)
            except Exception:
                break

    def send_cmd(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            self.log(f"[未连接串口，待发命令] > {cmd}")
            return
        try:
            self.ser.write((cmd.strip() + "\r\n").encode("utf-8"))
            self.log(f">>> {cmd}")
        except Exception as e:
            self.log(f"[发送失败] {e}")

    def send_custom_cmd(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.send_cmd(cmd)
            self.cmd_entry.delete(0, self.tk.END)

    def _poll_rx_queue(self):
        updated_motor = False
        while not self.rx_queue.empty():
            line = self.rx_queue.get_nowait()
            self._parse_line(line)
            if line.startswith("MCAL_"):
                updated_motor = True
        if updated_motor:
            self._update_fit_summary()
            self.redraw_motor_canvas()
        self.root.after(40, self._poll_rx_queue)

    def _parse_line(self, line: str):
        if not line.startswith("CSV,"):
            self.log(line)

        parts = line.split(",")
        if parts[0] == "MCAL_STEP" and len(parts) >= 8:
            mode = parts[1].strip()
            p_idx = str(int(parts[2]))
            s_idx = int(parts[4])
            duty, vl, vr = float(parts[5]), float(parts[6]), float(parts[7])
            self.active_mode = mode if mode in self.mcal_data else "AIR"
            m = self.mcal_data[self.active_mode]
            m["passes"].setdefault(p_idx, {})[str(s_idx)] = [duty, vl, vr]
            for s in range(len(MCAL_DUTIES)):
                vls, vrs = [], []
                for pk in m["passes"]:
                    st = m["passes"][pk].get(str(s)) or m["passes"][pk].get(s)
                    if st:
                        vls.append(st[1])
                        vrs.append(st[2])
                if vls:
                    m["curve_L"][s] = sum(vls) / len(vls)
                    m["curve_R"][s] = sum(vrs) / len(vrs)
            m["fit"] = fit_motor_curve(MCAL_DUTIES, m["curve_L"], m["curve_R"])

        elif parts[0] == "MCAL_CURVE" and len(parts) >= 5:
            s_idx = int(parts[1])
            if 0 <= s_idx < len(MCAL_DUTIES):
                m = self.mcal_data[self.active_mode]
                m["curve_L"][s_idx] = float(parts[3])
                m["curve_R"][s_idx] = float(parts[4])

        elif parts[0] == "MCAL_DONE" and len(parts) >= 6:
            mode = parts[1].strip()
            self.active_mode = mode if mode in self.mcal_data else "AIR"
            m = self.mcal_data[self.active_mode]
            m["fit"] = {"db_L": float(parts[2]), "db_R": float(parts[3]), "fs_L": float(parts[4]), "fs_R": float(parts[5])}
            # 地面带载始终优先作为循迹主模型；仅在尚未标定 GROUND 时才自动切换到 AIR
            has_gnd = any(v > 0 for v in self.mcal_data["GROUND"]["curve_L"])
            self.use_mode_var.set("GROUND" if has_gnd else self.active_mode)
            save_calibration_files(self.mcal_data, self.ir_cal, [], mode_tag=self.active_mode)

        elif parts[0] == "VTEST_DONE" and len(parts) >= 9:
            kv = dict(item.split("=", 1) for item in parts[1:] if "=" in item)
            self.verify_label.config(
                text=f"实测结果: 目标 {kv.get('tL')}/{kv.get('tR')} -> 前馈 uL={kv.get('uL')}, uR={kv.get('uR')} | 实测 vL={kv.get('vL')} ({kv.get('errL')}), vR={kv.get('vR')} ({kv.get('errR')}) mm/s"
            )

        elif line.startswith("CAL "):
            tokens = line.split()
            if len(tokens) >= 4 and tokens[1].isdigit():
                ch = int(tokens[1])
                if 0 <= ch < 4:
                    for tok in tokens[2:]:
                        if tok.startswith("black="):
                            self.ir_cal["black"][ch] = int(tok.split("=")[1])
                        elif tok.startswith("white="):
                            self.ir_cal["white"][ch] = int(tok.split("=")[1])

        elif (row := parse_csv_line(line)) is not None and self.recording_csv:
            self.csv_rows.append(row)
            if len(self.csv_rows) % 10 == 0:
                self.csv_count_label.config(text=f"已录制实车遥测样本: {len(self.csv_rows)} 帧")

    def _update_fit_summary(self):
        lines = []
        fits = {}
        for mode_key, title in [
            ("GROUND", "【★ 地面带载主用模型 (GROUND, 15档 0.06~0.95)】"),
            ("AIR", "【空载体检参考模型 (AIR, 15档, 仅对比)】"),
        ]:
            m = self.mcal_data[mode_key]
            fit = m["fit"] or fit_motor_curve(MCAL_DUTIES, m["curve_L"], m["curve_R"])
            m["fit"] = fit
            fits[mode_key] = fit
            lines.append(f"{title} (已采集 {len(m['passes'])} 轮):")
            lines.append(f"  左轮死区 db_L = {fit['db_L']:.4f} | 满速 fs_L = {fit['fs_L']:.1f} mm/s")
            lines.append(f"  右轮死区 db_R = {fit['db_R']:.4f} | 满速 fs_R = {fit['fs_R']:.1f} mm/s")
            lines.append(f"  左右死区差    = {fit['db_L'] - fit['db_R']:+.4f} | 满速差 = {fit['fs_L'] - fit['fs_R']:+.1f} mm/s")
            lines.append("-" * 54)
        if self.mcal_data["GROUND"]["passes"] and self.mcal_data["AIR"]["passes"]:
            dg_l = fits["GROUND"]["db_L"] - fits["AIR"]["db_L"]
            dg_r = fits["GROUND"]["db_R"] - fits["AIR"]["db_R"]
            lines.append(f"地面滚动摩擦死区增量: 左 {dg_l:+.4f}, 右 {dg_r:+.4f} (实车循迹用 GROUND)")
        self.fit_text.delete("1.0", self.tk.END)
        self.fit_text.insert(self.tk.END, "\n".join(lines))

    def redraw_motor_canvas(self):
        c = self.canvas
        c.delete("all")
        w = int(c.winfo_width() or 690)
        h = int(c.winfo_height() or 440)
        pad_l, pad_r, pad_t, pad_b = 58, 25, 30, 42
        pw = max(100, w - pad_l - pad_r)
        ph = max(100, h - pad_t - pad_b)

        m = self.mcal_data[self.use_mode_var.get()]
        max_v = max([600.0] + m["curve_L"] + m["curve_R"])
        y_top = float(int((max_v + 149) // 150) * 150)

        def to_xy(duty: float, speed: float):
            x = pad_l + (duty / 1.00) * pw
            y = pad_t + ph - (clamp(speed, 0.0, y_top) / y_top) * ph
            return x, y

        for d_step in [0.0, 0.20, 0.40, 0.60, 0.80, 1.00]:
            x, _ = to_xy(d_step, 0)
            c.create_line(x, pad_t, x, pad_t + ph, fill="#333333")
            c.create_text(x, pad_t + ph + 15, text=f"{d_step:.2f}", fill="#cccccc", font=("Consolas", 9))
        for k in range(6):
            v_step = k * y_top / 5.0
            _, y = to_xy(0, v_step)
            c.create_line(pad_l, y, pad_l + pw, y, fill="#333333")
            c.create_text(pad_l - 28, y, text=f"{int(v_step)}", fill="#cccccc", font=("Consolas", 9))

        c.create_text(
            pad_l + pw / 2, 14,
            text=f"15档全量程 PWM 占空比 (0.06~0.95) vs 实测轮速 (mm/s) — 当前: {self.use_mode_var.get()}",
            fill="#ffffff", font=("Microsoft YaHei", 10, "bold"),
        )

        for pk, steps_dict in m["passes"].items():
            for sk, tup in steps_dict.items():
                xl, yl = to_xy(tup[0], tup[1])
                xr, yr = to_xy(tup[0], tup[2])
                c.create_oval(xl - 2, yl - 2, xl + 2, yl + 2, fill="#3a86ff", outline="")
                c.create_oval(xr - 2, yr - 2, xr + 2, yr + 2, fill="#ff595e", outline="")

        pts_l, pts_r = [], []
        for s_idx, duty in enumerate(MCAL_DUTIES):
            vl, vr = m["curve_L"][s_idx], m["curve_R"][s_idx]
            if vl > 0 or vr > 0:
                pts_l.append(to_xy(duty, vl))
                pts_r.append(to_xy(duty, vr))
        for i in range(len(pts_l) - 1):
            c.create_line(*pts_l[i], *pts_l[i + 1], fill="#4cc9f0", width=2)
        for i in range(len(pts_r) - 1):
            c.create_line(*pts_r[i], *pts_r[i + 1], fill="#ff6b6b", width=2)
        for x, y in pts_l:
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#4cc9f0", outline="#ffffff")
        for x, y in pts_r:
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#ff6b6b", outline="#ffffff")

    def push_calibration_to_ram(self):
        mode = self.use_mode_var.get()
        m = self.mcal_data[mode]
        fit = m["fit"] or fit_motor_curve(MCAL_DUTIES, m["curve_L"], m["curve_R"])
        self.send_cmd("stop")
        time.sleep(0.05)
        for cmd_str in [
            f"set db_l {fit['db_L']:.4f}",
            f"set db_r {fit['db_R']:.4f}",
            f"set fs_l {fit['fs_L']:.1f}",
            f"set fs_r {fit['fs_R']:.1f}",
        ]:
            self.send_cmd(cmd_str)
            time.sleep(0.015)
        for s_idx in range(len(MCAL_DUTIES)):
            vl, vr = m["curve_L"][s_idx], m["curve_R"][s_idx]
            if vl > 0 or vr > 0:
                self.send_cmd(f"mcurve {s_idx} {vl:.1f} {vr:.1f}")
                time.sleep(0.015)
        self.log(f"[系统] 已将 {mode} 模式的 15 点曲线与死区/斜率同步至小车 RAM（未写Flash）。")

    def run_feedforward_verify(self):
        try:
            target = float(self.test_speed_var.get())
        except ValueError:
            return
        self.push_calibration_to_ram()
        self.send_cmd(f"vtest {target:.1f} {target:.1f} 1000")

    def clear_csv_rows(self):
        self.csv_rows.clear()
        self.csv_count_label.config(text="已录制实车遥测样本: 0 帧")

    def run_hw_pid_optimizer(self):
        mode = self.use_mode_var.get()
        m = self.mcal_data[mode]
        fit = m["fit"] or fit_motor_curve(MCAL_DUTIES, m["curve_L"], m["curve_R"])
        rep = evaluate_analog_pid_from_data(self.csv_rows, fit)
        if not rep["empirical_ready"]:
            lines = [
                "================================================================================",
                "   纯硬件模拟 PID 实测数据反哺评估器（等待实车采样中）",
                "================================================================================",
                f"当前电机模型 ({mode}): db_L={fit['db_L']:.4f}, db_R={fit['db_R']:.4f}, fs_L={fit['fs_L']:.1f}, fs_R={fit['fs_R']:.1f} mm/s",
                f"当前已录制有效红外遥测帧数: {rep['valid_frames']} 帧 (< 20 帧)",
                "",
                "【严谨工程说明】",
                f"  {rep['note']}",
                "  原因：实车红外管 TSAL6200 + BPV10NF 具有一定光斑宽度，且受真实安装高度、环境光和",
                "  20mm 黑胶带材质影响。未取得实地扫线或跑圈电压前，不能凭理想点模型断定 R39/R40 必须改小。",
                "",
                "【基于当前电路与电机死区模型的基速/跳线预设检查】",
                f"  - 若期望纯硬件基准占空比 D0 ≈ {rep['target_d0_pct']:.1f}% -> V_BASE (U7 Pin14) 目标 ≈ {rep['v_base_target_v']:.3f} V",
                f"  - {rep['jp_v_note']}",
                f"  - 左右电机天然差异：在 D0 下预估 vL={rep['vl_at_d0_mm_s']} vs vR={rep['vr_at_d0_mm_s']} mm/s (差值 {rep['speed_mismatch_mm_s']:+.1f} mm/s)",
                "",
                "下一步操作：请用小车在赛道上跑一圈软件循迹（或手推车沿弯道慢扫），录制 >= 20 帧后点击上方按钮分析！",
                "================================================================================",
            ]
        else:
            lines = [
                "================================================================================",
                f"   基于实测赛道数据的【纯硬件模拟 PID】反哺评估报告 (有效样本 {rep['valid_frames']} 帧)",
                "================================================================================",
                f"1. 实测四路红外黑白压差 (V): {rep['ir_spans_v']}  (均值 {rep['avg_span_v']:.3f} V)",
                f"2. 实测边缘单调性检验 (R39/R40=150k, R41/R42=300k):",
                f"   - {rep['weight_advice']}",
                "   - 实测各候选阻值对比 (R41/R42 固定 300kΩ):",
            ]
            for c in rep["candidate_resistors"]:
                lines.append(
                    f"     R39/R40={c['R39_R40_kohm']:3d}kΩ ({c['weight_ratio']:4.2f}:1) -> 与软件位置相关性={c['corr_with_sw_pos']:.4f}, max|e|={c['max_abs_e_V']:.3f}V, 运放余量={c['u6a_rail_margin_V']:.3f}V"
                )
            lines.extend([
                "",
                "3. 纯硬件模拟 PID 旋钮与电阻定量建议：",
                f"   - 基速电位器 RV_V: 调至 V_BASE (U7 Pin14) ≈ {rep['v_base_target_v']:.3f} V (对应 D0 ≈ {rep['target_d0_pct']:.1f}%)",
                f"   - 跳线建议: {rep['jp_v_note']}",
                f"   - 比例支路: 建议总 Kp ≈ {rep['kp_total_recommended']} -> R60 用 {rep['r60_recommended_kohm']} kΩ，RV_P ≈ {rep['rv_p_kohm']} kΩ",
                f"   - 微分支路: RV_D ≈ {rep['rv_d_kohm']} kΩ，JP_I 保持拔除",
                f"   - 电机天然死区差 ({rep['duty_mismatch_pct']:+.2f}%): D0 下预估 vL={rep['vl_at_d0_mm_s']} vs vR={rep['vr_at_d0_mm_s']} mm/s",
                "================================================================================",
            ])
        self.hw_report_text.delete("1.0", self.tk.END)
        self.hw_report_text.insert(self.tk.END, "\n".join(lines))

    def save_session_files(self):
        paths = save_calibration_files(self.mcal_data, self.ir_cal, self.csv_rows, mode_tag=self.use_mode_var.get())
        self.log(f"[归档完成] 已保存至: {paths['session_json']}")

    def load_session_file(self):
        path = self.filedialog.askopenfilename(initialdir=DATA_DIR, title="选择历史标定 JSON 文件", filetypes=[("JSON files", "*.json")])
        if not path:
            return
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if "mcal_data" in data:
            self.mcal_data = data["mcal_data"]
            self._update_fit_summary()
            self.redraw_motor_canvas()
            self.run_hw_pid_optimizer()
            self.log(f"[加载成功] {path}")


def run_selftest() -> int:
    """运行全流程算法与数据逻辑自检（15 档拟合、分段前馈、实测样本分析）。"""
    assert len(MCAL_DUTIES) == 15 and MCAL_DUTIES[0] == 0.06 and MCAL_DUTIES[-1] == 0.95
    speeds_l = [0.0 if u <= 0.15 else (u - 0.15) / 0.85 * 430.0 for u in MCAL_DUTIES]
    speeds_r = [0.0 if u <= 0.09 else (u - 0.09) / 0.91 * 510.0 for u in MCAL_DUTIES]
    fit = fit_motor_curve(MCAL_DUTIES, speeds_l, speeds_r)
    assert fit["db_L"] > fit["db_R"] + 0.03
    ul = interpolate_duty_for_speed(200.0, MCAL_DUTIES, speeds_l, fit["db_L"], fit["fs_L"])
    ur = interpolate_duty_for_speed(200.0, MCAL_DUTIES, speeds_r, fit["db_R"], fit["fs_R"])
    assert ul > ur + 0.04

    rep_empty = evaluate_analog_pid_from_data([], fit)
    assert rep_empty["empirical_ready"] is False

    w_adc, b_adc = 3400, 500
    synth_rows = [
        {"raw0": w_adc, "raw1": b_adc, "raw2": b_adc, "raw3": w_adc, "q0": 0, "q1": 1, "q2": 1, "q3": 0, "pos": 0.0, "confidence": 2.0},
        {"raw0": w_adc, "raw1": w_adc, "raw2": b_adc, "raw3": b_adc, "q0": 0, "q1": 0, "q2": 1, "q3": 1, "pos": 0.675, "confidence": 2.0},
        {"raw0": w_adc, "raw1": w_adc, "raw2": w_adc, "raw3": b_adc, "q0": 0, "q1": 0, "q2": 0, "q3": 1, "pos": 1.0, "confidence": 1.0},
        {"raw0": b_adc, "raw1": b_adc, "raw2": w_adc, "raw3": w_adc, "q0": 1, "q1": 1, "q2": 0, "q3": 0, "pos": -0.675, "confidence": 2.0},
        {"raw0": b_adc, "raw1": w_adc, "raw2": w_adc, "raw3": w_adc, "q0": 1, "q1": 0, "q2": 0, "q3": 0, "pos": -1.0, "confidence": 1.0},
    ] * 5
    rep_real = evaluate_analog_pid_from_data(synth_rows, fit)
    assert rep_real["empirical_ready"] is True
    assert rep_real["edge_drop_observed"] is True
    bom_chk = translate_hwemu_to_bom(w_out=0.5, kp_stage=1.8, kd_stage_ms=25.0, v_base=2.20, coast_mode=False)
    assert abs(bom_chk["R39_R40_kohm"] - 100.0) < 0.2 and not bom_chk["D_feasible_on_stock_100n"]
    bom_grp_b = translate_hwemu_to_bom(
        netlist_mode=True, r39_kohm=47.0, r41_kohm=110.0, r40_kohm=100.0, r42_kohm=120.0,
        r43_kohm=100.0, r45_kohm=120.0, r45_to_5v=True,
        kp_stage=3.5, kd_stage_ms=35.0, v_base=2.50, c50_nf=470.0,
        tri_min_v=0.42, tri_max_v=2.82, tri_freq_hz=21500.0,
    )
    assert bom_grp_b["D_feasible_on_C50"] is True and bom_grp_b["TRI_freq_Hz"] == 21500
    wd = OffTrackWatchdog(max_offtrack_mm=140.0)
    assert not wd.feed_line("CSV,1000,3,0,900,1100,800,2100,0.2,0.7,0.7,0.1,0.0,1.7,150,150,150,150,0.3,0.3")
    assert not wd.feed_line("CSV,1200,3,0,1150,2060,1500,2390,0.0,0.0,0.0,0.0,0.8,0.0,320,320,150,150,0.4,0.4")
    assert not wd.feed_line("CSV,1400,3,0,1150,2060,1500,2390,0.0,0.0,0.0,0.0,0.8,0.0,320,320,150,150,0.4,0.4")
    assert wd.feed_line("CSV,1600,3,0,1150,2060,1500,2390,0.0,0.0,0.0,0.0,0.8,0.0,320,320,150,150,0.4,0.4")
    assert wd.reason and wd.reason.startswith("auto_stop_off_track_white")
    wd_dark = OffTrackWatchdog(max_offtrack_mm=140.0)
    assert not wd_dark.feed_line("CSV,1000,3,0,900,1100,800,2100,0.2,0.7,0.7,0.1,0.0,1.7,150,150,150,150,0.3,0.3")
    assert not wd_dark.feed_line("CSV,1200,3,0,350,550,420,750,0.95,0.98,0.98,0.95,0.0,3.86,300,300,150,150,0.3,0.3")
    assert not wd_dark.feed_line("CSV,1400,3,0,350,550,420,750,0.95,0.98,0.98,0.95,0.0,3.86,300,300,150,150,0.3,0.3")
    assert wd_dark.feed_line("CSV,1550,3,0,350,550,420,750,0.95,0.98,0.98,0.95,0.0,3.86,300,300,150,150,0.3,0.3")
    assert wd_dark.reason and wd_dark.reason.startswith("auto_stop_off_board_dark")
    pr = parse_probe_line("PROBE,12345,9,1,LASER,132,1,1520,1520,130,450.5,-90.25,1,1")
    assert pr is not None and pr["face"] == 1 and pr["phase"] == "LASER" and pr["laser"] == 1 and pr["obs_mm"] == 132
    print("PASS host_station selftest:", json.dumps({"duties_count": len(MCAL_DUTIES), "fit": fit, "uL_200": round(ul, 3), "uR_200": round(ur, 3), "hw_pid": rep_real, "bom_sample": bom_chk, "bom_grp_b": bom_grp_b, "probe_sample": pr}, ensure_ascii=False))
    return 0


def build_arg_parser():
    p = argparse.ArgumentParser(description="STM32G474 智能小车蓝牙上位机 (Agent CLI + GUI)")
    p.add_argument("--selftest", action="store_true", help="运行算法自检并退出")
    sub = p.add_subparsers(dest="subcmd")

    sp_ports = sub.add_parser("ports", help="列出本机所有可用蓝牙/串口 (JSON)")
    sp_ports.set_defaults(func=cli_ports)

    sp_setp = sub.add_parser("set-port", help="保存默认蓝牙 COM 端口至 data/port_config.json")
    sp_setp.add_argument("port", help="如 COM5")
    sp_setp.set_defaults(func=cli_set_port)

    sp_cmd = sub.add_parser("cmd", help="向小车发送一条或多条控制命令并返回结构化 JSON")
    sp_cmd.add_argument("--port", default="AUTO")
    sp_cmd.add_argument("--wait", type=float, default=0.8)
    sp_cmd.add_argument("commands", nargs="+")
    sp_cmd.set_defaults(func=cli_cmd)

    sp_mcal = sub.add_parser("mcal", help="执行 5 轮 15 档电机标定 (ground 主用 或 air 参考) 并保存 JSON+CSV")
    sp_mcal.add_argument("--port", default="AUTO")
    sp_mcal.add_argument("--mode", choices=["ground", "air"], default="ground")
    sp_mcal.add_argument("--passes", type=int, default=5)
    sp_mcal.set_defaults(func=cli_mcal)

    sp_push = sub.add_parser("push", help="将 data/latest_calibration.json 的曲线与死区下发至小车 RAM")
    sp_push.add_argument("--port", default="AUTO")
    sp_push.add_argument("--mode", choices=["GROUND", "AIR"], default="GROUND")
    sp_push.set_defaults(func=cli_push)

    sp_vt = sub.add_parser("vtest", help="同步曲线后批量测试多档目标轮速的前馈精度 (mm/s)")
    sp_vt.add_argument("--port", default="AUTO")
    sp_vt.add_argument("--mode", choices=["GROUND", "AIR"], default="GROUND")
    sp_vt.add_argument("--speeds", default="80,120,160,220,300")
    sp_vt.add_argument("--ms", type=int, default=1000)
    sp_vt.set_defaults(func=cli_vtest)

    sp_cap = sub.add_parser("capture", help="录制实车赛道 CSV 遥测并自动评估硬件模拟 PID")
    sp_cap.add_argument("--port", default="AUTO")
    sp_cap.add_argument("--seconds", type=float, default=15.0)
    sp_cap.add_argument("--cmd", default="none", help="录制前发送的命令，如 start 或 none")
    sp_cap.add_argument("--stop-after", action="store_true")
    sp_cap.add_argument("--max-offtrack-mm", type=float, default=140.0, help="跑出循迹线超过该距离或冲出赛道板时主动停止 (0=禁用)")
    sp_cap.set_defaults(func=cli_capture)

    sp_ana = sub.add_parser("analyze", help="离线分析已录制的赛道 CSV 文件与电机模型")
    sp_ana.add_argument("--file", required=True)
    sp_ana.add_argument("--motor", default=LATEST_CAL_PATH)
    sp_ana.set_defaults(func=cli_analyze)

    sp_hw = sub.add_parser("hwemu", help="1:1 仿真硬件模拟 PID 模式运行整圈赛道并翻译物理 BOM 元件方案")
    sp_hw.add_argument("--port", default="AUTO")
    sp_hw.add_argument("--enable", type=int, choices=[0, 1], default=1)
    sp_hw.add_argument("--preset", choices=["stock", "group_b"], default="group_b")
    sp_hw.add_argument("--netlist", type=int, choices=[0, 1], default=1)
    sp_hw.add_argument("--w-out", type=float, default=1.0 / 3.0)
    sp_hw.add_argument("--w-in", type=float, default=1.0 / 6.0)
    sp_hw.add_argument("--adc-vref", type=float, default=None, help="覆盖 MCU VREF+(V)，默认保留芯片自动实测 ~2.897V")
    sp_hw.add_argument("--v5v", type=float, default=5.000, help="实测主扩展板 PWM_5V (V)")
    sp_hw.add_argument("--vmid", type=float, default=2.500, help="实测主扩展板 PWM_MID (V)")
    sp_hw.add_argument("--e-trim", type=float, default=0.0)
    sp_hw.add_argument("--r39", type=float, default=None)
    sp_hw.add_argument("--r41", type=float, default=None)
    sp_hw.add_argument("--r40", type=float, default=None)
    sp_hw.add_argument("--r42", type=float, default=None)
    sp_hw.add_argument("--r43", type=float, default=None)
    sp_hw.add_argument("--r45", type=float, default=None)
    sp_hw.add_argument("--r45-5v", type=int, choices=[0, 1], default=None)
    sp_hw.add_argument("--r-ir", type=float, default=None, help="红外子板输出串阻 (kΩ，如 R033=1.0k)")
    sp_hw.add_argument("--kp", type=float, default=2.5)
    sp_hw.add_argument("--rv-p", type=float, default=None, help="直接输入实测 RV_P 阻值 (kΩ)，自动算 kp_stage")
    sp_hw.add_argument("--r60", type=float, default=20.0)
    sp_hw.add_argument("--kd-ms", type=float, default=35.0)
    sp_hw.add_argument("--rv-d", type=float, default=None, help="直接输入实测 RV_D 阻值 (kΩ)，配合 --c50-nf 自动算 kd_ms")
    sp_hw.add_argument("--c50-nf", type=float, default=470.0, help="实际焊接电容 C50 (nF，原板 100nF，改焊 470nF)")
    sp_hw.add_argument("--r54", type=float, default=4.7, help="实际焊接电阻 R54 (kΩ，原板 22k，改焊 4.7k)")
    sp_hw.add_argument("--c60-nf", type=float, default=10.0)
    sp_hw.add_argument("--r61", type=float, default=20.0)
    sp_hw.add_argument("--tau1-ms", type=float, default=None)
    sp_hw.add_argument("--tau2-ms", type=float, default=None)
    sp_hw.add_argument("--v-base", type=float, default=2.44)
    sp_hw.add_argument("--atten", type=float, default=0.625, help="R74~77 实际分压系数 ANA_SPEED/RAW")
    sp_hw.add_argument("--spd-tau-ms", type=float, default=0.2475)
    sp_hw.add_argument("--tri-min", type=float, default=0.351, help="实测三角波谷底电压 (V)")
    sp_hw.add_argument("--tri-max", type=float, default=2.851, help="实测三角波波峰电压 (V)")
    sp_hw.add_argument("--tri-hz", type=float, default=20000.0, help="实测三角波频率 (Hz，TIM1 将 1:1 同步此载波)")
    sp_hw.add_argument("--i", type=int, choices=[0, 1], default=0)
    sp_hw.add_argument("--coast", type=int, choices=[0, 1], default=1)
    sp_hw.add_argument("--run", action="store_true", help="配置完成后自动发车跑完整圈并录数")
    sp_hw.add_argument("--seconds", type=float, default=20.0)
    sp_hw.add_argument("--stop-after", action="store_true")
    sp_hw.add_argument("--max-offtrack-mm", type=float, default=140.0, help="跑出循迹线超过该距离或冲出赛道板时主动停止 (0=禁用)")
    sp_hw.set_defaults(func=cli_hwemu)

    sp_ps = sub.add_parser("probe-scan", help="蓝牙控制云台+超声步进扫掠标定，输出最佳正对脉宽并保存 CSV")
    sp_ps.add_argument("--port", default="AUTO")
    sp_ps.add_argument("--center", type=int, default=None, help="可选：先设定 servo_center_us")
    sp_ps.add_argument("--span", type=int, default=220, help="单侧扫掠半幅 us (默认 220)")
    sp_ps.add_argument("--step", type=int, default=20, help="步进 us (默认 20)")
    sp_ps.add_argument("--dwell", type=int, default=140, help="每步驻留采样时间 ms (默认 140)")
    sp_ps.set_defaults(func=cli_probe_scan)

    sp_po = sub.add_parser("probe-odom", help="蓝牙执行单段直行(drive)或原地转弯(turn)里程计标定")
    sp_po.add_argument("--port", default="AUTO")
    sp_po.add_argument("action", choices=["drive", "turn"], help="drive=直行(mm), turn=原地转弯(deg, 正=左转CCW, 负=右转CW)")
    sp_po.add_argument("target", type=float, help="目标距离(mm)或目标角度(deg)")
    sp_po.add_argument("--speed", type=float, default=100.0, help="线速度 mm/s (默认 100)")
    sp_po.add_argument("--push", action="store_true", help="运动前先同步 latest_calibration.json 电机曲线")
    sp_po.set_defaults(func=cli_probe_odom)

    sp_pr = sub.add_parser("probe-run", help="蓝牙启动四面顺时针打靶任务并全程录制 PROBE 遥测 CSV")
    sp_pr.add_argument("--port", default="AUTO")
    sp_pr.add_argument("--push", action="store_true", help="启动前先同步 latest_calibration.json 电机曲线")
    sp_pr.add_argument("--cw", type=int, choices=[0, 1], default=None)
    sp_pr.add_argument("--faces", type=int, choices=[1, 2, 3, 4], default=None)
    sp_pr.add_argument("--aim-ms", type=int, default=None)
    sp_pr.add_argument("--sweep-us", type=int, default=None)
    sp_pr.add_argument("--laser-ms", type=int, default=None, help="单面激光时间 ms (正式 10000，调试可设 2000)")
    sp_pr.add_argument("--turn-deg", type=float, default=None)
    sp_pr.add_argument("--edge1", type=float, default=None, help="离面越角直行距离 mm")
    sp_pr.add_argument("--edge2", type=float, default=None, help="沿侧面对中直行距离 mm")
    sp_pr.add_argument("--approach", type=float, default=None, help="逼近最大距离 mm")
    sp_pr.add_argument("--servo-center", type=int, default=None)
    sp_pr.add_argument("--timeout", type=float, default=185.0)
    sp_pr.set_defaults(func=cli_probe_run)
    return p


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    if hasattr(args, "func"):
        sys.exit(args.func(args))

    import tkinter as tk
    root = tk.Tk()
    HostStationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
