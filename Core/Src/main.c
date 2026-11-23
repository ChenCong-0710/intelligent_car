/**
 * @file    main.c
 * @brief   STM32G474 智能小车顶层应用调度、按键交互、蓝牙串口命令控制台与实时遥测
 * @details
 * 本文件包含以下独立功能块：
 *   1. 全局控制器实例 (g_lf) 与 ST-LINK Live Watch 实时监控快照 (g_debug)
 *   2. 顶层动作触发封装：停止 (stop)、红外校准 (calibrate)、电机 5 轮空载/地面标定 (calibrate_motors)、
 *      起跑 (start)、四面打靶 (start_probe)、状态与纯硬件 PID 辅助诊断打印 (status, print_diag)
 *   3. 蓝牙/上位机在线参数修改引擎 (set_parameter) 与命令行解析器 (command, console_poll)
 *   4. 三路物理按键状态机 (buttons)：25 ms 消抖 + 短按/长按 1s 识别
 *   5. 200 Hz (5.0 ms) 硬件同步控制主循环与 10 Hz (100 ms) 非阻塞 CSV 蓝牙遥测
 */

#include "board.h"
#include "stm32g4xx.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 距离障碍物 <15 cm 停稳后，自动立刻开始“舵机 + 超声波”寻找障碍物垂直(法线)方向：1 = 自动（比赛流程），0 = 需手动按探测键 */
#define AUTO_PROBE_ON_OBSTACLE 1

/* 主动短接制动停稳后额外等待多久再启动云台扫掠 (ms)：留出机械停稳与舵机 5V 电源建立时间 */
#define AUTO_PROBE_SETTLE_MS 300U

/* 循迹行驶期间把云台舵机锁定并保持供电在中位：1 = 启用（推荐，保证 <15cm 停车测距方向始终朝前） */
#define TRACK_SERVO_CENTER 1

/* 单面打靶台架测试：1 = 只打一面、无激光、用核心板 LED 闪烁表示完成；
 * 0 = 正常多面流程（面数/激光/扫掠由 probe_* 参数决定，Gate 0 与 M4 用这个）。 */
#define PROBE_TEST_ONE_FACE 0
#define PROBE_TEST_LASER_MS 10000U  /* 锁定目标后的保持/模拟发射时间（LED 闪烁） */
#define PROBE_TEST_BLINK_MS 10000U  /* 完成后 LED 继续闪烁 10 s，期间保持瞄准 */
#define PROBE_TEST_SWEEP_US 600U    /* 单侧扫掠半幅：1500±600 = 900..2100us */
#define PROBE_TEST_AIM_MS 2000U     /* 大角度扫掠给足时间，含起始端稳定 150ms */

/* ============================================================================
 * 1. 全局控制器与调试器 Live Watch 实时观测结构体
 * ============================================================================ */

/** @brief 智能小车核心控制器全局实例 */
LfController g_lf;

/**
 * @brief 供 VS Code ST-LINK 调试器在 Live Watch 中直接触发动作的请求变量
 * @note  在调试器 Watch 窗口向 g_debug_request 写入：
 *        1 = 启动 8s 红外校准 | 2 = 启动循迹 | 3 = 立即停止 | 4 = 启动四面探测
 *        5 = 启动 5 轮架空电机标定 (mcal air) | 6 = 启动 5 轮地面电机标定 (mcal ground)
 */
volatile uint32_t g_debug_request;

/**
 * @brief 供 ST-LINK Live Watch 实时无阻塞观测的全局状态快照（每 5 ms 自动刷新）
 */
typedef struct {
    uint32_t ms, state, fault, bench, uart_drops;
    uint16_t raw[4], black[4], white[4], obstacle_mm, probe_face;
    float darkness[4], position, confidence, speed[2], target[2], duty[2];
    float wheel_deadband[2], wheel_free_speed[2];
    float distance_mm, heading_deg;
} DebugSnapshot;

volatile DebugSnapshot g_debug;
static bool bench_active, vtest_active, outtest_active, outtest_led, telemetry = true;
static bool scan_active, move_active, move_is_turn, probe_log_en;
static uint16_t telemetry_ms = 25U; /* 默认 25 ms (40 Hz) 高密度遥测；可用 log 10..1000 在线调整 */
static float bench_duty[2], vtest_target[2], vtest_duty[2], vtest_sum_speed[2];
static uint32_t bench_until, vtest_since, vtest_until, vtest_samples, outtest_until;
static uint16_t scan_cur_us, scan_end_us, scan_step_us, scan_dwell_ms, scan_best_us, scan_best_mm;
static uint32_t scan_step_since;
static float move_target, move_speed, move_start_mm, move_start_rad;
static uint32_t move_since, move_timeout_ms;
static LfOutput outtest_output;
/* 舵机连续往返扫掠台架测试：servosweep [period_ms] / servosweep stop */
static bool servo_sweep_active;
static uint32_t servo_sweep_since;
static uint16_t servo_sweep_period_ms = 2000U;
#if PROBE_TEST_ONE_FACE
static uint32_t probe_done_blink_until;
#endif
/* 打靶结束后保持侧视角的时间戳 (ms)：0=不保持（与 PROBE_TEST_ONE_FACE 无关） */
static uint32_t probe_hold_until;

static void clear_aux_modes(void) {
    bench_active = false; vtest_active = false; outtest_active = false;
    outtest_led = false; scan_active = false; move_active = false;
    servo_sweep_active = false;
}

/* ============================================================================
 * 2. 顶层动作封装与纯硬件 PID 辅助诊断报告
 * ============================================================================ */

/** @brief 立即停止一切测试或循迹任务 */
static void stop(uint32_t now) {
    clear_aux_modes();
    lf_stop(&g_lf, now);
}

/** @brief 启动 8 秒四路红外传感器黑白扫描校准 */
static void calibrate(uint32_t now) {
    clear_aux_modes(); board_emergency_off(); lf_calibrate(&g_lf, now);
    board_log("CAL: motors off; sweep ALL FOUR sensors over black and white for 8 s.\r\n");
}

/** @brief 启动左右轮电机死区与全量程转速 5 轮自动扫描标定（支持空载 air 与地面 ground 两种模式） */
static void calibrate_motors(LfMcalMode mode, uint8_t passes, uint32_t now) {
    clear_aux_modes();
    if (lf_calibrate_motors(&g_lf, mode, passes, now)) {
        board_log("MCAL_START,mode=%s,passes=%u,steps=%u\r\n",
                  mode == LF_MCAL_GROUND ? "GROUND" : "AIR",
                  (unsigned)g_lf.mcal_total_passes, (unsigned)g_lf.mcal_max_steps);
    } else {
        board_log("DENIED: stop and clear fault before running mcal.\r\n");
    }
}

/** @brief 启动 2 秒倒计时并发车循迹 */
static void start(uint32_t now) {
    clear_aux_modes();
    board_log(lf_start(&g_lf, now) ? "START: 2 s arming delay.\r\n" :
              "DENIED: stop/clear fault and calibrate first.\r\n");
}

/** @brief 启动顺时针四面障碍物目标探测与 10 秒激光打靶 */
static void start_probe(uint32_t now) {
#if PROBE_TEST_ONE_FACE
    /* 台架测试：强制单面，缩短模拟发射时间。参数在 Flash 恢复之后覆盖。 */
    g_lf.cfg.probe.faces = 1U;
    g_lf.cfg.probe.laser_ms = PROBE_TEST_LASER_MS;
    g_lf.cfg.probe.sweep_us = PROBE_TEST_SWEEP_US;
    g_lf.cfg.probe.aim_ms = PROBE_TEST_AIM_MS;
#endif
    clear_aux_modes();
    if (lf_start_probe(&g_lf, now)) {
        board_log("PROBE: starting %u-face %s target probing (scan=%c, laser=%ums, aim=%ums, edge=%.0f/%.0fmm, turn=%.1fdeg).\r\n",
                  (unsigned)g_lf.cfg.probe.faces,
                  g_lf.cfg.probe.scan_mode != 0U
                      ? (g_lf.cfg.probe.side_left ? "CCW-side" : "CW-side")
                      : (g_lf.cfg.probe.clockwise ? "CW" : "CCW"),
                  g_lf.cfg.probe.scan_mode != 0U ? 'B' : 'A',
                  (unsigned)g_lf.cfg.probe.laser_ms, (unsigned)g_lf.cfg.probe.aim_ms,
                  (double)g_lf.cfg.probe.edge1_mm, (double)g_lf.cfg.probe.edge2_mm,
                  (double)g_lf.cfg.probe.turn_deg);
        board_log("PROBE_MODE,scan=%c,side=%c,us=%u\r\n",
                  g_lf.cfg.probe.scan_mode != 0U ? 'B' : 'A',
                  g_lf.cfg.probe.side_left ? 'L' : 'R', (unsigned)g_lf.cfg.probe.side_us);
    } else {
        board_log("DENIED: must be in OBSTACLE_STOP or STOPPED with no fault.\r\n");
    }
}

/**
 * @brief 台架侧扫（M2 验收用）：跳过 AIM/避障流程，直接 SIDE_LOCK -> SIDE_SCAN -> SIDE_MOVE -> LASER
 * @param push true = 手推模式（电机断电，人工沿面推车），对应《问题排查手册》§17.12 第 7 步
 * @note  车必须由人工摆成与目标面平行、云台轴距面约 180 mm；盒子在车的 probe_side_left 指定一侧。
 */
static void start_side_probe(uint32_t now, bool push) {
    clear_aux_modes();
    if (lf_start_side_probe(&g_lf, now, push)) {
        board_log("SIDE: starting side-view bench scan (push=%u, us=%u, spd=%.0fmm/s, reach=%.0fmm, verify=%u).\r\n",
                  (unsigned)(push ? 1U : 0U), (unsigned)g_lf.cfg.probe.side_us,
                  (double)g_lf.cfg.probe.side_speed_mm_s, (double)g_lf.cfg.probe.side_reach_mm,
                  (unsigned)(g_lf.cfg.probe.side_verify ? 1U : 0U));
        board_log("PROBE_MODE,scan=B,side=%c,us=%u\r\n",
                  g_lf.cfg.probe.side_left ? 'L' : 'R', (unsigned)g_lf.cfg.probe.side_us);
    } else {
        board_log("SIDE DENIED (need STOPPED/OBSTACLE_STOP, no fault, and probe_scan_mode=1).\r\n");
    }
}

/**
 * @brief 台架绕行（M3 验收用）：用上一次 side 扫描的 w/kind/h 跑一段 LEG -> 下一面 SIDE_LOCK/SCAN/MOVE
 * @note  必须先 side（或 side push）扫到中点，否则 face.w/side_h 无效，命令会被拒绝。
 */
static void start_leg_probe(uint32_t now) {
    clear_aux_modes();
    if (lf_start_leg_probe(&g_lf, now)) {
        board_log("LEG: starting one-leg bench (w=%.1f,h=%.1f,kind=%u,pan=%.0f).\r\n",
                  (double)g_lf.face.w, (double)g_lf.side_h, (unsigned)g_lf.leg_kind,
                  (double)g_lf.cfg.probe.pan_x_mm);
        board_log("PROBE_MODE,scan=B,side=%c,us=%u\r\n",
                  g_lf.cfg.probe.side_left ? 'L' : 'R', (unsigned)g_lf.cfg.probe.side_us);
    } else {
        board_log("LEG DENIED (need STOPPED/OBSTACLE_STOP, no fault, scan_mode=1, and a prior side scan with w/h)\r\n");
    }
}

/** @brief 将上电 HC-04 波特率探测结果枚举转换为可读标签 */
static const char *bt_probe_label(uint8_t probe_state) {
    switch (probe_state) {
    case 1U: return "115200_OK";
    case 2U: return "9600->115200_UPGRADED";
    case 3U: return "9600_FALLBACK";
    default: return "NO_AT(PASSTHRU_OR_TTL)";
    }
}

/** @brief 返回当前正在生效用于循迹前馈的电机标定数据集名称 */
static const char *active_mcal_name(void) {
    if (!g_lf.mcal_valid) return "DEFAULT";
    return g_lf.mcal_active_mode == LF_MCAL_GROUND ? "GROUND" : "AIR_REF";
}

/** @brief 打印当前控制器状态、PID 参数、左右轮死区补偿参数、1:1硬件仿真状态与红外标定值 */
static void status(void) {
    board_log("STATE=%s FAULT=%s CAL=%u MCAL=%u(%s gnd=%u air=%u) HW_EMU=%u speed=%.1f kp=%.2f kd=%.3f k3=%.2f speed_loop=%u reverse=%u brake_pwm=%u obs=%u(%umm)\r\n",
        lf_state_name(g_lf.state), lf_fault_name(g_lf.fault), (unsigned)g_lf.cal.valid,
        (unsigned)g_lf.mcal_valid, active_mcal_name(),
        (unsigned)g_lf.mcal_ground_valid, (unsigned)g_lf.mcal_air_valid,
        (unsigned)g_lf.cfg.hw.enable,
        (double)g_lf.cfg.cruise_mm_s, (double)g_lf.cfg.line_kp,
        (double)g_lf.cfg.line_kd, (double)g_lf.cfg.line_k3,
        (unsigned)g_lf.cfg.speed_loop, (unsigned)g_lf.cfg.allow_reverse,
        (unsigned)g_lf.cfg.drive_brake, (unsigned)g_lf.cfg.obstacle_enable,
        (unsigned)g_lf.cfg.obstacle_stop_mm);
    board_log("MOTOR db_L=%.3f db_R=%.3f (diff=%+.3f) fs_L=%.1f fs_R=%.1f mm/s | BT=%s PC10=%c drops=%lu\r\n",
        (double)g_lf.cfg.wheel_deadband[0], (double)g_lf.cfg.wheel_deadband[1],
        (double)(g_lf.cfg.wheel_deadband[0] - g_lf.cfg.wheel_deadband[1]),
        (double)g_lf.cfg.wheel_free_speed[0], (double)g_lf.cfg.wheel_free_speed[1],
        bt_probe_label(board_bt_probe_status()), board_bt_status_pin() ? 'H' : 'L',
        (unsigned long)board_uart_drops());
    board_log("HW_SIM netlist=%u vref=%.3f v5v=%.3f vmid=%.3f trim=%+.3f kp_stg=%.2f R60=%.1fk kd=%.2fms R61=%.1fk tau1=%.2fms tau2=%.2fms Vbase=%.3fV I=%u coast=%u\r\n",
        (unsigned)g_lf.cfg.hw.netlist_mode, (double)g_lf.cfg.hw.adc_vref_v,
        (double)g_lf.cfg.hw.v_5v, (double)g_lf.cfg.hw.v_mid, (double)g_lf.cfg.hw.e_trim_v,
        (double)g_lf.cfg.hw.kp_stage, (double)g_lf.cfg.hw.r60_kohm,
        (double)g_lf.cfg.hw.kd_stage_ms, (double)g_lf.cfg.hw.r61_kohm,
        (double)g_lf.cfg.hw.d_tau1_ms, (double)g_lf.cfg.hw.d_tau2_ms,
        (double)g_lf.cfg.hw.v_base, (unsigned)g_lf.cfg.hw.i_enable,
        (unsigned)g_lf.cfg.hw.coast_mode);
    board_log("HW_TRI min=%.3fV max=%.3fV tri_hz=%.0f pwm_hz=%lu atten=%.3f spd_tau=%.3fms | R_U6A=%.0f/%.0f/%.0f/%.0f R43=%.0f R45=%.0f(%s) Rfb=%.0f Rsrc=%.1f\r\n",
        (double)g_lf.cfg.hw.tri_min_v, (double)g_lf.cfg.hw.tri_max_v,
        (double)g_lf.cfg.hw.tri_freq_hz, (unsigned long)board_get_pwm_hz(),
        (double)g_lf.cfg.hw.ana_atten, (double)g_lf.cfg.hw.speed_tau_ms,
        (double)g_lf.cfg.hw.r39_kohm, (double)g_lf.cfg.hw.r41_kohm,
        (double)g_lf.cfg.hw.r40_kohm, (double)g_lf.cfg.hw.r42_kohm,
        (double)g_lf.cfg.hw.r43_kohm, (double)g_lf.cfg.hw.r45_kohm,
        g_lf.cfg.hw.r45_to_5v ? "5V" : "MID",
        (double)g_lf.cfg.hw.r_fb_kohm, (double)g_lf.cfg.hw.r_ir_src_kohm);
    for (unsigned i = 0; i < 4; ++i)
        board_log("CAL %u black=%u white=%u raw=%u\r\n", i,
            (unsigned)g_lf.cal.black[i], (unsigned)g_lf.cal.white[i], (unsigned)g_lf.raw[i]);
}

/** @brief 将纯硬件模拟 PID 仿真模式下调试好的等效参数 1:1 翻译为扩展板物理元件值/电位器位置 */
static void print_hwmap(void) {
    const LfHwPidConfig *h = &g_lf.cfg.hw;
    float kp_tot = (20.0f / h->r60_kohm) * h->kp_stage;
    float rvp_k = (h->kp_stage - 1.0f) * 10.0f;
    float tri_span = h->tri_max_v - h->tri_min_v;
    float raw0 = h->v_mid + 2.0f * (h->v_base - h->v_mid);
    float d0_pct = ((h->ana_atten * raw0 - h->tri_min_v) / tri_span) * 100.0f;
    if (d0_pct < 0.0f) d0_pct = 0.0f;
    if (d0_pct > 100.0f) d0_pct = 100.0f;
    float d_stock_rvd = h->kd_stage_ms / 0.1f - 4.7f;
    float d_470n_rvd = h->kd_stage_ms / 0.47f - 4.7f;
    float v_bot = h->v_5v * (6.2f / 22.4f);
    float v_top = h->v_5v * (16.2f / 22.4f);
    float rvv_pct = (h->v_base - v_bot) / (v_top - v_bot) * 100.0f;
    if (h->netlist_mode) {
        board_log("HWMAP_IR: netlist R39=%.1fk R41=%.1fk R40=%.1fk R42=%.1fk R43=%.1fk R45=%.1fk(%s) Rfb=%.1fk Rsrc=%.1fk trim=%+.3fV\r\n",
            (double)h->r39_kohm, (double)h->r41_kohm, (double)h->r40_kohm, (double)h->r42_kohm,
            (double)h->r43_kohm, (double)h->r45_kohm, h->r45_to_5v ? "PWM_5V" : "PWM_MID",
            (double)h->r_fb_kohm, (double)h->r_ir_src_kohm, (double)h->e_trim_v);
    } else {
        float r39_k = 50.0f / h->w_outer;
        float r41_k = 50.0f / h->w_inner;
        board_log("HWMAP_IR: sym w_out=%.3f w_in=%.3f -> keep R44||R46=50k; R39/R40=%.1fk, R41/R42=%.1fk\r\n",
            (double)h->w_outer, (double)h->w_inner, (double)r39_k, (double)r41_k);
    }
    board_log("HWMAP_P : kp_total=%.2f (R60=%.1fk, kp_stg=%.2f -> RV_P=%.1fk / 100k; %s)\r\n",
        (double)kp_tot, (double)h->r60_kohm, (double)h->kp_stage, (double)rvp_k,
        rvp_k < 0 ? "NEED R60>20k!" : (rvp_k > 100 ? "NEED R60<20k!" : "RV_P IN RANGE"));
    board_log("HWMAP_D : kd=%.2fms R61=%.1fk tau1=%.2fms tau2=%.2fms | 100nF->RV_D=%.1fk | 470nF->RV_D=%.1fk (%s)\r\n",
        (double)h->kd_stage_ms, (double)h->r61_kohm, (double)h->d_tau1_ms, (double)h->d_tau2_ms,
        (double)d_stock_rvd, (double)d_470n_rvd,
        d_470n_rvd < 0 ? "BELOW 2.2ms" : (d_470n_rvd > 100 ? ">100k ON 470nF" : "470nF IN RANGE"));
    board_log("HWMAP_VB: Vbase=%.3fV (Vmid=%.3fV V5V=%.3fV TRI=[%.3f..%.3f]@%.0fHz) -> D0=%.1f%% | JP_V=%s | RV_V(2-3)=%.0f%% | JP_I=%s | DRV=%s\r\n",
        (double)h->v_base, (double)h->v_mid, (double)h->v_5v,
        (double)h->tri_min_v, (double)h->tri_max_v, (double)h->tri_freq_hz,
        (double)d0_pct, h->v_base < h->v_mid - 0.02f ? "2-3(R68=6.2k)" : "2-3 or 1-2",
        (double)rvv_pct, h->i_enable ? "INSTALL" : "REMOVE",
        h->coast_mode ? "STOCK_U4_COAST(IN2=0)" : "MOD_SLOW_DECAY");
}

static const char *vref_mode_label(uint8_t mode) {
    switch (mode) {
    case 1U: return "EXT_3V3";
    case 2U: return "INT_VREFBUF_2V9";
    default: return "FALLBACK";
    }
}

/**
 * @brief 蓝牙辅助诊断报告 (diag)：同时输出软件参数与纯硬件模拟 PID 等效电压/电机不对称度
 */
static void print_diag(void) {
    const LfHwPidConfig *h = &g_lf.cfg.hw;
    float vref = h->adc_vref_v > 0.5f ? h->adc_vref_v : board_adc_vref_v();
    float v[4];
    for (unsigned i = 0; i < 4; ++i) v[i] = (float)g_lf.raw[i] * (vref / 4095.0f);
    float wp = 0.0f, pid_e = 0.0f, vp_off = 0.0f, pid_c = 0.0f, spd[2] = {0}, dut[2] = {0};
    lf_hw_eval_static(&g_lf, &wp, &pid_e, &vp_off, &pid_c, spd, dut);
    float e_ana = pid_e - h->v_mid;
    float tri_span = h->tri_max_v - h->tri_min_v;
    float raw0 = h->v_mid + 2.0f * (h->v_base - h->v_mid);
    float d0 = (h->ana_atten * raw0 - h->tri_min_v) / tri_span;
    if (d0 < 0.0f) d0 = 0.0f;
    if (d0 > g_lf.cfg.max_duty) d0 = g_lf.cfg.max_duty;
    float vl_est = d0 > g_lf.cfg.wheel_deadband[0] ?
        (d0 - g_lf.cfg.wheel_deadband[0]) / (1.0f - g_lf.cfg.wheel_deadband[0]) * g_lf.cfg.wheel_free_speed[0] : 0;
    float vr_est = d0 > g_lf.cfg.wheel_deadband[1] ?
        (d0 - g_lf.cfg.wheel_deadband[1]) / (1.0f - g_lf.cfg.wheel_deadband[1]) * g_lf.cfg.wheel_free_speed[1] : 0;
    board_log("DIAG IR_V(VREF=%.3fV/%s): L2=%.3fV L1=%.3fV R1=%.3fV R2=%.3fV | WEIGHT_P=%.3fV HW_PID_E=%.3fV (e=%+.3fV)\r\n",
        (double)vref, vref_mode_label(board_adc_vref_mode()),
        (double)v[0], (double)v[1], (double)v[2], (double)v[3],
        (double)wp, (double)pid_e, (double)e_ana);
    board_log("DIAG HW_STG(net=%u V5=%.3f Vmid=%.3f): V_P=%+.3fV PID_C=%.3fV | ANA_SPD=(%.3fV,%.3fV) -> d=(%.1f%%,%.1f%%) @ %.0fHz | sw_pos=%+.3f\r\n",
        (unsigned)h->netlist_mode, (double)h->v_5v, (double)h->v_mid,
        (double)vp_off, (double)pid_c,
        (double)spd[0], (double)spd[1], (double)(dut[0] * 100.0f), (double)(dut[1] * 100.0f),
        (double)h->tri_freq_hz, (double)g_lf.position);
    board_log("DIAG MOTOR(%s): db_L=%.3f db_R=%.3f (diff=%+.3f) | At D0=%.1f%%: vL_est=%.1f vR_est=%.1f mm/s (diff=%+.1f)\r\n",
        active_mcal_name(),
        (double)g_lf.cfg.wheel_deadband[0], (double)g_lf.cfg.wheel_deadband[1],
        (double)(g_lf.cfg.wheel_deadband[0] - g_lf.cfg.wheel_deadband[1]),
        (double)(d0 * 100.0f), (double)vl_est, (double)vr_est, (double)(vl_est - vr_est));
}

/**
 * @brief 蓝牙 I/O 自检诊断 (io)：实时读取三路按键引脚原始高低电平、故障与编码器计数值
 */
static void print_io(void) {
    BoardIoSnapshot io = {0};
    board_read_io(&io);
    board_log("IO_IN,PD0=%c(track=%u),PD1=%c(probe=%u),PD15=%c(core=%u),PC4_nFAULT=%c,PC10_BT=%c,PC6_ECHO=%c,VREF=%.3fV(%s)\r\n",
        io.key_track_pin ? 'H' : 'L', (unsigned)board_track_pressed(),
        io.key_probe_pin ? 'H' : 'L', (unsigned)board_probe_pressed(),
        io.key_core_pin ? 'H' : 'L', (unsigned)board_core_pressed(),
        io.drv_nfault_pin ? 'H' : 'L',
        io.bt_status_pin ? 'H' : 'L',
        io.us_echo_pin ? 'H' : 'L',
        (double)board_adc_vref_v(), vref_mode_label(board_adc_vref_mode()));
    board_log("IO_ENC,TIM4_L=%u,TIM3_R=%u,ADC=%u,%u,%u,%u\r\n",
        (unsigned)io.enc_left_cnt, (unsigned)io.enc_right_cnt,
        (unsigned)io.raw_adc[0], (unsigned)io.raw_adc[1],
        (unsigned)io.raw_adc[2], (unsigned)io.raw_adc[3]);
    /* 舵机/云台硬件自检：TIM2 是否在计数、CCR1 脉宽、PA15 复用、PC7 电源使能 */
    board_log("IO_SERVO,TIM2_CR1=0x%04X,CNT=%u,CCR1=%u,ARR=%u,PA15_MODER=%u,PA15_AF=%u,PC7_ODR=%u\r\n",
        (unsigned)TIM2->CR1, (unsigned)TIM2->CNT, (unsigned)TIM2->CCR1,
        (unsigned)TIM2->ARR,
        (unsigned)((GPIOA->MODER >> 30) & 3U),
        (unsigned)((GPIOA->AFR[1] >> 28) & 0xFU),
        (unsigned)((GPIOC->ODR >> 7) & 1U));
    board_log("IO_NOTE: Keys idle=H(PD0=3.185V,PD1/PD15=3.3V), press=L(0V). Outputs require J_C2.6=GND -> U3.2(MODE_MCU=5V).\r\n");
}

/**
 * @brief 打印超声波、舵机、编码器里程计与四面打靶参数实时状态 (us 命令)
 */
static void print_us(void) {
    const LfProbeConfig *pr = &g_lf.cfg.probe;
    board_log("US_ODOM,obs_mm=%u,obs_ok=%u,servo_center=%u,dist_mm=%.1f,hdg_deg=%.2f,x_mm=%.1f,y_mm=%.1f,vL=%.1f,vR=%.1f\r\n",
        (unsigned)g_lf.last_obstacle_mm, (unsigned)g_lf.last_obstacle_valid,
        (unsigned)g_lf.cfg.servo_center_us,
        (double)g_lf.odo.distance_mm, (double)(g_lf.odo.heading_rad * 57.2957795f),
        (double)g_lf.odo.x_mm, (double)g_lf.odo.y_mm,
        (double)g_lf.measured_mm_s[0], (double)g_lf.measured_mm_s[1]);
    board_log("PROBE_CFG,cw=%u,faces=%u,aim_ms=%u,sweep_us=%u,laser_ms=%u,turn_deg=%.1f,edge1=%.0f,edge2=%.0f,approach=%.0f,spd=%.0f/%.0f/%.0f\r\n",
        (unsigned)pr->clockwise, (unsigned)pr->faces, (unsigned)pr->aim_ms, (unsigned)pr->sweep_us,
        (unsigned)pr->laser_ms, (double)pr->turn_deg, (double)pr->edge1_mm, (double)pr->edge2_mm,
        (double)pr->approach_max_mm, (double)pr->turn_speed_mm_s, (double)pr->drive_speed_mm_s,
        (double)pr->approach_speed_mm_s);
    board_log("PROBE_SIDE_CFG,scan=%u,left=%u,us=%u,spd=%.0f,reach=%.0f,band=%.1f,jump=%.1f,verify=%u,gamma=%.1f,pan_x=%.0f,fit_rms=%.1f\r\n",
        (unsigned)pr->scan_mode, (unsigned)pr->side_left, (unsigned)pr->side_us,
        (double)pr->side_speed_mm_s, (double)pr->side_reach_mm, (double)pr->side_band_mm,
        (double)pr->side_jump_mm, (unsigned)pr->side_verify, (double)pr->beam_half_deg,
        (double)pr->pan_x_mm, (double)pr->fit_rms_mm);
}

/* ============================================================================
 * 3. 在线参数修改与蓝牙/上位机串口命令行解析器
 * ============================================================================ */

typedef struct { const char *name; size_t offset; float minimum, maximum; } Parameter;
#define PARAM(name, field, lo, hi) {name, offsetof(LfConfig, field), lo, hi}
static const Parameter parameters[] = {
    PARAM("kp", line_kp, 0, 400), PARAM("ki", line_ki, 0, 100), PARAM("kd", line_kd, 0, 10),
    PARAM("k3", line_k3, 0, 3.0f), PARAM("d_tau", derivative_tau_s, .003f, .2f),
    PARAM("speed", cruise_mm_s, 20, 500), PARAM("curve_min", curve_min_mm_s, 10, 300),
    PARAM("max_wheel", max_wheel_mm_s, 50, 700), PARAM("max_turn", max_turn_mm_s, 20, 500),
    PARAM("accel", acceleration_mm_s2, 50, 5000), PARAM("slew", duty_slew_s, .5f, 20.0f),
    PARAM("max_duty", max_duty, .1f, .95f), PARAM("speed_kp", speed_kp, 0, .01f),
    PARAM("speed_ki", speed_ki, 0, .03f), PARAM("free_speed", free_speed_mm_s, 100, 1500),
    PARAM("deadband", feedforward_deadband, 0, .4f), PARAM("cpr", encoder_counts_per_rev, 20, 100000),
    PARAM("wheel_mm", wheel_diameter_mm, 10, 150), PARAM("track_mm", track_width_mm, 40, 300),
    PARAM("hw_w_out", hw.w_outer, .02f, 3.0f), PARAM("hw_w_in", hw.w_inner, .01f, 3.0f),
    PARAM("hw_adc_vref", hw.adc_vref_v, 2.0f, 3.6f), PARAM("hw_v5v", hw.v_5v, 4.2f, 5.6f),
    PARAM("hw_vmid", hw.v_mid, 1.8f, 3.2f), PARAM("hw_e_trim", hw.e_trim_v, -1.5f, 1.5f),
    PARAM("hw_r39", hw.r39_kohm, 1.0f, 2000.0f), PARAM("hw_r41", hw.r41_kohm, 1.0f, 2000.0f),
    PARAM("hw_r40", hw.r40_kohm, 1.0f, 2000.0f), PARAM("hw_r42", hw.r42_kohm, 1.0f, 2000.0f),
    PARAM("hw_r43", hw.r43_kohm, 1.0f, 2000.0f), PARAM("hw_r45", hw.r45_kohm, 1.0f, 2000.0f),
    PARAM("hw_r_fb", hw.r_fb_kohm, 1.0f, 2000.0f), PARAM("hw_r_ir", hw.r_ir_src_kohm, 0.0f, 20.0f),
    PARAM("hw_kp", hw.kp_stage, .2f, 20.0f), PARAM("hw_r60", hw.r60_kohm, 1.0f, 1000.0f),
    PARAM("hw_kd_ms", hw.kd_stage_ms, 0.0f, 500.0f), PARAM("hw_r61", hw.r61_kohm, 1.0f, 5000.0f),
    PARAM("hw_tau1_ms", hw.d_tau1_ms, .05f, 100.0f), PARAM("hw_tau2_ms", hw.d_tau2_ms, .01f, 50.0f),
    PARAM("hw_ti_ms", hw.ti_ms, 20.0f, 5000.0f), PARAM("hw_v_base", hw.v_base, 1.0f, 4.5f),
    PARAM("hw_atten", hw.ana_atten, .1f, 1.0f), PARAM("hw_spd_tau_ms", hw.speed_tau_ms, .01f, 20.0f),
    PARAM("hw_tri_min", hw.tri_min_v, .05f, 2.4f), PARAM("hw_tri_max", hw.tri_max_v, 2.5f, 5.2f),
    PARAM("hw_tri_hz", hw.tri_freq_hz, 4000.0f, 60000.0f),
    PARAM("probe_turn_deg", probe.turn_deg, 30.0f, 180.0f),
    PARAM("probe_edge1_mm", probe.edge1_mm, 30.0f, 1500.0f),
    PARAM("probe_edge2_mm", probe.edge2_mm, 30.0f, 1500.0f),
    PARAM("probe_approach_mm", probe.approach_max_mm, 30.0f, 1000.0f),
    PARAM("probe_turn_spd", probe.turn_speed_mm_s, 20.0f, 300.0f),
    PARAM("probe_drive_spd", probe.drive_speed_mm_s, 20.0f, 400.0f),
    PARAM("probe_approach_spd", probe.approach_speed_mm_s, 20.0f, 300.0f),
    /* M2 侧视横扫（float 参数统一走 offsetof 表；uint8/bool 在下面单独处理） */
    PARAM("probe_fit_rms_mm", probe.fit_rms_mm, 2.0f, 20.0f),
    PARAM("probe_side_spd_mm_s", probe.side_speed_mm_s, 20.0f, 100.0f),
    PARAM("probe_side_reach_mm", probe.side_reach_mm, 80.0f, 400.0f),
    PARAM("probe_side_band_mm", probe.side_band_mm, 4.0f, 30.0f),
    PARAM("probe_side_jump_mm", probe.side_jump_mm, 10.0f, 80.0f),
    PARAM("probe_beam_half_deg", probe.beam_half_deg, 2.0f, 15.0f),
    PARAM("probe_pan_x_mm", probe.pan_x_mm, 0.0f, 120.0f)
};

static bool set_parameter(const char *name, float value) {
    if (!isfinite(value)) return false;
    LfConfig candidate = g_lf.cfg;
    bool known = false;
    for (unsigned i = 0; i < sizeof(parameters) / sizeof(parameters[0]); ++i) {
        const Parameter *p = &parameters[i];
        if (!strcmp(name, p->name)) {
            if (value < p->minimum || value > p->maximum) return false;
            *(float *)((char *)&candidate + p->offset) = value;
            if (!strcmp(name, "deadband"))
                candidate.wheel_deadband[0] = candidate.wheel_deadband[1] = value;
            if (!strcmp(name, "free_speed"))
                candidate.wheel_free_speed[0] = candidate.wheel_free_speed[1] = value;
            known = true; break;
        }
    }
    /* 支持单独设置左/右轮死区 (db_l, db_r) 与左/右轮满占空比转速 (fs_l, fs_r) */
    if (!strcmp(name, "db_l") || !strcmp(name, "db_r")) {
        if (value < 0 || value > .45f) return false;
        candidate.wheel_deadband[!strcmp(name, "db_r") ? 1 : 0] = value;
        candidate.feedforward_deadband = .5f * (candidate.wheel_deadband[0] + candidate.wheel_deadband[1]);
        known = true;
    }
    if (!strcmp(name, "fs_l") || !strcmp(name, "fs_r")) {
        if (value < 100 || value > 1500) return false;
        candidate.wheel_free_speed[!strcmp(name, "fs_r") ? 1 : 0] = value;
        candidate.free_speed_mm_s = .5f * (candidate.wheel_free_speed[0] + candidate.wheel_free_speed[1]);
        known = true;
    }
    if (!strcmp(name, "speed_loop") || !strcmp(name, "reverse") ||
        !strcmp(name, "brake_pwm") || !strcmp(name, "obstacle") ||
        !strcmp(name, "hw_emu") || !strcmp(name, "hw_i") || !strcmp(name, "hw_coast") ||
        !strcmp(name, "hw_netlist") || !strcmp(name, "hw_r45_5v")) {
        if (value != 0 && value != 1) return false;
        if (!strcmp(name, "speed_loop")) candidate.speed_loop = value != 0;
        if (!strcmp(name, "reverse")) candidate.allow_reverse = value != 0;
        if (!strcmp(name, "brake_pwm")) candidate.drive_brake = value != 0;
        if (!strcmp(name, "obstacle")) candidate.obstacle_enable = value != 0;
        if (!strcmp(name, "hw_emu")) candidate.hw.enable = value != 0;
        if (!strcmp(name, "hw_i")) candidate.hw.i_enable = value != 0;
        if (!strcmp(name, "hw_coast")) candidate.hw.coast_mode = value != 0;
        if (!strcmp(name, "hw_netlist")) candidate.hw.netlist_mode = value != 0;
        if (!strcmp(name, "hw_r45_5v")) candidate.hw.r45_to_5v = value != 0;
        known = true;
    }
    if (!strcmp(name, "enc_l") || !strcmp(name, "enc_r") || !strcmp(name, "motor_l") || !strcmp(name, "motor_r")) {
        if (value != -1 && value != 1) return false;
        if (!strcmp(name, "enc_l")) candidate.encoder_sign[0] = (int8_t)value;
        if (!strcmp(name, "enc_r")) candidate.encoder_sign[1] = (int8_t)value;
        if (!strcmp(name, "motor_l")) candidate.motor_sign[0] = (int8_t)value;
        if (!strcmp(name, "motor_r")) candidate.motor_sign[1] = (int8_t)value;
        known = true;
    }
    if (!strcmp(name, "search_ms") || !strcmp(name, "lost_brake_ms") ||
        !strcmp(name, "cross_ms") || !strcmp(name, "obs_mm") || !strcmp(name, "servo_us") ||
        !strcmp(name, "probe_cw") || !strcmp(name, "probe_faces") || !strcmp(name, "probe_aim_ms") ||
        !strcmp(name, "probe_sweep_us") || !strcmp(name, "probe_step_us") || !strcmp(name, "probe_laser_ms") ||
        !strcmp(name, "probe_aim_min") || !strcmp(name, "probe_aim_max") || !strcmp(name, "probe_timeout_ms") ||
        !strcmp(name, "probe_scan_mode") || !strcmp(name, "probe_side_left") ||
        !strcmp(name, "probe_side_us") || !strcmp(name, "probe_side_verify")) {
        if (floorf(value) != value) return false;
        if (!strcmp(name, "search_ms") && value >= 0 && value <= 1000) { candidate.search_ms = (uint16_t)value; known = true; }
        if (!strcmp(name, "lost_brake_ms") && value >= 0 && value <= 500) { candidate.lost_brake_ms = (uint16_t)value; known = true; }
        if (!strcmp(name, "cross_ms") && value >= 20 && value <= 1000) { candidate.ambiguous_ms = (uint16_t)value; known = true; }
        if (!strcmp(name, "obs_mm") && value >= 50 && value <= 500) { candidate.obstacle_stop_mm = (uint16_t)value; known = true; }
        if (!strcmp(name, "servo_us") && value >= 900 && value <= 2100) { candidate.servo_center_us = (uint16_t)value; known = true; }
        if (!strcmp(name, "probe_cw") && (value == 0 || value == 1)) { candidate.probe.clockwise = (value != 0); known = true; }
        if (!strcmp(name, "probe_faces") && value >= 1 && value <= 4) { candidate.probe.faces = (uint8_t)value; known = true; }
        if (!strcmp(name, "probe_aim_ms") && value >= 200 && value <= 5000) { candidate.probe.aim_ms = (uint16_t)value; known = true; }
        if (!strcmp(name, "probe_sweep_us") && value >= 20 && value <= 600) { candidate.probe.sweep_us = (uint16_t)value; known = true; }
        if (!strcmp(name, "probe_step_us") && value >= 20 && value <= 200) { candidate.probe.scan_step_us = (uint16_t)value; known = true; }
        if (!strcmp(name, "probe_laser_ms") && value >= 200 && value <= 30000) { candidate.probe.laser_ms = (uint16_t)value; known = true; }
        if (!strcmp(name, "probe_aim_min") && value >= 20 && value <= 500) { candidate.probe.aim_min_mm = (uint16_t)value; known = true; }
        if (!strcmp(name, "probe_aim_max") && value >= 100 && value <= 2000) { candidate.probe.aim_max_mm = (uint16_t)value; known = true; }
        if (!strcmp(name, "probe_timeout_ms") && value >= 500 && value <= 15000) { candidate.probe.phase_timeout_ms = (uint16_t)value; known = true; }
        /* M2 侧视横扫开关类参数 */
        if (!strcmp(name, "probe_scan_mode") && (value == 0 || value == 1)) { candidate.probe.scan_mode = (uint8_t)value; known = true; }
        if (!strcmp(name, "probe_side_left") && (value == 0 || value == 1)) { candidate.probe.side_left = value != 0; known = true; }
        if (!strcmp(name, "probe_side_us") && value >= (float)LF_SERVO_US_MIN && value <= (float)LF_SERVO_US_MAX) {
            candidate.probe.side_us = (uint16_t)value; known = true;
        }
        if (!strcmp(name, "probe_side_verify") && (value == 0 || value == 1)) { candidate.probe.side_verify = value != 0; known = true; }
    }
    if (!known || !lf_config_valid(&candidate)) return false;
    g_lf.cfg = candidate;
    return true;
}

static bool is_stopped(void) {
    return (g_lf.state == LF_STOPPED || g_lf.state == LF_OBSTACLE_STOP || g_lf.state == LF_FAULT) &&
           !bench_active && !vtest_active && !outtest_active && !scan_active && !move_active &&
           !servo_sweep_active;
}

static void command(char *line, uint32_t now) {
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) return;
    if (!strcmp(cmd, "stop")) { stop(now); board_log("STOP\r\n"); return; }
    if (!strcmp(cmd, "status")) { status(); return; }
    if (!strcmp(cmd, "diag")) { print_diag(); return; }
    if (!strcmp(cmd, "hwmap")) { print_hwmap(); return; }
    if (!strcmp(cmd, "io")) { print_io(); return; }
    if (!strcmp(cmd, "us")) { print_us(); return; }
    if (!strcmp(cmd, "odom_reset")) { lf_odometry_reset(&g_lf); board_log("ODOM_RESET OK\r\n"); return; }
    if (!strcmp(cmd, "probe_log")) {
        char *v = strtok(NULL, " \t");
        if (v && !strcmp(v, "1")) probe_log_en = true;
        else if (v && !strcmp(v, "0")) probe_log_en = false;
        board_log("PROBE_LOG=%u\r\n", (unsigned)probe_log_en);
        return;
    }
    if (!strcmp(cmd, "lcdflip")) {
        char *v = strtok(NULL, " \t");
        bool flip = v && !strcmp(v, "1");
        board_lcd_flip(flip);
        board_log("LCDFLIP=%u (%s)\r\n", (unsigned)flip, flip ? "0x00_FLIP180" : "0xC0_UPRIGHT");
        return;
    }
    if (!strcmp(cmd, "help")) {
        board_log("cal | mcal [ground|air] [passes=5] | use_mcal [ground|air] | start | probe | stop | status | diag | hwmap | io | us | odom_reset | log 0/1 | probe_log 0/1\r\n");
        board_log("set kp/ki/kd/k3/speed/curve_min/max_turn/max_duty/speed_kp/speed_ki VALUE\r\n");
        board_log("set db_l/db_r/fs_l/fs_r VALUE; set speed_loop/reverse/brake_pwm/obstacle 0/1; enc_l/r motor_l/r -1/1\r\n");
        board_log("PROBE: set probe_cw/probe_faces/probe_aim_ms/probe_sweep_us/probe_step_us/probe_laser_ms/probe_turn_deg/probe_edge1_mm/probe_edge2_mm/probe_approach_mm V\r\n");
        board_log("SIDE:  set probe_scan_mode(0=A/1=B) probe_side_left probe_side_us probe_side_spd_mm_s probe_side_reach_mm\r\n");
        board_log("       set probe_side_band_mm probe_side_jump_mm probe_side_verify probe_beam_half_deg probe_pan_x_mm probe_fit_rms_mm V\r\n");
        board_log("SIDE-BENCH: side          (自走侧扫台架)  |  side push  (电机断电，人工推车扫)\r\n");
        board_log("LEG-BENCH:  leg            (用上一次 side 的 w/h 绕到下一面，M3 验收)\r\n");
        board_log("PROBE-CAL: scan [span_us=220] [step_us=20] [dwell_ms=140] | drive MM [SPD=120] | turn DEG [SPD=90] | outtest lamp/buz/laser/servo/led/stop VAL [MS]\r\n");
        board_log("SERVO: servosweep [period_ms=2000] (continuous 900..2100us) | servosweep stop | stop\r\n");
        board_log("HW-EMU: hw_preset stock|group_b; set hw_emu/hw_netlist/hw_r45_5v/hw_i/hw_coast 0/1; mcurve/vtest/bench/calset/defaults\r\n");
        return;
    }
    if (!strcmp(cmd, "log")) {
        char *v = strtok(NULL, " \t");
        if (!v) {
            board_log("LOG=%u period=%u ms\r\n", (unsigned)telemetry, (unsigned)telemetry_ms);
            return;
        }
        char *endp = NULL;
        unsigned long val = strtoul(v, &endp, 10);
        if (endp == v || *endp != 0) { board_log("LOG ERROR (0, 1, or 10..1000 ms)\r\n"); return; }
        if (val == 0UL) telemetry = false;
        else if (val == 1UL) telemetry = true;
        else if (val >= 10UL && val <= 1000UL) { telemetry_ms = (uint16_t)val; telemetry = true; }
        else board_log("LOG ERROR (0, 1, or 10..1000 ms)\r\n");
        return;
    }
    if (!strcmp(cmd, "start")) { start(now); return; }
    if (!strcmp(cmd, "probe")) { start_probe(now); return; }
    if (!strcmp(cmd, "side")) {
        /* 台架侧扫：side = 自走扫描；side push = 电机断电、人工推车扫（§17.12 第 7 步） */
        char *m = strtok(NULL, " \t");
        if (m && strcmp(m, "push")) { board_log("SIDE ERROR (usage: side | side push)\r\n"); return; }
        start_side_probe(now, m != NULL);
        return;
    }
    if (!strcmp(cmd, "leg")) { start_leg_probe(now); return; }
    if (!strcmp(cmd, "servosweep")) {
        /* 舵机连续往返台架测试：默认 2 s 一个完整 900->2100->900 周期，直到 stop 或 servosweep stop */
        char *v = strtok(NULL, " \t");
        if (v && !strcmp(v, "stop")) {
            servo_sweep_active = false;
            board_log("SERVOSWEEP STOP\r\n");
            return;
        }
        char *endp = NULL;
        unsigned long period = v ? strtoul(v, &endp, 10) : 2000UL;
        if ((v && (endp == v || *endp != 0)) || period < 500UL || period > 10000UL) {
            board_log("SERVOSWEEP ERROR (usage: servosweep [period_ms 500..10000] | servosweep stop)\r\n");
            return;
        }
        if (g_lf.fault != LF_OK) { board_log("SERVOSWEEP DENIED (fault)\r\n"); return; }
        if (g_lf.state != LF_STOPPED) lf_stop(&g_lf, now);
        clear_aux_modes();
        servo_sweep_period_ms = (uint16_t)period;
        servo_sweep_since = now;
        servo_sweep_active = true;
        board_log("SERVOSWEEP START period_ms=%lu range=900..2100us (send stop or servosweep stop)\r\n", period);
        return;
    }
    if (!strcmp(cmd, "outtest")) {
        /* 单路硬件输出点亮自检 (outtest lamp/buz/laser/servo/led/stop val [ms])，不驱动电机以保证台架安全 */
        char *ch = strtok(NULL, " \t");
        if (!ch || !strcmp(ch, "stop")) {
            outtest_active = false; outtest_led = false; servo_sweep_active = false;
            memset(&outtest_output, 0, sizeof(outtest_output));
            board_log("OUTTEST STOP\r\n");
            return;
        }
        if ((g_lf.state != LF_STOPPED && g_lf.state != LF_OBSTACLE_STOP) ||
            bench_active || vtest_active || g_lf.fault != LF_OK) {
            board_log("OUTTEST DENIED (must be STOPPED with no fault)\r\n");
            return;
        }
        char *v = strtok(NULL, " \t"), *d = strtok(NULL, " \t");
        char *ev = NULL, *ed = NULL;
        long val = v ? strtol(v, &ev, 10) : -1;
        unsigned long ms = d ? strtoul(d, &ed, 10) : 1000UL;
        const unsigned long ms_max = !strcmp(ch, "servo") ? 30000UL : 3000UL;
        if (!v || *ev != 0 || (d && *ed != 0) || ms < 100UL || ms > ms_max) {
            board_log("OUTTEST ERROR (usage: outtest lamp|buz|laser|led 0/1 [ms], outtest servo 600..2400 [ms])\r\n");
            return;
        }
        LfOutput next_out = {.servo_us = g_lf.cfg.servo_center_us};
        bool next_led = false;
        bool valid = true;
        if (!strcmp(ch, "lamp") && (val == 0 || val == 1)) next_out.park_lamp = (val == 1);
        else if (!strcmp(ch, "buz") && (val == 0 || val == 1)) next_out.buzzer = (val == 1);
        else if (!strcmp(ch, "laser") && (val == 0 || val == 1)) next_out.laser = (val == 1);
        else if (!strcmp(ch, "led") && (val == 0 || val == 1)) next_led = (val == 1);
        else if (!strcmp(ch, "servo") && val >= (long)LF_SERVO_US_MIN && val <= (long)LF_SERVO_US_MAX) {
            next_out.servo_enable = true; next_out.servo_us = (uint16_t)val;
        } else valid = false;
        if (!valid) { board_log("OUTTEST ERROR\r\n"); return; }
        if (g_lf.state != LF_STOPPED) lf_stop(&g_lf, now);
        clear_aux_modes();
        outtest_output = next_out;
        outtest_led = next_led;
        outtest_until = now + (uint32_t)ms;
        outtest_active = true;
        board_log("OUTTEST_START ch=%s val=%ld ms=%lu\r\n", ch, val, ms);
        return;
    }
    if (!is_stopped()) { board_log("DENIED: stop before changing parameters/calibration.\r\n"); return; }
    if (!strcmp(cmd, "cal")) { calibrate(now); return; }
    if (!strcmp(cmd, "use_mcal")) {
        char *m = strtok(NULL, " \t");
        LfMcalMode target = (m && !strcmp(m, "air")) ? LF_MCAL_AIR : LF_MCAL_GROUND;
        board_log(lf_select_mcal_mode(&g_lf, target) ? "USE_MCAL OK active=%s\r\n" : "USE_MCAL ERROR (%s not calibrated)\r\n",
                  target == LF_MCAL_GROUND ? "GROUND" : "AIR");
        return;
    }
    if (!strcmp(cmd, "mcal")) {
        /* 默认使用地面带载 (GROUND) 作为实车循迹主模型；支持 "mcal ground 5" / "mcal air 5" */
        LfMcalMode mode = LF_MCAL_GROUND;
        unsigned passes = 5U;
        char *arg1 = strtok(NULL, " \t");
        char *arg2 = strtok(NULL, " \t");
        if (arg1) {
            if (!strcmp(arg1, "ground") || !strcmp(arg1, "gnd")) {
                mode = LF_MCAL_GROUND;
                if (arg2) passes = (unsigned)strtoul(arg2, NULL, 10);
            } else if (!strcmp(arg1, "air")) {
                mode = LF_MCAL_AIR;
                if (arg2) passes = (unsigned)strtoul(arg2, NULL, 10);
            } else {
                passes = (unsigned)strtoul(arg1, NULL, 10);
            }
        }
        calibrate_motors(mode, (uint8_t)(passes ? passes : 5U), now);
        return;
    }
    if (!strcmp(cmd, "mcurve")) {
        /* 供上位机将保存在电脑上的 15 点地面主曲线直接下发到单片机 RAM */
        char *a = strtok(NULL, " \t"), *b = strtok(NULL, " \t"), *d = strtok(NULL, " \t");
        unsigned step = a ? (unsigned)strtoul(a, NULL, 10) : 99U;
        float vl = b ? strtof(b, NULL) : NAN, vr = d ? strtof(d, NULL) : NAN;
        if (step < LF_MCAL_STEPS && isfinite(vl) && isfinite(vr) && vl >= 0 && vr >= 0) {
            g_lf.mcal_curve_speed[step][0] = g_lf.mcal_ground_curve[step][0] = vl;
            g_lf.mcal_curve_speed[step][1] = g_lf.mcal_ground_curve[step][1] = vr;
            if (step + 1U > g_lf.mcal_max_steps) g_lf.mcal_max_steps = (uint8_t)(step + 1U);
            if (g_lf.mcal_max_steps >= 4U) {
                g_lf.mcal_valid = g_lf.mcal_ground_valid = true;
                g_lf.mcal_active_mode = LF_MCAL_GROUND;
            }
            board_log("MCURVE OK step=%u duty=%.2f vL=%.1f vR=%.1f\r\n",
                      step, (double)lf_mcal_step_duty((uint8_t)step), (double)vl, (double)vr);
        } else board_log("MCURVE ERROR\r\n");
        return;
    }
    if (!strcmp(cmd, "save")) {
        board_log(board_flash_save(&g_lf.cal, &g_lf.cfg) ? "SAVE OK\r\n" : "SAVE ERROR\r\n");
        return;
    }
    if (!strcmp(cmd, "load")) {
        board_log(board_flash_load(&g_lf.cal, &g_lf.cfg) ? "LOAD OK\r\n" : "LOAD ERROR\r\n");
        return;
    }
    if (!strcmp(cmd, "hw_preset")) {
        char *m = strtok(NULL, " \t");
        if (!m || (strcmp(m, "stock") && strcmp(m, "group_b"))) {
            board_log("HW_PRESET ERROR (use: hw_preset stock | group_b)\r\n");
            return;
        }
        lf_hw_apply_preset(&g_lf.cfg.hw, !strcmp(m, "group_b"));
        board_log("HW_PRESET OK (%s)\r\n", m);
        print_hwmap();
        return;
    }
    if (!strcmp(cmd, "defaults")) {
        g_lf.cfg = lf_default_config();
        g_lf.cfg.hw.adc_vref_v = board_adc_vref_v();
        g_lf.mcal_valid = g_lf.mcal_ground_valid = g_lf.mcal_air_valid = false;
        g_lf.mcal_active_mode = LF_MCAL_GROUND;
        board_log("DEFAULTS OK (RAM config reset)\r\n");
        return;
    }
    if (!strcmp(cmd, "set")) {
        char *key = strtok(NULL, " \t"); char *text = strtok(NULL, " \t"); char *end = NULL;
        float value = text ? strtof(text, &end) : NAN;
        bool good = key && text && end != text && *end == 0 && set_parameter(key, value);
        board_log(good ? "SET OK\r\n" : "SET ERROR (unknown/range/config conflict)\r\n"); return;
    }
    if (!strcmp(cmd, "order")) {
        LfConfig candidate = g_lf.cfg; bool good = true;
        for (unsigned i = 0; i < 4; ++i) {
            char *p = strtok(NULL, " \t");
            if (!p || strlen(p) != 1 || *p < '0' || *p > '3') { good = false; break; }
            candidate.channel_order[i] = (uint8_t)(*p - '0');
        }
        if (good && lf_config_valid(&candidate)) {
            g_lf.cfg = candidate; g_lf.cal.valid = false; g_lf.filter_ready = false;
            board_log("ORDER OK: calibration invalidated; run cal again.\r\n");
        } else board_log("ORDER ERROR\r\n");
        return;
    }
    if (!strcmp(cmd, "calset")) {
        unsigned ch, black, white;
        char *a = strtok(NULL, " \t"), *b = strtok(NULL, " \t"), *d = strtok(NULL, " \t");
        if (a && b && d && sscanf(a, "%u", &ch) == 1 && sscanf(b, "%u", &black) == 1 && sscanf(d, "%u", &white) == 1 &&
            ch < 4 && black < white && white <= 4095 && white - black >= g_lf.cfg.calibration_min_span) {
            g_lf.cal.black[ch] = (uint16_t)black; g_lf.cal.white[ch] = (uint16_t)white;
            g_lf.cal.valid = true;
            for (unsigned i = 0; i < 4; ++i)
                if ((int)g_lf.cal.white[i] - (int)g_lf.cal.black[i] < g_lf.cfg.calibration_min_span) g_lf.cal.valid = false;
            g_lf.fault = LF_OK;
            board_log("CALSET OK; valid=%u\r\n", (unsigned)g_lf.cal.valid);
        } else board_log("CALSET ERROR\r\n");
        return;
    }
    if (!strcmp(cmd, "bench")) {
        char *a = strtok(NULL, " \t"), *b = strtok(NULL, " \t"), *d = strtok(NULL, " \t");
        char *ea = NULL, *eb = NULL, *ed = NULL;
        float left = a ? strtof(a, &ea) : NAN, right = b ? strtof(b, &eb) : NAN;
        unsigned long ms = d ? strtoul(d, &ed, 10) : 0;
        if (a && b && d && *ea == 0 && *eb == 0 && *ed == 0 && isfinite(left) && isfinite(right) &&
            fabsf(left) <= .95f && fabsf(right) <= .95f && ms > 0 && ms <= 1500 && g_lf.fault == LF_OK) {
            bench_duty[0] = left; bench_duty[1] = right; bench_until = now + (uint32_t)ms; bench_active = true;
            board_log("BENCH %.2f %.2f %lu ms\r\n", (double)left, (double)right, ms);
        } else board_log("BENCH ERROR\r\n");
        return;
    }
    if (!strcmp(cmd, "vtest")) {
        /* 速度前馈闭环验证命令：按目标轮速 (mm/s) 直接调用 lf_wheel_feedforward() 计算补偿占空比并实测转速 */
        char *a = strtok(NULL, " \t"), *b = strtok(NULL, " \t"), *d = strtok(NULL, " \t");
        char *ea = NULL, *eb = NULL, *ed = NULL;
        float vl = a ? strtof(a, &ea) : NAN, vr = b ? strtof(b, &eb) : NAN;
        unsigned long ms = d ? strtoul(d, &ed, 10) : 0;
        if (a && b && d && *ea == 0 && *eb == 0 && *ed == 0 && isfinite(vl) && isfinite(vr) &&
            fabsf(vl) <= g_lf.cfg.max_wheel_mm_s && fabsf(vr) <= g_lf.cfg.max_wheel_mm_s &&
            ms >= 350 && ms <= 2500 && g_lf.fault == LF_OK) {
            vtest_target[0] = vl; vtest_target[1] = vr;
            float u0 = lf_wheel_feedforward(&g_lf, 0, vl);
            float u1 = lf_wheel_feedforward(&g_lf, 1, vr);
            float lim = g_lf.cfg.max_duty;
            vtest_duty[0] = u0 > lim ? lim : (u0 < -lim ? -lim : u0);
            vtest_duty[1] = u1 > lim ? lim : (u1 < -lim ? -lim : u1);
            vtest_sum_speed[0] = vtest_sum_speed[1] = 0;
            vtest_samples = 0;
            vtest_since = now;
            vtest_until = now + (uint32_t)ms;
            vtest_active = true;
            board_log("VTEST_START,tL=%.1f,tR=%.1f,uL=%.3f,uR=%.3f,ms=%lu\r\n",
                      (double)vl, (double)vr, (double)vtest_duty[0], (double)vtest_duty[1], ms);
        } else board_log("VTEST ERROR\r\n");
        return;
    }
    if (!strcmp(cmd, "scan")) {
        /* 水平云台 + 超声波步进扫掠标定：scan [span_us=220] [step_us=20] [dwell_ms=140] */
        char *a = strtok(NULL, " \t"), *b = strtok(NULL, " \t"), *d = strtok(NULL, " \t");
        unsigned long span = a ? strtoul(a, NULL, 10) : (unsigned long)g_lf.cfg.probe.sweep_us;
        unsigned long step = b ? strtoul(b, NULL, 10) : 20UL;
        unsigned long dwell = d ? strtoul(d, NULL, 10) : 140UL;
        if (span >= 20 && span <= 600 && step >= 5 && step <= 100 && dwell >= 60 && dwell <= 500 && g_lf.fault == LF_OK) {
            int32_t lo = (int32_t)g_lf.cfg.servo_center_us - (int32_t)span;
            int32_t hi = (int32_t)g_lf.cfg.servo_center_us + (int32_t)span;
            if (lo < 900) lo = 900;
            if (hi > 2100) hi = 2100;
            if (g_lf.state != LF_STOPPED) lf_stop(&g_lf, now);
            clear_aux_modes();
            scan_cur_us = (uint16_t)lo;
            scan_end_us = (uint16_t)hi;
            scan_step_us = (uint16_t)step;
            scan_dwell_ms = (uint16_t)dwell;
            scan_best_us = g_lf.cfg.servo_center_us;
            scan_best_mm = UINT16_MAX;
            scan_step_since = now;
            scan_active = true;
            board_log("SCAN_START,lo=%u,hi=%u,step=%u,dwell=%u,center=%u\r\n",
                (unsigned)scan_cur_us, (unsigned)scan_end_us, (unsigned)scan_step_us,
                (unsigned)scan_dwell_ms, (unsigned)g_lf.cfg.servo_center_us);
        } else board_log("SCAN ERROR (usage: scan [span_us 20..600] [step_us 5..100] [dwell_ms 60..500])\r\n");
        return;
    }
    if (!strcmp(cmd, "drive") || !strcmp(cmd, "turn")) {
        /* 编码器里程计单段运动标定：drive MM [SPD=120] 或 turn DEG [SPD=90]（正=左转CCW，负=右转CW） */
        bool is_turn = !strcmp(cmd, "turn");
        char *a = strtok(NULL, " \t"), *b = strtok(NULL, " \t");
        char *ea = NULL, *eb = NULL;
        float tgt = a ? strtof(a, &ea) : NAN;
        float spd = b ? strtof(b, &eb) : (is_turn ? g_lf.cfg.probe.turn_speed_mm_s : g_lf.cfg.probe.drive_speed_mm_s);
        float max_tgt = is_turn ? 360.0f : 1500.0f;
        if (a && *ea == 0 && (!b || *eb == 0) && isfinite(tgt) && isfinite(spd) &&
            fabsf(tgt) >= 5.0f && fabsf(tgt) <= max_tgt && spd >= 25.0f && spd <= 300.0f && g_lf.fault == LF_OK) {
            if (g_lf.state != LF_STOPPED) lf_stop(&g_lf, now);
            clear_aux_modes();
            move_is_turn = is_turn;
            move_target = tgt;
            move_speed = spd;
            move_start_mm = g_lf.odo.distance_mm;
            move_start_rad = g_lf.odo.heading_rad;
            move_since = now;
            move_timeout_ms = is_turn ? 6000U : 10000U;
            move_active = true;
            board_log("MOVE_START,mode=%s,target=%.1f,speed=%.1f\r\n", is_turn ? "TURN" : "DRIVE", (double)tgt, (double)spd);
        } else board_log("MOVE ERROR (usage: drive MM [SPD] | turn DEG [SPD])\r\n");
        return;
    }
    board_log("Unknown command; help\r\n");
}

static void console_poll(uint32_t now) {
    static char line[128]; static unsigned length; static bool discard;
    for (unsigned i = 0; i < 48; ++i) {
        int value = board_uart_getchar();
        if (value == -1) break;
        if (value == -2) { length = 0; discard = true; continue; }
        if (value == '\r' || value == '\n') {
            if (!discard) { line[length] = 0; command(line, now); }
            discard = false; length = 0;
        } else if (!discard && value >= 32 && value < 127) {
            if (length + 1 < sizeof(line)) line[length++] = (char)value;
            else { length = 0; discard = true; }
        }
    }
}

/* ============================================================================
 * 4. 物理按键状态机与实时变量快照更新
 * ============================================================================ */

typedef struct { bool raw, stable, long_done; uint32_t changed, pressed; } Button;
static Button keys[3];

static void buttons(uint32_t now) {
    bool values[] = {board_track_pressed(), board_probe_pressed(), board_core_pressed()};
    for (unsigned i = 0; i < 3; ++i) {
        Button *b = &keys[i];
        if (values[i] != b->raw) { b->raw = values[i]; b->changed = now; }
        if (b->stable != b->raw && now - b->changed >= 25) {
            b->stable = b->raw;
            if (b->stable) {
                b->pressed = now; b->long_done = false;
                if (i == 0 && (g_lf.state == LF_STOPPED || g_lf.state == LF_OBSTACLE_STOP) && !bench_active) {
                    start(now);
                } else if (i == 1 && g_lf.state == LF_OBSTACLE_STOP && !bench_active) {
                    start_probe(now);
                } else {
                    stop(now);
                }
            }
        }
        if ((i == 1 || i == 2) && b->stable && !b->long_done && now - b->pressed >= 1000) {
            b->long_done = true;
            if (g_lf.state == LF_STOPPED) calibrate(now);
        }
    }
}

static void snapshot(uint32_t now, const LfInput *sample) {
    g_debug.ms = now; g_debug.state = g_lf.state; g_debug.fault = g_lf.fault;
    g_debug.bench = bench_active; g_debug.uart_drops = board_uart_drops();
    g_debug.obstacle_mm = sample->obstacle_valid ? sample->obstacle_mm : 0U;
    g_debug.probe_face = g_lf.probe_face;
    g_debug.position = g_lf.position; g_debug.confidence = g_lf.confidence;
    g_debug.distance_mm = g_lf.odo.distance_mm;
    g_debug.heading_deg = g_lf.odo.heading_rad * 57.2957795f;
    for (unsigned i = 0; i < 4; ++i) {
        g_debug.raw[i] = g_lf.raw[i]; g_debug.black[i] = g_lf.cal.black[i]; g_debug.white[i] = g_lf.cal.white[i];
        g_debug.darkness[i] = g_lf.darkness[i];
    }
    for (unsigned i = 0; i < 2; ++i) {
        g_debug.speed[i] = g_lf.measured_mm_s[i]; g_debug.target[i] = g_lf.target_mm_s[i]; g_debug.duty[i] = g_lf.out.duty[i];
        g_debug.wheel_deadband[i] = g_lf.cfg.wheel_deadband[i];
        g_debug.wheel_free_speed[i] = g_lf.cfg.wheel_free_speed[i];
    }
}

/* ============================================================================
 * 5. 主函数入口与 200 Hz 硬件同步主循环
 * ============================================================================ */

int main(void) {
    lf_init(&g_lf); board_init();
    /* 上电先尝试恢复上次 save 到片内 Flash 的红外标定与全部参数，保证“上电后按键即可发车” */
    bool cfg_loaded = board_flash_load(&g_lf.cal, &g_lf.cfg);
    g_lf.cfg.hw.adc_vref_v = board_adc_vref_v();
    board_log("\r\nG474 LINE FOLLOW v2.2; 80MHz/TIM6+ADC1 5x1kHz/control 200Hz/PWM 20kHz; BT=%s (PC10=%c)\r\n",
        bt_probe_label(board_bt_probe_status()), board_bt_status_pin() ? 'H' : 'L');
    board_log("RAM-first mode (no auto Flash wear). Commands: mcal air 5 | mcal ground 5 | cal | start | diag\r\n");
    board_log("BOOT_CFG,flash_load=%u,cal_valid=%u\r\n", (unsigned)cfg_loaded, (unsigned)g_lf.cal.valid);
    /* 二进制自证：板级左右对调开关。以后任何一份日志都能对上它跑的是哪版固件，
     * 不再靠"两个文件时间戳谁更新"去猜（2026-11-22 转向争议就是这么糊涂的）。 */
    board_log("PROBE_BUILD,swaplr=%u,h_pred=1\r\n", (unsigned)BOARD_SWAP_LR);
    board_log("CSV,ms,state,fault,raw0,raw1,raw2,raw3,q0,q1,q2,q3,pos,confidence,vL,vR,tL,tR,uL,uR\r\n");
    uint32_t last_log = board_millis();
    LfState previous_state = g_lf.state;
    LfProbePhase previous_probe_phase = g_lf.probe_phase;
    for (;;) {
        uint32_t now = 0;
        float dt = 0;
        if (board_control_ready(&now, &dt)) {
            if (bench_active && (int32_t)(now - bench_until) >= 0) stop(now);
            if (outtest_active && (int32_t)(now - outtest_until) >= 0) {
                outtest_active = false; outtest_led = false;
                board_log("OUTTEST_DONE\r\n");
            }
            if (vtest_active && (int32_t)(now - vtest_until) >= 0) {
                float avgL = vtest_samples ? vtest_sum_speed[0] / (float)vtest_samples : 0;
                float avgR = vtest_samples ? vtest_sum_speed[1] / (float)vtest_samples : 0;
                board_log("VTEST_DONE,tL=%.1f,tR=%.1f,uL=%.3f,uR=%.3f,vL=%.1f,vR=%.1f,errL=%.1f,errR=%.1f\r\n",
                    (double)vtest_target[0], (double)vtest_target[1],
                    (double)vtest_duty[0], (double)vtest_duty[1],
                    (double)avgL, (double)avgR,
                    (double)(avgL - vtest_target[0]), (double)(avgR - vtest_target[1]));
                stop(now);
            }
            LfInput sample = {0}; board_sample(&sample);
            buttons(now);

            uint32_t request = g_debug_request; g_debug_request = 0;
            if (request == 1) calibrate(now);
            if (request == 2) start(now);
            if (request == 3) stop(now);
            if (request == 4) start_probe(now);
            if (request == 5) calibrate_motors(LF_MCAL_AIR, 5, now);
            if (request == 6) calibrate_motors(LF_MCAL_GROUND, 5, now);
            lf_step(&g_lf, &sample, now, dt);

            /* M2 侧扫遥测事件：控制器置位 -> 这里打印 -> 清零（事件只在发生的那个 5 ms 拍有效） */
            if (g_lf.side_event != (uint8_t)LF_SIDE_EVENT_NONE) {
                uint8_t side_ev = g_lf.side_event;
                g_lf.side_event = (uint8_t)LF_SIDE_EVENT_NONE;
                const LfProbeConfig *sp = &g_lf.cfg.probe;
                float side_reach = g_lf.side_h * tanf((float)sp->beam_half_deg * 0.01745329252f);
                /* 事件是位掩码：REV 找到第二个边界的那一拍会同时置 EDGE 和 MID，两条日志都要打 */
                if ((side_ev & (uint8_t)LF_SIDE_EVENT_EDGE) && g_lf.side_edges > 0U) {
                    /* 0 号是 FWD 先发现的 +s 侧（右）边界，1 号是 REV 发现的 -s 侧（左）边界 */
                    uint8_t i = (uint8_t)(g_lf.side_edges - 1U);
                    float s = g_lf.side_s_edge[i];
                    float s_true = i == 0U ? s - side_reach : s + side_reach;
                    board_log("PROBE_SIDE_EDGE,i=%u,s=%.1f,s_true=%.1f,n=%u\r\n",
                              (unsigned)i, (double)s, (double)s_true, (unsigned)g_lf.side_edge_frames);
                }
                if (side_ev & (uint8_t)LF_SIDE_EVENT_MID) {
                    /* k = d(h)/ds：只作诊断（车/盒残余斜率）。2026-11-22 21:36 起不再参与 LEG 控制 */
                    board_log("PROBE_SIDE_MID,s_m=%.1f,w=%.1f,kind=%s,retry=%u,k=%.3f\r\n",
                              (double)g_lf.face.s_m, (double)g_lf.face.w,
                              g_lf.face.kind == 0U ? "NARROW" : "WIDE", (unsigned)g_lf.side_warn,
                              (double)g_lf.side_k);
                }
                if (side_ev & (uint8_t)LF_SIDE_EVENT_YAW) {
                    /* 平移测斜：侧距单调漂移 → 车头相对箱面不平行，换算成修正角 */
                    board_log("PROBE_SIDE_YAW,dh=%.1f,ds=%.1f,corr=%.2f\r\n",
                              (double)g_lf.side_move_dh, (double)g_lf.side_move_ds,
                              (double)(g_lf.side_move_corr_rad * 57.2957795f));
                }
                if (side_ev & (uint8_t)LF_SIDE_EVENT_FAIL) {
                    board_log("PROBE_SIDE_FAIL,lock_stage=%u,edges=%u,retry=%u,h=%.1f\r\n",
                              (unsigned)g_lf.side_lock_stage, (unsigned)g_lf.side_edges,
                              (unsigned)g_lf.side_retry, (double)g_lf.side_h);
                }
            }

            /* M3 LEG 遥测：LEG 三段在同一个相位里，不换相位，所以由控制器置事件位、这里打印后清零。
             * TURN 用 turn_cmd/ach/ms；DRIVE1/2 用 leg_cmd/ach/hdg。 */
            if (g_lf.leg_event != 0U) {
                uint8_t leg_ev = g_lf.leg_event;
                g_lf.leg_event = 0U;
                if (leg_ev & (uint8_t)LF_LEG_EVENT_DRIVE1) {
                    board_log("PROBE_LEG,seg=DRIVE1,len=%.1f,ach=%.1f,hdg=%.2f\r\n",
                              (double)g_lf.leg_cmd_mm, (double)g_lf.leg_ach_mm,
                              (double)g_lf.leg_hdg_deg);
                }
                if (leg_ev & (uint8_t)LF_LEG_EVENT_TURN) {
                    /* dir = 命令符号（L=+90 逆时针）。h_pred = 转完后云台轴到下一面的期望侧距
                     * ≈ h_leg + margin：下一面 SIDE_LOCK 实测 h 与它一比就知道"转向/认面"对不对，
                     * 不必等 SIDE_SCAN 去和窄面窗口较劲（2026-11-22 的 389 mm 就是这么漏判的）。 */
                    float h_leg = g_lf.side_h + g_lf.side_k * g_lf.face.s_m;
                    board_log("PROBE_LEG,seg=TURN,cmd=%.1f,ach=%.1f,ms=%lu,dir=%c,h_pred=%.0f\r\n",
                              (double)g_lf.turn_cmd_deg, (double)g_lf.turn_ach_deg,
                              (unsigned long)g_lf.turn_ms,
                              g_lf.turn_cmd_deg >= 0.0f ? 'L' : 'R',
                              (double)(h_leg + LF_LEG_MARGIN_MM));
                }
                if (leg_ev & (uint8_t)LF_LEG_EVENT_DRIVE2) {
                    board_log("PROBE_LEG,seg=DRIVE2,len=%.1f,ach=%.1f,hdg=%.2f\r\n",
                              (double)g_lf.leg_cmd_mm, (double)g_lf.leg_ach_mm,
                              (double)g_lf.leg_hdg_deg);
                }
                if (leg_ev & (uint8_t)LF_LEG_EVENT_PROX) {
                    /* 行进中的侧距轨迹：窄面经过时 h 落在 pred 附近，无关挡块会明显偏小 */
                    board_log("PROBE_LEG,seg=PROX,stage=%u,h=%u,min=%u,max=%u,ms=%lu\r\n",
                              (unsigned)g_lf.leg_stage, (unsigned)g_lf.leg_prox_last,
                              (unsigned)g_lf.leg_prox_min, (unsigned)g_lf.leg_prox_max,
                              (unsigned long)(now - g_lf.leg_since));
                }
                if (leg_ev & (uint8_t)LF_LEG_EVENT_BACKUP) {
                    /* 收尾倒回：DRIVE2 末尾面已滑出侧视，按固定距离退回后重新交给 SIDE_LOCK */
                    board_log("PROBE_LEG,seg=BACKUP,len=%.1f,ach=%.1f,h=%u\r\n",
                              (double)LF_LEG_BACKUP_MM,
                              (double)(g_lf.leg_backup_start_mm - g_lf.odo.distance_mm),
                              (unsigned)g_lf.leg_prox_now);
                }
                if (leg_ev & (uint8_t)LF_LEG_EVENT_PROX2) {
                    board_log("PROBE_LEG,seg=PROX2,pred=%.0f,n=%u,face_n=%u,face_h=%u,h_min=%u,h_max=%u\r\n",
                              (double)g_lf.leg_h_expected, (unsigned)g_lf.leg_prox_n,
                              (unsigned)g_lf.leg_face2_n, (unsigned)g_lf.leg_face2_h,
                              (unsigned)g_lf.leg_prox_min, (unsigned)g_lf.leg_prox_max);
                }
                if (leg_ev & (uint8_t)LF_LEG_EVENT_FAIL) {
                    board_log("PROBE_LEG,seg=FAIL,stage=%u,why=%u,h=%.1f\r\n",
                              (unsigned)g_lf.leg_stage, (unsigned)g_lf.leg_fail,
                              (double)g_lf.side_h);
                }
            }

#if TRACK_SERVO_CENTER
            /* 起跑/循迹/取线/障碍停车期间把云台锁在机械中位并保持 5V 供电：
             * 否则舵机断电后会被振动带偏，导致“<15 cm 停车”的超声测距方向不再是车头正前方。 */
            if (g_lf.state == LF_ARMING || g_lf.state == LF_TRACKING ||
                g_lf.state == LF_LOST_BRAKE || g_lf.state == LF_SEARCHING ||
                g_lf.state == LF_OBSTACLE_STOP) {
                g_lf.out.servo_enable = true;
                g_lf.out.servo_us = g_lf.cfg.servo_center_us;
            }
#endif

            /* --- 自动打靶：<15 cm 主动短接制动停稳后，立刻开始“舵机 + 超声波”寻找垂直(法线)方向 --- */
            if (AUTO_PROBE_ON_OBSTACLE && g_lf.state == LF_OBSTACLE_STOP &&
                (int32_t)(now - (g_lf.state_since + AUTO_PROBE_SETTLE_MS)) >= 0) {
                start_probe(now);
            }

#if PROBE_TEST_ONE_FACE
            /* 目前没有激光器：单面测试期间关闭激光和蜂鸣器，改用 LED 指示。 */
            if (g_lf.state == LF_PROBING) {
                g_lf.out.laser = false;
                g_lf.out.buzzer = false;
            }
            /* 完成后状态虽回到 STOPPED，仍给舵机供电并保持锁定角，直到 LED 闪烁结束。 */
            if (g_lf.state == LF_STOPPED && probe_done_blink_until != 0U &&
                (int32_t)(probe_done_blink_until - now) > 0) {
                g_lf.out.servo_enable = true;
                g_lf.out.servo_us = g_lf.best_servo_us ? g_lf.best_servo_us : g_lf.cfg.servo_center_us;
            }
#endif

            /* 打靶/台架探测结束后 10 s 内保持侧视角：侧视锁定之后没有"回正"的理由，
             * 现场要能直接看/量窄面；到期才照常断电（board.c 仍是先停波后断电）。 */
            if (g_lf.state == LF_STOPPED && probe_hold_until != 0U) {
                if ((int32_t)(probe_hold_until - now) > 0 && g_lf.cfg.probe.scan_mode != 0U) {
                    g_lf.out.servo_enable = true;
                    g_lf.out.servo_us = g_lf.cfg.probe.side_us;
                } else {
                    probe_hold_until = 0U;
                }
            }

            if (vtest_active && g_lf.state == LF_STOPPED && g_lf.fault == LF_OK && (now - vtest_since >= 250U)) {
                vtest_sum_speed[0] += g_lf.measured_mm_s[0];
                vtest_sum_speed[1] += g_lf.measured_mm_s[1];
                ++vtest_samples;
            }

            /* 独立云台扫掠+超声测距标定状态机 (scan 命令) */
            if (scan_active && g_lf.state == LF_STOPPED && g_lf.fault == LF_OK) {
                g_lf.out = (LfOutput){.servo_enable = true, .servo_us = scan_cur_us};
                if ((uint32_t)(now - scan_step_since) >= scan_dwell_ms) {
                    board_log("SCAN_PT,%u,%u,%u\r\n",
                        (unsigned)scan_cur_us, (unsigned)sample.obstacle_mm, (unsigned)sample.obstacle_valid);
                    if (sample.obstacle_valid &&
                        sample.obstacle_mm >= g_lf.cfg.probe.aim_min_mm &&
                        sample.obstacle_mm <= g_lf.cfg.probe.aim_max_mm &&
                        sample.obstacle_mm < scan_best_mm) {
                        scan_best_mm = sample.obstacle_mm;
                        scan_best_us = scan_cur_us;
                    }
                    if (scan_cur_us + scan_step_us > scan_end_us) {
                        scan_active = false;
                        g_lf.out = (LfOutput){.servo_enable = false, .servo_us = g_lf.cfg.servo_center_us};
                        board_log("SCAN_DONE,best_us=%u,best_mm=%u,center_us=%u\r\n",
                            (unsigned)scan_best_us,
                            (unsigned)(scan_best_mm == UINT16_MAX ? 0U : scan_best_mm),
                            (unsigned)g_lf.cfg.servo_center_us);
                    } else {
                        scan_cur_us = (uint16_t)(scan_cur_us + scan_step_us);
                        scan_step_since = now;
                    }
                }
            }

            /* 独立里程计直行/转向标定状态机 (drive / turn 命令) */
            if (move_active && g_lf.state == LF_STOPPED && g_lf.fault == LF_OK) {
                float d_mm = g_lf.odo.distance_mm - move_start_mm;
                float d_deg = (g_lf.odo.heading_rad - move_start_rad) * 57.2957795f;
                uint32_t el = now - move_since;
                bool done = move_is_turn ? (fabsf(d_deg) >= fabsf(move_target)) : (fabsf(d_mm) >= fabsf(move_target));
                if (done || el >= move_timeout_ms) {
                    move_active = false;
                    g_lf.out = (LfOutput){.enable = true, .brake = true, .servo_us = g_lf.cfg.servo_center_us};
                    board_log("MOVE_DONE,mode=%s,target=%.1f,actual_mm=%.1f,actual_deg=%.2f,ms=%lu,timeout=%u\r\n",
                        move_is_turn ? "TURN" : "DRIVE", (double)move_target,
                        (double)d_mm, (double)d_deg, (unsigned long)el, (unsigned)(!done));
                } else {
                    float vL = 0.0f, vR = 0.0f;
                    if (move_is_turn) {
                        float s = move_target >= 0.0f ? 1.0f : -1.0f; /* 正=左转CCW，负=右转CW */
                        vL = -s * move_speed;
                        vR =  s * move_speed;
                    } else {
                        float s = move_target >= 0.0f ? 1.0f : -1.0f;
                        float corr = d_deg * 2.0f;
                        if (corr > 25.0f) corr = 25.0f;
                        if (corr < -25.0f) corr = -25.0f;
                        vL = s * move_speed + corr;
                        vR = s * move_speed - corr;
                    }
                    float u0 = lf_wheel_feedforward(&g_lf, 0, vL);
                    float u1 = lf_wheel_feedforward(&g_lf, 1, vR);
                    float lim = g_lf.cfg.max_duty;
                    g_lf.out = (LfOutput){
                        .enable = true,
                        .duty = {u0 > lim ? lim : (u0 < -lim ? -lim : u0),
                                 u1 > lim ? lim : (u1 < -lim ? -lim : u1)},
                        .servo_us = g_lf.cfg.servo_center_us
                    };
                }
            }

            /* 电机标定单步完成：实时通过蓝牙发送该轮该步数据给上位机 */
            if (g_lf.mcal_step_ready) {
                g_lf.mcal_step_ready = false;
                board_log("MCAL_STEP,%s,%u,%u,%u,%.2f,%.1f,%.1f\r\n",
                    g_lf.mcal_mode == LF_MCAL_GROUND ? "GROUND" : "AIR",
                    (unsigned)g_lf.mcal_last_pass, (unsigned)g_lf.mcal_total_passes,
                    (unsigned)g_lf.mcal_last_step, (double)g_lf.mcal_last_duty,
                    (double)g_lf.mcal_last_speed[0], (double)g_lf.mcal_last_speed[1]);
            }
            /* 电机 5 轮标定全部完成：回传 15 点平均曲线与死区/满速拟合结果（地面带载为主模型，空载仅参考） */
            if (g_lf.mcal_done_event) {
                g_lf.mcal_done_event = false;
                bool is_gnd = g_lf.mcal_mode == LF_MCAL_GROUND;
                const float *curve_rep = is_gnd ? &g_lf.mcal_ground_curve[0][0] : &g_lf.mcal_air_curve[0][0];
                const float *db_rep = is_gnd ? g_lf.mcal_ground_db : g_lf.mcal_air_db;
                const float *fs_rep = is_gnd ? g_lf.mcal_ground_fs : g_lf.mcal_air_fs;
                for (unsigned s = 0; s < g_lf.mcal_max_steps; ++s) {
                    board_log("MCAL_CURVE,%u,%.2f,%.1f,%.1f\r\n",
                        s, (double)lf_mcal_step_duty((uint8_t)s),
                        (double)curve_rep[s * 2U], (double)curve_rep[s * 2U + 1U]);
                }
                board_log("MCAL_DONE,%s,%.4f,%.4f,%.1f,%.1f,active=%s\r\n",
                    is_gnd ? "GROUND" : "AIR",
                    (double)db_rep[0], (double)db_rep[1],
                    (double)fs_rep[0], (double)fs_rep[1],
                    active_mcal_name());
                print_diag();
            }

            if ((bench_active || vtest_active) && g_lf.state == LF_STOPPED && g_lf.fault == LF_OK) {
                g_lf.out = (LfOutput){.enable = true,
                                      .duty = {vtest_active ? vtest_duty[0] : bench_duty[0],
                                               vtest_active ? vtest_duty[1] : bench_duty[1]},
                                      .servo_us = g_lf.cfg.servo_center_us};
            } else if (outtest_active && g_lf.state == LF_STOPPED && g_lf.fault == LF_OK) {
                g_lf.out = outtest_output;
            } else if (servo_sweep_active && g_lf.state == LF_STOPPED && g_lf.fault == LF_OK) {
                /* 连续三角波：900us -> 2100us -> 900us，周期 servo_sweep_period_ms */
                uint32_t period = servo_sweep_period_ms ? servo_sweep_period_ms : 2000U;
                uint32_t half = period / 2U;
                uint32_t phase = (now - servo_sweep_since) % period;
                uint16_t us;
                if (phase < half)
                    us = (uint16_t)(900U + (1200U * phase) / half);
                else
                    us = (uint16_t)(2100U - (1200U * (phase - half)) / half);
                g_lf.out = (LfOutput){.servo_enable = true, .servo_us = us};
            } else if (g_lf.state != LF_STOPPED) {
                clear_aux_modes();
            }
            board_set_pwm_hz(g_lf.cfg.hw.enable ? (uint32_t)(g_lf.cfg.hw.tri_freq_hz + 0.5f) : 20000U);
            board_apply(&g_lf.out, &g_lf.cfg); snapshot(now, &sample);
            bool probe_indicator_blink = false;
#if PROBE_TEST_ONE_FACE
            probe_indicator_blink =
                (g_lf.state == LF_PROBING &&
                 (g_lf.probe_phase == LF_PROBE_LASER || g_lf.probe_phase == LF_PROBE_COMPLETE_BEEPS)) ||
                (probe_done_blink_until != 0U &&
                 (int32_t)(probe_done_blink_until - now) > 0);
#endif
            bool status_led_on;
            if (outtest_active) status_led_on = outtest_led;
            else if (probe_indicator_blink) status_led_on = ((now / 150U) % 2U) != 0U;
            else status_led_on = (g_lf.state == LF_TRACKING || g_lf.state == LF_PROBING ||
                 ((g_lf.state == LF_CALIBRATING || g_lf.state == LF_MOTOR_CAL ||
                   g_lf.state == LF_ARMING || g_lf.state == LF_FAULT) && (now / 100) % 2));
            board_led(status_led_on);
            if (g_lf.state != previous_state) {
                board_log("EVENT %lu %s %s\r\n", (unsigned long)now, lf_state_name(g_lf.state), lf_fault_name(g_lf.fault));
                if (previous_state == LF_CALIBRATING) status();
                /* 探测结束：10 s 内云台留在侧视角（不回正、不断电），到期自动释放 */
                if (previous_state == LF_PROBING) probe_hold_until = now + 10000U;
#if PROBE_TEST_ONE_FACE
                if (previous_state == LF_PROBING && g_lf.state == LF_STOPPED) {
                    probe_done_blink_until = now + PROBE_TEST_BLINK_MS;
                    board_log("PROBE_TEST_DONE: single-face done, stopped; LED blink %u ms\r\n",
                              (unsigned)PROBE_TEST_BLINK_MS);
                }
#endif
                previous_state = g_lf.state;
            }
            if (g_lf.probe_phase != previous_probe_phase) {
                /* AIM 结束：模式 B 转 SIDE_LOCK，模式 A / AIM 失败则直接进 LASER（两种都要打 PROBE_FACE） */
                if (previous_probe_phase == LF_PROBE_AIM &&
                    (g_lf.probe_phase == LF_PROBE_LASER || g_lf.probe_phase == LF_PROBE_SIDE_LOCK ||
                     g_lf.probe_phase == LF_PROBE_TURN_PAR)) {
                    /* 2026-11-23：成功路径是 AIM→TURN_PAR，原来只在 AIM→LASER/SIDE_LOCK 打，
                     * 导致「转向为什么不是 90°」一直看不到 φ/ρ，这里补上。 */
                    /* M1：面模型（法线角/垂直距离/残差/内点数），侧视横扫的自检与绕行距离都靠它 */
                    board_log("PROBE_FACE,phi=%.2f,rho=%.1f,rms=%.1f,hits=%u,valid=%u\r\n",
                        (double)(g_lf.face.phi * 57.2957795f), (double)g_lf.face.rho,
                        (double)g_lf.face.rms, (unsigned)g_lf.face.hits, (unsigned)g_lf.face.valid);
                    /* 本轮扫掠找到障碍物平面的垂直(法线)方向：打印瞄准结果，便于现场核对偏转角 */
                    float angle_deg = ((float)g_lf.best_servo_us - (float)g_lf.cfg.servo_center_us) /
                                      LF_PROBE_SCAN_US_PER_DEG;
                    board_log("PROBE_LOCK,face=%u,servo_us=%u,center_us=%u,angle_deg=%.1f,dist_mm=%u,points=%u\r\n",
                        (unsigned)g_lf.probe_face, (unsigned)g_lf.best_servo_us,
                        (unsigned)g_lf.cfg.servo_center_us, (double)angle_deg,
                        (unsigned)(g_lf.best_obstacle_mm == UINT16_MAX ? 0U : g_lf.best_obstacle_mm),
                        (unsigned)g_lf.scan_points);
                }
                /* 侧视锁定自检结果：h 与 rho + pan_x 的差就是「转角不对/认错面」的唯一报警 */
                if (previous_probe_phase == LF_PROBE_SIDE_LOCK) {
                    const LfProbeConfig *lp = &g_lf.cfg.probe;
                    float pred = g_lf.face.rho > 1.0f ? g_lf.face.rho + lp->pan_x_mm : 0.0f;
                    board_log("PROBE_SIDE_LOCK,h=%.1f,pred=%.1f,us=%u,ok=%u,h1=%.1f,h2=%.1f,h3=%.1f,corr=%.1f,warn=%u\r\n",
                              (double)g_lf.side_h, (double)pred, (unsigned)lp->side_us,
                              (unsigned)(g_lf.probe_phase == LF_PROBE_SIDE_SCAN ? 1U : 0U),
                              (double)g_lf.side_h1, (double)g_lf.side_h2, (double)g_lf.side_h3,
                              (double)(g_lf.side_tilt_corr_rad * 57.2957795f), (unsigned)g_lf.side_warn);
                }
                if (previous_probe_phase == LF_PROBE_TURN_PAR) {
                    /* TURN_PAR 结束（成功进 SIDE_LOCK 或失败停机都打）：cmd/ach 是停稳后的值。
                     * dir 是命令符号：箱体在左（side_left=1）时**应当**是 R（从面对盒子转成平行），
                     * 不要把它当成"转反了"——绕角那一段（LEG TURN）才是 L。 */
                    board_log("PROBE_TURN,tag=PAR,cmd=%.1f,ach=%.1f,ms=%lu,ok=%u,dir=%c\r\n",
                              (double)g_lf.turn_cmd_deg, (double)g_lf.turn_ach_deg,
                              (unsigned long)g_lf.turn_ms, (unsigned)g_lf.turn_ok,
                              g_lf.turn_cmd_deg >= 0.0f ? 'L' : 'R');
                }
                board_log("PROBE_EVENT,%lu,%u,%s,%u,%u,%.1f,%.2f\r\n",
                    (unsigned long)now, (unsigned)g_lf.probe_face, lf_probe_phase_name(g_lf.probe_phase),
                    (unsigned)g_lf.last_obstacle_mm, (unsigned)g_lf.out.servo_us,
                    (double)g_lf.odo.distance_mm, (double)(g_lf.odo.heading_rad * 57.2957795f));
                previous_probe_phase = g_lf.probe_phase;
            }
        }
        uint32_t poll_now = board_millis();
        console_poll(poll_now);
        board_lcd_update(&g_lf);
        if (telemetry && g_lf.state != LF_MOTOR_CAL && !scan_active && poll_now - last_log >= telemetry_ms) {
            last_log = poll_now;
            if (probe_log_en || g_lf.state == LF_PROBING || g_lf.state == LF_OBSTACLE_STOP) {
                /* M2 只追加 3 个侧扫字段，旧字段顺序不变（《接口契约》§4「新字段只追加」） */
                board_log("PROBE,%lu,%u,%u,%s,%u,%u,%u,%u,%u,%.1f,%.2f,%u,%u,%.1f,%.1f,%u\r\n",
                    (unsigned long)poll_now, (unsigned)g_lf.state, (unsigned)g_lf.probe_face,
                    lf_probe_phase_name(g_lf.probe_phase),
                    (unsigned)g_lf.last_obstacle_mm, (unsigned)g_lf.last_obstacle_valid,
                    (unsigned)g_lf.out.servo_us, (unsigned)g_lf.best_servo_us,
                    (unsigned)(g_lf.best_obstacle_mm == UINT16_MAX ? 0U : g_lf.best_obstacle_mm),
                    (double)g_lf.odo.distance_mm, (double)(g_lf.odo.heading_rad * 57.2957795f),
                    (unsigned)g_lf.out.laser, (unsigned)g_lf.out.buzzer,
                    (double)g_lf.side_h, (double)g_lf.face.s_m, (unsigned)g_lf.side_edges);
            } else {
                board_log("CSV,%lu,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f\r\n",
                    (unsigned long)poll_now, (unsigned)g_lf.state, (unsigned)g_lf.fault,
                    (unsigned)g_lf.raw[0], (unsigned)g_lf.raw[1], (unsigned)g_lf.raw[2], (unsigned)g_lf.raw[3],
                    (double)g_lf.darkness[0], (double)g_lf.darkness[1], (double)g_lf.darkness[2], (double)g_lf.darkness[3],
                    (double)g_lf.position, (double)g_lf.confidence, (double)g_lf.measured_mm_s[0], (double)g_lf.measured_mm_s[1],
                    (double)g_lf.target_mm_s[0], (double)g_lf.target_mm_s[1], (double)g_lf.out.duty[0], (double)g_lf.out.duty[1]);
            }
        }
        board_watchdog_feed(); board_idle();
    }
}
