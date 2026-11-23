/**
 * @file    line_follower.c
 * @brief   智能小车核心控制算法与全流程状态机实现
 * @details
 * 本文件包含以下独立功能模块：
 *   1. 基础数学工具与状态切换辅助函数 (limit, approach, reset_loops)
 *   2. 默认参数配置与合法性边界检查 (lf_default_config, lf_config_valid)
 *   3. 外部控制命令入口 (lf_calibrate, lf_calibrate_motors, lf_start, lf_start_probe, lf_stop)
 *   4. 四路红外信号滤波、黑白归一化、全黑横线保持与质心计算 (read_line)
 *   5. 4 拍 (20 ms) 编码器滑动窗口测速与双轮航位推算里程计 (update_odometry)
 *   6. 左右轮独立死区/斜率 + 15 点实测分段插值前馈 + 非对称加减速速度环 (lf_wheel_feedforward, speed_control)
 *   7. 5 轮空载 (LF_MCAL_AIR) 与 5 轮地面带载自动往返 (LF_MCAL_GROUND) 电机标定状态机 (step_motor_cal)
 *   8. 顺时针四面障碍物瞄准与 10 秒激光打靶状态机 (step_probing)
 *   9. 200 Hz 主控制步进与安全故障诊断引擎 (lf_step)
 */

#include "line_follower.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * 模块 1：基础数学工具与内部状态复位函数
 * ============================================================================ */

/** @brief 将浮点数 x 限制在闭区间 [lo, hi] 内 */
static float limit(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

/** @brief 斜率限制器：每次调用最多允许 old 向目标 value 靠近 step 幅度 */
static float approach(float old, float value, float step) {
    return old + limit(value - old, -step, step);
}

/**
 * @brief 电机自动标定使用的 15 档递增 PWM 占空比阶梯 (6% ~ 95%)
 * @details
 *   - 低占空比区 (0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24)：以 3% 细步长精确捕捉左右电机不同的静摩擦/滚动摩擦起转死区；
 *   - 中高占空比区 (0.30, 0.38, 0.46, 0.55, 0.65, 0.75, 0.85, 0.95)：覆盖 0.50 以上高速工作区直至 95%，
 *     完整测出空载与地面带载下“占空比—实际转速”全量程非线性曲线。
 */
static const float k_mcal_duties[LF_MCAL_STEPS] = {
    0.06f, 0.09f, 0.12f, 0.15f, 0.18f, 0.21f, 0.24f,
    0.30f, 0.38f, 0.46f, 0.55f, 0.65f, 0.75f, 0.85f, 0.95f
};

float lf_mcal_step_duty(uint8_t step) {
    return step < LF_MCAL_STEPS ? k_mcal_duties[step] : 0.0f;
}

/**
 * @brief 清零位置环与速度环的所有动态积分器、目标速度与故障计时器
 */
static void reset_loops(LfController *c) {
    c->line_integral = c->derivative = 0;
    c->derivative_ready = false;
    c->white_pending = c->ambiguous_pending = false;
    c->obstacle_hits = 0;
    c->hw_d_ready = false;
    c->hw_d_lp1 = c->hw_d_lp2 = c->hw_v_i = c->hw_v_p = c->hw_v_d = 0;
    c->hw_weight_p = c->cfg.hw.v_mid;
    c->hw_pid_c = c->cfg.hw.v_mid;
    c->hw_ana_speed[0] = c->hw_ana_speed[1] = c->cfg.hw.ana_atten * c->cfg.hw.v_mid;
    for (unsigned i = 0; i < 2; ++i) {
        c->target_mm_s[i] = c->speed_integral[i] = c->applied_duty[i] = 0;
        c->wrong_direction_ms[i] = c->stall_ms[i] = 0;
    }
}

/** @brief 切换顶层运行状态并记录进入该状态的时间戳 */
static void change_state(LfController *c, LfState s, uint32_t now) {
    c->state = s;
    c->state_since = now;
}

/** @brief 清空云台步进扫掠采样缓存并复位本轮瞄准结果（具体实现见模块 8a） */
static void probe_scan_reset(LfController *c);
static float probe_scan_dir(const LfController *c);
static float probe_scan_us_at(const LfController *c, float index);

/* M2 侧视横扫的辅助函数定义在模块 8b（文件靠后），这里先给 lf_start_side_probe 用到的两个做前置声明 */
static void probe_motion_reset(LfController *c);
static void probe_side_lock_begin(LfController *c, uint32_t now);
static bool probe_side_width_ok(float w);

/** @brief 切换四面打靶子阶段，同时记录该阶段起始时间戳、起始里程与起始航向角 */
static void change_probe_phase(LfController *c, LfProbePhase p, uint32_t now) {
    c->probe_phase = p;
    c->phase_since = now;
    c->phase_start_mm = c->odo.distance_mm;
    c->phase_start_rad = c->odo.heading_rad;
    c->phase_start_x_mm = c->odo.x_mm;
    c->phase_start_y_mm = c->odo.y_mm;
    /* 每次进入瞄准阶段都重新开始一轮全新扫掠，避免上一面的回波数据污染本轮拟合 */
    if (p == LF_PROBE_AIM) probe_scan_reset(c);
}

/* ============================================================================
 * 模块 2：默认参数配置与参数合法性校验
 * ============================================================================ */

LfConfig lf_default_config(void) {
    return (LfConfig){
        /* 位置环参数：非线性三次方增益 k3=0.35 强化外侧探头纠偏，微分时间常数 16ms */
        .line_kp = 115, .line_ki = 0, .line_kd = 1.8f, .line_k3 = .35f, .derivative_tau_s = .016f,
        /* 速度与加减速参数：默认直线 120 mm/s，急弯减速下限 55 mm/s，加速度 1200 mm/s² */
        .cruise_mm_s = 120, .curve_min_mm_s = 55, .max_wheel_mm_s = 280,
        .max_turn_mm_s = 190, .acceleration_mm_s2 = 1200, .duty_slew_s = 5,
        /* 速度环与左右轮独立死区/满速参数（可通过 mcal air / mcal ground 5 轮实测自动更新） */
        .speed_kp = .0012f, .speed_ki = .003f, .free_speed_mm_s = 450,
        .feedforward_deadband = .14f,
        .wheel_deadband = {.14f, .14f}, .wheel_free_speed = {450, 450},
        .max_duty = .95f, .integral_limit = .25f,
        /* 机械几何参数：840 脉冲/圈，轮径 43 mm，探头坐标 {-1, -0.35, +0.35, +1}
         * 轮距用“等效里程计轮距”112.5：2026-11-22 台架实测 LEG 命令 90° 实车约 104°（多 14°），
         * 按 130×90/104≈112.5 标定（物理轮距仍约 130；turn 90 60 复测后再微调）。 */
        .encoder_counts_per_rev = 840, .wheel_diameter_mm = 43, .track_width_mm = 112.5f,
        .sensor_positions = {-1.0f, -.35f, .35f, 1.0f},
        /* 状态机时序与超声避障阈值：障碍物 < 150 mm (15 cm) 自动停车 */
        .calibration_min_span = 200, .calibration_ms = 8000, .arm_delay_ms = 2000,
        .lost_confirm_ms = 10, .lost_brake_ms = 60, .search_ms = 350,
        .stop_brake_ms = 100, .ambiguous_ms = 80, .obstacle_stop_mm = 150,
        .servo_center_us = 1500, .encoder_sign = {1, 1}, .motor_sign = {1, 1},
        .channel_order = {0, 1, 2, 3}, .speed_loop = true,
        .allow_reverse = false, .drive_brake = true, .obstacle_enable = true,
        /* 1:1 硬件模拟 PID 默认对应主扩展板网表实值 (R39/40=150k, R41/42=300k, R60/61/62=20k, C50=100nF, R54=22k) */
        .hw = {
            .enable = false, .i_enable = false, .coast_mode = true,
            .netlist_mode = false, .r45_to_5v = false,
            .w_outer = 1.0f / 3.0f, .w_inner = 1.0f / 6.0f,
            .adc_vref_v = 3.300f, .v_5v = 5.000f, .v_mid = 2.500f, .e_trim_v = 0.0f,
            .r39_kohm = 150.0f, .r41_kohm = 300.0f, .r40_kohm = 150.0f, .r42_kohm = 300.0f,
            .r43_kohm = 100.0f, .r45_kohm = 100.0f, .r_fb_kohm = 50.0f, .r_ir_src_kohm = 0.0f,
            .kp_stage = 2.5f, .r60_kohm = 20.0f,
            .kd_stage_ms = 5.5f, .r61_kohm = 20.0f,
            .d_tau1_ms = 2.2f, .d_tau2_ms = 0.55f,
            .ti_ms = 600.0f, .v_base = 2.500f,
            .ana_atten = 0.625f, .speed_tau_ms = 0.2475f,
            .tri_min_v = 0.351f, .tri_max_v = 2.851f, .tri_freq_hz = 20000.0f
        },
        /* 四面顺时针障碍物探测与 10s 激光打靶默认参数（均可经蓝牙 set probe_* 在线标定） */
        .probe = {
            .clockwise = true, .faces = 4U,
            /* aim_ms 2000→3000（2026-11-23）：第二段改成同幅独立复测后点数受驻留时长限制
             * （max_n = sweep_ms/180）；3000 ms 让第二段拿到 15 点、第一段 11 点。
             * 全流程只发生一次 AIM，多花 1 s 可以接受。 */
            .aim_ms = 3000U, .sweep_us = 300U, .scan_step_us = 60U, .laser_ms = 10000U,
            .aim_min_mm = 40U, .aim_max_mm = 400U, .phase_timeout_ms = 4500U,
            .turn_speed_mm_s = 90.0f, .drive_speed_mm_s = 120.0f, .approach_speed_mm_s = 85.0f,
            .turn_deg = 90.0f, .edge1_mm = 220.0f, .edge2_mm = 220.0f, .approach_max_mm = 260.0f,
            /* M2 侧视横扫：主方案默认开启（scan_mode=1），箱体在左、舵机右掰 90° 侧视 */
            .scan_mode = 1U, .side_left = true, .side_us = 2400U,
            .fit_rms_mm = LF_PROBE_FIT_RMS_MM,
            .side_speed_mm_s = 50.0f, .side_reach_mm = 260.0f,
            /* 台架实测（2026-11-22 上车）：车在动时侧视读数噪声约 ±15 mm（静止只有 ±2 mm），
             * band 8 会把大部分面内样本当噪声丢掉 → 放宽到 15；背景在 500~900 mm，jump 放到 40 仍安全。
             * 波束半角：4 次干净扫描反推 (span 202/185/199/191, h≈166) → γ≈7.6°，
             * 所以仍然用 7.5°；早先“γ=10.5”是把一次边界偏差较大的扫描当成了真值。 */
            .side_band_mm = 15.0f, .side_jump_mm = 40.0f,
            .beam_half_deg = 7.5f, .pan_x_mm = 30.0f, .side_verify = true
        }
    };
}

void lf_odometry_reset(LfController *c) {
    memset(&c->odo, 0, sizeof(c->odo));
    c->measured_mm_s[0] = c->measured_mm_s[1] = 0;
}

void lf_init(LfController *c) {
    memset(c, 0, sizeof(*c));
    c->cfg = lf_default_config();
    c->state = LF_STOPPED;
    c->mcal_active_mode = LF_MCAL_GROUND;
    c->hw_weight_p = c->cfg.hw.v_mid;
    c->hw_pid_e = c->cfg.hw.v_mid;
    c->hw_pid_c = c->cfg.hw.v_mid;
    c->hw_ana_speed[0] = c->hw_ana_speed[1] = c->cfg.hw.ana_atten * c->cfg.hw.v_mid;
}

void lf_hw_apply_preset(LfHwPidConfig *hw, bool group_b) {
    if (!hw) return;
    if (group_b) {
        hw->netlist_mode = true;
        hw->r45_to_5v = true;
        hw->r39_kohm = 47.0f;
        hw->r41_kohm = 110.0f;
        hw->r40_kohm = 100.0f;
        hw->r42_kohm = 120.0f;
        hw->r43_kohm = 100.0f;
        hw->r45_kohm = 120.0f;
        hw->r_fb_kohm = 50.0f;
        hw->r_ir_src_kohm = 1.0f;
        hw->e_trim_v = 0.0f;
        hw->r60_kohm = 20.0f;
        hw->r61_kohm = 20.0f;
        hw->d_tau1_ms = 2.21f; /* R54=4.7kΩ * C50=470nF */
        hw->d_tau2_ms = (hw->kd_stage_ms / 0.470f) * 0.010f;
        if (hw->d_tau2_ms < 0.05f) hw->d_tau2_ms = 0.05f;
    } else {
        hw->netlist_mode = false;
        hw->r45_to_5v = false;
        hw->w_outer = 1.0f / 3.0f;
        hw->w_inner = 1.0f / 6.0f;
        hw->r39_kohm = 150.0f;
        hw->r41_kohm = 300.0f;
        hw->r40_kohm = 150.0f;
        hw->r42_kohm = 300.0f;
        hw->r43_kohm = 100.0f;
        hw->r45_kohm = 100.0f;
        hw->r_fb_kohm = 50.0f;
        hw->r_ir_src_kohm = 0.0f;
        hw->e_trim_v = 0.0f;
        hw->r60_kohm = 20.0f;
        hw->r61_kohm = 20.0f;
        hw->d_tau1_ms = 2.20f;
        hw->d_tau2_ms = 0.55f;
    }
}

bool lf_config_valid(const LfConfig *p) {
    const float values[] = {
        p->line_kp, p->line_ki, p->line_kd, p->line_k3, p->derivative_tau_s,
        p->cruise_mm_s, p->curve_min_mm_s, p->max_wheel_mm_s, p->max_turn_mm_s,
        p->acceleration_mm_s2, p->duty_slew_s, p->speed_kp, p->speed_ki,
        p->free_speed_mm_s, p->feedforward_deadband,
        p->wheel_deadband[0], p->wheel_deadband[1],
        p->wheel_free_speed[0], p->wheel_free_speed[1],
        p->max_duty, p->integral_limit,
        p->encoder_counts_per_rev, p->wheel_diameter_mm, p->track_width_mm
    };
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
        if (!isfinite(values[i]) || values[i] < 0) return false;
    if (p->encoder_counts_per_rev < 1 || p->wheel_diameter_mm < 1 || p->track_width_mm < 30 ||
        p->free_speed_mm_s < 1 || p->wheel_free_speed[0] < 1 || p->wheel_free_speed[1] < 1 ||
        p->max_duty <= 0 || p->max_duty > 1 ||
        p->feedforward_deadband >= 1 || p->wheel_deadband[0] >= 1 || p->wheel_deadband[1] >= 1 ||
        p->line_k3 > 3.0f || p->curve_min_mm_s > p->cruise_mm_s ||
        p->cruise_mm_s > p->max_wheel_mm_s || p->acceleration_mm_s2 <= 0 ||
        p->duty_slew_s <= 0 || p->servo_center_us < 900 || p->servo_center_us > 2100) return false;
    unsigned seen = 0;
    for (unsigned i = 0; i < LF_CHANNELS; ++i) {
        if (p->channel_order[i] >= LF_CHANNELS || (seen & (1U << p->channel_order[i])) ||
            !isfinite(p->sensor_positions[i])) return false;
        seen |= 1U << p->channel_order[i];
    }
    for (unsigned i = 0; i < 2; ++i)
        if ((p->encoder_sign[i] != 1 && p->encoder_sign[i] != -1) ||
            (p->motor_sign[i] != 1 && p->motor_sign[i] != -1)) return false;
    const LfHwPidConfig *h = &p->hw;
    const float hw_vals[] = {
        h->w_outer, h->w_inner, h->adc_vref_v, h->v_5v, h->v_mid,
        h->r39_kohm, h->r41_kohm, h->r40_kohm, h->r42_kohm,
        h->r43_kohm, h->r45_kohm, h->r_fb_kohm, h->r_ir_src_kohm,
        h->kp_stage, h->r60_kohm, h->kd_stage_ms, h->r61_kohm,
        h->d_tau1_ms, h->d_tau2_ms, h->ti_ms, h->v_base,
        h->ana_atten, h->speed_tau_ms, h->tri_min_v, h->tri_max_v, h->tri_freq_hz
    };
    for (unsigned i = 0; i < sizeof(hw_vals) / sizeof(hw_vals[0]); ++i)
        if (!isfinite(hw_vals[i]) || hw_vals[i] < 0) return false;
    if (!isfinite(h->e_trim_v) || h->e_trim_v < -1.5f || h->e_trim_v > 1.5f) return false;
    if (h->w_outer < .02f || h->w_outer > 3.0f || h->w_inner < .01f || h->w_inner > 3.0f ||
        h->adc_vref_v < 2.0f || h->adc_vref_v > 3.6f ||
        h->v_5v < 4.2f || h->v_5v > 5.6f || h->v_mid < 1.8f || h->v_mid > 3.2f ||
        h->v_mid >= h->v_5v - 0.5f ||
        h->r39_kohm < 1.0f || h->r39_kohm > 2000.0f || h->r41_kohm < 1.0f || h->r41_kohm > 2000.0f ||
        h->r40_kohm < 1.0f || h->r40_kohm > 2000.0f || h->r42_kohm < 1.0f || h->r42_kohm > 2000.0f ||
        h->r43_kohm < 1.0f || h->r43_kohm > 2000.0f || h->r45_kohm < 1.0f || h->r45_kohm > 2000.0f ||
        h->r_fb_kohm < 1.0f || h->r_fb_kohm > 2000.0f || h->r_ir_src_kohm > 20.0f ||
        h->kp_stage < .2f || h->kp_stage > 20.0f || h->r60_kohm < 1.0f || h->r60_kohm > 1000.0f ||
        h->kd_stage_ms > 500.0f || h->r61_kohm < 1.0f || h->r61_kohm > 5000.0f ||
        h->d_tau1_ms < .05f || h->d_tau1_ms > 100.0f || h->d_tau2_ms < .01f || h->d_tau2_ms > 50.0f ||
        h->ti_ms < 20.0f || h->ti_ms > 5000.0f || h->v_base < 1.0f || h->v_base > 4.5f ||
        h->ana_atten < 0.1f || h->ana_atten > 1.0f ||
        h->speed_tau_ms < 0.01f || h->speed_tau_ms > 20.0f ||
        h->tri_min_v < 0.05f || h->tri_max_v > h->v_5v - 0.05f || h->tri_max_v <= h->tri_min_v + 0.2f ||
        h->tri_freq_hz < 4000.0f || h->tri_freq_hz > 60000.0f) return false;
    const LfProbeConfig *pr = &p->probe;
    const float pr_vals[] = {
        pr->turn_speed_mm_s, pr->drive_speed_mm_s, pr->approach_speed_mm_s,
        pr->turn_deg, pr->edge1_mm, pr->edge2_mm, pr->approach_max_mm
    };
    for (unsigned i = 0; i < sizeof(pr_vals) / sizeof(pr_vals[0]); ++i)
        if (!isfinite(pr_vals[i]) || pr_vals[i] <= 0.0f) return false;
    if (pr->faces < 1U || pr->faces > 4U ||
        pr->aim_ms < 200U || pr->aim_ms > 5000U ||
        pr->sweep_us < 20U || pr->sweep_us > 600U ||
        pr->scan_step_us < 20U || pr->scan_step_us > 200U ||
        pr->laser_ms < 200U || pr->laser_ms > 30000U ||
        pr->aim_min_mm < 20U || pr->aim_max_mm <= pr->aim_min_mm || pr->aim_max_mm > 2000U ||
        pr->phase_timeout_ms < 500U || pr->phase_timeout_ms > 15000U ||
        pr->turn_speed_mm_s < 20.0f || pr->turn_speed_mm_s > 300.0f ||
        pr->drive_speed_mm_s < 20.0f || pr->drive_speed_mm_s > 400.0f ||
        pr->approach_speed_mm_s < 20.0f || pr->approach_speed_mm_s > 300.0f ||
        pr->turn_deg < 30.0f || pr->turn_deg > 180.0f ||
        pr->edge1_mm < 30.0f || pr->edge1_mm > 1500.0f ||
        pr->edge2_mm < 30.0f || pr->edge2_mm > 1500.0f ||
        pr->approach_max_mm < 30.0f || pr->approach_max_mm > 1000.0f) return false;
    /* M2 侧视横扫参数：范围与《接口契约_M1.md》§2.3 一致 */
    const float side_vals[] = {
        pr->fit_rms_mm, pr->side_speed_mm_s, pr->side_reach_mm,
        pr->side_band_mm, pr->side_jump_mm, pr->beam_half_deg
    };
    for (unsigned i = 0; i < sizeof(side_vals) / sizeof(side_vals[0]); ++i)
        if (!isfinite(side_vals[i]) || side_vals[i] <= 0.0f) return false;
    if (!isfinite(pr->pan_x_mm) || pr->pan_x_mm < 0.0f) return false; /* pan_x 允许为 0（云台轴与轮轴重合） */
    if (pr->scan_mode > 1U ||
        pr->side_us < LF_SERVO_US_MIN || pr->side_us > LF_SERVO_US_MAX ||
        pr->fit_rms_mm < 2.0f || pr->fit_rms_mm > 20.0f ||
        pr->side_speed_mm_s < 20.0f || pr->side_speed_mm_s > 100.0f ||
        pr->side_reach_mm < 80.0f || pr->side_reach_mm > 400.0f ||
        pr->side_band_mm < 4.0f || pr->side_band_mm > 30.0f ||
        pr->side_jump_mm < 10.0f || pr->side_jump_mm > 80.0f ||
        pr->side_jump_mm <= pr->side_band_mm ||
        pr->beam_half_deg < 2.0f || pr->beam_half_deg > 15.0f ||
        pr->pan_x_mm > 120.0f) return false;
    return true;
}

/* ============================================================================
 * 模块 3：外部触发命令接口（红外校准、电机死区标定、起跑、打靶、停止、故障）
 * ============================================================================ */

void lf_calibrate(LfController *c, uint32_t now) {
    reset_loops(c);
    c->out = (LfOutput){0};
    c->cal.valid = false;
    c->fault = LF_OK;
    c->samples = 0;
    for (unsigned i = 0; i < LF_CHANNELS; ++i) {
        c->minimum[i] = UINT16_MAX;
        c->maximum[i] = 0;
    }
    change_state(c, LF_CALIBRATING, now);
}

bool lf_calibrate_motors(LfController *c, LfMcalMode mode, uint8_t passes, uint32_t now) {
    if (c->state != LF_STOPPED || c->fault != LF_OK) return false;
    reset_loops(c);
    lf_odometry_reset(c);
    memset(c->mcal_sum_speed, 0, sizeof(c->mcal_sum_speed));
    memset(c->mcal_samples, 0, sizeof(c->mcal_samples));
    c->mcal_mode = mode;
    c->mcal_pass = 0;
    c->mcal_total_passes = passes < 1U ? 1U : (passes > 10U ? 10U : passes);
    c->mcal_step = 0;
    /* 空载与地面带载模式均完整测试全部 15 档占空比 (0.06 ~ 0.95) */
    c->mcal_max_steps = LF_MCAL_STEPS;
    c->mcal_pass_samples = 0;
    c->mcal_pass_sum[0] = c->mcal_pass_sum[1] = 0;
    c->mcal_returning = false;
    c->mcal_return_done = false;
    c->mcal_step_pause = false;
    c->mcal_step_ready = false;
    c->mcal_done_event = false;
    /* 若已有地面带载主曲线，重测空载仅更新参考池，不清除地面带载前馈 */
    if (mode == LF_MCAL_GROUND || !c->mcal_ground_valid) c->mcal_valid = false;
    c->phase_since = now;
    change_state(c, LF_MOTOR_CAL, now);
    return true;
}

bool lf_start(LfController *c, uint32_t now) {
    /* 1:1 硬件模拟仿真模式下无需提前软件 cal 归一化也可直接按原始电压差跑车 */
    bool ir_ready = c->cal.valid || c->cfg.hw.enable;
    if ((c->state != LF_STOPPED && c->state != LF_OBSTACLE_STOP) || !ir_ready || c->fault != LF_OK)
        return false;
    reset_loops(c);
    lf_odometry_reset(c);
    c->last_turn = 0;
    change_state(c, LF_ARMING, now);
    return true;
}

bool lf_start_probe(LfController *c, uint32_t now) {
    if ((c->state != LF_OBSTACLE_STOP && c->state != LF_STOPPED) || c->fault != LF_OK)
        return false;
    reset_loops(c);
    lf_odometry_reset(c);
    c->probe_face = 0;
    c->best_servo_us = c->cfg.servo_center_us;
    c->best_obstacle_mm = UINT16_MAX;
    /* M2：清掉上一轮侧扫的全部状态，避免残留 h / 原点 / 复测标志污染新一轮 */
    c->side_bench = 0U;
    c->side_push = 0U;
    c->side_locked = 0U;
    c->side_retry = 0U;
    c->side_rescan = 0U;
    c->side_verify_done = 0U;
    c->side_verify_mode = 0U;
    c->side_event = 0U;
    c->side_h = 0.0f;
    c->box_kind_ok = 0U; /* 新一轮首面重新锁存宽窄基准 */
    change_state(c, LF_PROBING, now);
    change_probe_phase(c, LF_PROBE_AIM, now);
    return true;
}

bool lf_start_side_probe(LfController *c, uint32_t now, bool push) {
    if ((c->state != LF_OBSTACLE_STOP && c->state != LF_STOPPED) || c->fault != LF_OK)
        return false;
    if (c->cfg.probe.scan_mode == 0U) return false; /* 侧扫台架命令只在侧视模式下可用 */
    reset_loops(c);
    lf_odometry_reset(c);
    /* 台架没有 AIM 结果：清掉面模型 → SIDE_LOCK 的 pred=0，会跳过 rho + pan_x 自检，
     * 只做 h∈[80,350] 的物理范围检查（人工摆位的平行度由人保证）。 */
    memset(&c->face, 0, sizeof(c->face));
    c->probe_face = 0U;
    c->h_ref_mm = 0.0f;
    c->best_servo_us = c->cfg.probe.side_us;
    c->best_obstacle_mm = UINT16_MAX;
    c->side_bench = 1U;
    c->side_push = push ? 1U : 0U;
    c->side_event = 0U;
    c->box_kind_ok = 0U; /* 台架从当前面重新起算 */
    probe_motion_reset(c);
    probe_side_lock_begin(c, now);
    change_state(c, LF_PROBING, now);
    change_probe_phase(c, LF_PROBE_SIDE_LOCK, now);
    return true;
}

/**
 * @brief 本段 LEG 的当前面宽窄：首面实测锁存 + 面序交替（宽↔窄）
 * @note  2026-11-23 复核历史日志：宽面实测 109.5~187.4，窄面 43.6~79.7，分界 100 分得开；
 *        原先的 130 会把 h≈200 处的宽面（118.5）误判成窄面（见 LF_SIDE_KIND_SPLIT_MM 注释）。
 *        仍保留「首面锁存 + 面号奇偶取反」：实测 w 有 ±15 mm 波动，后续面不再单独信 w。
 */
static uint8_t probe_leg_current_kind(LfController *c) {
    if (!c->box_kind_ok && c->face.w > 20.0f) {
        c->box_first_kind = (c->face.w >= LF_SIDE_KIND_SPLIT_MM) ? 1U : 0U;
        c->box_latch_index = (uint8_t)(c->probe_face > 0U ? c->probe_face - 1U : 0U);
        c->box_kind_ok = 1U;
    }
    uint8_t face_idx = (uint8_t)(c->probe_face > 0U ? c->probe_face - 1U : 0U);
    if (!c->box_kind_ok || face_idx < c->box_latch_index) return c->face.kind;
    return (uint8_t)(c->box_first_kind ^ ((face_idx - c->box_latch_index) & 1U));
}

bool lf_start_leg_probe(LfController *c, uint32_t now) {
    if ((c->state != LF_OBSTACLE_STOP && c->state != LF_STOPPED) || c->fault != LF_OK) return false;
    if (c->cfg.probe.scan_mode == 0U) return false;
    /* 复用上一次 side 扫描的几何：face.w/kind 与 side_h 必须有效，否则 L1/L2 无从算起。
     * 注意 side 入口会把 face.rho 清 0，所以下一面 SIDE_LOCK 的 pred=0（跳过 ρ 自检），
     * 这正是台架 leg 命令想要的：人工摆位/刚绕过来时 ρ 本来就不该拿来当基准。 */
    /* 2026-11-22 用户要求先测 M3 的 LEG 本体：宽度越界（valid=0，如 w=201.8）或
     * |k|>0.20（实测 0.25）只记 side_warn/日志，不再拒绝绕行（k 已不参与控制）；
     * 只保留 w>20、h>1 的最低几何可信度，否则 L1/L2 无从算起。 */
    if (!(c->face.w > 20.0f) || !(c->side_h > 1.0f)) return false;
    reset_loops(c);
    c->side_bench = 1U;   /* 单面台架：下一面 LASER 结束直接 COMPLETE，不进下一段 LEG */
    c->side_push = 0U;
    c->side_locked = 1U;  /* 保持 side_us 舵机角，直到下一面 SIDE_LOCK 自己清 0 */
    c->best_servo_us = c->cfg.probe.side_us;
    c->leg_started = 0U;
    c->leg_event = 0U;
    c->leg_fail = 0U;
    c->leg_stage = (uint8_t)LF_LEG_DRIVE1;
    c->turn_stage = (uint8_t)LF_TURN_IDLE;
    probe_motion_reset(c);
    if (c->probe_face < 3U) ++c->probe_face; /* 台架从第 1 面绕到第 2 面 */
    c->leg_kind = probe_leg_current_kind(c);
    change_state(c, LF_PROBING, now);
    change_probe_phase(c, LF_PROBE_LEG, now);
    return true;
}

void lf_stop(LfController *c, uint32_t now) {
    bool moving = c->out.enable || c->state == LF_TRACKING || c->state == LF_LOST_BRAKE ||
                  c->state == LF_SEARCHING || c->state == LF_PROBING ||
                  c->state == LF_MOTOR_CAL || c->state == LF_STOPPING;
    reset_loops(c);
    c->probe_phase = LF_PROBE_IDLE;
    c->fault = LF_OK;
    c->out = (LfOutput){.enable = moving, .brake = moving};
    change_state(c, moving ? LF_STOPPING : LF_STOPPED, now);
}

void lf_trip(LfController *c, LfFault why) {
    reset_loops(c);
    c->probe_phase = LF_PROBE_IDLE;
    c->out = (LfOutput){0};
    c->fault = why;
    c->state = LF_FAULT;
}

/* ============================================================================
 * 模块 4：四路红外传感器信号处理与黑线横向位置估计
 * ============================================================================ */

static bool read_line(LfController *c, const LfInput *in, float dt, bool *all_black_out) {
    float sum = 0, weighted = 0, peak = 0;
    bool all_black = true;
    for (unsigned i = 0; i < LF_CHANNELS; ++i) {
        unsigned j = c->cfg.channel_order[i];
        c->raw[i] = in->raw[j < LF_CHANNELS ? j : i];
        if (!c->filter_ready) c->filtered_raw[i] = c->raw[i];
        else c->filtered_raw[i] += dt / (.003f + dt) * (c->raw[i] - c->filtered_raw[i]);
        float span = (float)c->cal.white[i] - c->cal.black[i];
        float q = span > 0 ? limit((c->cal.white[i] - c->filtered_raw[i]) / span, 0, 1) : 0;
        if (q < .06f) q = 0;
        c->darkness[i] = q;
        if (q < .85f) all_black = false;
        sum += q;
        weighted += q * c->cfg.sensor_positions[i];
        peak = fmaxf(peak, q);
    }
    c->filter_ready = true;
    c->confidence = sum;
    *all_black_out = all_black;
    bool valid = peak >= .20f && sum >= .25f;
    if (valid) {
        if (all_black) {
            /* 穿过横线/交叉口全黑期间保持上一拍位置，抑制假微分甩尾 */
            c->position = c->derivative_ready ? c->last_position : 0;
        } else {
            c->position = limit(weighted / sum, -1, 1);
        }
    }
    /* 若采样无效（全白丢线），刻意保留上一次有效位置：全白 != 居中 */
    return valid;
}

static float hw_u6a_stage(const LfController *c, float *weight_p_out, float *e_ana_out) {
    const LfHwPidConfig *h = &c->cfg.hw;
    float v[LF_CHANNELS];
    float vref = h->adc_vref_v > 0.5f ? h->adc_vref_v : 3.300f;
    for (unsigned i = 0; i < LF_CHANNELS; ++i) {
        v[i] = (float)c->raw[i] * (vref / 4095.0f);
    }
    float rail_hi = h->v_5v - 0.05f;
    float vmid = h->v_mid;
    float g0 = 1.0f / (h->r39_kohm + h->r_ir_src_kohm);
    float g1 = 1.0f / (h->r41_kohm + h->r_ir_src_kohm);
    float gm = 1.0f / h->r43_kohm + (h->r45_to_5v ? 0.0f : (1.0f / h->r45_kohm));
    float g5 = h->r45_to_5v ? (1.0f / h->r45_kohm) : 0.0f;
    float gp = g0 + g1 + gm + g5;
    float vp_node = limit((g0 * v[0] + g1 * v[1] + gm * vmid + g5 * h->v_5v) / gp, 0.05f, rail_hi);
    if (weight_p_out) *weight_p_out = vp_node;

    float e_ana = 0.0f;
    if (h->netlist_mode) {
        float g3 = 1.0f / (h->r40_kohm + h->r_ir_src_kohm);
        float g2 = 1.0f / (h->r42_kohm + h->r_ir_src_kohm);
        float gfb = 1.0f / h->r_fb_kohm;
        float gn = g3 + g2 + gfb;
        float pid_e = limit((gn * vp_node - g3 * v[3] - g2 * v[2]) / gfb, 0.05f, rail_hi);
        e_ana = pid_e - vmid + h->e_trim_v;
    } else {
        e_ana = h->w_outer * (v[0] - v[3]) + h->w_inner * (v[1] - v[2]) + h->e_trim_v;
    }
    float pid_e_out = limit(vmid + e_ana, 0.05f, rail_hi);
    if (e_ana_out) *e_ana_out = pid_e_out - vmid;
    return pid_e_out;
}

float lf_compute_analog_pid_e(const LfController *c, float *e_ana_out) {
    return hw_u6a_stage(c, NULL, e_ana_out);
}

static void hw_mixer_pwm_stage(const LfHwPidConfig *h, float max_duty, float v_c_off,
                               float spd_state[2], bool do_rc_filter, float sub_dt, float duty_out[2]) {
    float rail_hi = h->v_5v - 0.05f;
    float base_off = 2.0f * (h->v_base - h->v_mid);
    float raw_L = limit(h->v_mid + base_off + 2.0f * v_c_off, 0.05f, rail_hi);
    float raw_R = limit(h->v_mid + base_off - 2.0f * v_c_off, 0.05f, rail_hi);
    float ss_L = h->ana_atten * raw_L;
    float ss_R = h->ana_atten * raw_R;
    if (do_rc_filter) {
        float tau_s = h->speed_tau_ms * 0.001f;
        float alpha = sub_dt / (tau_s + sub_dt);
        spd_state[0] += alpha * (ss_L - spd_state[0]);
        spd_state[1] += alpha * (ss_R - spd_state[1]);
    } else {
        spd_state[0] = ss_L;
        spd_state[1] = ss_R;
    }
    float tri_span = h->tri_max_v - h->tri_min_v;
    duty_out[0] = limit((spd_state[0] - h->tri_min_v) / tri_span, 0.0f, max_duty);
    duty_out[1] = limit((spd_state[1] - h->tri_min_v) / tri_span, 0.0f, max_duty);
}

void lf_hw_eval_static(const LfController *c, float *weight_p_v, float *pid_e_v,
                       float *vp_off_v, float *pid_c_v, float spd_v[2], float duty_out[2]) {
    const LfHwPidConfig *h = &c->cfg.hw;
    float wp = h->v_mid, e_ana = 0.0f;
    float pid_e = hw_u6a_stage(c, &wp, &e_ana);
    float lo = 0.05f - h->v_mid;
    float hi = (h->v_5v - 0.05f) - h->v_mid;
    float vp_off = limit(-h->kp_stage * e_ana, lo, hi);
    float vc_off = limit(-((20.0f / h->r60_kohm) * vp_off), lo, hi);
    float spd[2] = {0}, dut[2] = {0};
    hw_mixer_pwm_stage(h, c->cfg.max_duty, vc_off, spd, false, 0.0f, dut);
    if (weight_p_v) *weight_p_v = wp;
    if (pid_e_v) *pid_e_v = pid_e;
    if (vp_off_v) *vp_off_v = vp_off;
    if (pid_c_v) *pid_c_v = h->v_mid + vc_off;
    if (spd_v) { spd_v[0] = spd[0]; spd_v[1] = spd[1]; }
    if (duty_out) { duty_out[0] = dut[0]; duty_out[1] = dut[1]; }
}

/* ============================================================================
 * 模块 5：4 拍 (20 ms) 编码器滑动窗口测速与双轮航位推算里程计
 * ============================================================================ */

static void update_odometry(LfController *c, const LfInput *in, float dt) {
    const float mm_per_count = 3.14159265f * c->cfg.wheel_diameter_mm / c->cfg.encoder_counts_per_rev;
    uint8_t idx = c->odo.index;
    c->odo.dt_window[idx] = dt;
    float step_mm[2] = {0, 0};
    for (unsigned i = 0; i < 2; ++i) {
        int16_t signed_delta = (int16_t)(in->encoder_delta[i] * c->cfg.encoder_sign[i]);
        c->odo.window[idx][i] = signed_delta;
        step_mm[i] = signed_delta * mm_per_count;
        c->odo.wheel_mm[i] += step_mm[i];
    }
    c->odo.index = (uint8_t)((idx + 1U) % LF_ENC_WINDOW);
    if (c->odo.count < LF_ENC_WINDOW) ++c->odo.count;
    float sum_dt = 0;
    int32_t sum_counts[2] = {0, 0};
    for (unsigned k = 0; k < c->odo.count; ++k) {
        sum_dt += c->odo.dt_window[k];
        sum_counts[0] += c->odo.window[k][0];
        sum_counts[1] += c->odo.window[k][1];
    }
    if (sum_dt > 0) {
        for (unsigned i = 0; i < 2; ++i) {
            float win_speed = (sum_counts[i] * mm_per_count) / sum_dt;
            c->measured_mm_s[i] += dt / (.006f + dt) * (win_speed - c->measured_mm_s[i]);
        }
    }
    float ds = .5f * (step_mm[0] + step_mm[1]);
    float dtheta = (step_mm[1] - step_mm[0]) / c->cfg.track_width_mm;
    c->odo.distance_mm += ds;
    c->odo.heading_rad += dtheta;
    c->odo.x_mm += ds * cosf(c->odo.heading_rad);
    c->odo.y_mm += ds * sinf(c->odo.heading_rad);
}

/* ============================================================================
 * 模块 6：左右轮独立死区/斜率 + 15 点实测分段插值前馈与速度 PI 控制器
 * ============================================================================ */

/**
 * @brief 计算单侧驱动轮在目标线速度 target_mm_s 下所需的精确前馈占空比
 * @details
 * 优先级 1：若已完成 5 轮电机实测标定 (c->mcal_valid == true)，则直接在 15 点实测
 *          (mcal_curve_speed[s][wheel] -> k_mcal_duties[s]) 曲线上执行分段线性反插值！
 *          这样能同时补偿低占空比静摩擦死区、中占空比斜率差异以及高占空比 (>0.50~0.95) 的饱和弯曲。
 * 优先级 2：若尚未存有 15 点曲线或目标速度超出最高实测点，则使用该轮独立的线性死区+满速模型：
 *          u_ff = sign(t) * (db_i + |t| / vmax_i * (1 - db_i))
 */
float lf_wheel_feedforward(const LfController *c, unsigned wheel, float target_mm_s) {
    if (wheel > 1U || fabsf(target_mm_s) < 1.0f) return 0.0f;
    float abs_t = fabsf(target_mm_s);
    float db = c->cfg.wheel_deadband[wheel];
    float vmax = c->cfg.wheel_free_speed[wheel];
    float u_mag = db + abs_t / vmax * (1.0f - db);

    if (c->mcal_valid && c->mcal_max_steps >= 4U) {
        /* 在实测 (speed -> duty) 表中查找包含 abs_t 的相邻区间并做线性插值 */
        uint8_t n = c->mcal_max_steps;
        int first = -1;
        for (uint8_t s = 0; s < n; ++s) {
            if (c->mcal_curve_speed[s][wheel] > 8.0f) { first = (int)s; break; }
        }
        if (first >= 0) {
            float v_first = c->mcal_curve_speed[first][wheel];
            if (abs_t <= v_first) {
                float u0 = db < k_mcal_duties[first] ? db : k_mcal_duties[first] * 0.8f;
                u_mag = u0 + (abs_t / v_first) * (k_mcal_duties[first] - u0);
            } else {
                bool found = false;
                for (uint8_t s = (uint8_t)first; s + 1U < n; ++s) {
                    float v0 = c->mcal_curve_speed[s][wheel];
                    float v1 = c->mcal_curve_speed[s + 1U][wheel];
                    if (v1 > v0 + 1.0f && abs_t >= v0 && abs_t <= v1) {
                        u_mag = k_mcal_duties[s] +
                                (abs_t - v0) / (v1 - v0) * (k_mcal_duties[s + 1U] - k_mcal_duties[s]);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    float v_last = c->mcal_curve_speed[n - 1U][wheel];
                    float u_last = k_mcal_duties[n - 1U];
                    if (abs_t > v_last && vmax > v_last + 10.0f) {
                        u_mag = u_last + (abs_t - v_last) / (vmax - v_last) * (1.0f - u_last);
                    }
                }
            }
        }
    }
    return copysignf(limit(u_mag, 0.0f, 1.0f), target_mm_s);
}

static void speed_control(LfController *c, float left, float right, float dt, bool allow_rev) {
    float desired[2] = {left, right};
    c->out.enable = true;
    c->out.brake = false;
    for (unsigned i = 0; i < 2; ++i) {
        float lo = allow_rev ? -c->cfg.max_wheel_mm_s : 0;
        desired[i] = limit(desired[i], lo, c->cfg.max_wheel_mm_s);
        bool decelerating = fabsf(desired[i]) < fabsf(c->target_mm_s[i]) ||
                            (desired[i] * c->target_mm_s[i] < 0);
        float rate_scale = decelerating ? 3.5f : 1.0f;
        c->target_mm_s[i] = approach(c->target_mm_s[i], desired[i],
                                     c->cfg.acceleration_mm_s2 * rate_scale * dt);
        float t = c->target_mm_s[i];
        if (fabsf(t) < 1) {
            c->speed_integral[i] = c->applied_duty[i] = c->out.duty[i] = 0;
            continue;
        }
        /* 使用包含 15 点实测分段插值与独立死区/斜率的前馈计算 */
        float ff = lf_wheel_feedforward(c, i, t);
        float error = t - c->measured_mm_s[i];
        float old_integral = c->speed_integral[i];
        float candidate = limit(old_integral + c->cfg.speed_ki * error * dt,
                                -c->cfg.integral_limit, c->cfg.integral_limit);
        float u = ff;
        float duty_lo = allow_rev ? -c->cfg.max_duty : 0;
        if (c->cfg.speed_loop) {
            u += c->cfg.speed_kp * error + candidate;
            float clipped = limit(u, duty_lo, c->cfg.max_duty);
            if ((u - clipped) * error <= 0) c->speed_integral[i] = candidate;
            u = ff + c->cfg.speed_kp * error + c->speed_integral[i];
        } else c->speed_integral[i] = 0;
        u = limit(u, duty_lo, c->cfg.max_duty);
        c->applied_duty[i] = approach(c->applied_duty[i], u, c->cfg.duty_slew_s * rate_scale * dt);
        c->out.duty[i] = c->applied_duty[i];
    }
}

/* ============================================================================
 * 模块 7：5 轮空载 (LF_MCAL_AIR) 与 5 轮地面带载自动往返 (LF_MCAL_GROUND) 电机标定状态机
 * ============================================================================ */

/** @brief 将指定模式 (GROUND/AIR) 的 15 点曲线与死区/满速应用到实时循迹控制配置 */
static void apply_mcal_dataset(LfController *c, LfMcalMode mode) {
    const float *src_curve = (mode == LF_MCAL_GROUND) ? &c->mcal_ground_curve[0][0] : &c->mcal_air_curve[0][0];
    const float *src_db = (mode == LF_MCAL_GROUND) ? c->mcal_ground_db : c->mcal_air_db;
    const float *src_fs = (mode == LF_MCAL_GROUND) ? c->mcal_ground_fs : c->mcal_air_fs;
    memcpy(&c->mcal_curve_speed[0][0], src_curve, sizeof(c->mcal_curve_speed));
    for (unsigned w = 0; w < 2U; ++w) {
        c->cfg.wheel_deadband[w] = src_db[w];
        c->cfg.wheel_free_speed[w] = src_fs[w];
    }
    c->cfg.feedforward_deadband = 0.5f * (src_db[0] + src_db[1]);
    c->cfg.free_speed_mm_s = 0.5f * (src_fs[0] + src_fs[1]);
    c->mcal_active_mode = mode;
    c->mcal_valid = true;
}

bool lf_select_mcal_mode(LfController *c, LfMcalMode mode) {
    bool ok = (mode == LF_MCAL_GROUND) ? c->mcal_ground_valid : c->mcal_air_valid;
    if (!ok) return false;
    apply_mcal_dataset(c, mode);
    return true;
}

/**
 * @brief 完成全部 5 轮阶梯采集后，计算 15 点平均曲线并对左右轮独立做最小二乘线性回归拟合
 * @note  地面带载曲线 (GROUND) 为实车循迹主用模型；空载曲线 (AIR) 仅作机械参考，不覆盖已有 GROUND 模型
 */
static void finish_motor_cal(LfController *c, uint32_t now) {
    bool both_ok = true;
    float avg_curve[LF_MCAL_STEPS][2] = {{0}};
    float fit_db[2] = {0}, fit_fs[2] = {0};
    for (unsigned s = 0; s < c->mcal_max_steps; ++s) {
        uint16_t cnt = c->mcal_samples[s];
        avg_curve[s][0] = cnt ? c->mcal_sum_speed[s][0] / (float)cnt : 0;
        avg_curve[s][1] = cnt ? c->mcal_sum_speed[s][1] / (float)cnt : 0;
    }
    for (unsigned w = 0; w < 2; ++w) {
        float sum_u = 0, sum_v = 0, sum_uu = 0, sum_uv = 0;
        unsigned n_fit = 0;
        int first_spin = -1;
        for (unsigned s = 0; s < c->mcal_max_steps; ++s) {
            float v = avg_curve[s][w];
            if (first_spin < 0 && v > 15.0f) first_spin = (int)s;
            if (v > 25.0f) {
                float u = k_mcal_duties[s];
                sum_u += u; sum_v += v;
                sum_uu += u * u; sum_uv += u * v;
                ++n_fit;
            }
        }
        if (n_fit < 3U || first_spin < 0) { both_ok = false; break; }
        float denom = (float)n_fit * sum_uu - sum_u * sum_u;
        if (denom <= 1e-6f) { both_ok = false; break; }
        float slope = ((float)n_fit * sum_uv - sum_u * sum_v) / denom;
        float intercept = (sum_v - slope * sum_u) / (float)n_fit;
        if (slope < 100.0f) { both_ok = false; break; }
        float reg_db = limit(-intercept / slope, 0.02f, 0.35f);
        float break_db = first_spin > 0 ?
            0.5f * (k_mcal_duties[first_spin - 1] + k_mcal_duties[first_spin]) :
            k_mcal_duties[0];
        fit_db[w] = limit(0.65f * reg_db + 0.35f * break_db, 0.02f, 0.35f);
        fit_fs[w] = limit(slope * (1.0f - fit_db[w]), 150.0f, 1500.0f);
    }
    if (!both_ok) {
        lf_trip(c, LF_MOTOR_STALL);
        return;
    }
    if (c->mcal_mode == LF_MCAL_GROUND) {
        memcpy(c->mcal_ground_curve, avg_curve, sizeof(avg_curve));
        c->mcal_ground_db[0] = fit_db[0]; c->mcal_ground_db[1] = fit_db[1];
        c->mcal_ground_fs[0] = fit_fs[0]; c->mcal_ground_fs[1] = fit_fs[1];
        c->mcal_ground_valid = true;
        apply_mcal_dataset(c, LF_MCAL_GROUND);
    } else {
        memcpy(c->mcal_air_curve, avg_curve, sizeof(avg_curve));
        c->mcal_air_db[0] = fit_db[0]; c->mcal_air_db[1] = fit_db[1];
        c->mcal_air_fs[0] = fit_fs[0]; c->mcal_air_fs[1] = fit_fs[1];
        c->mcal_air_valid = true;
        /* 若地面带载尚未标定，则暂用空载作初始前馈；若地面已标定则始终保留地面主模型 */
        if (!c->mcal_ground_valid) apply_mcal_dataset(c, LF_MCAL_AIR);
    }
    c->mcal_done_event = true;
    c->out = (LfOutput){0};
    change_state(c, LF_STOPPED, now);
}

static void step_motor_cal(LfController *c, const LfInput *in, uint32_t now) {
    /* 地面模式下若超声波检测到前方 <18cm 障碍物，立即紧急停机防撞 */
    if (c->mcal_mode == LF_MCAL_GROUND && in->obstacle_valid &&
        in->obstacle_mm >= 20U && in->obstacle_mm < 180U && !c->mcal_returning) {
        lf_stop(c, now);
        return;
    }

    uint32_t elapsed = now - c->phase_since;

    /* 1. 每轮结束后的停轮制动与（地面模式）自动倒车回原点阶段 */
    if (c->mcal_step >= c->mcal_max_steps) {
        if (c->mcal_mode == LF_MCAL_GROUND && !c->mcal_return_done && !c->mcal_returning && c->odo.distance_mm > 25.0f) {
            if (elapsed < 150U) {
                c->out.enable = c->out.brake = true;
                return;
            }
            c->mcal_returning = true;
            c->phase_since = now;
            elapsed = 0;
        }
        if (c->mcal_returning) {
            /* 依托编码器里程计自动倒车回到每轮起跑原点（最小倒车占空比抬至 -0.36 防止地面静摩擦卡死），最长 9.0s */
            if (c->odo.distance_mm > 5.0f && elapsed < 9000U) {
                float rev = c->odo.distance_mm > 120.0f ? -0.48f : -0.36f;
                float corr = limit(c->odo.heading_rad * 0.18f, -0.06f, 0.06f);
                c->out.enable = true;
                c->out.brake = false;
                c->out.duty[0] = rev + corr;
                c->out.duty[1] = rev - corr;
                return;
            }
            c->mcal_returning = false;
            c->mcal_return_done = true;
            c->phase_since = now;
            elapsed = 0;
        }
        c->out.enable = c->out.brake = true;
        if (elapsed >= 250U) {
            ++c->mcal_pass;
            if (c->mcal_pass >= c->mcal_total_passes) {
                finish_motor_cal(c, now);
            } else {
                c->mcal_step = 0;
                c->mcal_step_pause = false;
                c->mcal_returning = false;
                c->mcal_return_done = false;
                c->odo.distance_mm = 0;
                c->odo.heading_rad = 0;
                c->phase_since = now;
            }
        }
        return;
    }

    /* 2. 地面模式下每步脉冲结束后的 120 ms 主动电磁刹车停顿，控制单步位移并让下档从静摩擦稳定再起步 */
    if (c->mcal_step_pause) {
        c->out.enable = c->out.brake = true;
        if (elapsed >= 120U) {
            c->mcal_step_pause = false;
            ++c->mcal_step;
            c->phase_since = now;
        }
        return;
    }

    /* 3. 当前占空比阶梯测试（给足稳速爬坡时间，避免过渡期低估稳态速度）：
     *    - 空载 (AIR)   : 每档 500 ms（前 250 ms 稳速 + 后 250 ms 采集 50 帧均值）
     *    - 地面 (GROUND): 每档 580 ms（前 340 ms 克服惯性爬上稳态 + 后 240 ms 采集 48 帧均值） */
    uint32_t settle_ms = (c->mcal_mode == LF_MCAL_GROUND) ? 340U : 250U;
    uint32_t step_ms = (c->mcal_mode == LF_MCAL_GROUND) ? 580U : 500U;
    float duty = k_mcal_duties[c->mcal_step];
    c->out.enable = true;
    c->out.brake = false;
    c->out.duty[0] = c->out.duty[1] = duty;
    if (elapsed >= settle_ms) {
        c->mcal_sum_speed[c->mcal_step][0] += c->measured_mm_s[0];
        c->mcal_sum_speed[c->mcal_step][1] += c->measured_mm_s[1];
        ++c->mcal_samples[c->mcal_step];
        c->mcal_pass_sum[0] += c->measured_mm_s[0];
        c->mcal_pass_sum[1] += c->measured_mm_s[1];
        ++c->mcal_pass_samples;
    }
    if (elapsed >= step_ms) {
        uint16_t p_cnt = c->mcal_pass_samples;
        c->mcal_last_pass = (uint8_t)(c->mcal_pass + 1U);
        c->mcal_last_step = c->mcal_step;
        c->mcal_last_duty = duty;
        c->mcal_last_speed[0] = p_cnt ? c->mcal_pass_sum[0] / (float)p_cnt : 0;
        c->mcal_last_speed[1] = p_cnt ? c->mcal_pass_sum[1] / (float)p_cnt : 0;
        c->mcal_pass_samples = 0;
        c->mcal_pass_sum[0] = c->mcal_pass_sum[1] = 0;
        c->mcal_step_ready = true;
        if (c->mcal_mode == LF_MCAL_GROUND) {
            c->mcal_step_pause = true;
            c->phase_since = now;
        } else {
            ++c->mcal_step;
            c->phase_since = now;
        }
    }
}

/* ============================================================================
 * 模块 8：提高部分（2）四面顺时针目标探测与 10 秒激光打靶状态机
 * ============================================================================ */

static bool morse_2short_1long(uint32_t elapsed_ms) {
    if (elapsed_ms >= 3600U) return false;
    uint32_t slot = elapsed_ms % 1200U;
    return (slot < 150U) || (slot >= 300U && slot < 450U) || (slot >= 600U && slot < 1050U);
}

/* ============================================================================
 * 模块 8a：云台步进扫掠 + 平面法线（垂直位置）最小二乘拟合
 * ============================================================================
 * 对一堵垂直于地面的平面，从不同水平角度 theta 测距满足 1/r(theta) = a*cos(theta) + b*sin(theta)，
 * 其法线（即小车真正需要“正对/垂直”的方向）位于 theta* = atan2(b, a)。
 * 因此本模块按固定脉宽步长离散驻留采样，再对连续有效回波段做二维最小二乘拟合求出平面法线，
 * 相比“只记录单点最小距离”能显著抵抗超声噪声、斜射回波衰减与多径干扰。
 */

/**
 * @brief 单个扫掠采样点的最短驻留时长 (ms)
 * @note  60 ms 覆盖一次硬件超声触发周期，另 60 ms 用于等舵机走完这一步（SG90 约 10 ms/deg，
 *        7.5° 一步约 75 ms）。驻留不足会让采样点混入上一角度的旧回波，使法线出现 1~2° 系统偏置。
 */
#define LF_PROBE_SCAN_MIN_DWELL_MS 180U

/** @brief 每步进之后丢弃的最初时长 (ms)：这段时间内的回波可能还是上一个角度触发的 */
#define LF_PROBE_SCAN_SKIP_MS 120U

/** @brief 瞄准阶段开始前让舵机在首点稳定下来的时间上限 (ms) */
#define LF_PROBE_AIM_SETTLE_MS 150U

/** @brief 第 index 个离散采样点对应的舵机脉宽 (us，按 ±sweep_us 线性分布并限幅到 600..2400 全行程) */
static uint16_t probe_scan_pulse(const LfController *c, uint16_t index) {
    float us = probe_scan_us_at(c, (float)index);
    if (us < (float)LF_SERVO_US_MIN) us = (float)LF_SERVO_US_MIN;
    if (us > (float)LF_SERVO_US_MAX) us = (float)LF_SERVO_US_MAX;
    return (uint16_t)(us + 0.5f);
}

/** @brief 只清空扫掠采样缓存（两段式扫掠的第二段复用） */
static void probe_scan_clear(LfController *c) {
    memset(c->scan_range_mm, 0, sizeof(c->scan_range_mm));
    memset(c->scan_sum_mm, 0, sizeof(c->scan_sum_mm));
    memset(c->scan_hits, 0, sizeof(c->scan_hits));
    c->scan_points = 0;
    c->scan_index = 0;
    c->best_servo_us = c->cfg.servo_center_us;
    c->best_obstacle_mm = UINT16_MAX;
}

/** @brief 清空云台步进扫掠采样缓存、复位本轮瞄准结果，并回到第一段（粗扫） */
static void probe_scan_reset(LfController *c) {
    probe_scan_clear(c);
    memset(&c->face, 0, sizeof(c->face));
    memset(&c->aim_first, 0, sizeof(c->aim_first));
    c->aim_span_us = c->cfg.probe.sweep_us;
    c->aim_center_us = c->cfg.servo_center_us;
    c->aim_pass = 0U;
}

/** @brief 依据扫掠半幅、步长与总时长规划本轮离散采样点数（保证每点驻留 >= LF_PROBE_SCAN_MIN_DWELL_MS） */
static uint16_t probe_scan_plan(LfController *c, uint32_t sweep_ms) {
    const LfProbeConfig *pr = &c->cfg.probe;
    uint32_t step = (c->aim_pass == 0U) ? (pr->scan_step_us ? (uint32_t)pr->scan_step_us : 60U)
                                        : (uint32_t)LF_PROBE_AIM2_STEP_US;
    uint32_t n = (2U * (uint32_t)c->aim_span_us) / step + 1U;
    uint32_t max_n = sweep_ms / LF_PROBE_SCAN_MIN_DWELL_MS;
    if (max_n < 3U) max_n = 3U;
    if (n < 3U) n = 3U;
    if (n > (uint32_t)LF_PROBE_SCAN_POINTS) n = (uint32_t)LF_PROBE_SCAN_POINTS;
    if (n > max_n) n = max_n;
    c->scan_points = (uint8_t)n;
    return (uint16_t)n;
}

/**
 * @brief 本轮扫掠的方向：+1 = 从 −span 往 +span 扫（第一段），−1 = 反过来（第二段反向扫）
 * @note  2026-11-23：第二段改成"与第一段同幅、但反向"的独立复测。
 *        依据：两轮实测里第二段相对第一段的 φ 差是 **+8.51° / +8.14°**（几乎恒定），
 *        这种"方向固定、偏差恒定"的东西是单向扫掠下舵机回差/滞环引起的角度滞后，
 *        取反方向扫一次再平均即可对消（噪声也顺带降 √2）。
 *        附带好处：第一段结束停在 +span，第二段正好从 +span 往回扫，舵机不用再空跑一趟。 */
static float probe_scan_dir(const LfController *c) {
    return (c->aim_pass == 1U) ? -1.0f : 1.0f;
}

/** @brief 离散（可为小数）序号 → 舵机脉宽 (us)。扫掠方向由 probe_scan_dir() 决定 */
static float probe_scan_us_at(const LfController *c, float index) {
    float span = c->scan_points > 1U ? (float)(c->scan_points - 1U) : 1.0f;
    return (float)c->aim_center_us +
           probe_scan_dir(c) * (-(float)c->aim_span_us +
                                2.0f * (float)c->aim_span_us * index / span);
}

/** @brief 第 index 个离散采样点对应的方位角 (rad，相对车头正前，向左为正) */
static float probe_scan_angle(const LfController *c, unsigned index) {
    float us = probe_scan_us_at(c, (float)index);
    return ((us - (float)c->cfg.servo_center_us) / LF_PROBE_SCAN_US_PER_DEG) * 0.01745329252f;
}

/**
 * @brief 内点判据：实测距离与该角度下的模型预测距离之差是否在带宽内
 * @param rho  面到云台轴的垂直距离 (mm)
 * @param phi  面法线方位角 (rad)
 * @param th   采样方位角 (rad)
 * @param d    该点实测距离 (mm，0 = 无回波)
 * @param band 允许偏差 (mm)
 */
static bool probe_inlier(float rho, float phi, float th, uint16_t d, float band) {
    if (d == 0U) return false;
    float pred = rho / cosf(th - phi);
    if (!(pred > 40.0f) || !(pred < 400.0f)) return false;
    return fabsf((float)d - pred) <= band;
}

/**
 * @brief 对最长连续有效回波段做 1/r = a*cos(theta) + b*sin(theta) 最小二乘拟合，求出平面法线方向
 * @param[out] pulse_us    正对平面法线方向的舵机脉宽 (us)
 * @param[out] distance_mm 该方向上实测的回波距离 (mm)
 * @return true = 拟合成功；false = 有效点不足或拟合残差 > 12%（调用方改用三点平滑最小值兜底）
 */
static bool probe_fit_normal(LfController *c, uint16_t *pulse_us, uint16_t *distance_mm) {
    unsigned n = c->scan_points;
    if (n < 3U) return false;
    /* 先把扫掠切成「同一物体」的连续段：回波为 0 或相邻两点距离突跳都算断点 */
    unsigned run_start = 0, run_len = 0;
    unsigned cand_start[LF_PROBE_MAX_RUNS], cand_len[LF_PROBE_MAX_RUNS];
    uint32_t cand_sum[LF_PROBE_MAX_RUNS];
    unsigned cand_n = 0U;
    for (unsigned i = 0; i <= n; ++i) {
        bool ok = (i < n) && (c->scan_range_mm[i] != 0U);
        if (ok && run_len > 0U) {
            int32_t prev = (int32_t)c->scan_range_mm[i - 1U];
            int32_t cur = (int32_t)c->scan_range_mm[i];
            int32_t jump = cur > prev ? cur - prev : prev - cur;
            if (jump > (int32_t)LF_PROBE_RUN_JUMP_MM) {
                if (cand_n < LF_PROBE_MAX_RUNS) {
                    uint32_t sum = 0U;
                    for (unsigned k = run_start; k < run_start + run_len; ++k) sum += c->scan_range_mm[k];
                    cand_start[cand_n] = run_start; cand_len[cand_n] = run_len; cand_sum[cand_n] = sum;
                    ++cand_n;
                }
                run_len = 0U;
            }
        }
        if (ok) {
            if (run_len == 0U) run_start = i;
            ++run_len;
        } else if (run_len > 0U) {
            if (cand_n < LF_PROBE_MAX_RUNS) {
                uint32_t sum = 0U;
                for (unsigned k = run_start; k < run_start + run_len; ++k) sum += c->scan_range_mm[k];
                cand_start[cand_n] = run_start; cand_len[cand_n] = run_len; cand_sum[cand_n] = sum;
                ++cand_n;
            }
            run_len = 0U;
        }
    }

    /* 选段：障碍物是「车停下来时最近的那个面」，所以在长度 >= 3 的候选段里取平均距离最小的；
     * 都不到 3 点时退回最长的一段。 */
    unsigned best_start = 0U, best_len = 0U;
    uint32_t best_mean = UINT32_MAX;
    for (unsigned k = 0; k < cand_n; ++k) {
        if (cand_len[k] < 3U) continue;
        uint32_t mean = cand_sum[k] / cand_len[k];
        if (mean < best_mean) {
            best_mean = mean; best_start = cand_start[k]; best_len = cand_len[k];
        } else if (mean == best_mean && cand_len[k] > best_len) {
            best_start = cand_start[k]; best_len = cand_len[k];
        }
    }
    if (best_len == 0U) {
        for (unsigned k = 0; k < cand_n; ++k) {
            if (cand_len[k] > best_len) { best_len = cand_len[k]; best_start = cand_start[k]; }
        }
    }
    if (best_len < 3U) return false;

    const float half_us = (float)c->aim_span_us;
    float offset_per_index = (2.0f * half_us) / (float)(n - 1U); /* 相邻采样点的脉宽差 (us) */

    /* 首尾各去掉一个点：目标恰好进出波束边缘时回波幅度最容易被低估 */
    unsigned fit_start = best_start + (best_len >= 5U ? 1U : 0U);
    unsigned fit_end = best_start + best_len - (best_len >= 5U ? 1U : 0U);
    float scc = 0, sss = 0, scs = 0, scz = 0, ssz = 0;
    for (unsigned i = fit_start; i < fit_end; ++i) {
        /* 一律用「相对车头正前」的绝对角度，保证两段式扫掠中心不同时模型仍然一致 */
        float radians = probe_scan_angle(c, i);
        float co = cosf(radians), si = sinf(radians);
        float z = 1.0f / (float)c->scan_range_mm[i];
        scc += co * co; sss += si * si; scs += co * si;
        scz += co * z;  ssz += si * z;
    }
    float det = scc * sss - scs * scs;
    float chosen_index = -1.0f;
    float plane_a = 0.0f, plane_b = 0.0f; /* 1/d = a*cos(th) + b*sin(th) 的两个系数 */
    bool model_ok = false;
    if (det > 0.0001f) {
        float a = (scz * sss - ssz * scs) / det;
        float b = (ssz * scc - scz * scs) / det;
        float normal_deg = atan2f(b, a) * 57.29577951f;
        plane_a = a;
        plane_b = b;
        model_ok = true;
        /* 法线角是「相对车头正前」的绝对角，而采样点是以 aim_center_us 为中心的，
         * 所以先把法线角换算成相对本轮扫掠中心的偏移，才能反解出采样点序号 */
        float center_deg = ((float)c->aim_center_us - (float)c->cfg.servo_center_us) /
                           LF_PROBE_SCAN_US_PER_DEG;
        chosen_index = (probe_scan_dir(c) * (normal_deg - center_deg) * LF_PROBE_SCAN_US_PER_DEG +
                        half_us) / offset_per_index;
        float relative_error = 0;
        for (unsigned i = fit_start; i < fit_end; ++i) {
            float radians = probe_scan_angle(c, i);
            float predicted = a * cosf(radians) + b * sinf(radians);
            float measured = 1.0f / (float)c->scan_range_mm[i];
            relative_error += fabsf(predicted - measured) / measured;
        }
        relative_error /= (float)(fit_end - fit_start);
        if (relative_error > 0.12f || chosen_index < (float)best_start ||
            chosen_index > (float)(best_start + best_len - 1U)) chosen_index = -1.0f;
    }
    if (chosen_index < 0.0f) {
        /* 曲面或回波抖动较大时退回三点平滑最小值（等价于参考工程 Tracker_ChooseNormal 的兜底分支） */
        uint32_t smallest = UINT32_MAX;
        unsigned fallback = best_start + best_len / 2U;
        for (unsigned i = best_start + 1U; i + 1U < best_start + best_len; ++i) {
            uint32_t sum = (uint32_t)c->scan_range_mm[i - 1U] + (uint32_t)c->scan_range_mm[i] +
                           (uint32_t)c->scan_range_mm[i + 1U];
            if (sum < smallest) { smallest = sum; fallback = i; }
        }
        chosen_index = (float)fallback;
    }
    unsigned nearest = (unsigned)(chosen_index + 0.5f);
    if (nearest < best_start) nearest = best_start;
    if (nearest >= best_start + best_len) nearest = best_start + best_len - 1U;
    float us = probe_scan_us_at(c, chosen_index);
    if (us < (float)LF_SERVO_US_MIN) us = (float)LF_SERVO_US_MIN;
    if (us > (float)LF_SERVO_US_MAX) us = (float)LF_SERVO_US_MAX;
    *pulse_us = (uint16_t)(us + 0.5f);
    *distance_mm = c->scan_range_mm[nearest];

    /* --- M1：把平面模型交给上层（phi/rho/rms/hits），供 PROBE_FACE 日志与 M2 侧视横扫复用 --- */
    memset(&c->face, 0, sizeof(c->face));
    if (model_ok) {
        float phi = atan2f(plane_b, plane_a);
        float rho = 1.0f / sqrtf(plane_a * plane_a + plane_b * plane_b);
        if (isfinite(phi) && isfinite(rho) && rho > 1.0f) {
            float e2 = 0.0f;
            unsigned used = 0, inl = 0;
            for (unsigned i = fit_start; i < fit_end; ++i) {
                float th = probe_scan_angle(c, i);
                float e = (float)c->scan_range_mm[i] - rho / cosf(th - phi);
                e2 += e * e;
                ++used;
                if (probe_inlier(rho, phi, th, c->scan_range_mm[i], LF_PROBE_INLIER_BAND_MM)) ++inl;
            }
            c->face.phi = phi;
            c->face.rho = rho;
            c->face.rms = used ? sqrtf(e2 / (float)used) : 99.0f;
            c->face.hits = (uint8_t)(inl > 255U ? 255U : inl);
            c->face.valid = (inl >= LF_PROBE_MIN_INLIERS) && (c->face.rms < c->cfg.probe.fit_rms_mm) &&
                            (rho > 60.0f) && (rho < 400.0f);
        }
    }
    return true;
}

/** @brief 结束一轮扫掠：各点累加值换算为平均距离，再拟合平面法线并锁存最佳瞄准脉宽/距离 */
static void probe_scan_finish(LfController *c) {
    unsigned n = c->scan_points;
    for (unsigned i = 0; i < n; ++i) {
        uint32_t hits = c->scan_hits[i];
        c->scan_range_mm[i] = hits ? (uint16_t)(c->scan_sum_mm[i] / hits) : 0U;
    }
    uint16_t best_mm = UINT16_MAX, best_us = c->cfg.servo_center_us;
    for (unsigned i = 0; i < n; ++i) {
        if (c->scan_range_mm[i] != 0U && c->scan_range_mm[i] < best_mm) {
            best_mm = c->scan_range_mm[i];
            best_us = probe_scan_pulse(c, (uint16_t)i);
        }
    }
    uint16_t fit_us = 0, fit_mm = 0;
    if (probe_fit_normal(c, &fit_us, &fit_mm)) { best_us = fit_us; best_mm = fit_mm; }
    c->best_servo_us = best_us;
    c->best_obstacle_mm = best_mm;
}

/* ============================================================================
 * 模块 8b：M2 侧视横扫（SIDE_LOCK / SIDE_SCAN / SIDE_MOVE + 发射前复测）
 * ============================================================================
 * 侧视法：车头平行于被测面，舵机掰到 ±90° 固定不动，车沿面的切线方向平移扫掠。
 *   - 扫描坐标 s = 车体（轮轴中点）沿扫描轴的有符号位移，由里程计给出（不是 distance_mm 路程）；
 *   - 面内回波基本是常数 h（法向入射，不需要 1/cos 修正），用 8 点直线 d = h0 + k·s 吃掉
 *     「车不平行」造成的 h 随 s 线性漂移；
 *   - 边界 = 回波跳变处，位置取「最后一个面内样本 + 第一个外点样本」的中点；
 *   - 中点 s_m = (s_L + s_R)/2；宽度 w = (s_R - s_L) - 2·h·tanγ（γ 只影响宽度，不影响中点）；
 *   - 开位：沿切线直行到 s_m 停车（非完整约束不影响），保持同一舵机角发射激光即命中面中心。
 * 关键性质（《方案_四面靶心定位与绕行.md》§3.3）：扫描与发射共用同一根转轴、同一个舵机角，
 * 边界位置与落点是同一个仿射映射，所以 pan_x、侧角是否正好 90°、车头残留几度不平行，
 * 都不会让光斑偏离面中心——本模块刻意不在 s_m 上做任何 pan_x 修正。
 * ========================================================================== */

/** @brief 清空滑动环与连续外点计数（换向、过边界后重建面内基线前调用） */
static void probe_side_ring_reset(LfController *c) {
    c->side_ring_n = 0U;
    c->side_ring_i = 0U;
    c->side_bad = 0U;
    c->side_out_zero = 0U;
    c->side_s_out_first = 0.0f;
}

/** @brief 把一帧面内样本压入 8 点滑动环（环形覆盖，最小二乘与顺序无关） */
static void probe_side_ring_push(LfController *c, float s, float d) {
    /* 长基线累加（面内样本）：给"扫描结束后的二次微调"用，判车头与面是否平行。
     * 8 点环只有 ~24 mm 基线、斜率还被 LF_SIDE_K_LIMIT 钳住；这里用整遍扫描的全部面内样本，
     * 基线 ±90 mm、样本上百个，实测 σ=0.55~0.70°。 */
    if (c->side_fit_n == 0U) { c->side_fit_min_s = s; c->side_fit_max_s = s; }
    else {
        if (s < c->side_fit_min_s) c->side_fit_min_s = s;
        if (s > c->side_fit_max_s) c->side_fit_max_s = s;
    }
    c->side_fit_sum_s += s;
    c->side_fit_sum_d += d;
    c->side_fit_sum_ss += s * s;
    c->side_fit_sum_sd += s * d;
    if (c->side_fit_n < UINT16_MAX) ++c->side_fit_n;
    c->side_ring_s[c->side_ring_i] = s;
    c->side_ring_d[c->side_ring_i] = d;
    c->side_ring_i = (uint8_t)((c->side_ring_i + 1U) % LF_SIDE_RING);
    if (c->side_ring_n < LF_SIDE_RING) ++c->side_ring_n;
    if (c->side_inface < UINT16_MAX) ++c->side_inface;
}

/** @brief 清空一遍侧扫的全部判定状态（不动 side_h / 原点 / 复测标志） */
static void probe_side_scan_reset(LfController *c) {
    c->side_stage = (uint8_t)LF_SIDE_FWD;
    c->side_edges = 0U;
    c->side_verify_mode = 0U;
    c->side_warn = 0U;
    c->side_s_last_good = 0.0f;
    c->side_s_turn = 0.0f;
    c->side_h0 = c->side_h;
    c->side_k = 0.0f;
    c->side_inface = 0U;
    c->side_edge_frames = 0U;
    c->side_fit_sum_s = 0.0f;
    c->side_fit_sum_d = 0.0f;
    c->side_fit_sum_ss = 0.0f;
    c->side_fit_sum_sd = 0.0f;
    c->side_fit_min_s = 0.0f;
    c->side_fit_max_s = 0.0f;
    c->side_fit_n = 0U;
    /* side_yaw_deg 不在这里清：它要在重扫之后仍然保留，供 PROBE_SIDE_YAW 日志与单测读；
     * 每面一次的清零在 probe_side_lock_begin() 里做。 */
    probe_side_ring_reset(c);
}

/**
 * @brief 侧扫轴坐标：相对「本遍扫描起点」沿扫描轴（起点处的车头方向）的有符号位移 (mm)
 * @note  刻意不用 phase_start_*：change_probe_phase() 每换一个子阶段都会重设它，
 *        而 SIDE_SCAN / SIDE_MOVE / 复测扫描必须共用同一根轴（《接口契约》§7 函数表）。
 */
static float probe_side_along(const LfController *c) {
    float dx = c->odo.x_mm - c->side_origin_x_mm;
    float dy = c->odo.y_mm - c->side_origin_y_mm;
    return dx * cosf(c->side_origin_rad) + dy * sinf(c->side_origin_rad);
}

/** @brief 锁存侧扫轴原点（车体位姿 + 航向）并按 60 ms 超声节拍重新起算采样计时 */
static void probe_side_begin(LfController *c, uint32_t now) {
    c->side_origin_x_mm = c->odo.x_mm;
    c->side_origin_y_mm = c->odo.y_mm;
    c->side_origin_rad = c->odo.heading_rad;
    c->side_ping_since = now;
    c->face.id = c->probe_face;
    probe_side_scan_reset(c);
}

/* 2026-11-22：扫描期按 side_k 修车头的 probe_drive_face 已撤掉。
 * 台架实测效果一般，且 side_k 会被电机死区造成的走弯污染（k≈0.25 里有一部分是弧线，
 * 不是静态斜停）。侧扫/开位一律回到 probe_drive_signed：锁 phase_start_rad。 */

/**
 * @brief 沿当前航向有符号直行（带里程计航向纠偏），侧扫与开位共用
 * @note  纠偏项与行驶方向无关：dθ = (右-左)/轮距，所以倒车时仍然是 (spd + corr, spd - corr) 拉回原航向。
 * @note  增益 120→300、限幅 30→40（2026-11-23）：SIDE_SCAN 第一遍扫实测航向漂了 **2.7°**
 *        （hdg −88.21→−85.51），直接把 w 从真值 150 拉成 124.9；而同一轮的复测扫只漂 0.49°、w 回到 145.7。
 *        120 意味着"每偏 1° 只给 2.1 mm/s 差速"（≈6°/s 的纠正能力），持续性漂移会留下 2~3° 稳态误差。
 *        300 对应 ≈15°/s，稳态误差应压到 ~1°。若出现低频摆动，把这两个数改回 120 / 30 即可。
 */
static void probe_drive_signed(LfController *c, float speed_mm_s, float dt) {
    float sign = speed_mm_s >= 0.0f ? 1.0f : -1.0f;
    float spd = fabsf(speed_mm_s);
    float corr = limit((c->odo.heading_rad - c->phase_start_rad) * 300.0f, -40.0f, 40.0f);
    speed_control(c, sign * spd + corr, sign * spd - corr, dt, true);
}

/** @brief 清空加减速斜坡状态（DRIVE 切主动制动时用，避免残留目标速度在下一次起步瞬间生效） */
static void probe_motion_reset(LfController *c) {
    for (unsigned i = 0; i < 2U; ++i) {
        c->target_mm_s[i] = 0.0f;
        c->applied_duty[i] = 0.0f;
        c->speed_integral[i] = 0.0f;
    }
}

/** @brief 对滑动环里的 (s, d) 做最小二乘直线拟合 d = h0 + k·s；样本不足返回 false */
static bool probe_side_fit(const LfController *c, float *h0, float *k) {
    float n = (float)c->side_ring_n;
    if (n < 2.0f) return false;
    float ss = 0.0f, sd = 0.0f, sss = 0.0f, ssd = 0.0f;
    for (uint8_t i = 0; i < c->side_ring_n; ++i) {
        float s = c->side_ring_s[i];
        float d = c->side_ring_d[i];
        ss += s;
        sd += d;
        sss += s * s;
        ssd += s * d;
    }
    float den = n * sss - ss * ss;
    if (fabsf(den) < 1.0e-3f) {
        *k = 0.0f;
        *h0 = sd / n;
        return true;
    }
    *k = limit((n * ssd - ss * sd) / den, -LF_SIDE_K_LIMIT, LF_SIDE_K_LIMIT);
    *h0 = (sd - (*k) * ss) / n;
    return true;
}

/**
 * @brief 侧扫单帧喂入：维护面内直线模型，连续外点判定边界
 * @param s 当前扫描坐标 (mm)
 * @param d 当前侧视回波 (mm，0 = 无回波/丢帧)
 */
static void probe_side_feed(LfController *c, float s, uint16_t d) {
    const LfProbeConfig *pr = &c->cfg.probe;
    /* 起步/换向后先重建基线：只收与 SIDE_LOCK 实测 h 接近的样本，避免背景墙（几百 mm）污染模型 */
    if (c->side_ring_n < LF_SIDE_RING_MIN) {
        if (d != 0U && fabsf((float)d - c->side_h) <= pr->side_jump_mm) {
            probe_side_ring_push(c, s, (float)d);
            c->side_s_last_good = s;
            c->side_bad = 0U;
            c->side_out_zero = 0U;
        }
        return;
    }
    float h0 = 0.0f, k = 0.0f;
    if (!probe_side_fit(c, &h0, &k)) return;
    float pred = h0 + k * s;
    float dev = d != 0U ? fabsf((float)d - pred) : 1.0e9f;
    if (dev <= pr->side_band_mm) {
        /* 面内：更新模型（k 吸收车不平行造成的 h 随 s 线性漂移） */
        probe_side_ring_push(c, s, (float)d);
        c->side_s_last_good = s;
        c->side_h0 = h0;
        c->side_k = k;
        c->side_bad = 0U;
        c->side_out_zero = 0U;
    } else if (dev > pr->side_jump_mm) {
        /* 外点。两类证据分开累计：
         *   ① 跳变型：有回波、且与基线 h 的差 > jump —— 说明确实换了一个物体（背景墙），3 帧确认；
         *   ② 无回波型：d == 0 —— 丢帧与真出界不可区分，只能靠持续时间，6 帧确认。
         * 只有"偏离预测"但回波仍接近 h（例如面外 20 mm 处有墙）时两类都不成立，当歧义继续扫，
         * 最终由 reach/timeout 判失败——宁可不打这一面，也不要拿假边界算中点。 */
        if (c->side_bad == 0U) c->side_s_out_first = s;
        if (c->side_bad < 255U) ++c->side_bad;
        if (d == 0U) {
            if (c->side_out_zero < 255U) ++c->side_out_zero;
        }
        bool jump_edge = (d != 0U) && (c->side_bad >= LF_SIDE_EDGE_JUMP_FRAMES) &&
                         (fabsf((float)d - c->side_h) > pr->side_jump_mm);
        bool zero_edge = c->side_out_zero >= LF_SIDE_EDGE_ZERO_FRAMES;
        /* 还要求自上次边界以来已积累足够的面内证据：防止「擦边丢帧」被当成边界 */
        bool enough_inface = c->side_inface >= LF_SIDE_EDGE_MIN_INFACE;
        if (enough_inface && (jump_edge || zero_edge)) {
            float edge = 0.5f * (c->side_s_last_good + c->side_s_out_first);
            if (c->side_edges < 2U) c->side_s_edge[c->side_edges++] = edge;
            c->side_edge_frames = c->side_inface > 255U ? 255U : (uint8_t)c->side_inface;
            c->side_inface = 0U;
            c->side_event |= (uint8_t)LF_SIDE_EVENT_EDGE;
            probe_side_ring_reset(c);
        }
    } else {
        /* 中间带（band < dev <= jump）：当噪声丢弃，不更新模型也不累加外点 */
        c->side_bad = 0U;
        c->side_out_zero = 0U;
    }
}

/**
 * @brief 由两个边界计算 s_l / s_r / s_m / w / kind（γ 补偿只作用在 s_l/s_r/w，中点天然对称抵消）
 * @return false = 边界不足 2 个
 */
static bool probe_side_center(LfController *c) {
    if (c->side_edges < 2U) return false;
    float e0 = c->side_s_edge[0];
    float e1 = c->side_s_edge[1];
    float lo = fminf(e0, e1);
    float hi = fmaxf(e0, e1);
    float gamma = c->cfg.probe.beam_half_deg * 0.01745329252f;
    float reach = c->side_h * tanf(gamma); /* 波束足印半宽：两侧同向外扩 → 中点不动 */
    c->face.s_l = lo + reach;
    c->face.s_r = hi - reach;
    c->face.s_m = 0.5f * (lo + hi);
    c->face.w = (hi - lo) - 2.0f * reach;
    c->face.h = c->side_h;
    c->face.kind = c->face.w < LF_SIDE_KIND_SPLIT_MM ? 0U : 1U;
    return true;
}

/**
 * @brief 扫描结束后自检：车头与面是否不平行到值得"转正后重扫一遍"
 * @return true = 需要转正；此时 side_yaw_rad 就是要转的相对角（正=左转/CCW）
 * @note  用整遍扫描的**全部面内样本**做长基线回归（不是 8 点环）：d(s) = h0 + k·s，
 *        dh/ds = -tan(δ)，δ 是车头相对面的偏航。要转正就是转 -δ = atan(k)（右面镜像取反）。
 *        实测 σ=0.55~0.70°（181~486 样本），4° 死区 ≈6σ；好工况（残余 2~3°）一次都不转。
 *        当年 7bded7f 那版要求平移 ≥100 mm、而 SIDE_MOVE 只走 |s_m|~20 mm，数学上永不触发；
 *        现在把判据放在**扫描结束**处（扫描行程 ±90 mm 起），才是它该在的位置。
 */
static bool probe_side_yaw_should_fix(LfController *c) {
    if (!LF_SIDE_YAW_ENABLE) return false;                   /* 总开关（见 LF_SIDE_YAW_ENABLE） */
    if (c->side_yaw_fix != 0U) return false;                 /* 每面只做一次 */
    if (c->side_fit_n < LF_SIDE_YAW_MIN_SAMPLES) return false;
    if (c->side_fit_max_s - c->side_fit_min_s < LF_SIDE_YAW_MIN_SPAN_MM) return false;
    /* 面本身都没量对就不许用它的斜率去转车：2026-11-23 22:33 第 4 面只擦到窄面的一条边，
     * w=16.4（两条边相距 16.5 mm），拟合却给出 atan(k)=−18.4°，车被转 18.4° 后重扫一条边
     * 都没有 —— 整面丢失。斜率只在"确实横穿了一整个面"时才有物理意义。 */
    if (!probe_side_width_ok(c->face.w)) return false;
    float n = (float)c->side_fit_n;
    float den = n * c->side_fit_sum_ss - c->side_fit_sum_s * c->side_fit_sum_s;
    if (den < 1.0e3f) return false;                           /* 退化：样本都挤在一点 */
    float k = (n * c->side_fit_sum_sd - c->side_fit_sum_s * c->side_fit_sum_d) / den;
    /* 推导：面沿 x、车在 y=h、车头与面夹角 ψ 时，侧视读数是 (h − s·sinψ)/cosψ
     *       → dh/ds = −tanψ → ψ = −atan(k)；要摆正就得转 −ψ = atan(k)。
     * 实测校验（probe_fix4）：k=−0.0432 → 应转 −2.47°，而由侧扫反推的正确航向
     * 也确实比 TURN_PAR 给的再低 2.47°。右视面几何镜像 → 取反。 */
    float rot = atanf(k);                                     /* 左视面：要转的角 */
    if (!c->cfg.probe.side_left) rot = -rot;                  /* 镜像面：符号取反 */
    c->side_yaw_deg = rot * 57.2957795f;
    if (fabsf(c->side_yaw_deg) < LF_SIDE_YAW_DEADBAND_DEG) return false;
    if (fabsf(c->side_yaw_deg) > LF_SIDE_YAW_MAX_DEG) return false;
    c->side_yaw_rad = rot;
    return true;
}

/** @brief 由最终确定的 s_m / w 反推 s_l / s_r，保证三者满足 s_l = s_m - w/2、s_r = s_m + w/2（复测平均后必须重算） */
static void probe_side_sync_lr(LfController *c) {
    c->face.s_l = c->face.s_m - 0.5f * c->face.w;
    c->face.s_r = c->face.s_m + 0.5f * c->face.w;
}

/** @brief 面宽是否落在两种面宽的合理测量区间：真值 50/150 与实车波束偏置后的 ~110/146~172 都接受 */
static bool probe_side_width_ok(float w) {
    return (w >= 30.0f && w <= 70.0f) || (w >= 80.0f && w <= 130.0f) ||
           (w >= 140.0f && w <= 190.0f);
}

/** @brief 取 side_ring_d[0..n-1] 的中位数（SIDE_LOCK 的 5 帧；插入排序，n 很小） */
static float probe_side_median(const LfController *c, uint8_t n) {
    float v[LF_SIDE_LOCK_FRAMES];
    if (n > (uint8_t)LF_SIDE_LOCK_FRAMES) n = (uint8_t)LF_SIDE_LOCK_FRAMES;
    for (uint8_t i = 0; i < n; ++i) v[i] = c->side_ring_d[i];
    for (uint8_t i = 1U; i < n; ++i) {
        float key = v[i];
        int j = (int)i - 1;
        while (j >= 0 && v[j] > key) {
            v[j + 1] = v[j];
            --j;
        }
        v[j + 1] = key;
    }
    return v[n / 2U];
}

/** @brief 波束足印半宽 h·tanγ (mm)：边界会向外多出这一段，行程预算与 γ 补偿都要用它 */
static float probe_side_footprint(const LfController *c) {
    return c->side_h * tanf(c->cfg.probe.beam_half_deg * 0.01745329252f);
}

/** @brief 边界确认距离 (mm)：连续 LF_SIDE_EDGE_ZERO_FRAMES 帧 × 每帧行程 + 余量 */
static float probe_side_confirm_mm(const LfController *c) {
    float step = c->cfg.probe.side_speed_mm_s * ((float)LF_SIDE_PING_MS * 0.001f);
    return (float)LF_SIDE_EDGE_ZERO_FRAMES * step + LF_SIDE_CONFIRM_EXTRA_MM;
}

/** @brief 侧扫引擎返回码（与 LfSideStage 的 LF_SIDE_DONE 区分命名） */
typedef enum { LF_SIDE_SCAN_RUN = 0, LF_SIDE_SCAN_OK = 1, LF_SIDE_SCAN_FAIL = 2 } LfSideResult;

/**
 * @brief 侧扫（或复测）一拍：先按 60 ms 超声节拍取一帧，再推进 FWD/REV 子状态并输出驱动
 * @return LF_SIDE_SCAN_RUN = 仍在扫；LF_SIDE_SCAN_OK = 两边界齐；LF_SIDE_SCAN_FAIL = 超时/超行程
 */
static LfSideResult probe_side_scan_tick(LfController *c, const LfInput *in, uint32_t now, float dt) {
    const LfProbeConfig *pr = &c->cfg.probe;
    uint32_t elapsed = now - c->phase_since;
    float s = probe_side_along(c);
    /* 1. 按超声 60 ms 触发节拍取一帧（不是每个 5 ms 控制拍都取，否则滑动环只有 40 ms 历史，斜率全被噪声吃掉） */
    if ((uint32_t)(now - c->side_ping_since) >= LF_SIDE_PING_MS) {
        c->side_ping_since = now;
        probe_side_feed(c, s, in->obstacle_valid ? in->obstacle_mm : 0U);
    }
    /* 2. 子状态推进：FWD 找 +s 侧边界 → REV 找 -s 侧边界 → DONE
     *    行程上限 **相对扫描原点左右对称**（契约 §2.3「单方向最大扫描行程」）：
     *    普通扫描 = ±side_reach；复测 = ±(w/2 + h·tanγ + 确认距离)，随 h/速度动态算。 */
    float reach = c->side_verify_mode
        ? (0.5f * c->face.w + probe_side_footprint(c) + probe_side_confirm_mm(c))
        : pr->side_reach_mm;
    uint32_t timeout_ms = c->side_push
        ? LF_SIDE_PUSH_TIMEOUT_MS
        : (uint32_t)((2.0f * pr->side_reach_mm + 300.0f) / fmaxf(pr->side_speed_mm_s, 20.0f) * 1000.0f) + 2500U;
    if (c->side_stage == (uint8_t)LF_SIDE_FWD) {
        if (c->side_edges >= 1U) {
            c->side_s_turn = s;
            c->side_stage = (uint8_t)LF_SIDE_REV;
            probe_side_ring_reset(c);
        } else if (s >= reach || elapsed >= timeout_ms) {
            return LF_SIDE_SCAN_FAIL;
        }
    } else if (c->side_stage == (uint8_t)LF_SIDE_REV) {
        if (c->side_edges >= 2U) {
            c->side_stage = (uint8_t)LF_SIDE_DONE;
            return LF_SIDE_SCAN_OK;
        }
        if (s <= -reach || elapsed >= timeout_ms) return LF_SIDE_SCAN_FAIL;
    } else {
        return LF_SIDE_SCAN_OK;
    }
    /* 3. 输出：台架手推模式断电（人工推车），正常模式沿面直行 */
    if (c->side_push) {
        c->out.enable = false;
        c->out.brake = false;
        c->out.duty[0] = c->out.duty[1] = 0.0f;
    } else {
        float spd = c->side_stage == (uint8_t)LF_SIDE_FWD ? pr->side_speed_mm_s : -pr->side_speed_mm_s;
        probe_drive_signed(c, spd, dt);
    }
    c->out.park_lamp = true;
    return LF_SIDE_SCAN_RUN;
}

/**
 * @brief 启动一次 SIDE_LOCK 沿面搜索：下一拍进 stage 7 前进 LF_SIDE_SEEK_MM 再重采
 * @note  只在 probe_face>0（刚从 LEG 过来）才有意义；面 0 没有前序里程可依，照旧直接失败。
 */
static void probe_side_seek_begin(LfController *c) {
    c->side_seek_n = (uint8_t)(c->side_seek_n + 1U);
    c->side_seek_start_mm = c->odo.distance_mm;
    c->side_warn = 11U;
    probe_motion_reset(c);
    c->side_lock_stage = 7U;
    c->side_h1 = 0.0f;
    c->side_ring_n = 0U;
}

/** @brief 进入 SIDE_LOCK：清空锁定/扫描状态，让 60 ms 采样节拍与 SIDE_MOVE 子状态从头开始 */
static void probe_side_lock_begin(LfController *c, uint32_t now) {
    c->side_h = 0.0f;
    c->side_pred_mm = 0.0f;
    c->side_seek_n = 0U;
    c->side_seek_start_mm = 0.0f;
    c->side_lock_sample_since = now;
    c->side_target = 0.0f;
    c->side_move_stage = (uint8_t)LF_SIDE_MOVE_DRIVE;
    c->side_verify_done = 0U;
    c->side_rescan = 0U;
    c->side_retry = 0U;
    c->side_locked = 0U;
    c->side_lock_warn = 0U;
    /* M3/M4e 预扫测斜状态 */
    c->side_lock_stage = 0U;
    c->side_h1 = c->side_h2 = c->side_h3 = 0.0f;
    c->side_delta_rad = 0.0f;
    c->side_tilt_corr_rad = 0.0f;
    c->side_tilt_eps_deg = 0.0f;
    c->side_tilt_iter = 0U;
    c->side_tilt_retry = 0U;
    /* 扫描结束后的"二次微调"：每面最多一次，进 SIDE_LOCK 时复位 */
    c->side_yaw_fix = 0U;
    c->side_yaw_rad = 0.0f;
    c->side_yaw_deg = 0.0f;
    /* 本面平移基线：按面宽选。面 0 还没扫过、不知道宽窄（首面基准在 LEG 入口才锁存），按宽面 60 mm 给；
     * 若它其实是 50 mm 窄面，平移会把波束带出去 → warn=8 → 本次跳过测斜，不影响正确性。
     * 面 1~3 用首面锁存 + 面序交替（与 probe_leg_current_kind() 同一套规则）：第 i 面 = first_kind ^ (i & 1)。 */
    uint8_t wide = 1U;
    if (c->probe_face > 0U && c->box_kind_ok) {
        wide = (uint8_t)(c->box_first_kind ^ (c->probe_face & 1U));
    }
    c->side_tilt_shift_mm = wide ? LF_SIDE_TILT_SHIFT_MM : LF_SIDE_TILT_SHIFT_NARROW_MM;
    c->turn_stage = (uint8_t)LF_TURN_IDLE;
    probe_side_begin(c, now); /* 这里会把 side_ping_since 归零、清扫描环 */
}

/**
 * @brief SIDE_LOCK 的 5 帧采样：每 60 ms 取一帧，返回 true 表示 5 帧齐
 * @note  预扫测斜的 h1/h2/h3 都用它；h2/h3 前把 side_ring_n 清 0。
 */
static bool probe_side_lock_sample(LfController *c, const LfInput *in, uint32_t now) {
    const LfProbeConfig *pr = &c->cfg.probe;
    if ((uint32_t)(now - c->side_ping_since) >= LF_SIDE_PING_MS) {
        c->side_ping_since = now;
        uint16_t d = in->obstacle_valid ? in->obstacle_mm : 0U;
        if (d >= pr->aim_min_mm && d <= pr->aim_max_mm && c->side_ring_n < LF_SIDE_LOCK_FRAMES)
            c->side_ring_d[c->side_ring_n++] = (float)d;
    }
    return c->side_ring_n >= LF_SIDE_LOCK_FRAMES;
}

/**
 * @brief 进入发射前复测：把侧扫轴原点改到当前（已开到中点）位置，行程收到 ±(w/2 + h·tanγ + 确认距离)
 * @return false = 复测不可用（此时按原 s_m 直接发射）
 */
static bool probe_side_verify_begin(LfController *c, uint32_t now) {
    if (!(c->face.w >= 20.0f)) return false;
    c->side_s_m_ref = c->face.s_m;
    c->side_verify_offset = probe_side_along(c);
    c->side_w_prev = c->face.w;
    probe_side_begin(c, now);
    c->side_verify_mode = 1U;
    return true;
}

/** @brief 当前应保持的舵机脉宽：侧视锁定后必须保持 side_us（扫描与发射同角是性质①的前提） */
static uint16_t probe_servo_hold(const LfController *c) {
    const LfProbeConfig *pr = &c->cfg.probe;
    if (pr->scan_mode != 0U && c->side_locked) return pr->side_us;
    return c->best_servo_us ? c->best_servo_us : c->cfg.servo_center_us;
}

/* ============================================================================
 * M3：原地转（TURN_PAR / LEG 转角 / LEG 前航向修正）
 * ============================================================================ */

/**
 * @brief 原地单向转到一个相对角（相对 base_rad，正=左转/CCW），到位后主动制动并停稳再测 ach
 * @param c         控制器
 * @param now       当前时间戳 (ms)
 * @param base_rad  本次转的基准航向 (rad)。TURN_PAR 用入口航向，LEG TURN 用 DRIVE1 结束航向
 * @param target_rad 相对转角 (rad，正=左转/CCW；要求 BOARD_SWAP_LR=0 才与实车一致)。单向逼近，不来回修
 * @param speed     单轮线速度 (mm/s)
 * @param tol_deg   停稳后 |ach−cmd| 的容差 (deg)，超过置 turn_ok=0
 * @param dt        控制周期 (s)
 * @return true = 已完成（turn_ok 表示是否在容差内）
 * @note  每次开始前调用方必须把 turn_stage 置 LF_TURN_IDLE（TURN_PAR / LEG TURN 都这样做）。
 *        超时不在这里判：TURN_PAR 用 phase elapsed，LEG 用 leg_since/leg_drive_since，
 *        这样「转角卡住」和「直行卡住」能给出不同的失败原因。
 */
static bool probe_turn_to(LfController *c, uint32_t now, float base_rad, float target_rad,
                          float speed, float tol_deg, float dt) {
    if (c->turn_stage == (uint8_t)LF_TURN_IDLE) {
        c->turn_stage = (uint8_t)LF_TURN_DRIVE;
        c->turn_base_rad = base_rad;
        c->turn_start_ms = now;
        c->turn_settle_ms = now;
        c->turn_ms = 0U;
        c->turn_cmd_deg = target_rad * 57.2957795f;
        c->turn_ach_deg = 0.0f;
        c->turn_ok = 0U;
    }
    float ach_rad = c->odo.heading_rad - c->turn_base_rad;
    if (c->turn_stage == (uint8_t)LF_TURN_DRIVE) {
        if (fabsf(ach_rad) >= fabsf(target_rad)) {
            /* 到位：先主动短接制动，150 ms 后再读航向，避免把滑行段算成"已到位" */
            probe_motion_reset(c);
            c->out.enable = c->out.brake = true;
            c->out.duty[0] = c->out.duty[1] = 0.0f;
            c->turn_stage = (uint8_t)LF_TURN_SETTLE;
            c->turn_settle_ms = now;
        } else {
            float dir = target_rad >= 0.0f ? 1.0f : -1.0f;
            /* dtheta = (右−左)/轮距：左轮负、右轮正 = 左转（正目标） */
            speed_control(c, dir > 0.0f ? -speed : speed, dir > 0.0f ? speed : -speed, dt, true);
        }
    } else if (c->turn_stage == (uint8_t)LF_TURN_SETTLE) {
        probe_motion_reset(c);
        c->out.enable = c->out.brake = true;
        c->out.duty[0] = c->out.duty[1] = 0.0f;
        if (now - c->turn_settle_ms >= LF_TURN_SETTLE_MS) {
            c->turn_ach_deg = (c->odo.heading_rad - c->turn_base_rad) * 57.2957795f;
            c->turn_ms = now - c->turn_start_ms;
            c->turn_ok = fabsf(c->turn_ach_deg - c->turn_cmd_deg) <= tol_deg ? 1U : 0U;
            c->turn_stage = (uint8_t)LF_TURN_DONE;
            return true;
        }
    } else {
        return true;
    }
    return false;
}

/* ============================================================================
 * M3：LEG 三段绕行（DRIVE1 → TURN → DRIVE2）
 * ============================================================================ */

/** @brief LEG 引擎返回码 */
typedef enum { LF_LEG_RUN = 0, LF_LEG_OK = 1, LF_LEG_FAIL = 2 } LfLegResult;

/** @brief LEG 失败收尾：锁存原因、置 FAIL 事件、主动制动 */
static LfLegResult probe_leg_fail(LfController *c, uint8_t why) {
    c->leg_fail = why;
    c->leg_event |= (uint8_t)LF_LEG_EVENT_FAIL;
    probe_motion_reset(c);
    c->out.enable = c->out.brake = true;
    c->out.duty[0] = c->out.duty[1] = 0.0f;
    return LF_LEG_FAIL;
}

/**
 * @brief LEG 一拍：入口锁存原点/航向/L1/L2 → DRIVE1 → TURN → DRIVE2
 * @return RUN / OK / FAIL
 * @note  距离基准全部是 LEG 自己的 leg_origin_mm / leg_turn_mm，**不要用 phase_start_mm**：
 *        DRIVE1 会把它用掉，而原地转时 distance_mm 不变，DRIVE2 的 d_mm>=L2 会立即成立。
 *        航向纠偏基准分别是 leg_hdg0（DRIVE1）和 leg_hdg2（DRIVE2），**不要用 probe_drive_signed**
 *        （它用 phase_start_rad，转完 90° 会命令 90° 差速修正）。
 */
static LfLegResult probe_leg_tick(LfController *c, const LfInput *in, uint32_t now, float dt) {
    const LfProbeConfig *pr = &c->cfg.probe;
    c->out.enable = c->out.brake = true; /* 默认保持制动；speed_control 会覆盖成驱动 */
    if (!c->leg_started) {
        c->leg_kind = probe_leg_current_kind(c);
        /* 面中心处的 h 就用 SIDE_LOCK 测的 side_h：side_k 是 24 mm 基线的钳位斜率（现场几乎恒 −0.250），
         * 不是倾角，拿它乘 s_m（最大 ±75 mm）会往 LEG 长度里注入最多 ±19 mm 的系统误差（M4e 止血②）。
         * 残余偏航 ≤7° 时面中心与扫描原点的 h 差 ≤|s_m|·sin7°≈9 mm，远小于 LEG 落点散布。 */
        float h_leg = c->side_h;
        h_leg = limit(h_leg, LF_SIDE_H_MIN_MM, LF_SIDE_H_MAX_MM);
        if (h_leg < LF_LEG_H_MIN_MM) return probe_leg_fail(c, 1U);
        float w_nom = (c->leg_kind == 0U) ? LF_BOX_SHORT_MM : LF_BOX_LONG_MM;
        float d_b = (c->leg_kind == 0U) ? LF_BOX_LONG_MM : LF_BOX_SHORT_MM;
        /* L1 从轮轴起算：SIDE_MOVE 把轮轴停在面中心后方 pan_x，云台轴才在面中心 */
        /* 站距基准用**首面**实测的 h_ref，不用本面的 h、也不再 +margin：
         * 每面都"比上一面再远 20 mm"，再加上转向滑移，会一级级往外走 ——
         * 2026-11-23 22:38 实测 160 → 206 → 298，第 3 面窄面已经在 30 cm 外，
         * 扫出来的 w = −20.3（两条边反序），整面丢失。 */
        if (c->h_ref_mm < 1.0f) c->h_ref_mm = h_leg;
        c->leg_l1_mm = 0.5f * w_nom + pr->pan_x_mm + c->h_ref_mm;
        /* L2 要让**云台轴**落在下一面中心，所以减掉 pan_x；否则窄面会从面外 30 mm 起扫 */
        c->leg_l2_mm = h_leg + 0.5f * d_b - pr->pan_x_mm;
        if (c->leg_l2_mm < 20.0f) c->leg_l2_mm = 20.0f;
        c->leg_origin_mm = c->odo.distance_mm;
        c->leg_hdg0 = c->odo.heading_rad;
        c->leg_since = now;
        c->leg_drive_since = now;
        c->leg_started = 1U;
        c->leg_stage = (uint8_t)LF_LEG_DRIVE1; /* 不做起步修正：k 只是诊断量，见 line_follower.h */
        c->turn_stage = (uint8_t)LF_TURN_IDLE;
        /* 侧距连续采样的入口：h_expected 是"下一面应该在哪"的唯一先验，DRIVE2 认面用 */
        c->leg_prox_since = now;
        c->leg_prox_n = 0U;
        c->leg_prox_last = 0U;
        c->leg_prox_min = 0U;
        c->leg_prox_max = 0U;
        c->leg_face2_n = 0U;
        c->leg_face2_h = 0U;
        c->leg_face2_sum = 0U;
        c->leg_prox_now = 0U;
        c->leg_prox_now_ok = 0U;
        c->leg_backup_start_mm = 0.0f;
        c->leg_h_expected = c->h_ref_mm;   /* 下一面的期望站距 = 首面基准（与 L1 一致） */
        c->leg_face_s_mm = 0.0f;
        c->leg_face_seen = 0U;
        c->leg_face_cnt = 0U;
        c->leg_stop_reason = 0U;
        c->leg_cmd_mm = 0.0f;
        c->leg_ach_mm = 0.0f;
        c->leg_hdg_deg = 0.0f;
        return LF_LEG_RUN;
    }
    /* 侧距连续采样：LEG 期间舵机一直压在 side_us（宽面转完不回正），超声本来就在连续测距，
     * 这里按 50 ms 留一帧轨迹。DRIVE2 里落在 h_expected±band 的帧就是"窄面从旁边过"的直接证据：
     * 停车后 SIDE_LOCK 若被无关物体挡住（实测 h≈93），有这条轨迹就能分清是面还是挡块。 */
    if ((uint32_t)(now - c->leg_prox_since) >= LF_LEG_PROX_PERIOD_MS) {
        c->leg_prox_since = now;
        c->leg_prox_now_ok = in->obstacle_valid ? 1U : 0U;
        if (in->obstacle_valid) c->leg_prox_now = in->obstacle_mm;
        if (in->obstacle_valid && in->obstacle_mm >= (uint16_t)LF_SIDE_H_MIN_MM &&
            in->obstacle_mm <= (uint16_t)LF_SIDE_H_MAX_MM) {
            uint16_t h = in->obstacle_mm;
            if (c->leg_prox_n == 0U || h < c->leg_prox_min) c->leg_prox_min = h;
            if (c->leg_prox_n == 0U || h > c->leg_prox_max) c->leg_prox_max = h;
            c->leg_prox_last = h;
            c->leg_prox_n = (uint16_t)(c->leg_prox_n + 1U);
            if (c->leg_stage == (uint8_t)LF_LEG_DRIVE2 && c->leg_h_expected > 1.0f &&
                fabsf((float)h - c->leg_h_expected) <= LF_LEG_PROX_FACE_BAND_MM) {
                c->leg_face2_sum += h;
                c->leg_face2_n = (uint16_t)(c->leg_face2_n + 1U);
            }
            /* 「找突变」认面（只认 DRIVE2 段：DRIVE1 沿上一面走，看到的不是下一面）：
             * 连续 LF_LEG_FACE_CONFIRM 帧落在 h_leg+margin±band → 下一面已经进侧视，锁存落点。 */
            if (c->leg_stage == (uint8_t)LF_LEG_DRIVE2 && c->leg_face_seen == 0U) {
                float h_pred = c->leg_h_expected + LF_LEG_MARGIN_MM;
                if (fabsf((float)h - h_pred) <= LF_LEG_FACE_BAND_MM) {
                    if (c->leg_face_cnt < 255U) ++c->leg_face_cnt;
                    if (c->leg_face_cnt >= LF_LEG_FACE_CONFIRM) {
                        c->leg_face_seen = 1U;
                        c->leg_face_s_mm = c->odo.distance_mm - c->leg_turn_mm;
                    }
                } else {
                    c->leg_face_cnt = 0U;
                }
            }
            if ((c->leg_prox_n & 1U) == 0U) c->leg_event |= (uint8_t)LF_LEG_EVENT_PROX;
        }
    }
    if ((uint32_t)(now - c->leg_since) >= LF_LEG_TIMEOUT_MS) {
        uint8_t why = c->leg_stage == (uint8_t)LF_LEG_DRIVE1 ? 2U
                    : c->leg_stage == (uint8_t)LF_LEG_TURN ? 3U : 4U;
        return probe_leg_fail(c, why);
    }
    switch (c->leg_stage) {
    case LF_LEG_DRIVE1: {
        float ach = c->odo.distance_mm - c->leg_origin_mm;
        if (ach >= c->leg_l1_mm) {
            c->leg_cmd_mm = c->leg_l1_mm;
            c->leg_ach_mm = ach;
            c->leg_hdg_deg = (c->odo.heading_rad - c->leg_hdg0) * 57.2957795f;
            c->leg_event |= (uint8_t)LF_LEG_EVENT_DRIVE1;
            c->leg_turn_mm = c->odo.distance_mm;
            c->leg_hdg1 = c->odo.heading_rad;
            c->leg_stage = (uint8_t)LF_LEG_TURN;
            c->turn_stage = (uint8_t)LF_TURN_IDLE;
            probe_motion_reset(c);
            break;
        }
        if ((uint32_t)(now - c->leg_drive_since) >= LF_LEG_DRIVE_TIMEOUT_MS)
            return probe_leg_fail(c, 2U);
        float corr = limit((c->odo.heading_rad - c->leg_hdg0) * 120.0f, -30.0f, 30.0f);
        speed_control(c, LF_LEG_SPEED_MM_S + corr, LF_LEG_SPEED_MM_S - corr, dt, true);
        break;
    }
    case LF_LEG_TURN: {
        /* 箱体在左 → 左转 90°（逆时针，+90）。BOARD_SWAP_LR=0 时命令符号与实车一致；
         * 21:34 的「该左转却右拐」是 BOARD_SWAP_LR=1 把电机/编码器同时镜像所致，不是这里符号反。 */
        float target = pr->side_left ? 90.0f : -90.0f;
        if (probe_turn_to(c, now, c->leg_hdg1, target * 0.01745329252f,
                          LF_LEG_TURN_SPEED_MM_S, LF_TURN_TOL_DEG, dt)) {
            c->leg_hdg2 = c->odo.heading_rad;
            c->leg_turn_ach_deg = c->turn_ach_deg;
            c->leg_turn_ms = c->turn_ms;
            c->leg_event |= (uint8_t)LF_LEG_EVENT_TURN;
            c->turn_stage = (uint8_t)LF_TURN_IDLE;
            c->leg_stage = (uint8_t)LF_LEG_DRIVE2;
            c->leg_drive_since = now;
            probe_motion_reset(c);
        }
        break;
    }
    case LF_LEG_DRIVE2: {
        float ach = c->odo.distance_mm - c->leg_turn_mm;
        /* 停车目标（事件优先，死算只是上限）：
         *   ① 认到面（h 进 h_leg+margin±band）→ 停在「面近边 + d_B/2 + 波束足印」，
         *      让云台轴落在面中心附近；只停在近边上时波束只压住面的一半，窄面回波很弱。
         *   ② 一直没认到面 → 窄面再往前找 LF_LEG_NARROW_SEARCH_MM（宽面可见窗口本来就宽，不加）。
         * 之后 SIDE_LOCK 自己还有 3 次沿面搜索（LF_SIDE_SEEK_*）兜底。 */
        float d_b = (c->leg_kind == 0U) ? LF_BOX_LONG_MM : LF_BOX_SHORT_MM;
        float stop_target = c->leg_l2_mm;
        bool stop_event = false;
        if (c->leg_face_seen != 0U) {
            stop_event = true;
            float adv = c->leg_face_s_mm + 0.5f * d_b + probe_side_footprint(c);
            stop_target = (adv < c->leg_l2_mm) ? adv : c->leg_l2_mm;
        } else if (c->leg_kind == 0U) {
            stop_target = c->leg_l2_mm + LF_LEG_NARROW_SEARCH_MM;
        }
        if (ach >= stop_target) {
            c->leg_stop_reason = stop_event ? 1U : 2U;
            c->leg_cmd_mm = stop_target;
            c->leg_ach_mm = ach;
            c->leg_hdg_deg = (c->odo.heading_rad - c->leg_hdg2) * 57.2957795f;
            /* DRIVE2 结束时给窄面侧距统计：face_n>0 说明行进中确实看到过面（h≈pred），
             * face_h 是它的均值；停车后 SIDE_LOCK 的 h1 与它一比就知道是不是被挡块顶替了。 */
            c->leg_face2_h = c->leg_face2_n ? (uint16_t)(c->leg_face2_sum / c->leg_face2_n) : 0U;
            c->leg_event |= (uint8_t)LF_LEG_EVENT_PROX2;
            c->leg_event |= (uint8_t)LF_LEG_EVENT_DRIVE2;
            /* 收尾位置自检：停机时面已滑出侧视（h>H_MAX 或丢回波），但行进中确实看到过面 →
             * 死算的 L2 越过窄面后沿了，倒回 LF_LEG_BACKUP_MM 再交给 SIDE_LOCK。 */
            bool face_gone = !c->leg_prox_now_ok ||
                             c->leg_prox_now > (uint16_t)LF_SIDE_H_MAX_MM;
            if (face_gone && c->leg_face2_n >= 5U) {
                c->leg_backup_start_mm = c->odo.distance_mm;
                c->leg_drive_since = now;
                probe_motion_reset(c);
                c->leg_stage = (uint8_t)LF_LEG_BACKUP;
                return LF_LEG_RUN;
            }
            c->leg_stage = (uint8_t)LF_LEG_DONE;
            probe_motion_reset(c);
            c->out.enable = c->out.brake = true;
            c->out.duty[0] = c->out.duty[1] = 0.0f;
            return LF_LEG_OK;
        }
        if ((uint32_t)(now - c->leg_drive_since) >= LF_LEG_DRIVE_TIMEOUT_MS)
            return probe_leg_fail(c, 4U);
        /* DRIVE2 兼「认面搜索」：降速到 LF_LEG_SEARCH_SPEED_MM_S，50 ms 一帧约 3 mm，
         * 认面判据和刹车滑行都更可控（原来 100 mm/s 时一帧 5 mm、刹停还要再滑几毫米）。 */
        float corr = limit((c->odo.heading_rad - c->leg_hdg2) * 120.0f, -30.0f, 30.0f);
        speed_control(c, LF_LEG_SEARCH_SPEED_MM_S + corr, LF_LEG_SEARCH_SPEED_MM_S - corr, dt, true);
        break;
    }
    case LF_LEG_BACKUP: {
        /* 倒回固定距离再交给 SIDE_LOCK：DRIVE2 末尾窄面已滑出侧视时的补救 */
        float back = c->leg_backup_start_mm - c->odo.distance_mm;
        if (back >= LF_LEG_BACKUP_MM) {
            probe_motion_reset(c);
            c->out.enable = c->out.brake = true;
            c->out.duty[0] = c->out.duty[1] = 0.0f;
            c->leg_event |= (uint8_t)LF_LEG_EVENT_BACKUP;
            c->leg_stage = (uint8_t)LF_LEG_DONE;
            return LF_LEG_OK;
        }
        if ((uint32_t)(now - c->leg_drive_since) >= LF_LEG_DRIVE_TIMEOUT_MS)
            return probe_leg_fail(c, 5U);
        speed_control(c, -LF_LEG_BACKUP_SPEED_MM_S, -LF_LEG_BACKUP_SPEED_MM_S, dt, true);
        break;
    }
    default:
        return LF_LEG_OK;
    }
    return LF_LEG_RUN;
}

static void step_probing(LfController *c, const LfInput *in, uint32_t now, float dt) {
    if (now - c->state_since >= 180000U) { lf_stop(c, now); return; }
    const LfProbeConfig *pr = &c->cfg.probe;
    uint32_t elapsed = now - c->phase_since;
    float d_mm = c->odo.distance_mm - c->phase_start_mm;
    float d_rad = c->odo.heading_rad - c->phase_start_rad;
    float target_rad = pr->turn_deg * (3.14159265f / 180.0f);
    float dir = pr->clockwise ? 1.0f : -1.0f; /* 顺时针绕行：先左转出角，再两次右转进角 */
    c->out.servo_enable = true;
    /* 侧视锁定后必须一直保持 side_us（扫描与发射同角），其余阶段保持 AIM 找到的法线角 */
    c->out.servo_us = probe_servo_hold(c);

    switch (c->probe_phase) {
    case LF_PROBE_AIM: {
        c->out.enable = c->out.brake = c->out.park_lamp = true;
        uint32_t settle_ms = pr->aim_ms / 4U;
        if (settle_ms > LF_PROBE_AIM_SETTLE_MS) settle_ms = LF_PROBE_AIM_SETTLE_MS;
        uint32_t sweep_ms = pr->aim_ms > settle_ms ? (pr->aim_ms - settle_ms) : 1U;
        uint16_t points = probe_scan_plan(c, sweep_ms);
        if (elapsed < settle_ms) {
            /* 首点驻留：等舵机从上一位置转到位，并让超声回波彻底稳定后再开始计点 */
            c->out.servo_us = probe_scan_pulse(c, 0U);
        } else {
            uint32_t s_el = elapsed - settle_ms;
            if (s_el > sweep_ms) s_el = sweep_ms;
            /* 由已用时间反推当前应驻留的离散角度点：舵机做“台阶式”步进而非连续扫掠，
             * 保证每个角度上都留足 >= 60 ms 去覆盖一次硬件超声触发周期 */
            uint32_t index = (s_el * (uint32_t)points) / sweep_ms;
            if (index >= (uint32_t)points) index = (uint32_t)points - 1U;
            c->scan_index = (uint8_t)index;
            c->out.servo_us = probe_scan_pulse(c, (uint16_t)index);
            /* 每步开头的 LF_PROBE_SCAN_SKIP_MS 内，超声可能还在按上一角度计时，丢弃这段样本 */
            uint32_t index_begin_ms = (index * sweep_ms) / (uint32_t)points;
            bool fresh = (s_el - index_begin_ms) >= LF_PROBE_SCAN_SKIP_MS;
            /* 仅当回波落在“正前方合理障碍距离窗口 [aim_min_mm, aim_max_mm]”内才计入该角度样本 */
            if (fresh && in->obstacle_valid &&
                in->obstacle_mm >= pr->aim_min_mm && in->obstacle_mm <= pr->aim_max_mm) {
                c->scan_sum_mm[index] += in->obstacle_mm;
                uint8_t hits = c->scan_hits[index];
                if (hits < 255) { ++hits; c->scan_hits[index] = hits; }
            }
        }
        if (elapsed >= pr->aim_ms) {
            probe_scan_finish(c);
            if (c->aim_pass == 0U && c->face.valid) {
                /* 第一段粗扫成功：做第二段**独立复测**（2026-11-23 重做）。
                 * 旧设计是「以粗测法线为中心、只扫 ±120 us（±12°）的细扫」，有三个叠加缺陷：
                 *   ① 力臂只有粗扫的 1/2.5 → 同样 5° 的角度偏差，距离变化只有 1/6.25；
                 *   ② 中心锁在粗扫的 φ 上 → 粗扫偏了就"错也自洽"，两遍不可能互相纠正；
                 *   ③ 驻留时长把点数压到 10 个。
                 * 实测两轮（真值由随后 SIDE_SCAN 的 187 mm 长基线反推）：
                 *   probe_fix2 真值 +1.40°：粗 −1.60 / 细 +6.91 / 平均 +2.66（误差 3.0 / 5.5 / 1.3）
                 *   probe_fix3 真值 +2.10°：粗 +4.10 / 细 +12.24 / 平均 +8.17（误差 2.0 / 10.1 / 6.1）
                 * → 细扫就是拖后腿的那个。现在改成：**同样的 ±sweep_us 全幅、中心固定在舵机中位、
                 *   步进更密**（点数受驻留时长限制，aim_ms=3000 时 15 点 vs 粗扫 11 点）。
                 * 两遍因此是同力臂的独立测量，合并（取平均）才有意义；两遍差 >25° 时按下面的门限退粗扫。 */
                c->aim_first = c->face;
                c->aim_center_us = c->cfg.servo_center_us;   /* 不再依赖粗扫的 φ，去掉自证 */
                c->aim_span_us = pr->sweep_us;               /* 与粗扫同幅，力臂一样长 */
                c->aim_pass = 1U;
                probe_scan_clear(c);
                memset(&c->face, 0, sizeof(c->face));
                c->phase_since = now;
                c->out.servo_us = probe_scan_pulse(c, 0U);
            } else {
                if (c->aim_pass == 1U) {
                    if (!c->face.valid) {
                        /* 细扫无效：整段退回粗扫结果 */
                        c->face = c->aim_first;
                    } else if (c->aim_first.valid) {
                        /* 两遍合并（2026-11-23）：粗细扫是**同一个静止场景的两次独立测量**。
                         * 粗扫基线 ±300 us（比细扫长 2.5 倍、对噪声更稳），细扫点更密但只覆盖 ±120 us；
                         * 固件自己的日志里两遍可以差 12°（193724：粗 −8.9 / 细 +1.14），单用任何一遍都偏 5° 量级。
                         * 标定用例 probe_fix2（人摆正、面在 146 mm）：真值 +1.40°（由随后 SIDE_SCAN 的
                         * 187 mm 长基线反推），粗 −1.60 / 细 +6.91 / 平均 +2.66 → 误差 3.00 / 5.51 / 1.26°。
                         * 两遍差超过门限说明至少一遍认错了东西，此时信基线更长的粗扫。 */
                        float dphi = fabsf(c->aim_first.phi - c->face.phi) * 57.2957795f;
                        if (dphi <= LF_PROBE_AIM_MERGE_MAX_DEG) {
                            c->face.phi = 0.5f * (c->aim_first.phi + c->face.phi);
                            c->face.rho = 0.5f * (c->aim_first.rho + c->face.rho);
                        } else {
                            c->face = c->aim_first;
                        }
                        /* 瞄准脉宽跟着合并后的 φ 走，保证 PROBE_LOCK / best_servo_us 与 PROBE_FACE 自洽 */
                        float us = (float)c->cfg.servo_center_us +
                                   c->face.phi * 57.2957795f * LF_PROBE_SCAN_US_PER_DEG;
                        us = limit(us + 0.5f, (float)LF_SERVO_US_MIN, (float)LF_SERVO_US_MAX);
                        c->best_servo_us = (uint16_t)us;
                    }
                }
                c->out.servo_us = probe_servo_hold(c);
                /* M3：AIM 成功 → 原地转到侧视方向（TURN_PAR）→ SIDE_LOCK。
                 * 注意：旧条件里的 side_bench 只由 lf_start_side_probe() 置 1，而它直接从
                 * SIDE_LOCK 开始、永远到不了 AIM —— 再判 side_bench 就是死代码。
                 * 侧向邻墙的误锁风险改由 TURN_PAR 后的 SIDE_LOCK 自检 + 失败即停兜住（M4 再加重试）。 */
                if (pr->scan_mode != 0U && c->face.valid && !c->side_push) {
                    c->turn_stage = (uint8_t)LF_TURN_IDLE;
                    c->turn_ok = 0U;
                    c->turn_backup = (c->face.rho < LF_TURN_PAR_BACKUP_RHO_MM) ? 0U : 1U;
                    c->leg_started = 0U;
                    c->leg_event = 0U;
                    change_probe_phase(c, LF_PROBE_TURN_PAR, now);
                } else {
                    change_probe_phase(c, LF_PROBE_LASER, now);
                }
            }
        }
        break;
    }
    case LF_PROBE_TURN_PAR: {
        /* 目标：让舵机侧视方向（名义 +90°）正对 AIM 拟合出的面法线 φ。
         * 车体要转的角 = φ − 90°（箱体在左）或 φ + 90°（箱体在右）。
         * 性质①只保证"扫描和发射用同一个舵机角→中点不偏"，**不保证**转完车头严格平行：
         * 因此这里允许的残差要靠 SIDE_SCAN 的 k 在 LEG 前修掉（M3 P1-1），不能当它不存在。 */
        c->out.servo_us = pr->side_us; /* 转身期间舵机就先甩到侧视，SIDE_LOCK 的 400 ms 只作余量 */
        c->out.park_lamp = false;
        float phi_deg = c->face.phi * 57.2957795f;
        /* probe_yaw_bias_deg 是**台架注入**的偏置（console: yawbias DEG，RAM-only，不进 Flash 配置）。
         * 用途：AIM 正常工作时，人为把车摆歪并不会留下残余偏航（TURN_PAR 会补偿掉），
         * 所以想把"扫描结束二次微调"的坏工况造出来，只能在转角命令上注入已知量。默认 0。 */
        float cmd_deg = (pr->side_left ? (phi_deg - 90.0f) : (phi_deg + 90.0f)) +
                        c->probe_yaw_bias_deg;
        if (fabsf(phi_deg) > LF_TURN_PAR_MAX_PHI_DEG) {
            /* φ 离谱（AIM 被邻面/棱角带偏）：不转，带着 ok=0 停机，避免为了对齐法线把车转进箱子 */
            c->turn_cmd_deg = cmd_deg;
            c->turn_ach_deg = (c->odo.heading_rad - c->phase_start_rad) * 57.2957795f;
            c->turn_ms = elapsed;
            c->turn_ok = 0U;
            c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
            lf_stop(c, now);
            break;
        }
        if (!c->turn_backup) {
            /* 现场可能停在 13~14 cm；先低速倒 20 mm，给原地转的前角留间隙（手册 §17.15.3b） */
            float back = c->phase_start_mm - c->odo.distance_mm;
            if (back >= LF_TURN_PAR_BACKUP_MM) {
                probe_motion_reset(c);
                c->out.enable = c->out.brake = true;
                c->out.duty[0] = c->out.duty[1] = 0.0f;
                c->turn_backup = 1U;
                c->phase_start_rad = c->odo.heading_rad;
                c->turn_stage = (uint8_t)LF_TURN_IDLE;
            } else {
                probe_drive_signed(c, -50.0f, dt);
            }
            break;
        }
        if (probe_turn_to(c, now, c->phase_start_rad, cmd_deg * 0.01745329252f,
                          LF_TURN_PAR_SPEED_MM_S, LF_TURN_TOL_DEG, dt)) {
            /* 精度不拦：|ach−cmd| ≤ LF_TURN_GROSS_DEG 就继续（角度残差交给扫描后的二次微调）。
             * 只有转不动/转错方向（>20°）才当机构失败停机，免得为 9° 丢掉整面。 */
            if (fabsf(c->turn_ach_deg - c->turn_cmd_deg) > LF_TURN_GROSS_DEG) {
                c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
                lf_stop(c, now);
                break;
            }
            probe_side_lock_begin(c, now);
            change_probe_phase(c, LF_PROBE_SIDE_LOCK, now);
        } else if (elapsed >= LF_TURN_PAR_TIMEOUT_MS) {
            c->turn_ach_deg = (c->odo.heading_rad - c->turn_base_rad) * 57.2957795f;
            c->turn_ms = elapsed;
            c->turn_ok = 0U;
            c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
            lf_stop(c, now);
        }
        break;
    }
    case LF_PROBE_SIDE_LOCK: {
        /* 舵机掰到侧视 ±90°（默认 2400us = +90° = 箱体在左），等它走完，然后采一次 h 就结束。
         * 2026-11-23 起这里**不做任何平移、不做任何转角**（扫描前自动测斜已整段删除，
         * 原因见 case 1 的注释）；残余偏航交给 SIDE_SCAN 结束后的"二次微调"。
         * 手推台架模式必须断电：否则 H 桥短接制动，人工推不动（与 side push 的承诺矛盾）。 */
        c->out.park_lamp = true;
        c->out.servo_us = pr->side_us;
        if (c->side_push) {
            c->out.enable = false;
            c->out.brake = false;
            c->out.duty[0] = c->out.duty[1] = 0.0f;
        } else {
            c->out.enable = c->out.brake = true;
        }
        bool fail = elapsed >= LF_SIDE_LOCK_TIMEOUT_MS;
        switch (c->side_lock_stage) {
        case 0: /* 等舵机 90° 走完 */
            if (elapsed < LF_SIDE_LOCK_WAIT_MS) break;
            c->side_ring_n = 0;
            c->side_ping_since = now;
            c->side_lock_stage = 1;
            break;
        case 1: { /* h1（搜索重采后可能多次进入） */
            if (!probe_side_lock_sample(c, in, now)) {
                uint32_t waited = now - c->side_lock_sample_since;
                uint32_t wait_need = (c->side_seek_n == 0U) ? LF_SIDE_LOCK_FALLBACK_MS
                                                            : LF_SIDE_SEEK_WAIT_MS;
                if (waited < wait_need) break;
                /* ① 停住后测不到面（回波被更远的墙抢走：现场行进中 251~263、停住稳定 661）时，
                 *    用 LEG 行进中已经量到的面均值兜底 —— 行进中确实见过面（face_n≥5）。 */
                if (c->side_seek_n == 0U && c->leg_face2_n >= 5U) {
                    float hf = (float)c->leg_face2_h;
                    if (hf >= LF_SIDE_H_MIN_MM && hf <= LF_SIDE_H_MAX_MM) {
                        c->side_h1 = hf;
                        c->side_warn = 10U;
                        c->side_h3 = c->side_h1;
                        c->side_lock_stage = 6;
                        break;
                    }
                }
                /* ② 还一帧都没采到、也搜够次数：沿面再进 30 mm 重采（见 LF_SIDE_SEEK_*） */
                if (c->probe_face > 0U && c->side_seek_n < LF_SIDE_SEEK_TRIES) {
                    probe_side_seek_begin(c);
                }
                break;
            }
            c->side_h1 = probe_side_median(c, c->side_ring_n);
            if (c->side_h1 < LF_SIDE_H_MIN_MM || c->side_h1 > LF_SIDE_H_MAX_MM) { fail = true; break; }
            /* 扫描前自动测斜（h1→平移→h2→退回→转正→h3）已于 2026-11-23 **整段删除**：
             * 它的输入是侧视超声在 side_us=2400 处的双稳态读数（同一位置 ±13~22 mm 跳变），
             * 60 mm 基线上的真实信号只有 5 mm，信噪比 0.3 —— 不是修正，是把噪声当信号再转一次。
             * 现场一次 eps=21° 把车从 −93.3° 转到 −67.1°、一次 eps=−53.8°→钳位−45° 甩转，
             * 都是它干的。扫描↔发射同轴（性质①）本来就不需要扫描前的任何转角。
             * 现在改为：只采一次 h，直接进 stage 6 自检；真歪了由 SIDE_SCAN 结束后的
             * '二次微调'（probe_side_yaw_should_fix）处理，那里有 ±90 mm 的长基线做依据。 */
            c->side_h3 = c->side_h1;
            c->side_lock_stage = 6;
            break;
        }
        case 7: { /* 沿面搜索：前进 LF_SIDE_SEEK_MM（舵机仍压在 side_us），到位后重采 5 帧 */
            float moved = c->odo.distance_mm - c->side_seek_start_mm;
            if (moved >= LF_SIDE_SEEK_MM) {
                probe_motion_reset(c);
                c->out.enable = c->out.brake = true;
                c->out.duty[0] = c->out.duty[1] = 0.0f;
                c->side_ring_n = 0U;
                c->side_ping_since = now;
                c->side_lock_sample_since = now;
                c->side_lock_stage = 1U;
                break;
            }
            probe_drive_signed(c, LF_SIDE_SEEK_SPEED_MM_S, dt);
            break;
        }
        default: { /* 6：ρ+pan_x 自检并通过才写 side_h */
            /* h3 丢面（>H_MAX 或 <H_MIN）而 h2 有效时用 h2 兜底：转正那一下常伴随一次丢回波 */
            if ((c->side_h3 < LF_SIDE_H_MIN_MM || c->side_h3 > LF_SIDE_H_MAX_MM) &&
                c->side_h2 >= LF_SIDE_H_MIN_MM && c->side_h2 <= LF_SIDE_H_MAX_MM) {
                c->side_h3 = c->side_h2;
                c->side_warn = 7U;
            }
            /* 期望侧距：面 0 没有 LEG，只能用 AIM 测到的 rho（车停在哪就量到哪）；
             * 面 1~3 刚跑完 LEG，LEG 自己就是用 h_leg 算的 L1，转完到这一面的期望侧距
             * 就是 h_leg + margin —— 与 main.c 打印的 h_pred 是同一个数。
             * 以前这里一律用 face.rho+pan_x（**第一面的** AIM 数，全程不更新）：AIM 停车位置
             * 差 20 mm，这一面的生死就变了。2026-11-23 21:36 实测 h=231、h_pred=187（差 44，
             * 本来该过），却拿 155.2 去比 → 75.8 > TOL 70，差 5.8 mm 停机，第二面白白没打。 */
            float pred = (c->face.valid && c->face.rho > 1.0f) ? c->face.rho + pr->pan_x_mm : 0.0f;
            if (c->probe_face > 0U && c->leg_h_expected > 1.0f)
                pred = c->leg_h_expected;
            c->side_pred_mm = pred;   /* 日志必须打这里：main.c 原来另算一遍旧公式，与实际判据不一致 */
            /* h1/h2/h3 里挑最接近 pred 的那个：退回/转正过程中丢一次面很常见，
             * 不能因为 h3=306 就把 h1=h2=240 这个真实读数丢掉。 */
            float h = c->side_h3;
            if (pred > 0.0f) {
                float best = h;
                float best_d = fabsf(h - pred);
                if (c->side_h2 >= LF_SIDE_H_MIN_MM && c->side_h2 <= LF_SIDE_H_MAX_MM &&
                    fabsf(c->side_h2 - pred) < best_d) {
                    best = c->side_h2;
                    best_d = fabsf(c->side_h2 - pred);
                }
                if (c->side_h1 >= LF_SIDE_H_MIN_MM && c->side_h1 <= LF_SIDE_H_MAX_MM &&
                    fabsf(c->side_h1 - pred) < best_d) {
                    best = c->side_h1;
                }
                h = best;
            }
            /* 只保留「回波本身可用」的绝对判据。|h − pred| 这个相对门禁**去掉**：
             * 它比的是"LEG 死算落点"和"实测站距"，而 LEG 的落点误差（转向滑移）实测
             * −106…+69 mm，与"认错面"在数值上无法区分；扫描的种子窗口用的是实测 h（自洽），
             * LEG 偏大只会让激光远一点，不会扫错面。留着它只会误杀整面（21:36 差 5.8、
             * 21:56 差 1.0 mm 都是它）。超差只记 warn=13，供日志判断。 */
            bool ok = h >= LF_SIDE_H_MIN_MM && h <= LF_SIDE_H_MAX_MM;
            if (ok && pred > 0.0f && fabsf(h - pred) > LF_SIDE_LOCK_TOL_MM) c->side_warn = 13U;
            /* 先把本阶段的告警存下来：成功路径紧接着的 probe_side_begin() 会清 side_warn，
             * 以前 warn=6/5/7 就是在这一拍被擦成 0，日志里永远看不到"平移卡住"。 */
            c->side_lock_warn = c->side_warn;
            if (ok) {
                c->side_h = h;
                c->face.h = h;
                c->side_locked = 1U;
                probe_side_begin(c, now);
                change_probe_phase(c, LF_PROBE_SIDE_SCAN, now);
            } else if (c->probe_face > 0U && c->side_seek_n < LF_SIDE_SEEK_TRIES) {
                /* 回波本身不可用（<80 或 >450）：沿面再挪一段重采（见 LF_SIDE_SEEK_*） */
                probe_side_seek_begin(c);
            } else {
                fail = true;
            }
            break;
        }
        }
        if (fail) {
            /* 自检不过 = 转角不对 / 认错面 / 侧向没有目标：立即可诊断地停机，不带着错误 h 继续扫。
             * （M3 仍失败即停；回 AIM 重试留给 M4。） */
            c->side_locked = 0U;
            c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
            lf_stop(c, now);
        }
        break;
    }
    case LF_PROBE_SIDE_SCAN: {
        c->out.servo_us = pr->side_us;
        if (c->side_yaw_fix == 1U) {
            /* "二次微调"第 2 步：原地转正（角度由长基线回归给出），转完**重扫一遍**。
             * 重扫由 probe_side_begin() 重新锁原点，最终 s_m 由这一次给出，
             * 所以"扫描↔发射同航向"（性质①）仍然成立 —— 这正是 2e00745 当年
             * 删掉"到点后转身"时那条硬规则（verify=0 时转完直接发射会偏 h·tanε≈26 mm）
             * 在本轮（verify=1）的精确放宽：要动角就必须紧接着重扫。 */
            if (!probe_turn_to(c, now, c->odo.heading_rad, c->side_yaw_rad,
                               LF_LEG_TURN_SPEED_MM_S, 3.0f, dt)) break;
            c->side_yaw_fix = 2U;
            c->turn_stage = (uint8_t)LF_TURN_IDLE;
            c->side_event |= (uint8_t)LF_SIDE_EVENT_YAW;  /* 再报一次：这次带上真实的 cmd/ach */

            /* 关键：**不重锁扫描原点**。转完先沿面退回原来那个扫描原点（s=0），
             * 再用已有的 side_rescan 机制从原点重扫一遍 —— 这样 s_m 仍在同一个坐标系里，
             * SIDE_MOVE 的 side_target 才有意义。（重锁原点会让车在"面边缘"起扫，
             * 单测 probeside 的 8° 用例就是靠这一点暴露出来的。） */
            c->side_target = 0.0f;
            c->side_rescan = 1U;
            c->side_verify_done = 1U;   /* 这次回退不做复测，重扫完成后再复位成 0 */
            c->side_retry = 0U;
            c->side_move_stage = (uint8_t)LF_SIDE_MOVE_DRIVE;
            probe_motion_reset(c);
            c->phase_since = now;
            change_probe_phase(c, LF_PROBE_SIDE_MOVE, now);
            break;
        }
        LfSideResult sr = probe_side_scan_tick(c, in, now, dt);
        if (sr == LF_SIDE_SCAN_OK) {
            probe_side_center(c);
            bool w_ok = probe_side_width_ok(c->face.w);
            bool catastrophic = !(c->face.w > 15.0f && c->face.w < 400.0f);
            c->face.valid = w_ok;
            c->side_warn = w_ok ? 0U : 1U;
            c->side_event |= (uint8_t)LF_SIDE_EVENT_MID;
            if (catastrophic && !c->side_push && c->side_retry == 0U) {
                /* 两个边界反序/间距离谱（w∉(15,400)）：当作扫描失败，回原点重扫一遍
                 * （普通超范围 80~120 不算 catastrophic，不重扫，交给复测纠正） */
                c->side_retry = 1U;
                c->side_rescan = 1U;
                c->side_verify_done = 1U;
                c->side_target = 0.0f;
                c->side_move_stage = (uint8_t)LF_SIDE_MOVE_DRIVE;
                probe_motion_reset(c);
                change_probe_phase(c, LF_PROBE_SIDE_MOVE, now);
            } else if (catastrophic) {
                /* 重扫过仍离谱：数据不可用，宁可这一面记失败也不带着错 s_m 发射 */
                c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
                lf_stop(c, now);
            } else if (c->side_push) {
                /* 台架手推模式：找齐边界即结束，供卷尺核对 PROBE_SIDE_EDGE / PROBE_SIDE_MID */
                c->out.enable = false;
                c->out.brake = false;
                c->out.duty[0] = c->out.duty[1] = 0.0f;
                change_probe_phase(c, LF_PROBE_COMPLETE_BEEPS, now);
            } else if (probe_side_yaw_should_fix(c)) {
                /* "二次微调"第 1 步：扫描自己量出车头与面不平行（|k| 超死区）→ 转正后重扫。
                 * 好工况（残余 2~3°）走不到这里，一个角都不动。 */
                c->side_yaw_fix = 1U;
                c->turn_stage = (uint8_t)LF_TURN_IDLE;
                c->turn_ok = 0U;
                c->side_event |= (uint8_t)LF_SIDE_EVENT_YAW;
                probe_motion_reset(c);
                c->out.enable = c->out.brake = true;
                c->out.duty[0] = c->out.duty[1] = 0.0f;
                break;  /* 同一拍只做决策，下一拍才开始转 */
            } else {
                probe_motion_reset(c);
                c->side_target = c->face.s_m;
                c->side_move_stage = (uint8_t)LF_SIDE_MOVE_DRIVE;
                c->side_verify_done = 0U;
                change_probe_phase(c, LF_PROBE_SIDE_MOVE, now);
            }
        } else if (sr == LF_SIDE_SCAN_FAIL) {
            c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
            if (c->side_push) {
                lf_stop(c, now);
            } else if (c->side_retry == 0U) {
                /* 失败一次：先沿面退回本遍扫描原点，再从同一几何起点重扫（否则重扫起点已在边界外，必失败） */
                c->side_retry = 1U;
                c->side_rescan = 1U;
                c->side_verify_done = 1U;
                c->side_target = 0.0f;
                c->side_move_stage = (uint8_t)LF_SIDE_MOVE_DRIVE;
                probe_motion_reset(c);
                change_probe_phase(c, LF_PROBE_SIDE_MOVE, now);
            } else {
                lf_stop(c, now);
            }
        }
        break;
    }
    case LF_PROBE_SIDE_MOVE: {
        c->out.servo_us = pr->side_us;
        c->out.park_lamp = true;
        switch (c->side_move_stage) {
        case LF_SIDE_MOVE_DRIVE: {
            float err = c->side_target - probe_side_along(c);
            if (fabsf(err) < LF_SIDE_ARRIVE_MM || elapsed >= LF_SIDE_MOVE_TIMEOUT_MS) {
                if (fabsf(err) >= LF_SIDE_ARRIVE_MM) {
                    /* 开位超时（打滑/被挡/编码器异常）：显式告警，不许"看起来成功"；
                     * 若复测开着，随后的 VERIFY 会以当前位置重新找边界并再开一次，有机会自愈。 */
                    c->side_warn = 4U;
                    c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
                }
                /* M4e 止血①：这里原先按平移段的侧距单调漂移原地转一次修正，**已删除**。
                 * 那个转身在 SIDE_SCAN 之后、LASER 之前：probe_side_verify=0 时转完直接发射，
                 * 光斑偏 h·tanε + pan_x·sinε（ε=7°、h≈180 时约 26 mm），而 50 mm 窄面半宽只有 25 mm。
                 * 硬规则：扫描开始到 LASER 结束之间一个角都不许动。侧距漂移只留诊断，不再进控制。 */
                probe_motion_reset(c);
                c->out.enable = c->out.brake = true;
                c->side_move_stage = (uint8_t)LF_SIDE_MOVE_SETTLE;
                c->phase_since = now;
            } else {
                float spd = fabsf(err) < LF_SIDE_SLOW_DIST_MM ? LF_SIDE_SLOW_SPEED_MM_S : pr->side_speed_mm_s;
                probe_drive_signed(c, err < 0.0f ? -spd : spd, dt);
            }
            break;
        }
        case LF_SIDE_MOVE_SETTLE:
            probe_motion_reset(c);
            c->out.enable = c->out.brake = true;
            if (elapsed >= LF_SIDE_SETTLE_MS) {
                if (pr->side_verify && !c->side_verify_done && probe_side_verify_begin(c, now)) {
                    c->phase_since = now; /* 复测扫描的计时/采样节拍从停稳后重新起算 */
                    c->side_move_stage = (uint8_t)LF_SIDE_MOVE_VERIFY;
                } else {
                    c->side_move_stage = (uint8_t)LF_SIDE_MOVE_DONE;
                }
            }
            break;
        case LF_SIDE_MOVE_VERIFY: {
            LfSideResult vr = probe_side_scan_tick(c, in, now, dt);
            if (vr == LF_SIDE_SCAN_RUN) break;
            bool have = false;
            float s2_local = 0.0f;
            if (vr == LF_SIDE_SCAN_OK) {
                have = probe_side_center(c);
                if (have) s2_local = c->face.s_m;
            } else {
                c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL; /* 复测失败也要能诊断 */
            }
            /* 复测宽度合法才采用，否则回退初测的宽度/分类（复测只用来校正中点） */
            if (have && probe_side_width_ok(c->face.w)) {
                c->face.valid = 1U; /* 初测宽度越界、复测正常的面也要恢复成可信 */
            } else {
                c->face.w = c->side_w_prev;
                c->face.kind = c->face.w < LF_SIDE_KIND_SPLIT_MM ? 0U : 1U;
            }
            float s2 = c->side_verify_offset + s2_local;
            float dh = s2 - c->side_s_m_ref;
            if (have && fabsf(dh) < LF_SIDE_VERIFY_TOL_MM) {
                c->face.s_m = 0.5f * (c->side_s_m_ref + s2); /* 一致：取两次平均 */
                c->side_warn = 2U;
            } else {
                c->face.s_m = have ? s2 : c->side_s_m_ref;
                c->side_warn = 3U; /* 不一致：以复测为准（无复测结果则保持初测），照样发射 */
            }
            probe_side_sync_lr(c); /* 复测可能改了 s_m/w，三者必须自洽 */
            c->side_event |= (uint8_t)LF_SIDE_EVENT_MID;
            c->side_target = c->face.s_m - c->side_verify_offset;
            c->side_verify_mode = 0U;
            c->side_verify_done = 1U;
            c->side_move_stage = (uint8_t)LF_SIDE_MOVE_DRIVE;
            c->phase_since = now;
            probe_motion_reset(c);
            break;
        }
        default: /* LF_SIDE_MOVE_DONE */
            probe_motion_reset(c);
            c->out.enable = c->out.brake = true;
            if (c->side_rescan) {
                c->side_rescan = 0U;
                probe_side_begin(c, now);
                change_probe_phase(c, LF_PROBE_SIDE_SCAN, now);
            } else {
                change_probe_phase(c, LF_PROBE_LASER, now);
            }
            break;
        }
        break;
    }
    case LF_PROBE_LASER: {
        c->out.enable = c->out.brake = c->out.park_lamp = true;
        /* 发射期间不许动舵机：侧视法要求扫描与发射严格同角（《方案》§3.3 性质①）。
         * M1 兜底（侧视锁定失败）时舵机要从侧视角甩回法线角，先等 400 ms 再点激光，
         * 避免「10 s 窗口」的前 0.4 s 打在还没转到的方向上；窗口长度因此顺延。 */
        c->out.servo_us = probe_servo_hold(c);
        uint32_t settle = (pr->scan_mode != 0U && !c->side_locked) ? LF_SIDE_LOCK_WAIT_MS : 0U;
        c->out.laser = c->out.buzzer = (elapsed >= settle) && (elapsed < settle + pr->laser_ms);
        if (elapsed >= settle + pr->laser_ms) {
            c->out.laser = c->out.buzzer = false;
            bool more = !c->side_bench && (uint8_t)(c->probe_face + 1U) < pr->faces;
            if (pr->scan_mode != 0U && c->side_locked) {
                if (!more) {
                    change_probe_phase(c, LF_PROBE_COMPLETE_BEEPS, now);
                } else {
                    /* 实测宽度不可信（重扫/微调后拟合崩）**不拦绕行**：LEG 只用 kind 的标称宽窄
                     * （LF_BOX_LONG/SHORT），probe_leg_current_kind 只看 box_first_kind+面序，
                     * 与实测 w 无关。2026-11-23 22:23 第 2 面重扫得 w=19.9 就把整轮停掉了。 */
                    if (!(c->face.w > 20.0f)) {
                        c->face.w = (c->side_w_prev > 20.0f) ? c->side_w_prev
                                  : ((c->leg_kind == 0U) ? LF_BOX_SHORT_MM : LF_BOX_LONG_MM);
                        c->side_warn = 12U;
                    }
                    /* 面号在这里前进：PROBE_EVENT 的 face、face.id、下一面的 faces 判断都靠它。
                     * 不能等 LEG 结束再 ++，否则 LEG 期间日志里的面号还是上一面。 */
                    ++c->probe_face;
                    c->leg_started = 0U;
                    c->leg_event = 0U;
                    c->leg_fail = 0U;
                    c->leg_stage = (uint8_t)LF_LEG_DRIVE1;
                    c->turn_stage = (uint8_t)LF_TURN_IDLE;
                    probe_motion_reset(c);
                    c->out.servo_us = pr->side_us;
                    change_probe_phase(c, LF_PROBE_LEG, now);
                }
            } else if (!more) {
                change_probe_phase(c, LF_PROBE_COMPLETE_BEEPS, now);
            } else if (pr->scan_mode != 0U) {
                /* 侧视模式下面没锁上（AIM valid=0 / SIDE_LOCK 失败）= 这一面打不了。
                 * 以前这里会掉进模式 A 的方阵巡边（TURN_OUT→DRIVE_EDGE1→TURN_CORNER→…），
                 * 实车表现就是"绕过箱子盲走一圈再回来"：2026-11-23 20:50 那次 AIM 无效后正是这样，
                 * 白跑 12 s 并朝箱子开过去。侧视模式一律失败即停——宁可少打一面，也不带着错误位姿乱走。 */
                c->side_locked = 0U;
                c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
                lf_stop(c, now);
            } else {
                /* 模式 A（scan_mode=0）兜底：仍沿用方阵巡边；M4 不再走这条 */
                c->side_locked = 0U;
                c->out.servo_us = c->cfg.servo_center_us;
                change_probe_phase(c, LF_PROBE_TURN_OUT, now);
            }
        }
        break;
    }
    case LF_PROBE_LEG: {
        /* 侧视模式下绕到下一面：LEG 自带 DRIVE1/TURN/DRIVE2，不换相位；
         * 舵机保持 side_us，不甩回法线角（下一面 SIDE_LOCK 会重置 side_locked）。 */
        c->out.servo_us = pr->side_us;
        c->out.park_lamp = false;
        c->out.laser = c->out.buzzer = false;
        c->side_locked = 1U;
        LfLegResult lr = probe_leg_tick(c, in, now, dt);
        if (lr == LF_LEG_OK) {
            probe_side_lock_begin(c, now);
            change_probe_phase(c, LF_PROBE_SIDE_LOCK, now);
        } else if (lr == LF_LEG_FAIL) {
            c->side_event |= (uint8_t)LF_SIDE_EVENT_FAIL;
            lf_stop(c, now);
        }
        break;
    }
    case LF_PROBE_TURN_OUT:
        c->out.servo_us = c->cfg.servo_center_us;
        speed_control(c, -dir * pr->turn_speed_mm_s, dir * pr->turn_speed_mm_s, dt, true);
        if (fabsf(d_rad) >= target_rad || elapsed >= pr->phase_timeout_ms) {
            change_probe_phase(c, LF_PROBE_DRIVE_EDGE1, now);
        }
        break;
    case LF_PROBE_DRIVE_EDGE1: {
        c->out.servo_us = c->cfg.servo_center_us;
        float hdg_corr = limit(d_rad * 120.0f, -30.0f, 30.0f);
        speed_control(c, pr->drive_speed_mm_s + hdg_corr, pr->drive_speed_mm_s - hdg_corr, dt, true);
        if (d_mm >= pr->edge1_mm || elapsed >= pr->phase_timeout_ms) {
            change_probe_phase(c, LF_PROBE_TURN_CORNER, now);
        }
        break;
    }
    case LF_PROBE_TURN_CORNER:
        c->out.servo_us = c->cfg.servo_center_us;
        speed_control(c, dir * pr->turn_speed_mm_s, -dir * pr->turn_speed_mm_s, dt, true);
        if (fabsf(d_rad) >= target_rad || elapsed >= pr->phase_timeout_ms) {
            change_probe_phase(c, LF_PROBE_DRIVE_EDGE2, now);
        }
        break;
    case LF_PROBE_DRIVE_EDGE2: {
        c->out.servo_us = c->cfg.servo_center_us;
        float hdg_corr = limit(d_rad * 120.0f, -30.0f, 30.0f);
        speed_control(c, pr->drive_speed_mm_s + hdg_corr, pr->drive_speed_mm_s - hdg_corr, dt, true);
        if (d_mm >= pr->edge2_mm || elapsed >= pr->phase_timeout_ms) {
            change_probe_phase(c, LF_PROBE_TURN_IN, now);
        }
        break;
    }
    case LF_PROBE_TURN_IN:
        c->out.servo_us = c->cfg.servo_center_us;
        speed_control(c, dir * pr->turn_speed_mm_s, -dir * pr->turn_speed_mm_s, dt, true);
        if (fabsf(d_rad) >= target_rad || elapsed >= pr->phase_timeout_ms) {
            change_probe_phase(c, LF_PROBE_APPROACH, now);
        }
        break;
    case LF_PROBE_APPROACH: {
        c->out.servo_us = c->cfg.servo_center_us;
        float hdg_corr = limit(d_rad * 100.0f, -25.0f, 25.0f);
        speed_control(c, pr->approach_speed_mm_s + hdg_corr, pr->approach_speed_mm_s - hdg_corr, dt, true);
        if ((in->obstacle_valid && in->obstacle_mm >= pr->aim_min_mm && in->obstacle_mm <= c->cfg.obstacle_stop_mm) ||
            d_mm >= pr->approach_max_mm || elapsed >= pr->phase_timeout_ms) {
            ++c->probe_face;
            c->best_servo_us = c->cfg.servo_center_us;
            c->best_obstacle_mm = UINT16_MAX;
            reset_loops(c);
            change_probe_phase(c, LF_PROBE_AIM, now);
        }
        break;
    }
    case LF_PROBE_COMPLETE_BEEPS:
        c->out.enable = c->out.brake = c->out.park_lamp = true;
        /* 单面/最后一面完成后继续保持在最佳瞄准角，不再立刻回正前方。 */
        c->out.servo_us = probe_servo_hold(c);
        c->out.buzzer = morse_2short_1long(elapsed);
        if (elapsed >= 3600U) {
            c->out = (LfOutput){0};
            c->probe_phase = LF_PROBE_IDLE;
            c->side_bench = 0U;
            c->side_push = 0U;
            c->side_locked = 0U;
            change_state(c, LF_STOPPED, now);
        }
        break;
    default:
        lf_stop(c, now);
        break;
    }
}

/* ============================================================================
 * 模块 9：1:1 硬件模拟 PID 仿真内核、200 Hz 主控制步进与快速循迹闭环
 * ============================================================================ */

/**
 * @brief 按主扩展板纯硬件模拟 PID 真实电路拓扑 (U6A->U6B/C/D->U7A->U7B/C->U10) 运行一拍
 * @note  此模式下刻意关闭软件归一化、三次外推和失线记忆，让所有可调参数与物理电阻/电位器 1:1 对应
 */
static void step_hw_pid_emu(LfController *c, float dt) {
    const LfHwPidConfig *h = &c->cfg.hw;
    float e_ana = 0.0f;
    c->hw_pid_e = hw_u6a_stage(c, &c->hw_weight_p, &e_ana);

    float lo = 0.05f - h->v_mid;
    float hi = (h->v_5v - 0.05f) - h->v_mid;

    /* 1. U6B 比例级：V_P - V_MID = -[(R49 + RV_P) / R48] * e */
    float v_p = limit(-h->kp_stage * e_ana, lo, hi);
    c->hw_v_p = v_p;

    /* 2. U6C 连续时间微分级 (10x 过采样 sub_dt=0.5ms 匹配 2.2ms / 0.5ms RC 极点) */
    float tau1 = h->d_tau1_ms * 0.001f;
    float tau2 = h->d_tau2_ms * 0.001f;
    bool first_step = !c->hw_d_ready;
    if (first_step) {
        c->hw_d_lp1 = e_ana;
        c->hw_d_lp2 = 0.0f;
        c->hw_d_ready = true;
    }
    const unsigned sub_n = 10U;
    float sub_dt = dt / (float)sub_n;
    float a1 = sub_dt / (tau1 + sub_dt);
    float a2 = sub_dt / (tau2 + sub_dt);
    for (unsigned s = 0; s < sub_n; ++s) {
        float prev_lp1 = c->hw_d_lp1;
        c->hw_d_lp1 += a1 * (e_ana - c->hw_d_lp1);
        float raw_de = (c->hw_d_lp1 - prev_lp1) / sub_dt;
        c->hw_d_lp2 += a2 * (raw_de - c->hw_d_lp2);
    }
    float v_d = limit(-(h->kd_stage_ms * 0.001f) * c->hw_d_lp2, lo, hi);
    c->hw_v_d = v_d;

    /* 3. U6D 带漏积分级 (受 JP_I = i_enable 控制，R57*C61 ≈ 1 s 泄漏时间常数) */
    if (h->i_enable) {
        float decay = 1.0f / (1.0f + dt);
        c->hw_v_i = limit(c->hw_v_i * decay - (e_ana * dt) / (h->ti_ms * 0.001f), lo, hi);
    } else {
        c->hw_v_i = 0.0f;
    }

    /* 4. U7A 反向加法器 (R62=20kΩ, R47=100kΩ -> I 支路权重 0.2) */
    float sum_p = (20.0f / h->r60_kohm) * v_p;
    float sum_d = (20.0f / h->r61_kohm) * v_d;
    float sum_i = h->i_enable ? (0.20f * c->hw_v_i) : 0.0f;
    float v_c_off = limit(-(sum_p + sum_d + sum_i), lo, hi);
    c->hw_pid_c = h->v_mid + v_c_off;

    /* 5. U7B/C 混合 -> R74~77 / C62~63 (0.625, 0.2475ms) -> U10 实测三角波比较器 */
    float dut[2] = {0};
    hw_mixer_pwm_stage(h, c->cfg.max_duty, v_c_off, c->hw_ana_speed, !first_step, dt, dut);

    c->out.enable = true;
    c->out.brake = false;
    c->out.duty[0] = dut[0];
    c->out.duty[1] = dut[1];
    c->applied_duty[0] = dut[0];
    c->applied_duty[1] = dut[1];
    c->target_mm_s[0] = c->target_mm_s[1] = 0.0f;
}

void lf_step(LfController *c, const LfInput *in, uint32_t now, float dt) {
    c->out = (LfOutput){.servo_us = c->cfg.servo_center_us};
    if (!lf_config_valid(&c->cfg)) { lf_trip(c, LF_BAD_CONFIG); return; }
    if (!isfinite(dt) || dt < .001f || dt > .020f) { lf_trip(c, LF_BAD_TIMING); return; }
    if (!in->driver_ok) { lf_trip(c, LF_DRIVER_FAULT); return; }
    if (!in->adc_ok) { lf_trip(c, LF_ADC_STALE); return; }

    bool all_black = false;
    bool line_valid = read_line(c, in, dt, &all_black);
    update_odometry(c, in, dt);
    c->last_obstacle_mm = in->obstacle_mm;
    c->last_obstacle_valid = in->obstacle_valid;
    /* 在所有状态下实时更新 U6A 硬件等效电压 PID_E / WEIGHT_P，供 LCD 屏与蓝牙诊断直接读取 */
    c->hw_pid_e = hw_u6a_stage(c, &c->hw_weight_p, NULL);

    if (c->state == LF_FAULT || c->state == LF_STOPPED) return;
    if (c->state == LF_CALIBRATING) {
        ++c->samples;
        for (unsigned i = 0; i < LF_CHANNELS; ++i) {
            uint16_t v = (uint16_t)c->filtered_raw[i];
            if (v < c->minimum[i]) c->minimum[i] = v;
            if (v > c->maximum[i]) c->maximum[i] = v;
        }
        if (now - c->state_since >= c->cfg.calibration_ms) {
            bool good = c->samples >= 100;
            for (unsigned i = 0; i < LF_CHANNELS; ++i) {
                c->cal.black[i] = c->minimum[i];
                c->cal.white[i] = c->maximum[i];
                if ((unsigned)(c->maximum[i] - c->minimum[i]) < c->cfg.calibration_min_span) good = false;
            }
            c->cal.valid = good;
            c->fault = good ? LF_OK : LF_BAD_CALIBRATION;
            change_state(c, LF_STOPPED, now);
        }
        return;
    }
    if (c->state == LF_MOTOR_CAL) {
        step_motor_cal(c, in, now);
        return;
    }
    if (c->state == LF_STOPPING) {
        c->out.enable = c->out.brake = true;
        if (now - c->state_since >= c->cfg.stop_brake_ms) {
            c->out = (LfOutput){0};
            change_state(c, LF_STOPPED, now);
        }
        return;
    }
    if (c->state == LF_OBSTACLE_STOP) {
        c->out.enable = c->out.brake = c->out.park_lamp = true;
        c->out.buzzer = (now - c->state_since) < 1000U;
        return;
    }
    if (c->state == LF_PROBING) {
        step_probing(c, in, now, dt);
        return;
    }
    if (c->state == LF_ARMING) {
        if (now - c->state_since < c->cfg.arm_delay_ms) return;
        if (!line_valid && !c->cfg.hw.enable) { lf_trip(c, LF_LINE_LOST); return; }
        change_state(c, LF_TRACKING, now);
    }

    if (c->cfg.obstacle_enable && in->obstacle_valid &&
        in->obstacle_mm >= 20U && in->obstacle_mm < c->cfg.obstacle_stop_mm) {
        if (++c->obstacle_hits >= 2U) {
            reset_loops(c);
            c->out.enable = c->out.brake = c->out.park_lamp = c->out.buzzer = true;
            change_state(c, LF_OBSTACLE_STOP, now);
            return;
        }
    } else {
        c->obstacle_hits = 0;
    }

    if (c->cfg.hw.enable && c->state == LF_TRACKING) {
        step_hw_pid_emu(c, dt);
        return;
    }

    if (all_black) {
        if (!c->ambiguous_pending) { c->ambiguous_pending = true; c->ambiguous_since = now; }
        if (now - c->ambiguous_since >= c->cfg.ambiguous_ms) { lf_trip(c, LF_AMBIGUOUS_LINE); return; }
    } else c->ambiguous_pending = false;
    if (c->state == LF_TRACKING && !line_valid) {
        if (!c->white_pending) { c->white_pending = true; c->white_since = now; }
        if (now - c->white_since >= c->cfg.lost_confirm_ms) {
            reset_loops(c);
            change_state(c, c->cfg.lost_brake_ms > 0 ? LF_LOST_BRAKE : LF_SEARCHING, now);
        } else {
            c->out.enable = c->out.brake = true;
            return;
        }
    } else if (line_valid) c->white_pending = false;
    if (c->state == LF_LOST_BRAKE) {
        c->out.enable = c->out.brake = true;
        if (now - c->state_since < c->cfg.lost_brake_ms) return;
        if (line_valid && !all_black) {
            reset_loops(c);
            c->last_position = c->position;
            c->derivative_ready = true;
            change_state(c, LF_TRACKING, now);
        } else if (c->cfg.search_ms > 0 && fabsf(c->last_turn) > .05f) {
            change_state(c, LF_SEARCHING, now);
        } else {
            lf_trip(c, LF_LINE_LOST);
            return;
        }
    }
    if (c->state == LF_SEARCHING) {
        if (line_valid && !all_black) {
            reset_loops(c);
            c->last_position = c->position;
            c->derivative_ready = true;
            change_state(c, LF_TRACKING, now);
        } else if (now - c->state_since >= c->cfg.search_ms) {
            lf_trip(c, LF_LINE_LOST);
            return;
        } else {
            float outer = fmaxf(85.0f, .78f * c->cfg.cruise_mm_s);
            float inner = c->cfg.allow_reverse ? -fmaxf(45.0f, .42f * c->cfg.cruise_mm_s) : 0;
            speed_control(c, c->last_turn > 0 ? outer : inner,
                          c->last_turn > 0 ? inner : outer, dt, c->cfg.allow_reverse);
            return;
        }
    }
    if (c->state != LF_TRACKING) return;

    if (!c->derivative_ready) { c->last_position = c->position; c->derivative_ready = true; }
    float derivative = all_black ? 0 : (c->position - c->last_position) / dt;
    c->derivative += dt / (c->cfg.derivative_tau_s + dt) * (derivative - c->derivative);
    c->last_position = c->position;
    if (fabsf(c->position) > .08f) c->last_turn = c->position;

    /* 外侧单探头外推：当相邻内探头降到 <0.16 时按比例平滑过渡至外推区间，消除硬阈值阶跃顿挫 */
    float edge_pos = c->position;
    if (!all_black) {
        if (c->darkness[3] >= .20f && c->darkness[0] == 0 && c->darkness[1] == 0 &&
            c->darkness[2] < .16f && c->darkness[3] < .90f) {
            float blend = (.16f - c->darkness[2]) / .16f;
            float ext = 1.0f + .60f * (.90f - c->darkness[3]) / .70f;
            edge_pos = fmaxf(c->position, c->position + blend * (ext - c->position));
        } else if (c->darkness[0] >= .20f && c->darkness[3] == 0 && c->darkness[2] == 0 &&
                   c->darkness[1] < .16f && c->darkness[0] < .90f) {
            float blend = (.16f - c->darkness[1]) / .16f;
            float ext = -(1.0f + .60f * (.90f - c->darkness[0]) / .70f);
            edge_pos = fminf(c->position, c->position + blend * (ext - c->position));
        }
    }
    float pos_eff = edge_pos * (1.0f + c->cfg.line_k3 * edge_pos * edge_pos);
    float candidate = limit(c->line_integral + c->position * dt, -.5f, .5f);
    float turn = c->cfg.line_kp * pos_eff + c->cfg.line_ki * candidate + c->cfg.line_kd * c->derivative;
    float clipped = limit(turn, -c->cfg.max_turn_mm_s, c->cfg.max_turn_mm_s);
    /* S形平滑调速：直道小摆(|pos|<=0.12)全速不掉速，进弯按S型曲线平滑衔接至curve_min，适配连续近弯 */
    float u = limit((fabsf(edge_pos) - .12f) / .88f, 0.0f, 1.0f);
    float smooth_curve = u * u * (3.0f - 2.0f * u);
    float d_damp = 1.0f / (1.0f + .018f * fminf(fabsf(c->derivative), 18.0f));
    float base = fmaxf(c->cfg.curve_min_mm_s,
                       c->cfg.curve_min_mm_s +
                           (c->cfg.cruise_mm_s - c->cfg.curve_min_mm_s) * (1.0f - .95f * smooth_curve) * d_damp);
    float minimum_wheel = c->cfg.allow_reverse ? -c->cfg.max_wheel_mm_s : 0;
    float actual_turn = (limit(base + clipped, minimum_wheel, c->cfg.max_wheel_mm_s) -
                         limit(base - clipped, minimum_wheel, c->cfg.max_wheel_mm_s)) * .5f;
    if ((turn - actual_turn) * c->position <= 0) c->line_integral = candidate;
    turn = limit(c->cfg.line_kp * pos_eff + c->cfg.line_ki * c->line_integral +
                 c->cfg.line_kd * c->derivative, -c->cfg.max_turn_mm_s, c->cfg.max_turn_mm_s);
    speed_control(c, base + turn, base - turn, dt, c->cfg.allow_reverse);

    if (c->cfg.speed_loop && now - c->state_since > 300) {
        unsigned ms = (unsigned)(dt * 1000 + .5f);
        for (unsigned i = 0; i < 2; ++i) {
            bool wrong = fabsf(c->target_mm_s[i]) > 40 &&
                         c->target_mm_s[i] * c->measured_mm_s[i] < -1200;
            bool stall = fabsf(c->target_mm_s[i]) > 50 && fabsf(c->out.duty[i]) > .25f &&
                         fabsf(c->measured_mm_s[i]) < 8;
            c->wrong_direction_ms[i] = wrong ? c->wrong_direction_ms[i] + ms : 0;
            c->stall_ms[i] = stall ? c->stall_ms[i] + ms : 0;
            if (c->wrong_direction_ms[i] >= 100) { lf_trip(c, LF_ENCODER_DIRECTION); return; }
            if (c->stall_ms[i] >= 600) { lf_trip(c, LF_MOTOR_STALL); return; }
        }
    }
}

const char *lf_state_name(LfState s) {
    static const char *const names[] = {"STOPPED", "CALIBRATING", "ARMING", "TRACKING",
        "LOST_BRAKE", "SEARCHING", "STOPPING", "FAULT", "OBSTACLE_STOP", "PROBING", "MOTOR_CAL"};
    return (unsigned)s < sizeof(names) / sizeof(names[0]) ? names[s] : "UNKNOWN";
}

const char *lf_fault_name(LfFault f) {
    static const char *const names[] = {"OK", "CALIBRATION", "ADC_STALE", "DRIVER",
        "LINE_LOST", "ALL_BLACK", "ENCODER_DIRECTION", "STALL", "TIMING", "CONFIG"};
    return (unsigned)f < sizeof(names) / sizeof(names[0]) ? names[f] : "UNKNOWN";
}

const char *lf_probe_phase_name(LfProbePhase p) {
    /* 顺序必须与 LfProbePhase 枚举一致（《接口契约_M1.md》§3：0~9 数值不变，10~18 追加在末尾） */
    static const char *const names[] = {"IDLE", "AIM", "LASER", "TURN_OUT",
        "DRIVE_EDGE1", "TURN_CORNER", "DRIVE_EDGE2", "TURN_IN", "APPROACH", "COMPLETE_BEEPS",
        "SQUARE", "EDGE", "CENTER", "ALIGN", "TURN_PAR", "SIDE_LOCK", "SIDE_SCAN", "SIDE_MOVE", "LEG"};
    return (unsigned)p < sizeof(names) / sizeof(names[0]) ? names[p] : "UNKNOWN";
}
