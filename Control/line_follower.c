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

/** @brief 切换四面打靶子阶段，同时记录该阶段起始时间戳、起始里程与起始航向角 */
static void change_probe_phase(LfController *c, LfProbePhase p, uint32_t now) {
    c->probe_phase = p;
    c->phase_since = now;
    c->phase_start_mm = c->odo.distance_mm;
    c->phase_start_rad = c->odo.heading_rad;
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
        /* 机械几何参数：840 脉冲/圈，轮径 43 mm，轮距 130 mm，探头坐标 {-1, -0.35, +0.35, +1} */
        .encoder_counts_per_rev = 840, .wheel_diameter_mm = 43, .track_width_mm = 130,
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
            .aim_ms = 1200U, .sweep_us = 220U, .laser_ms = 10000U,
            .aim_min_mm = 40U, .aim_max_mm = 400U, .phase_timeout_ms = 4500U,
            .turn_speed_mm_s = 90.0f, .drive_speed_mm_s = 120.0f, .approach_speed_mm_s = 85.0f,
            .turn_deg = 90.0f, .edge1_mm = 220.0f, .edge2_mm = 220.0f, .approach_max_mm = 260.0f
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
    change_state(c, LF_PROBING, now);
    change_probe_phase(c, LF_PROBE_AIM, now);
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

static void step_probing(LfController *c, const LfInput *in, uint32_t now, float dt) {
    if (now - c->state_since >= 180000U) { lf_stop(c, now); return; }
    const LfProbeConfig *pr = &c->cfg.probe;
    uint32_t elapsed = now - c->phase_since;
    float d_mm = c->odo.distance_mm - c->phase_start_mm;
    float d_rad = c->odo.heading_rad - c->phase_start_rad;
    float target_rad = pr->turn_deg * (3.14159265f / 180.0f);
    float dir = pr->clockwise ? 1.0f : -1.0f; /* 顺时针绕行：先左转出角，再两次右转进角 */
    c->out.servo_enable = true;
    c->out.servo_us = c->best_servo_us ? c->best_servo_us : c->cfg.servo_center_us;

    switch (c->probe_phase) {
    case LF_PROBE_AIM: {
        c->out.enable = c->out.brake = c->out.park_lamp = true;
        uint32_t settle_ms = pr->aim_ms / 4U;
        if (settle_ms > 150U) settle_ms = 150U;
        uint32_t sweep_ms = pr->aim_ms > settle_ms ? (pr->aim_ms - settle_ms) : 1U;
        int32_t sweep_off = -(int32_t)pr->sweep_us;
        if (elapsed > settle_ms) {
            uint32_t s_el = elapsed - settle_ms;
            if (s_el > sweep_ms) s_el = sweep_ms;
            sweep_off = -(int32_t)pr->sweep_us + (int32_t)((2U * (uint32_t)pr->sweep_us * s_el) / sweep_ms);
        }
        int32_t raw_us = (int32_t)c->cfg.servo_center_us + sweep_off;
        if (raw_us < 900) raw_us = 900;
        if (raw_us > 2100) raw_us = 2100;
        uint16_t cmd_us = (uint16_t)raw_us;
        c->out.servo_us = cmd_us;
        if (elapsed >= settle_ms && in->obstacle_valid &&
            in->obstacle_mm >= pr->aim_min_mm && in->obstacle_mm <= pr->aim_max_mm &&
            in->obstacle_mm < c->best_obstacle_mm) {
            c->best_obstacle_mm = in->obstacle_mm;
            c->best_servo_us = cmd_us;
        }
        if (elapsed >= pr->aim_ms) {
            c->out.servo_us = c->best_servo_us ? c->best_servo_us : c->cfg.servo_center_us;
            change_probe_phase(c, LF_PROBE_LASER, now);
        }
        break;
    }
    case LF_PROBE_LASER:
        c->out.enable = c->out.brake = c->out.park_lamp = true;
        c->out.laser = c->out.buzzer = (elapsed < pr->laser_ms);
        if (elapsed >= pr->laser_ms) {
            c->out.laser = c->out.buzzer = false;
            if ((uint8_t)(c->probe_face + 1U) >= pr->faces) {
                change_probe_phase(c, LF_PROBE_COMPLETE_BEEPS, now);
            } else {
                c->out.servo_us = c->cfg.servo_center_us;
                change_probe_phase(c, LF_PROBE_TURN_OUT, now);
            }
        }
        break;
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
        c->out.servo_us = c->cfg.servo_center_us;
        c->out.buzzer = morse_2short_1long(elapsed);
        if (elapsed >= 3600U) {
            c->out = (LfOutput){0};
            c->probe_phase = LF_PROBE_IDLE;
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
    static const char *const names[] = {"IDLE", "AIM", "LASER", "TURN_OUT",
        "DRIVE_EDGE1", "TURN_CORNER", "DRIVE_EDGE2", "TURN_IN", "APPROACH", "COMPLETE_BEEPS"};
    return (unsigned)p < sizeof(names) / sizeof(names[0]) ? names[p] : "UNKNOWN";
}
