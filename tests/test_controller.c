/**
 * @file    test_controller.c
 * @brief   智能小车控制层 (Control/) 23 项主机端自动化单元测试
 * @details
 * 本测试程序在电脑主机端 (MinGW GCC) 直接编译运行，无需连接单片机硬件，全面验证：
 *   1. calibration   : 八秒红外黑白扫描校准通过与跨度不足拒绝
 *   2. centroid      : 四路归一化位置质心单调性（外侧单路绝不回落）
 *   3. lost          : 丢线消抖确认、位置记忆保持与短时制动
 *   4. reacquire     : 丢线旋回搜索与重捕获黑线恢复正常循迹
 *   5. timeout       : 丢线搜索超时进入 LF_LINE_LOST 故障保护
 *   6. stop          : 运动中停机先执行 100 ms 主动电磁制动再撤销使能
 *   7..10. faults    : DRV_nFAULT 驱动故障、ADC 过期、周期越界、参数非法保护
 *   11..12. enc      : 编码器方向接反 (LF_ENCODER_DIRECTION) 与电机堵转 (LF_MOTOR_STALL) 诊断
 *   13. pid          : 速度环抗积分饱和与允许反转时的内轮负目标速度
 *   14. bridge       : DRV8833 驱动/制动（慢衰减）与驱动/滑行（快衰减）PWM 映射
 *   15. rollover     : 32 位毫秒计数器溢出回绕 (UINT32_MAX) 安全性
 *   16. allblack     : 持续四路全黑超时触发 LF_AMBIGUOUS_LINE 保护
 *   17. weakline     : 起跑瞬间信号过弱立即拒动保护
 *   18. integral     : 位置环饱和抗积分windup校验
 *   19. extrapolation: 外侧单路边缘单调外推与短时全黑横线平顺保持
 *   20. scurve       : 连续 S 弯快速换向（非对称 3.5× 减速度在 35 ms 内完成内轮降速）
 *   21. obstacle     : <15cm 超声波自动主动短接制动停车 + 停车灯常亮 + 1.0s 蜂鸣器长鸣
 *   22. probe        : 四面顺时针障碍物瞄准 + 10s 激光与蜂鸣器同步发射 + 3 组“两短一长”鸣叫
 *   23. motorcal     : 左右轮不同死区与斜率的 3 轮自动扫描拟合及前馈不对称补偿验证
 *   24. probeaim     : 云台步进扫掠 + 平面法线(垂直位置)最小二乘拟合精度验证
 *   25. probefit    : M1 面模型（phi/rho/rms/hits）在无噪声/噪声/丢帧/无回波下的精度
 *   26. probeside    : M2 侧视横扫（SIDE_LOCK / SIDE_SCAN / SIDE_MOVE + 复测）在合成侧扫世界里的中点/宽度精度
 *   27~28. probeturn / probetour : M3/M4 骨架（现为 SKIP）
 *   29. probesidetiltoff : SIDE_LOCK 一趟里**一次转都不许发生**（扫描前自动测斜已关闭，见 line_follower.h）
 *   30. probemovenoturn  : SIDE_MOVE 侧距单调漂移也不再转身（扫描到发射之间航向不变）
 */

#include "line_follower.h"
#include "bridge_logic.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(expr) do { if (!(expr)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#expr); exit(EXIT_FAILURE); } } while (0)

static LfInput input(uint16_t a,uint16_t b,uint16_t c,uint16_t d) {
    return (LfInput){.raw={a,b,c,d},.adc_ok=true,.driver_ok=true};
}
static void ready(LfController *c) {
    lf_init(c);c->cal.valid=true;c->cfg.speed_loop=false;c->cfg.arm_delay_ms=0;
    for(unsigned i=0;i<4;++i){c->cal.black[i]=500;c->cal.white[i]=3000;}
    assert(lf_start(c,0));
}
static void tick(LfController *c,LfInput *in,uint32_t *now,unsigned count) {
    for(unsigned i=0;i<count;++i){*now+=5;lf_step(c,in,*now,.005f);}
}
static void calibration(void) {
    LfController c;lf_init(&c);assert(!lf_start(&c,0));
    lf_calibrate(&c,0);LfInput in=input(2800,2800,2800,2800);uint32_t now=0;
    tick(&c,&in,&now,200);
    for(unsigned channel=0;channel<4;++channel){in=input(2800,2800,2800,2800);in.raw[channel]=600;tick(&c,&in,&now,200);}
    in=input(2800,2800,2800,2800);tick(&c,&in,&now,600);
    assert(c.state==LF_STOPPED && c.cal.valid && c.fault==LF_OK);
    for(unsigned i=0;i<4;++i)assert(c.cal.white[i]-c.cal.black[i]>2000);
    lf_calibrate(&c,now);tick(&c,&in,&now,1600);
    assert(!c.cal.valid && c.fault==LF_BAD_CALIBRATION);
}
static void centroid(void) {
    LfController c;ready(&c);uint32_t now=0;
    LfInput in=input(3000,500,500,3000);tick(&c,&in,&now,20);
    assert(fabsf(c.position)<.001f && c.confidence>1.9f);
    in=input(3000,3000,500,500);tick(&c,&in,&now,30);float pair=c.position;
    assert(pair>.6f && c.target_mm_s[0]>c.target_mm_s[1]);
    in=input(3000,3000,3000,500);tick(&c,&in,&now,30);
    assert(c.position>pair && c.position>.99f); /* Outer-only must not reduce position. */
    in=input(500,3000,3000,3000);tick(&c,&in,&now,30);assert(c.position<-.99f);
}
static void lost(void) {
    LfController c;ready(&c);uint32_t now=0;LfInput in=input(3000,3000,3000,500);
    tick(&c,&in,&now,30);float pos=c.position;
    in=input(3000,3000,3000,3000);tick(&c,&in,&now,8);
    assert(c.state==LF_LOST_BRAKE && c.out.brake && c.position==pos);
    assert(c.out.duty[0]==0 && c.out.duty[1]==0);
}
static void reacquire(void) {
    LfController c;ready(&c);uint32_t now=0;LfInput in=input(3000,3000,3000,500);
    tick(&c,&in,&now,30);in=input(3000,3000,3000,3000);tick(&c,&in,&now,25);
    assert(c.state==LF_SEARCHING);
    in=input(3000,500,500,3000);tick(&c,&in,&now,30);
    assert(c.state==LF_TRACKING && c.fault==LF_OK && fabsf(c.position)<.001f);
}
static void timeout(void) {
    LfController c;ready(&c);uint32_t now=0;LfInput in=input(3000,3000,3000,500);
    tick(&c,&in,&now,30);in=input(3000,3000,3000,3000);tick(&c,&in,&now,150);
    assert(c.state==LF_FAULT && c.fault==LF_LINE_LOST && !c.out.enable);
}
static void stopping(void) {
    LfController c;ready(&c);uint32_t now=0;LfInput in=input(3000,500,500,3000);
    tick(&c,&in,&now,30);lf_stop(&c,now);tick(&c,&in,&now,1);
    assert(c.state==LF_STOPPING && c.out.brake);tick(&c,&in,&now,25);
    assert(c.state==LF_STOPPED && !c.out.enable);
}
static void faults(const char *name) {
    LfController c;ready(&c);LfInput in=input(3000,500,500,3000);
    LfFault expected;
    if(!strcmp(name,"driver")){in.driver_ok=false;expected=LF_DRIVER_FAULT;}
    else if(!strcmp(name,"stale")){in.adc_ok=false;expected=LF_ADC_STALE;}
    else if(!strcmp(name,"timing")){expected=LF_BAD_TIMING;}
    else {c.cfg.encoder_counts_per_rev=0;expected=LF_BAD_CONFIG;}
    lf_step(&c,&in,5,!strcmp(name,"timing")?.1f:.005f);
    assert(c.state==LF_FAULT && c.fault==expected && !c.out.enable);
}
static void encoder_fault(bool wrong_direction) {
    LfController c;ready(&c);c.cfg.speed_loop=true;uint32_t now=0;
    LfInput in=input(3000,500,500,3000);
    if(wrong_direction)in.encoder_delta[0]=in.encoder_delta[1]=-4;
    tick(&c,&in,&now,400);
    assert(c.state==LF_FAULT && !c.out.enable);
    assert(c.fault==(wrong_direction?LF_ENCODER_DIRECTION:LF_MOTOR_STALL));
}
static void pid(void) {
    LfController c;ready(&c);c.cfg.speed_loop=true;c.cfg.allow_reverse=true;
    c.cfg.max_duty=.15f;uint32_t now=0;LfInput in=input(3000,500,500,3000);
    tick(&c,&in,&now,50);
    assert(fabsf(c.out.duty[0])<=.15001f && fabsf(c.out.duty[1])<=.15001f);
    assert(fabsf(c.speed_integral[0])<.001f && fabsf(c.speed_integral[1])<.001f);
    c.cfg.allow_reverse=true;c.cfg.line_kp=300;in=input(3000,3000,3000,500);
    tick(&c,&in,&now,100);assert(c.target_mm_s[1]<0);
}
static void bridge(void) {
    BridgeCompare q=bridge_compare(.25f,true,false,true,4000);assert(q.in1==4000 && q.in2==3000);
    q=bridge_compare(-.25f,true,false,true,4000);assert(q.in1==3000 && q.in2==4000);
    q=bridge_compare(0,true,false,true,4000);assert(q.in1==4000 && q.in2==4000);
    q=bridge_compare(.25f,true,false,false,4000);assert(q.in1==1000 && q.in2==0);
    q=bridge_compare(-.25f,true,false,false,4000);assert(q.in1==0 && q.in2==1000);
    q=bridge_compare(.6f,false,true,true,4000);assert(q.in1==0 && q.in2==0);
    q=bridge_compare(NAN,true,false,true,4000);assert(q.in1==0 && q.in2==0);
}
static void rollover(void) {
    LfController c;ready(&c);uint32_t now=UINT32_MAX-100;
    LfInput in=input(3000,3000,3000,500);tick(&c,&in,&now,30);
    in=input(3000,3000,3000,3000);tick(&c,&in,&now,150);
    assert(c.state==LF_FAULT && c.fault==LF_LINE_LOST);
}
static void allblack(void) {
    LfController c;ready(&c);uint32_t now=0;LfInput in=input(500,500,500,500);
    tick(&c,&in,&now,30);
    assert(c.state==LF_FAULT && c.fault==LF_AMBIGUOUS_LINE && !c.out.enable);
}
static void weakline(void) {
    LfController c;ready(&c);uint32_t now=0;LfInput in=input(2875,2875,2875,2875);
    tick(&c,&in,&now,1);
    assert(c.state==LF_FAULT && c.fault==LF_LINE_LOST && !c.out.enable);
}
static void integral(void) {
    LfController c;ready(&c);c.cfg.line_ki=100;c.cfg.line_kd=0;
    uint32_t now=0;LfInput in=input(3000,3000,3000,500);tick(&c,&in,&now,200);
    assert(fabsf(c.line_integral)<.001f && c.target_mm_s[1]>=0);
}
static void extrapolation(void) {
    LfController c;ready(&c);c.cfg.max_turn_mm_s=400;c.cfg.max_wheel_mm_s=500;uint32_t now=0;
    /* Full outer sensor -> pos == 1.0; fading outer sensor -> monotonic turn boost */
    LfInput in=input(3000,3000,3000,500);tick(&c,&in,&now,30);
    assert(c.position>.99f && c.position<=1.0f);
    float full_outer_left=c.target_mm_s[0];
    in=input(3000,3000,3000,2000);tick(&c,&in,&now,30);
    assert(c.position>.99f && c.target_mm_s[0]>full_outer_left+25.0f);
    /* Short cross-line (all black for 30 ms < 80 ms) does not trip fault */
    in=input(500,500,500,500);tick(&c,&in,&now,6);
    assert(c.state==LF_TRACKING && c.fault==LF_OK);
}
static void scurve(void) {
    LfController c;ready(&c);uint32_t now=0;
    LfInput in=input(3000,3000,500,500);tick(&c,&in,&now,30);
    assert(c.target_mm_s[0]>120.0f && c.target_mm_s[1]<20.0f);
    /* Sudden S-curve reversal to left bend: left wheel must decelerate in <= 35 ms (7 ticks) */
    in=input(500,500,3000,3000);tick(&c,&in,&now,7);
    assert(c.target_mm_s[0]<35.0f && c.target_mm_s[1]>c.target_mm_s[0]);
}
static void obstacle(void) {
    LfController c;ready(&c);uint32_t now=0;
    LfInput in=input(3000,500,500,3000);
    in.obstacle_valid=true;in.obstacle_mm=135; /* < 150 mm (15 cm) */
    tick(&c,&in,&now,2);
    assert(c.state==LF_OBSTACLE_STOP && c.out.enable && c.out.brake);
    assert(c.out.park_lamp && c.out.buzzer);
    /* After 1000 ms, buzzer turns off while active brake hold and park lamp stay ON */
    tick(&c,&in,&now,210);
    assert(c.state==LF_OBSTACLE_STOP && c.out.enable && c.out.brake);
    assert(c.out.park_lamp && !c.out.buzzer);
}
static void probe(void) {
    LfController c;ready(&c);uint32_t now=0;
    /* 本用例验证模式 A 的完整「AIM -> LASER -> 方阵巡边 -> APPROACH -> 下一面」流程；
     * M2 起默认 probe_scan_mode=1（侧视），故这里显式选 0 保持旧流程回归覆盖。 */
    c.cfg.probe.scan_mode=0U;
    LfInput in=input(3000,500,500,3000);
    in.obstacle_valid=true;in.obstacle_mm=130;
    tick(&c,&in,&now,2);assert(c.state==LF_OBSTACLE_STOP);
    assert(lf_start_probe(&c,now));
    assert(c.state==LF_PROBING && c.probe_phase==LF_PROBE_AIM);
    for(unsigned face=0;face<4;++face){
        /* 两段式 aim 扫掠：粗扫 aim_ms + 同幅独立复测 aim_ms（aim_ms 可调，按相位退出，别写死 tick 数） */
        in.obstacle_mm=125;
        for(unsigned k=0;k<4000 && c.probe_phase==LF_PROBE_AIM;++k) tick(&c,&in,&now,1);
        tick(&c,&in,&now,2); /* 换相位那一拍只做相位切换，输出在下一拍才落到 out 上 */
        assert(c.probe_phase==LF_PROBE_LASER && c.out.laser && c.out.buzzer && c.out.park_lamp);
        tick(&c,&in,&now,2000); /* 10 s laser + synchronous buzzer */
        if(face<3){
            in.obstacle_mm=300;
            /* Phase 3: TURN_OUT (CCW 90 deg -> left=-, right=+) */
            in.encoder_delta[0]=-4;in.encoder_delta[1]=4;
            for(unsigned k=0;k<400 && c.probe_phase==LF_PROBE_TURN_OUT;++k) tick(&c,&in,&now,1);
            assert(c.probe_phase==LF_PROBE_DRIVE_EDGE1);
            /* Phase 4: DRIVE_EDGE1 (straight 220 mm) */
            in.encoder_delta[0]=4;in.encoder_delta[1]=4;
            for(unsigned k=0;k<500 && c.probe_phase==LF_PROBE_DRIVE_EDGE1;++k) tick(&c,&in,&now,1);
            assert(c.probe_phase==LF_PROBE_TURN_CORNER);
            /* Phase 5: TURN_CORNER (CW 90 deg -> left=+, right=-) */
            in.encoder_delta[0]=4;in.encoder_delta[1]=-4;
            for(unsigned k=0;k<400 && c.probe_phase==LF_PROBE_TURN_CORNER;++k) tick(&c,&in,&now,1);
            assert(c.probe_phase==LF_PROBE_DRIVE_EDGE2);
            /* Phase 6: DRIVE_EDGE2 (straight 220 mm) */
            in.encoder_delta[0]=4;in.encoder_delta[1]=4;
            for(unsigned k=0;k<500 && c.probe_phase==LF_PROBE_DRIVE_EDGE2;++k) tick(&c,&in,&now,1);
            assert(c.probe_phase==LF_PROBE_TURN_IN);
            /* Phase 7: TURN_IN (CW 90 deg -> left=+, right=-) */
            in.encoder_delta[0]=4;in.encoder_delta[1]=-4;
            for(unsigned k=0;k<400 && c.probe_phase==LF_PROBE_TURN_IN;++k) tick(&c,&in,&now,1);
            assert(c.probe_phase==LF_PROBE_APPROACH);
            /* Phase 8: APPROACH until obstacle_mm <= 150 mm */
            in.obstacle_mm=135;in.encoder_delta[0]=3;in.encoder_delta[1]=3;
            tick(&c,&in,&now,2);
            assert(c.probe_phase==LF_PROBE_AIM && c.probe_face==face+1U);
        }
    }
    assert(c.probe_phase==LF_PROBE_COMPLETE_BEEPS && !c.out.laser);
    tick(&c,&in,&now,730); /* 3.65 s completion 3x (2 short, 1 long) beeps */
    assert(c.state==LF_STOPPED && !c.out.enable && !c.out.buzzer);
    /* Verify probe phase name helper & config validation */
    assert(!strcmp(lf_probe_phase_name(LF_PROBE_DRIVE_EDGE1), "DRIVE_EDGE1"));
    assert(!strcmp(lf_probe_phase_name(LF_PROBE_TURN_CORNER), "TURN_CORNER"));
    LfConfig bad = lf_default_config(); bad.probe.faces = 0; assert(!lf_config_valid(&bad));
    bad = lf_default_config(); bad.probe.turn_deg = 10.0f; assert(!lf_config_valid(&bad));
}
/* 云台步进扫掠 + 平面法线(垂直位置)拟合精度：
 * 把 20° 方向的人造平面回波（r = d / cos(theta - 20°)）喂给瞄准状态机，
 * 扫掠结束后锁定的舵机脉宽必须落在 1500 + 20*US_PER_DEG 附近，
 * 而不是简单返回“单点最小距离”所在的 ±60° 边缘。 */
static void probeaim(void) {
    LfController c;ready(&c);uint32_t now=0;
    /* 本用例只验证 AIM 面拟合本身（进入 LASER 后即结束），固定走模式 A 的过渡流程，
     * 避免将来 M3 把自动流程改成 AIM→TURN_PAR→SIDE_LOCK 时影响本用例。 */
    c.cfg.probe.scan_mode=0U;
    LfInput in=input(3000,500,500,3000);
    in.obstacle_valid=true;in.obstacle_mm=130;
    tick(&c,&in,&now,2);assert(c.state==LF_OBSTACLE_STOP);
    assert(lf_start_probe(&c,now));
    const float normal_deg=20.0f, plane_mm=150.0f;
    unsigned guard=0, ping_ticks=0;
    while(c.probe_phase==LF_PROBE_AIM && ++guard<4000){
        /* 真实硬件上 TIM17 每 60 ms 才触发一次超声，回波值在两次触发之间保持不变 */
        if(++ping_ticks>=12U){
            ping_ticks=0U;
            float deg=((float)c.out.servo_us-(float)c.cfg.servo_center_us)/LF_PROBE_SCAN_US_PER_DEG;
            float rel=(deg-normal_deg)*0.01745329252f;
            in.obstacle_mm=(uint16_t)(plane_mm/cosf(rel)+0.5f);
        }
        now+=5;lf_step(&c,&in,now,.005f);
    }
    assert(c.probe_phase==LF_PROBE_LASER);
    assert(c.scan_points>=9U);
    float found_deg=((float)c.best_servo_us-(float)c.cfg.servo_center_us)/LF_PROBE_SCAN_US_PER_DEG;
    assert(fabsf(found_deg-normal_deg)<4.0f);
    assert(c.best_obstacle_mm>=140U && c.best_obstacle_mm<=170U);
    /* 步进扫掠步长参数必须受配置校验约束 */
    LfConfig bad=lf_default_config();bad.probe.scan_step_us=5U;assert(!lf_config_valid(&bad));
    bad=lf_default_config();bad.probe.scan_step_us=500U;assert(!lf_config_valid(&bad));
    assert(lf_config_valid(&c.cfg));
}
/* ============================================================================
 * M1~M4 瞄准/导航类用例的公共工具与骨架
 * ==========================================================================*/

/** 合成「带波束外扩」的超声回波，供瞄准类单测使用。
 *  面垂直于云台轴、距轴 rho；面中心横向偏移 c；半宽 a；波束半角 gamma。
 *  只要波束边缘还能擦到面就有回波，所以量到的边界会比真实边界外扩 rho*tan(gamma)。
 *  返回 0 表示该角度没有回波。 */
static uint16_t synth_mm(float th_deg, float rho, float c, float a, float gamma_deg) {
    const float rad = 0.01745329252f;
    float y = rho * tanf(th_deg * rad);
    float reach = a + rho * tanf(gamma_deg * rad);
    if (fabsf(y - c) > reach) return 0U;
    return (uint16_t)(rho / cosf(th_deg * rad) + 0.5f);
}

/** 已注册但尚未实现的用例：打印 SKIP 并正常退出（保持 -Test 全绿）。 */
static void probe_pending(const char *name) {
    printf("SKIP %s (待实现，见 接口契约_M1.md)\n", name);
}

/* ---- 合成回波的噪声/丢帧版本（M1~M4 共用） ---- */
static uint32_t g_synth_rng = 20261122U;
static float synth_rand01(void) {
    g_synth_rng = g_synth_rng * 1664525U + 1013904223U;
    return (float)((g_synth_rng >> 8) & 0xFFFFU) / 65535.0f;
}
/** 在 synth_mm 基础上叠加 [0,sigma] 均匀噪声与 drop 丢帧率；返回 0 表示该帧无回波 */
static uint16_t synth_noisy(float th_deg, float rho, float half_w, float gamma_deg, float sigma, float drop) {
    if (drop > 0.0f && synth_rand01() < drop) return 0U;
    uint16_t d = synth_mm(th_deg, rho, 0.0f, half_w, gamma_deg);
    if (d == 0U || sigma <= 0.0f) return d;
    float v = (float)d + (synth_rand01() * 2.0f - 1.0f) * sigma;
    return (uint16_t)(v < 1.0f ? 1.0f : v + 0.5f);
}
/** 跑完一次 AIM（按真实硬件的 60 ms 超声节拍喂合成回波），返回时 face 已填好 */
static void aim_run(LfController *c, LfInput *in, uint32_t *now,
                    float phi_deg, float rho, float half_w, float sigma, float drop) {
    unsigned guard = 0, ping_ticks = 0;
    while (c->probe_phase == LF_PROBE_AIM && ++guard < 4000U) {
        if (++ping_ticks >= 12U) {
            ping_ticks = 0U;
            float deg = ((float)c->out.servo_us - (float)c->cfg.servo_center_us) / LF_PROBE_SCAN_US_PER_DEG;
            in->obstacle_mm = synth_noisy(deg - phi_deg, rho, half_w, 7.5f, sigma, drop);
            in->obstacle_valid = in->obstacle_mm != 0U;
        }
        *now += 5U;
        lf_step(c, in, *now, 0.005f);
    }
}
/** 起一次完整的单面探测流程（模式 A 过渡流程：AIM 后直接进 LASER，供 AIM 面拟合类用例使用） */
static void probe_start(LfController *c, LfInput *in, uint32_t *now) {
    ready(c);
    c->cfg.probe.scan_mode = 0U;
    *in = input(3000, 500, 500, 3000);
    in->obstacle_valid = true;
    in->obstacle_mm = 130;
    tick(c, in, now, 2);
    assert(c->state == LF_OBSTACLE_STOP);
    assert(lf_start_probe(c, *now));
}

/** M1：面模型（phi/rho/rms/hits）在无噪声基线与噪声/丢帧下的精度。 */
static void probefit(void) {
    /* ① 合成回波器自检：面心正对 150 mm、半宽 25 mm 的窄面 */
    assert(synth_mm(0.0f, 150.0f, 0.0f, 25.0f, 7.5f) == 150U);
    assert(synth_mm(10.0f, 150.0f, 0.0f, 25.0f, 7.5f) == 152U);
    assert(synth_mm(30.0f, 150.0f, 0.0f, 25.0f, 7.5f) == 0U);

    /* ② 无噪声基线：拟合本身不许带系统偏置 */
    {
        LfController c; LfInput in; uint32_t now = 0;
        probe_start(&c, &in, &now);
        aim_run(&c, &in, &now, 0.0f, 150.0f, 75.0f, 0.0f, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        assert(c.face.valid);
        assert(fabsf(c.face.phi * 57.2957795f) < 0.3f);
        assert(fabsf(c.face.rho - 150.0f) < 2.0f);
        assert(c.face.hits >= 3U);
    }
    /* ③ 宽面 + 12° 斜置 + 3 mm 噪声 + 10% 丢帧 */
    {
        LfController c; LfInput in; uint32_t now = 0;
        probe_start(&c, &in, &now);
        aim_run(&c, &in, &now, -12.0f, 150.0f, 75.0f, 2.0f, 0.10f);
        float err_deg = fabsf(c.face.phi * 57.2957795f + 12.0f);
        assert(err_deg < 1.5f);
        assert(fabsf(c.face.rho - 150.0f) < 5.0f);
        assert(c.face.rms < 6.0f);
        assert(c.face.valid);
    }
    /* ④ 窄面 + 8° 斜置 + 噪声/丢帧：面只有 50 mm 宽，是最难的一档 */
    {
        LfController c; LfInput in; uint32_t now = 0;
        probe_start(&c, &in, &now);
        aim_run(&c, &in, &now, 8.0f, 150.0f, 25.0f, 2.0f, 0.15f);
        float err_deg = fabsf(c.face.phi * 57.2957795f - 8.0f);
        /* 50 mm 窄面在 ±30° 扫掠里只有 3 个采样点，3 点拟合的法线精度约 3°；
           侧视横扫只用 φ 决定转多少度（仿射性质保证中点不受影响），3° 足够 */
        assert(err_deg < 3.5f);
        assert(fabsf(c.face.rho - 150.0f) < 5.0f);
        assert(c.face.rms < 6.0f);
        assert(c.face.valid);
    }
    /* ⑤ 无回波：不许崩，且必须判为无效 */
    {
        LfController c; LfInput in; uint32_t now = 0;
        probe_start(&c, &in, &now);
        unsigned guard = 0;
        in.obstacle_valid = false; in.obstacle_mm = 0;
        while (c.probe_phase == LF_PROBE_AIM && ++guard < 4000U) { now += 5U; lf_step(&c, &in, now, 0.005f); }
        assert(!c.face.valid);
    }
}
/* ============================================================================
 * M2：侧视横扫（SIDE_LOCK / SIDE_SCAN / SIDE_MOVE + 复测）的合成侧扫世界
 * ==========================================================================*/

/** @brief 侧扫虚拟世界：面沿扫描轴的区间、侧视站立距离、车不平行斜率、噪声/丢帧/背景回波 */
typedef struct {
    float center_mm;  /**< 面中心在扫描坐标（车初始轮轴位置为 0）里的位置 */
    float half_w_mm;  /**< 面半宽：25 = 窄面 50 mm，75 = 宽面 150 mm */
    float h_mm;       /**< s=0 处的侧视站立距离 */
    float tilt;       /**< 横向倾斜：h(s) = h_mm + tilt·s（模拟车头与面不平行） */
    float gamma_deg;  /**< 超声波束半角 */
    float sigma_mm;   /**< 回波距离噪声（±均匀） */
    float drop;       /**< 丢帧率 0~1 */
    float bg_mm;      /**< 面外回波距离（0 = 无回波；>0 模拟旁边的墙/邻面） */
} SynthSide;

/** @brief 合成侧视回波：波束足印 [s-fp, s+fp] 与面区间有交集才有回波，否则返回背景距离或 0 */
static uint16_t synth_side_echo(const SynthSide *w, float s) {
    float h = w->h_mm + w->tilt * s;
    if (h < 20.0f) h = 20.0f;
    float fp = h * tanf(w->gamma_deg * 0.01745329252f); /* 波束足印半宽 = h·tanγ */
    float d;
    if (fabsf(s - w->center_mm) <= w->half_w_mm + fp) d = h;
    else d = w->bg_mm;
    if (d > 0.0f && w->sigma_mm > 0.0f) {
        d += (synth_rand01() * 2.0f - 1.0f) * w->sigma_mm;
        if (d < 1.0f) d = 1.0f;
    }
    if (w->drop > 0.0f && synth_rand01() < w->drop) d = 0.0f;
    return d > 0.0f ? (uint16_t)(d + 0.5f) : 0U;
}

/** @brief 台架侧扫启动：不跑 AIM/避障流程，人工已把车摆成与面平行，直接进 SIDE_LOCK */
static void side_start(LfController *c, LfInput *in, uint32_t *now, bool verify, bool push,
                       uint16_t side_us) {
    lf_init(c);
    c->cal.valid = true;
    c->cfg.arm_delay_ms = 0;
    for (unsigned i = 0; i < 4U; ++i) { c->cal.black[i] = 500; c->cal.white[i] = 3000; }
    c->cfg.probe.side_verify = verify;
    if (side_us != 0U) c->cfg.probe.side_us = side_us;
    *in = input(3000, 500, 500, 3000);
    in->obstacle_valid = true;
    in->obstacle_mm = 180U;
    *now = 0;
    assert(lf_start_side_probe(c, *now, push));
    assert(c->state == LF_PROBING && c->probe_phase == LF_PROBE_SIDE_LOCK);
}

/**
 * @brief 跑完一次台架侧扫：按 60 ms 超声节拍喂合成回波，并按控制器目标轮速（或人工推车速度）
 *        生成编码器脉冲，直到进入 LASER / COMPLETE_BEEPS 或退出 PROBING。
 */
static void side_run(LfController *c, LfInput *in, uint32_t *now, const SynthSide *w,
                     bool push, float push_mm_s) {
    const float mm_per_count = 3.14159265f * c->cfg.wheel_diameter_mm / c->cfg.encoder_counts_per_rev;
    float frac[2] = {0.0f, 0.0f};
    unsigned ping = 0U, guard = 0U;
    while (c->state == LF_PROBING && ++guard < 80000U) {
        if (c->probe_phase == LF_PROBE_LASER || c->probe_phase == LF_PROBE_COMPLETE_BEEPS) break;
        if (++ping >= 12U) { /* 12 × 5 ms = 60 ms：与 board.c TIM17 超声触发节拍一致 */
            ping = 0U;
            uint16_t d = synth_side_echo(w, c->odo.x_mm);
            in->obstacle_mm = d;
            in->obstacle_valid = d != 0U;
        }
        float v[2] = {0.0f, 0.0f};
        if (push && c->probe_phase == LF_PROBE_SIDE_SCAN) {
            /* 手推模式：人跟着控制器的 FWD/REV 子状态来回推 */
            float dir = c->side_stage == (uint8_t)LF_SIDE_FWD ? 1.0f : -1.0f;
            v[0] = v[1] = push_mm_s * dir;
        } else if (!push) {
            v[0] = c->target_mm_s[0];
            v[1] = c->target_mm_s[1];
        }
        for (unsigned i = 0; i < 2U; ++i) {
            float counts = v[i] * 0.005f / mm_per_count + frac[i];
            int16_t whole = (int16_t)(counts >= 0.0f ? floorf(counts) : ceilf(counts));
            frac[i] = counts - (float)whole;
            in->encoder_delta[i] = whole;
        }
        *now += 5U;
        lf_step(c, in, *now, 0.005f);
    }
    assert(guard < 80000U);
}

/** @brief 侧扫用例公共断言：两边界齐、宽度 γ 补偿正确、中点误差在容差内、宽窄分类正确 */
static void side_expect(const LfController *c, float center_mm, float half_w_mm, float mid_tol,
                        float w_tol) {
    assert(c->side_edges == 2U);
    assert(c->face.valid);
    assert(fabsf(c->face.w - 2.0f * half_w_mm) <= w_tol);
    assert(fabsf(c->face.s_m - center_mm) <= mid_tol);
    assert(c->face.kind == (half_w_mm < 50.0f ? 0U : 1U));
}

/**
 * M2：侧视横扫。合成回波按「波束足印 ∩ 面区间」生成（γ=7.5°，与固件默认 probe_beam_half_deg 一致），覆盖 12 组场景：
 *   宽面居中（含 SIDE_MOVE 实际到位检查）、宽面+复测（含 side_warn=2、LASER 保持 side_us）、
 *   偏心 40 mm、窄面、车不平行 8°（斜率拟合必须吸收漂移）、15% 丢帧×3 种子、面外背景墙、
 *   近距背景（band<dev<jump，不许误判边界）、手推台架模式、完全无回波、
 *   SIDE_SCAN 失败→退回原点重扫→停机、SIDE_LOCK 的 rho+pan_x 自检失败→FAIL+停机、
 *   M2 自动流程保持 M1（AIM→LASER，不进侧扫）。
 * 另含 11 个新参数越界校验与 5 个新相位名。
 * 判据来自《方案》§12 / 《接口契约》§8：宽面 |s_m|<3 mm、w=150±8；偏心 |s_m-40|<5；
 * 窄面 |s_m|<5 且判窄；不平行 |s_m|<5；15% 丢帧不误判边界；激光必须保持扫描侧角（性质①）。
 */
static void probeside(void) {
    /* ① 宽面居中：±75 mm、h=180、σ=1 mm、不丢帧（关复测，验证基本流程 + SIDE_MOVE 到位）。
     *    这里只验"干净世界"的流程与 3 mm 判据；丢帧鲁棒性由 ⑥（15% 丢帧×3 种子）覆盖。
     *    （SIDE_LOCK 两点测斜修好后会多采 5 帧回波，噪声序列整体后移；10% 丢帧下
     *      「最后一个面内样本」会被丢帧提前 ~5 mm，s_m 的 3 mm 判据本来就在边缘。） */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.0f, 7.5f, 1.0f, 0.0f, 0.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        assert(c.side_locked == 1U);
        side_expect(&c, 0.0f, 75.0f, 3.0f, 8.0f);
        assert(fabsf(c.face.h - 180.0f) < 5.0f);
        assert(fabsf(c.odo.x_mm - c.face.s_m) < 4.0f); /* SIDE_MOVE 确实开到了中点 */
        assert(c.face.id == 0U);
    }
    /* ② 宽面居中 + 发射前复测：SIDE_VERIFY 跑完、两次中点一致 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, true, false, 2300U);
        /* 把 AIM 的"最佳角"故意设成中位 1500：激光阶段若误用 best_servo_us 就会被下面一条断言抓住 */
        c.best_servo_us = c.cfg.servo_center_us;
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        assert(c.side_verify_done == 1U);
        assert(c.side_warn == 2U); /* 2 = 复测一致、已取平均 */
        side_expect(&c, 0.0f, 75.0f, 3.0f, 8.0f);
        assert(fabsf(c.odo.x_mm - c.face.s_m) < 4.0f);
        /* 《方案》§3.3 性质①：发射必须保持扫描时的 side_us（而不是 AIM 的 best_servo_us） */
        tick(&c, &in, &now, 1U);
        assert(c.out.servo_us == 2300U);
        assert(c.out.laser && c.out.park_lamp);
    }
    /* ③ 宽面偏心 40 mm：中点必须跟着偏 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {40.0f, 75.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        side_expect(&c, 40.0f, 75.0f, 5.0f, 8.0f);
        assert(fabsf(c.odo.x_mm - 40.0f) < 5.0f);
    }
    /* ④ 窄面 ±25 mm：补偿后 50 mm，必须判为窄面 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 25.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        /* 窄面：过去 60 mm 平移会把光束带出 ±25 mm 面（h2 凑不满 3 帧），再退回原点用 20 mm 小基线重测。
         * 该路径随扫描前自动测斜一起停用（LF_SIDE_TILT_PROBE_ENABLE=0，见 line_follower.h）：
         * 现在窄面也不平移、不回退重测，h 直接用 h1，扫描照常跑完（不许卡到 9 s 超时）。 */
        assert(LF_SIDE_TILT_PROBE_ENABLE == 0);
        assert(c.side_tilt_retry == 0U);
        assert(c.side_shift_peak_mm < 1.0f);
        side_expect(&c, 0.0f, 25.0f, 5.0f, 8.0f);
    }
    /* ④b 远距离 h=300 mm + 复测：复测行程必须按 h·tanγ（≈39.5 mm）与确认距离动态算，
     *      固定 w/2+50 会在边界确认完成前撞上限；REV 也必须相对原点对称，否则折返点再走 260 mm 不够 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 300.0f, 0.0f, 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, true, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        side_expect(&c, 0.0f, 75.0f, 5.0f, 8.0f);
        assert(fabsf(c.face.h - 300.0f) < 6.0f);
        assert(c.side_warn == 2U); /* 复测成功且取平均 */
    }
    /* ⑤ 车不平行 8°（tan8°≈0.14，比实车 3~5° 更狠）：h 随 s 漂移 ±14 mm，
     *    面内判据必须靠 8 点拟合（含斜率 k）把它吃掉、不能判成外点 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.1405f, 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        side_expect(&c, 0.0f, 75.0f, 5.0f, 8.0f);
    }
    /* ⑥ 15% 丢帧 × 3 个随机种子：单帧/双帧丢回波不许被当成边界 */
    for (unsigned seed = 0; seed < 3U; ++seed) {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.15f, 0.0f};
        g_synth_rng = 20261122U + seed * 7919U;
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        side_expect(&c, 0.0f, 75.0f, 5.0f, 8.0f);
    }
    /* ⑦ 邻面/背景回波：面外 60 mm 处有墙，不许把墙当边界，也不许把宽度扫大 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.10f, 240.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        side_expect(&c, 0.0f, 75.0f, 5.0f, 8.0f);
    }
    /* ⑧ 台架手推模式：电机断电、人工跟 FWD/REV 来回推，找齐边界后进完成提示 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, false, true, 2400U);
        side_run(&c, &in, &now, &w, true, 40.0f);
        assert(c.probe_phase == LF_PROBE_COMPLETE_BEEPS);
        assert(c.side_push == 1U);
        side_expect(&c, 0.0f, 75.0f, 5.0f, 8.0f);
    }
    /* ⑨ 完全无回波：SIDE_LOCK 超时 -> 台架模式直接优雅停机，不许崩 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide dead = {1000.0f, 75.0f, 180.0f, 0.0f, 7.5f, 0.0f, 0.0f, 0.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &dead, false, 0.0f);
        assert(c.state != LF_PROBING && c.fault == LF_OK);
        assert(c.side_locked == 0U);
        tick(&c, &in, &now, 30U);
        assert(c.state == LF_STOPPED);
    }
    /* ⑨b SIDE_SCAN 失败 → 退回本遍扫描原点重扫一次 → 仍失败 → 优雅停机
     *     （failure 时车已跑出 200+ mm，这条"先回原点再重扫"的路径最容易写错） */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        c.cfg.probe.side_reach_mm = 80.0f; /* 真实边界在 ±98.7 mm，reach 不够 → 两次都必然失败 */
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.state != LF_PROBING && c.fault == LF_OK);
        assert(c.side_retry == 1U); /* 失败一次 + 退回原点重扫一次 */
        assert((c.side_event & (uint8_t)LF_SIDE_EVENT_FAIL) != 0U);
        assert((c.side_event & (uint8_t)LF_SIDE_EVENT_EDGE) == 0U); /* 一个边界都没扫到 */
        tick(&c, &in, &now, 30U);
        assert(c.state == LF_STOPPED);
    }
    /* ⑨c 面外回波只比 h 远 20 mm（落在 band=15 与 jump=40 之间的歧义带）：不许当成边界，
     *     扫到 reach 仍找不到边界 → 重扫一次 → 停机（宁可这一面失败，也不拿假边界算中点） */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, 0.0f, 7.5f, 2.0f, 0.10f, 200.0f};
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.state != LF_PROBING && c.fault == LF_OK);
        assert(c.side_retry == 1U);
        assert((c.side_event & (uint8_t)LF_SIDE_EVENT_FAIL) != 0U);
        tick(&c, &in, &now, 30U);
        assert(c.state == LF_STOPPED);
    }
    /* ⑩ M3 的自动流程：默认 probe_scan_mode=1，AIM 成功后必须进 TURN_PAR（不再是 M2 的 LASER）；
     *     这正是 M2 接入分支 side_bench 死代码被修掉后的行为变化 */
    {
        LfController c; ready(&c); uint32_t now = 0;
        LfInput in = input(3000, 500, 500, 3000);
        in.obstacle_valid = true; in.obstacle_mm = 130U;
        tick(&c, &in, &now, 2U);
        assert(c.state == LF_OBSTACLE_STOP);
        assert(lf_start_probe(&c, now));
        assert(c.cfg.probe.scan_mode == 1U); /* 默认就是侧视模式 */
        aim_run(&c, &in, &now, 0.0f, 150.0f, 75.0f, 2.0f, 0.10f);
        assert(c.probe_phase == LF_PROBE_TURN_PAR); /* M3：AIM → TURN_PAR → SIDE_LOCK */
        assert(c.side_locked == 0U);
        assert((c.side_event & (uint8_t)LF_SIDE_EVENT_EDGE) == 0U);
        assert(c.face.valid);
    }
    /* ⑩b SIDE_LOCK 的期望值自检已改为**只诊断不拦**（2026-11-23 22:25）：
     *      h 落在 [80,450] 但和 rho+pan_x 差 80 mm 时照样锁定并进扫描，只记 warn=13。
     *      理由：LEG 只用 kind 的标称宽窄（不依赖实测 w/h 的绝对值），扫描种子窗口用的是
     *      实测 h（自洽）；用"死算落点"当门禁会把 21:36（差 5.8 mm）、21:56（差 1.0 mm）
     *      这种正常落点判死。 */
    {
        LfController c; LfInput in; uint32_t now;
        side_start(&c, &in, &now, false, false, 2400U);
        c.face.valid = true;   /* 注入 AIM 面模型（真实台架入口没有 AIM，故 pred=0 跳过自检） */
        c.face.rho = 150.0f;   /* pred = rho + pan_x = 180 mm；实测 100 → 差 80 */
        unsigned guard = 0U;
        while (c.probe_phase != LF_PROBE_SIDE_SCAN && ++guard < 2000U) {
            in.obstacle_mm = 100U;
            in.obstacle_valid = true;
            in.encoder_delta[0] = in.encoder_delta[1] = 0;
            now += 5U;
            lf_step(&c, &in, now, 0.005f);
        }
        assert(guard < 2000U);
        assert(c.probe_phase == LF_PROBE_SIDE_SCAN);   /* 不再停机 */
        assert(c.side_locked == 1U);
        assert(fabsf(c.side_h - 100.0f) < 0.01f);      /* 实测值照样写进去 */
        assert(c.side_lock_warn == 13U);               /* 超差只留诊断标记 */
        lf_stop(&c, now);
    }
    /* ⑪ 配置校验：M2 新增参数越界必须被拒 */
    {
        LfConfig def = lf_default_config();
        assert(lf_config_valid(&def));
        LfConfig bad = def; bad.probe.side_band_mm = 2.0f; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.side_jump_mm = 5.0f; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.side_jump_mm = bad.probe.side_band_mm; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.side_speed_mm_s = 5.0f; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.side_reach_mm = 20.0f; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.side_us = 500U; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.side_us = 2500U; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.scan_mode = 2U; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.fit_rms_mm = 50.0f; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.beam_half_deg = 30.0f; assert(!lf_config_valid(&bad));
        bad = def; bad.probe.pan_x_mm = 200.0f; assert(!lf_config_valid(&bad));
    }
    /* ⑫ 新相位名（日志解析用）与台架入口守卫 */
    assert(!strcmp(lf_probe_phase_name(LF_PROBE_TURN_PAR), "TURN_PAR"));
    assert(!strcmp(lf_probe_phase_name(LF_PROBE_SIDE_LOCK), "SIDE_LOCK"));
    assert(!strcmp(lf_probe_phase_name(LF_PROBE_SIDE_SCAN), "SIDE_SCAN"));
    assert(!strcmp(lf_probe_phase_name(LF_PROBE_SIDE_MOVE), "SIDE_MOVE"));
    assert(!strcmp(lf_probe_phase_name(LF_PROBE_LEG), "LEG"));
    {
        LfController c;
        lf_init(&c);
        c.cal.valid = true;
        c.cfg.probe.scan_mode = 0U;
        assert(!lf_start_side_probe(&c, 0U, false)); /* 角度法模式下台架侧扫入口必须拒绝 */
    }
}
/**
 * @brief 只跑到 SIDE_LOCK 结束（进 SIDE_SCAN）或停机：给只验锁定/测斜行为的用例用（不跑整条侧扫）
 * @note  编码器按控制器自己的 target_mm_s 生成，测斜的 60 mm 平移/退回与原地转都能真的走。
 */
static void side_run_lock(LfController *c, LfInput *in, uint32_t *now, const SynthSide *w) {
    const float mm_per_count = 3.14159265f * c->cfg.wheel_diameter_mm / c->cfg.encoder_counts_per_rev;
    float frac[2] = {0.0f, 0.0f};
    unsigned ping = 0U, guard = 0U;
    while (c->state == LF_PROBING && c->probe_phase == LF_PROBE_SIDE_LOCK && ++guard < 4000U) {
        if (++ping >= 12U) { /* 60 ms 超声节拍 */
            ping = 0U;
            uint16_t d = synth_side_echo(w, c->odo.x_mm);
            in->obstacle_mm = d;
            in->obstacle_valid = d != 0U;
        }
        for (unsigned i = 0; i < 2U; ++i) {
            float counts = c->target_mm_s[i] * 0.005f / mm_per_count + frac[i];
            int16_t whole = (int16_t)(counts >= 0.0f ? floorf(counts) : ceilf(counts));
            frac[i] = counts - (float)whole;
            in->encoder_delta[i] = whole;
        }
        *now += 5U;
        lf_step(c, in, *now, 0.005f);
    }
    assert(guard < 4000U);
}

/**
 * 扫描前自动测斜已关闭（LF_SIDE_TILT_PROBE_ENABLE=0）的回归用例。
 * 现场依据（2026-11-23 20:41 台架）：第一次转 TURN_PAR（cmd −98.8 / ach −98.9）之后车头是正的，
 * 紧接着本测斜给出 h1=179 / h2=202 / h3=167、eps=21.0° → corr=+21°，车头从 −93.3° 被转到 −67.1°
 * （实测 +26.1°，230 ms），此后整轮跑偏。而 179/202/167 三个读数全落在侧视超声 side_us=2400 处的
 * 双稳态格点上（同一静止位置在 167/179/192/202 之间跳，跨次不可重复）——这一步不是修正，是把噪声当信号。
 * 现在的要求：SIDE_LOCK 一趟里
 *   ① 只采 h1，不做 60/30 mm 平移（side_shift_peak_mm == 0）；
 *   ② 一次都不许转（turn_start_ms == 0、side_tilt_iter == 0、corr == 0）；
 *   ③ h 直接用 h1、h3 沿用 h1，然后照常进 SIDE_SCAN。
 * 合成世界里故意把面摆斜 20°（h 随 s 明显变化）：旧代码在这里会转 20°，新代码必须一动不动。
 */
static void probesidetiltoff(void) {
    const float d2r = 3.14159265f / 180.0f;
    LfController c; LfInput in; uint32_t now;
    SynthSide w = {0.0f, 75.0f, 180.0f, tanf(20.0f * d2r), 7.5f, 0.6f, 0.0f, 0.0f};
    side_start(&c, &in, &now, false, false, 2400U);
    side_run_lock(&c, &in, &now, &w);
    assert(LF_SIDE_TILT_PROBE_ENABLE == 0);                       /* 开关必须是关的 */
    assert(c.side_shift_peak_mm < 1.0f);                          /* 没有 60/30 mm 平移 */
    assert(c.turn_start_ms == 0U);                                /* 没有调用 probe_turn_to */
    assert(c.side_tilt_iter == 0U);                               /* 一次都没转 */
    assert(c.side_tilt_corr_rad == 0.0f);
    assert(c.side_h3 == c.side_h1);                               /* h3 沿用 h1 */
    assert(c.side_h >= LF_SIDE_H_MIN_MM && c.side_h <= LF_SIDE_H_MAX_MM);
    assert(c.side_locked == 1U);
    assert(c.state == LF_PROBING && c.probe_phase == LF_PROBE_SIDE_SCAN); /* 照常进侧扫 */
}

/**
 * 扫描结束后的"二次微调"（长基线判偏航 → 转正 → 退回原点重扫）：
 *   ① 面斜 10°（>4° 死区）：必须触发一次转正，转完退回原扫描原点重扫，最终 s_m/w 仍然正确；
 *   ② 面斜 2°（<4° 死区）：一次都不许转（好工况零打扰）。
 * 背景：7bded7f 那版把判据放在 SIDE_MOVE，要求平移 >= 100 mm，而 SIDE_MOVE 只走 |s_m|~20 mm，
 *       那个 if 数学上永不触发（所以"从来没生效过"）；2e00745 删除后立下"扫描→发射间不许动角"
 *       的硬规则，其依据是 verify=0 时转完直接发射会偏 h·tanε≈26 mm。现在 verify=1，
 *       规则放宽为"要动角就必须紧接着重扫"，由本用例守着。
 */
static void probesideyawfix(void) {
    const float d2r = 3.14159265f / 180.0f;
    /* 2026-11-24 起二次微调重新启用（LF_SIDE_YAW_ENABLE=1），死区按现场要求收到 ±1°：
     *   ① 面斜 10°（>> 1°）：必须触发转正 → 退回原扫描原点重扫 → 仍正常走到 LASER；
     *   ② 面斜 0.4°（< 1°）：一次都不许转（好工况零打扰）。
     * 硬规则不变：要动角就必须紧接着重扫（verify=1 下性质①才成立）。 */
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, tanf(10.0f * d2r), 7.5f, 2.0f, 0.10f, 0.0f};
        side_start(&c, &in, &now, true, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        assert(c.side_yaw_fix >= 1U);      /* 10° 必须转正 */
        assert(c.turn_start_ms != 0U);     /* probe_turn_to 被调用过 */
        assert(fabsf(c.side_yaw_deg) > 1.0f);
    }
    {
        LfController c; LfInput in; uint32_t now;
        SynthSide w = {0.0f, 75.0f, 180.0f, tanf(0.4f * d2r), 7.5f, 0.0f, 0.0f, 0.0f};
        side_start(&c, &in, &now, true, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        assert(c.side_yaw_fix == 0U);      /* 死区内不许转 */
        assert(c.turn_start_ms == 0U);
        assert(fabsf(c.side_yaw_deg) < LF_SIDE_YAW_DEADBAND_DEG);
        side_expect(&c, 0.0f, 75.0f, 5.0f, 8.0f);
    }
}

/**
 * M4e 顺序：SIDE_MOVE 到位后不许再转身。合成一段「侧距单调增大」的平移
 * （真值倾斜 10°、沿面走 150 mm → h 增大 26 mm，正好越过旧的漂移门限 ds≥100 / dh≥12 / 2°≤e≤12°），
 * 断言航向不变、没有调用 probe_turn_to、扫描后纠偏（side_yaw_fix）保持 0。
 * 旧代码会在这段末尾 corr≈+9.8° 原地转一次 —— 那一步在 SIDE_SCAN 之后、LASER 之前，会把光斑打偏。
 */
static void probemovenoturn(void) {
    const float d2r = 3.14159265f / 180.0f;
    LfController c; LfInput in; uint32_t now;
    /* 面中心 +75、半宽 75：车从 x=0 走到 x=150 全程都在面内；倾斜 10° → 侧距单调增大 26 mm */
    SynthSide w = {75.0f, 75.0f, 180.0f, tanf(10.0f * d2r), 7.5f, 0.3f, 0.0f, 0.0f};
    side_start(&c, &in, &now, false, false, 2400U); /* 只为锁存 side_origin_* 并进入 PROBING */
    assert(c.probe_phase == LF_PROBE_SIDE_LOCK);
    c.cfg.probe.side_speed_mm_s = 100.0f; /* 50 ms 采样间距 5 mm：漂移 0.88 mm/帧，稳过 ±0.3 mm 噪声 */
    c.side_target = 150.0f;
    c.probe_phase = LF_PROBE_SIDE_MOVE;   /* 直接注入 SIDE_MOVE（与 probepar 注入 TURN_PAR 同法） */
    const float hdg0 = c.odo.heading_rad;
    const float mmpc = 3.14159265f * c.cfg.wheel_diameter_mm / c.cfg.encoder_counts_per_rev;
    float frac[2] = {0.0f, 0.0f};
    unsigned ping = 0U, guard = 0U;
    while (c.state == LF_PROBING && c.probe_phase == LF_PROBE_SIDE_MOVE && ++guard < 8000U) {
        if (++ping >= 12U) {
            ping = 0U;
            uint16_t d = synth_side_echo(&w, c.odo.x_mm);
            in.obstacle_mm = d;
            in.obstacle_valid = d != 0U;
        }
        for (unsigned i = 0; i < 2U; ++i) {
            float counts = c.target_mm_s[i] * 0.005f / mmpc + frac[i];
            int16_t whole = (int16_t)(counts >= 0.0f ? floorf(counts) : ceilf(counts));
            frac[i] = counts - (float)whole;
            in.encoder_delta[i] = whole;
        }
        now += 5U;
        lf_step(&c, &in, now, 0.005f);
    }
    assert(guard < 8000U);
    assert(c.side_move_stage == (uint8_t)LF_SIDE_MOVE_DONE); /* 开到 150 mm 并停稳 */
    assert(fabsf(c.odo.x_mm - 150.0f) < 4.0f);
    assert(c.side_yaw_fix == 0U);                           /* 扫描后纠偏未触发（面确实平行） */
    assert(c.turn_start_ms == 0U);                           /* 没有 probe_turn_to */
    assert(fabsf(c.odo.heading_rad - hdg0) < 0.5f * d2r);    /* 航向没变 */
}
/**
 * M3 TURN_PAR：直接注入 TURN_PAR 相位，用控制器自己的 target_mm_s 生成编码器脉冲。
 * 覆盖 φ 的符号/幅度（−20/0/+20）、side_left=0/1、|φ|>35 拒绝、堵转超时。
 */
static void probepar(void) {
    const float d2r = 3.14159265f / 180.0f;
    const struct { float phi_deg; bool left; bool block; float expect_deg; } cases[] = {
        {  0.0f, true,  false,  -90.0f },
        { 20.0f, true,  false,  -70.0f },
        {-20.0f, true,  false, -110.0f },
        {  0.0f, false, false,   90.0f },
        { 40.0f, true,  false,    0.0f },  /* |φ|>35：拒绝 TURN_PAR，失败停机 */
        {  0.0f, true,  true,     0.0f },  /* 堵转：3 s 超时失败 */
    };
    for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); ++k) {
        LfController c;
        lf_init(&c);
        c.cal.valid = true;
        c.cfg.arm_delay_ms = 0;
        c.cfg.probe.scan_mode = 1;
        c.cfg.probe.side_left = cases[k].left;
        c.state = LF_PROBING;
        c.probe_phase = LF_PROBE_TURN_PAR;
        c.state_since = 0;
        c.phase_since = 0;
        c.phase_start_rad = 0.0f;
        c.turn_stage = (uint8_t)LF_TURN_IDLE;
        c.turn_backup = 1U; /* 本用例只测转角，不测低 ρ 倒车 */
        c.face.valid = 1;
        c.face.phi = cases[k].phi_deg * d2r;
        LfInput in = input(3000, 500, 500, 3000);
        in.obstacle_valid = true;
        in.obstacle_mm = 180U;
        const float mmpc = 3.14159265f * c.cfg.wheel_diameter_mm / c.cfg.encoder_counts_per_rev;
        float frac[2] = {0.0f, 0.0f};
        uint32_t now = 0;
        unsigned guard = 0;
        while (c.state == LF_PROBING && c.probe_phase == LF_PROBE_TURN_PAR && ++guard < 2000U) {
            for (unsigned i = 0; i < 2U; ++i) {
                float counts = (cases[k].block ? 0.0f : c.target_mm_s[i]) * 0.005f / mmpc + frac[i];
                int16_t whole = (int16_t)(counts >= 0.0f ? floorf(counts) : ceilf(counts));
                frac[i] = counts - (float)whole;
                in.encoder_delta[i] = whole;
            }
            now += 5U;
            lf_step(&c, &in, now, 0.005f);
        }
        assert(guard < 2000U);
        if (cases[k].phi_deg > 35.0f || cases[k].block) {
            assert(c.turn_ok == 0U);
            assert(c.probe_phase != LF_PROBE_SIDE_LOCK);
            if (cases[k].block) assert(c.turn_ms >= 2900U);
        } else {
            assert(c.probe_phase == LF_PROBE_SIDE_LOCK);
            assert(c.turn_ok == 1U);
            assert(fabsf(c.turn_cmd_deg - cases[k].expect_deg) < 1.0f);
            assert(fabsf(c.turn_ach_deg - cases[k].expect_deg) < 8.0f);
        }
    }
}

/**
 * M3 LEG：先 side 扫到宽面中点，再 leg 绕到下一面，用独立 truth 位姿断言终点。
 * 用 1.00 / 1.02 两种轮径比例误差，确认「控制器自测 OK」之外真值也落在下一面中心附近。
 */
static void probeleg_run(bool feed_face) {
    for (unsigned variant = 0; variant < 2U; ++variant) {
        const float scale = variant == 0U ? 1.00f : 1.02f;
        LfController c;
        LfInput in;
        uint32_t now;
        SynthSide w = {0};
        w.center_mm = 0.0f;
        w.half_w_mm = 75.0f;
        w.h_mm = 180.0f;
        w.tilt = 0.0f;
        w.gamma_deg = 7.5f;
        w.sigma_mm = 0.0f;
        w.drop = 0.0f;
        w.bg_mm = 0.0f;
        side_start(&c, &in, &now, false, false, 2400U);
        side_run(&c, &in, &now, &w, false, 0.0f);
        assert(c.probe_phase == LF_PROBE_LASER);
        assert(c.face.kind == 1U && c.face.w > 130.0f && c.side_h > 150.0f);
        lf_stop(&c, now);
        tick(&c, &in, &now, 30U);
        assert(c.state == LF_STOPPED);
        assert(lf_start_leg_probe(&c, now));
        assert(c.state == LF_PROBING && c.probe_phase == LF_PROBE_LEG);
        /* truth：轮轴在面中心后方 pan_x、距面 h；宽面→下一面窄面，x=+75，中心 y=+25 */
        float tx = -c.cfg.probe.pan_x_mm;
        float ty = -c.side_h;
        float tyaw = 0.0f;
        const float mmpc = 3.14159265f * c.cfg.wheel_diameter_mm / c.cfg.encoder_counts_per_rev;
        float frac[2] = {0.0f, 0.0f};
        unsigned guard = 0;
        while (c.state == LF_PROBING && c.probe_phase == LF_PROBE_LEG && ++guard < 8000U) {
            float v[2] = {c.target_mm_s[0], c.target_mm_s[1]};
            float dL = v[0] * 0.005f * scale;
            float dR = v[1] * 0.005f * scale;
            float ds = 0.5f * (dL + dR);
            float dth = (dR - dL) / c.cfg.track_width_mm;
            tx += ds * cosf(tyaw + 0.5f * dth);
            ty += ds * sinf(tyaw + 0.5f * dth);
            tyaw += dth;
            for (unsigned i = 0; i < 2U; ++i) {
                float counts = v[i] * 0.005f / mmpc + frac[i];
                int16_t whole = (int16_t)(counts >= 0.0f ? floorf(counts) : ceilf(counts));
                frac[i] = counts - (float)whole;
                in.encoder_delta[i] = whole;
            }
            /* 侧距：feed_face=false → 无回波，DRIVE2 走 L2 兜底（旧路径原样不动）；
             * feed_face=true → 恒定给出 h_leg+margin，DRIVE2 第二帧就命中 → 事件停车。 */
            in.obstacle_valid = feed_face ? 1U : 0U;
            in.obstacle_mm = feed_face ? (uint16_t)(c.side_h + LF_LEG_MARGIN_MM) : 0U;
            now += 5U;
            lf_step(&c, &in, now, 0.005f);
        }
        assert(guard < 8000U);
        assert(c.probe_phase == LF_PROBE_SIDE_LOCK);
        assert(c.leg_fail == 0U);
        assert(c.leg_stage == (uint8_t)LF_LEG_DONE);
        assert((c.leg_event & (uint8_t)LF_LEG_EVENT_DRIVE2) != 0U);
        assert(fabsf(c.leg_turn_ach_deg - 90.0f) < 8.0f);
        assert(fabsf(tyaw - 1.5707963f) < 0.10f);
        if (feed_face) {
            /* 事件停车：转完刚进 DRIVE2 就认到面 → 车几乎没走就停，剩下的交给 SIDE_SCAN。
             * 21:36 现场的「转完又多开 162 mm」就是这里被死算 L2 拖走的。 */
            assert(c.leg_face_seen == 1U);
            assert(c.leg_stop_reason == 1U);
            /* 认到面后还要补 d_B/2 + 波束足印（窄面 ≈27 mm）才停，不再停在面近边上 */
            assert(c.leg_ach_mm > 15.0f && c.leg_ach_mm < 80.0f);
            assert(fabsf(c.leg_cmd_mm - c.leg_ach_mm) < 15.0f);
            assert(ty < -100.0f);   /* 明显还没到下一面中心 y=−5 */
        } else {
            /* 没认到面 → 退回旧路径：走到 L2 死算值，真值终点 x=75+h+margin=275、y=−5 */
            assert(c.leg_face_seen == 0U);
            assert(c.leg_stop_reason == 2U);
            assert(fabsf(c.leg_cmd_mm - c.leg_l2_mm) < 0.01f);
            /* L1 现在瞄准首面站距 h_ref（不再 +margin）：x = 75 + h_ref */
            assert(fabsf(tx - (75.0f + c.h_ref_mm)) < 15.0f);
            assert(fabsf(ty + 5.0f) < 15.0f);
        }
    }
}

/**
 * M4e：SIDE_LOCK 静止无回波时用 LEG 行进中量到的面均值兜底。
 * 现场 22:08：DRIVE2 行进中侧距 251~263（face_h=242），一停住回波被 66 cm 外的墙抢走并稳定 661，
 * 全部被 aim_max 400 拒收 → 5 帧凑不齐 → 干等 9 s 超时，第二面白丢。
 */
static void probesidefallback(void) {
    LfController c;
    LfInput in;
    uint32_t now = 0;
    lf_init(&c);
    in = input(3000, 500, 500, 3000);
    in.obstacle_valid = 0U;      /* 静止一帧回波都收不到 */
    in.obstacle_mm = 0U;
    c.state = LF_PROBING;
    c.probe_phase = LF_PROBE_SIDE_LOCK;
    c.side_lock_stage = 0U;
    c.phase_since = now;
    c.side_ping_since = now;
    c.probe_face = 1U;           /* 面 1~3：期望侧距 = h_leg + margin */
    c.leg_h_expected = 180.0f;
    c.leg_face2_n = 8U;
    c.leg_face2_h = 200U;
    for (unsigned i = 0; i < 700U && c.probe_phase == LF_PROBE_SIDE_LOCK; ++i) {
        now += 5U;
        lf_step(&c, &in, now, 0.005f);
    }
    assert(c.probe_phase == LF_PROBE_SIDE_SCAN);   /* 兜底后进扫描，不是超时停机 */
    assert(c.side_lock_warn == 10U);   /* 成功路径会被 probe_side_begin() 清掉 side_warn，看锁存值 */
    assert(fabsf(c.side_h - 200.0f) < 0.01f);
    assert(fabsf(c.side_pred_mm - 180.0f) < 0.01f);   /* pred = h_ref（不再 +margin） */
}

/** M3 LEG：没认到面 → 死算 L2 兜底（真值终点断言）。 */
static void probeleg(void) { probeleg_run(false); }
/** M4e LEG：DRIVE2 认到面 → 事件停车（转完立刻停，居中交给 SIDE_SCAN）。 */
static void probelegevent(void) { probeleg_run(true); }

/** M3：车体对正 SQUARE + 偏心挪车 ALIGN。 */
static void probeturn(void) { probe_pending("probeturn"); }
/** M4：切线绕行（edge1/edge2 由实测值计算）。 */
static void probetour(void) { probe_pending("probetour"); }

static void motorcal(void) {
    /* Simulate asymmetric motors across 15 steps (0.06..0.95 duty range):
     * Left motor : true deadband = 0.16, true free speed = 420 mm/s
     * Right motor: true deadband = 0.09, true free speed = 540 mm/s */
    LfController c;lf_init(&c);c.cal.valid=true;c.cfg.speed_loop=false;c.cfg.arm_delay_ms=0;
    for(unsigned i=0;i<4;++i){c.cal.black[i]=500;c.cal.white[i]=3000;}
    uint32_t now=0;
    assert(lf_calibrate_motors(&c,LF_MCAL_AIR,5,now));
    assert(c.state==LF_MOTOR_CAL && c.mcal_total_passes==5 && c.mcal_max_steps==15);
    const float mm_per_count=3.14159265f*c.cfg.wheel_diameter_mm/c.cfg.encoder_counts_per_rev;
    for(unsigned step_ticks=0;step_ticks<30000 && c.state==LF_MOTOR_CAL;++step_ticks){
        float uL=c.out.enable && !c.out.brake ? c.out.duty[0] : 0;
        float uR=c.out.enable && !c.out.brake ? c.out.duty[1] : 0;
        float vL=uL>.16f ? (uL-.16f)/(1.0f-.16f)*420.0f : 0;
        float vR=uR>.09f ? (uR-.09f)/(1.0f-.09f)*540.0f : 0;
        LfInput in=input(3000,500,500,3000);
        in.encoder_delta[0]=(int16_t)(vL*.005f/mm_per_count+.5f);
        in.encoder_delta[1]=(int16_t)(vR*.005f/mm_per_count+.5f);
        now+=5;lf_step(&c,&in,now,.005f);
    }
    assert(c.state==LF_STOPPED && c.fault==LF_OK && c.mcal_valid && c.mcal_done_event);
    assert(c.cfg.wheel_deadband[0]>c.cfg.wheel_deadband[1]+.03f);
    assert(c.cfg.wheel_free_speed[0]<c.cfg.wheel_free_speed[1]-40.0f);
    /* Verify piecewise feedforward interpolation: commanding 200 mm/s produces duties matching true motor model */
    float ffL=lf_wheel_feedforward(&c,0,200.0f);
    float ffR=lf_wheel_feedforward(&c,1,200.0f);
    float true_uL=.16f+200.0f/420.0f*(1.0f-.16f);
    float true_uR=.09f+200.0f/540.0f*(1.0f-.09f);
    assert(fabsf(ffL-true_uL)<.025f && fabsf(ffR-true_uR)<.025f);
    /* Verify that when driving straight on a centered line, Left wheel gets higher duty to compensate! */
    assert(lf_start(&c,now));
    LfInput straight=input(3000,500,500,3000);
    tick(&c,&straight,&now,30);
    assert(c.state==LF_TRACKING && c.out.duty[0]>c.out.duty[1]+.04f);

    /* Also verify 5-pass GROUND mode across ALL 15 steps (0.06..0.95) with step braking & auto return-to-origin */
    lf_stop(&c,now);tick(&c,&straight,&now,25);
    assert(lf_calibrate_motors(&c,LF_MCAL_GROUND,5,now));
    assert(c.mcal_max_steps==15);
    for(unsigned step_ticks=0;step_ticks<30000 && c.state==LF_MOTOR_CAL;++step_ticks){
        float uL=c.out.enable && !c.out.brake ? c.out.duty[0] : 0;
        float uR=c.out.enable && !c.out.brake ? c.out.duty[1] : 0;
        float vL=0,vR=0;
        if(uL>.18f)vL=(uL-.18f)/(1.0f-.18f)*390.0f;
        else if(uL<-.18f)vL=(uL+.18f)/(1.0f-.18f)*390.0f;
        if(uR>.11f)vR=(uR-.11f)/(1.0f-.11f)*500.0f;
        else if(uR<-.11f)vR=(uR+.11f)/(1.0f-.11f)*500.0f;
        LfInput in=input(3000,500,500,3000);
        in.encoder_delta[0]=(int16_t)(vL*.005f/mm_per_count+(vL>=0?.5f:-.5f));
        in.encoder_delta[1]=(int16_t)(vR*.005f/mm_per_count+(vR>=0?.5f:-.5f));
        now+=5;lf_step(&c,&in,now,.005f);
    }
    assert(c.state==LF_STOPPED && c.fault==LF_OK && c.mcal_valid);
    assert(c.mcal_ground_valid && c.mcal_active_mode==LF_MCAL_GROUND);
}
static void ground_priority(void) {
    /* 1. 先跑地面带载标定，后跑空载标定：确认地面带载始终为主模型，空载仅作参考 */
    LfController c; lf_init(&c); c.cal.valid=true;
    for(unsigned i=0;i<4;++i){c.cal.black[i]=500;c.cal.white[i]=3000;}
    const float mm_per_count=3.14159265f*c.cfg.wheel_diameter_mm/c.cfg.encoder_counts_per_rev;
    uint32_t now=0;
    assert(lf_calibrate_motors(&c,LF_MCAL_GROUND,1,now));
    for(unsigned s=0;s<12000 && c.state==LF_MOTOR_CAL;++s){
        float uL=c.out.enable && !c.out.brake ? c.out.duty[0] : 0;
        float uR=c.out.enable && !c.out.brake ? c.out.duty[1] : 0;
        float vL=uL>.20f ? (uL-.20f)/.80f*380.0f : (uL<-.20f ? (uL+.20f)/.80f*380.0f : 0);
        float vR=uR>.18f ? (uR-.18f)/.82f*410.0f : (uR<-.18f ? (uR+.18f)/.82f*410.0f : 0);
        LfInput in=input(3000,500,500,3000);
        in.encoder_delta[0]=(int16_t)(vL*.005f/mm_per_count+(vL>=0?.5f:-.5f));
        in.encoder_delta[1]=(int16_t)(vR*.005f/mm_per_count+(vR>=0?.5f:-.5f));
        now+=5; lf_step(&c,&in,now,.005f);
    }
    assert(c.mcal_ground_valid && c.mcal_active_mode==LF_MCAL_GROUND);
    float gnd_db_l=c.cfg.wheel_deadband[0];
    assert(lf_calibrate_motors(&c,LF_MCAL_AIR,1,now));
    for(unsigned s=0;s<6000 && c.state==LF_MOTOR_CAL;++s){
        float uL=c.out.enable && !c.out.brake ? c.out.duty[0] : 0;
        float uR=c.out.enable && !c.out.brake ? c.out.duty[1] : 0;
        float vL=uL>.10f ? (uL-.10f)/.90f*550.0f : 0;
        float vR=uR>.10f ? (uR-.10f)/.90f*550.0f : 0;
        LfInput in=input(3000,500,500,3000);
        in.encoder_delta[0]=(int16_t)(vL*.005f/mm_per_count+.5f);
        in.encoder_delta[1]=(int16_t)(vR*.005f/mm_per_count+.5f);
        now+=5; lf_step(&c,&in,now,.005f);
    }
    /* 空载完成不应覆盖地面带载主死区与主曲线 */
    assert(c.mcal_air_valid && c.mcal_ground_valid && c.mcal_active_mode==LF_MCAL_GROUND);
    assert(fabsf(c.cfg.wheel_deadband[0]-gnd_db_l)<.001f);
    assert(c.mcal_air_db[0]<gnd_db_l-.04f);
    assert(lf_select_mcal_mode(&c,LF_MCAL_AIR) && c.mcal_active_mode==LF_MCAL_AIR);
    assert(lf_select_mcal_mode(&c,LF_MCAL_GROUND) && c.mcal_active_mode==LF_MCAL_GROUND);

    /* 2. 倒车归位阻塞/超时防重入回归测试：即使倒车阶段轮速为0导致distance_mm始终>25mm，也不能死循环卡死 */
    assert(lf_calibrate_motors(&c,LF_MCAL_GROUND,2,now));
    for(unsigned s=0;s<9000 && c.state==LF_MOTOR_CAL;++s){
        float uL=c.out.enable && !c.out.brake ? c.out.duty[0] : 0;
        float uR=c.out.enable && !c.out.brake ? c.out.duty[1] : 0;
        float vL=uL>.20f ? (uL-.20f)/.80f*380.0f : 0;
        float vR=uR>.18f ? (uR-.18f)/.82f*410.0f : 0;
        LfInput in=input(3000,500,500,3000);
        in.encoder_delta[0]=(int16_t)(vL*.005f/mm_per_count+.5f);
        in.encoder_delta[1]=(int16_t)(vR*.005f/mm_per_count+.5f);
        now+=5; lf_step(&c,&in,now,.005f);
    }
    assert(c.state==LF_STOPPED && c.fault==LF_OK && c.mcal_ground_valid && c.mcal_pass==2U);
}
static void hwemu(void) {
    /* 验证 1:1 硬件模拟 PID 仿真模式：免 cal 启动、名义居中 D0≈48.5%、左偏线左减速右加速、可调权值与超声停车 */
    LfController c; lf_init(&c);
    c.cfg.arm_delay_ms=0; c.cfg.hw.enable=true;
    assert(!c.cal.valid && lf_start(&c,0));
    uint32_t now=0;
    LfInput center=input(3100,600,600,3100);
    tick(&c,&center,&now,20);
    assert(c.state==LF_TRACKING);
    assert(fabsf(c.hw_pid_e-2.500f)<.005f);
    assert(fabsf(c.out.duty[0]-0.4846f)<.01f && fabsf(c.out.duty[1]-0.4846f)<.01f);

    /* 黑线偏左 (L2/L1=600, R1/R2=3100 -> V_PID_E 降低 -> D_L 降 D_R 升左转纠偏) */
    LfInput left_line=input(600,600,3100,3100);
    tick(&c,&left_line,&now,30);
    assert(c.hw_pid_e<1.8f && c.out.duty[0]<c.out.duty[1]-.40f);

    /* 超声波 <15cm 在硬件仿真模式下仍保持安全主动锁停保护 */
    LfInput near_obs=center; near_obs.obstacle_valid=true; near_obs.obstacle_mm=120U;
    tick(&c,&near_obs,&now,5);
    assert(c.state==LF_OBSTACLE_STOP && c.out.brake && c.out.park_lamp);
}
static void hwemu_netlist(void) {
    /* 验证按实测 VREF / PWM_5V / PWM_MID / 三角波上下限 / 选组B U6A电阻网表的 1:1 全链路仿真 */
    LfController c; lf_init(&c);
    lf_hw_apply_preset(&c.cfg.hw, true); /* Group B: 47k/110k/100k/120k + 120k->5V + R_src=1k */
    c.cfg.hw.adc_vref_v = 2.897f;
    c.cfg.hw.v_5v = 5.000f;
    c.cfg.hw.v_mid = 2.500f;
    c.cfg.hw.tri_min_v = 0.420f;
    c.cfg.hw.tri_max_v = 2.820f;
    c.cfg.hw.tri_freq_hz = 21500.0f;
    c.cfg.hw.v_base = 2.500f;
    c.cfg.hw.kp_stage = 3.5f;
    c.cfg.hw.kd_stage_ms = 35.0f;
    c.cfg.arm_delay_ms = 0;
    c.cfg.hw.enable = true;

    /* 验证原实测停车静止值 [1106, 1352, 904, 2513] 在 Group B 网表下归零到 -0.017 V 左右 */
    LfInput park = input(1106, 1352, 904, 2513);
    assert(lf_start(&c, 0));
    uint32_t now = 0;
    tick(&c, &park, &now, 20);
    assert(c.state == LF_TRACKING);
    float e_park = c.hw_pid_e - c.cfg.hw.v_mid;
    assert(fabsf(e_park - (-0.0167f)) < 0.010f);

    /* 验证实测三角波公式 d = (0.625*V_mid - tri_min)/(tri_max - tri_min) */
    LfController cm; lf_init(&cm);
    cm.cfg.hw.v_5v = 5.040f; cm.cfg.hw.v_mid = 2.520f; cm.cfg.hw.v_base = 2.520f;
    cm.cfg.hw.tri_min_v = 0.420f; cm.cfg.hw.tri_max_v = 2.820f;
    cm.cfg.arm_delay_ms = 0; cm.cfg.hw.enable = true;
    LfInput balanced = input(2000, 1000, 1000, 2000);
    assert(lf_start(&cm, 0));
    uint32_t t2 = 0;
    tick(&cm, &balanced, &t2, 10);
    float expected_d0 = (0.625f * 2.520f - 0.420f) / (2.820f - 0.420f);
    assert(fabsf(cm.out.duty[0] - expected_d0) < 0.003f);
    assert(fabsf(cm.hw_ana_speed[0] - 0.625f * 2.520f) < 0.005f);

    /* 验证右偏移 (R2跌黑) 在 Group B 下产生正 e_ana 并输出 dL > dR */
    LfInput right_line = input(1106, 1352, 904, 1050);
    tick(&c, &right_line, &now, 15);
    assert(c.hw_pid_e > c.cfg.hw.v_mid + 0.45f);
    assert(c.out.duty[0] > c.out.duty[1] + 0.30f);

    /* 非法三角波上下限应被配置校验拒绝 */
    LfConfig bad = c.cfg;
    bad.hw.tri_min_v = 2.90f; bad.hw.tri_max_v = 2.80f;
    assert(!lf_config_valid(&bad));
}
int main(int argc,char **argv) {
    assert(argc==2);const char *name=argv[1];
    if(!strcmp(name,"calibration"))calibration();
    else if(!strcmp(name,"centroid"))centroid();
    else if(!strcmp(name,"lost"))lost();
    else if(!strcmp(name,"reacquire"))reacquire();
    else if(!strcmp(name,"timeout"))timeout();
    else if(!strcmp(name,"stop"))stopping();
    else if(!strcmp(name,"driver")||!strcmp(name,"stale")||!strcmp(name,"timing")||!strcmp(name,"config"))faults(name);
    else if(!strcmp(name,"direction"))encoder_fault(true);
    else if(!strcmp(name,"stall"))encoder_fault(false);
    else if(!strcmp(name,"pid"))pid();
    else if(!strcmp(name,"bridge"))bridge();
    else if(!strcmp(name,"rollover"))rollover();
    else if(!strcmp(name,"allblack"))allblack();
    else if(!strcmp(name,"weakline"))weakline();
    else if(!strcmp(name,"integral"))integral();
    else if(!strcmp(name,"extrapolation"))extrapolation();
    else if(!strcmp(name,"scurve"))scurve();
    else if(!strcmp(name,"obstacle"))obstacle();
    else if(!strcmp(name,"probe"))probe();
    else if(!strcmp(name,"probeaim"))probeaim();
    else if(!strcmp(name,"probefit"))probefit();
    else if(!strcmp(name,"probeside"))probeside();
    else if(!strcmp(name,"probesidetiltoff"))probesidetiltoff();
    else if(!strcmp(name,"probesideyawfix"))probesideyawfix();
    else if(!strcmp(name,"probemovenoturn"))probemovenoturn();
    else if(!strcmp(name,"probepar"))probepar();
    else if(!strcmp(name,"probeleg"))probeleg();
    else if(!strcmp(name,"probelegevent"))probelegevent();
    else if(!strcmp(name,"probesidefallback"))probesidefallback();
    else if(!strcmp(name,"probeturn"))probeturn();
    else if(!strcmp(name,"probetour"))probetour();
    else if(!strcmp(name,"motorcal"))motorcal();
    else if(!strcmp(name,"ground_priority"))ground_priority();
    else if(!strcmp(name,"hwemu"))hwemu();
    else if(!strcmp(name,"hwemu_netlist"))hwemu_netlist();
    else assert(!"unknown test");
    printf("PASS %s\n",name);return 0;
}
