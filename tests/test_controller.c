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
    LfInput in=input(3000,500,500,3000);
    in.obstacle_valid=true;in.obstacle_mm=130;
    tick(&c,&in,&now,2);assert(c.state==LF_OBSTACLE_STOP);
    assert(lf_start_probe(&c,now));
    assert(c.state==LF_PROBING && c.probe_phase==LF_PROBE_AIM);
    for(unsigned face=0;face<4;++face){
        /* 2000 ms stepped aim sweep (400 ticks @ 5 ms) */
        in.obstacle_mm=125;tick(&c,&in,&now,420);
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
    LfInput in=input(3000,500,500,3000);
    in.obstacle_valid=true;in.obstacle_mm=130;
    tick(&c,&in,&now,2);assert(c.state==LF_OBSTACLE_STOP);
    assert(lf_start_probe(&c,now));
    const float normal_deg=20.0f, plane_mm=150.0f;
    unsigned guard=0, ping_ticks=0;
    while(c.probe_phase==LF_PROBE_AIM && ++guard<2000){
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
    else if(!strcmp(name,"motorcal"))motorcal();
    else if(!strcmp(name,"ground_priority"))ground_priority();
    else if(!strcmp(name,"hwemu"))hwemu();
    else if(!strcmp(name,"hwemu_netlist"))hwemu_netlist();
    else assert(!"unknown test");
    printf("PASS %s\n",name);return 0;
}
