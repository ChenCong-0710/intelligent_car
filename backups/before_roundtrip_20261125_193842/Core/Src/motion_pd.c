/* TEST application compatibility shim. New projects use dual_motor.h directly. */
#include "motion_pd.h"
volatile uint32_t g_motor_pulses[2];
static void mirror(const MotionPd *m){g_motor_pulses[0]=m->counts[0];g_motor_pulses[1]=m->counts[1];}
void motion_init(MotionPd *m,uint16_t l,uint16_t r){(void)dual_motor_init(m,0,l,r);mirror(m);}
void motion_start(MotionPd *m){dual_motor_start(m);mirror(m);}
void motion_stop(MotionPd *m){dual_motor_stop(m);mirror(m);}
void motion_trip(MotionPd *m,MotionError e){dual_motor_trip(m,e);mirror(m);}
void motion_tick(MotionPd *m,uint16_t l,uint16_t r,bool ok){dual_motor_tick_1ms(m,l,r,ok);mirror(m);}
float motion_pulse_duty(unsigned w,float t,float v,float d,float lag){DualMotorConfig c;(void)lag;dual_motor_default_config(&c);return dual_motor_pd_duty(&c,w,t,v,d);}
const char *motion_action_name(unsigned a){return dual_motor_action_name(a);}
const char *motion_phase_name(MotionPhase p){return dual_motor_phase_name(p);}
const char *motion_error_name(MotionError e){return dual_motor_error_name(e);}
