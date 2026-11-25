/* TEST two-segment application. Controller remains portable and hardware-free. */
#include "motion_pd.h"
#include <string.h>
volatile uint32_t g_motor_pulses[2];
int32_t g_motion_results[2][2];
static MotionSequenceStage sequence;
static uint32_t wait_since;
static void mirror(const MotionPd *m){g_motor_pulses[0]=m->counts[0];g_motor_pulses[1]=m->counts[1];}
bool motion_busy(const MotionPd *m){return m->running||sequence==MS_FORWARD||sequence==MS_WAIT||sequence==MS_REVERSE;}
MotionSequenceStage motion_sequence_stage(void){return sequence;}
const char *motion_sequence_name(void){static const char *n[]={"IDLE","FORWARD","WAIT_2S","REVERSE","DONE","CANCELLED","FAULT"};return n[sequence];}
uint32_t motion_wait_remaining(const MotionPd *m){uint32_t age=m->now-wait_since;return sequence==MS_WAIT&&age<2000U?2000U-age:0;}
void motion_init(MotionPd *m,uint16_t l,uint16_t r){
 DualMotorConfig c;dual_motor_default_config(&c);
 /* A40 profile: slower cruise, gradual count synchronization while moving.
    Keep the portable core defaults as the proven V4 profile. */
 c.cruise_target=40.0f;c.soft_stop_ms=300;
 c.sync_count_kp=.45f;c.sync_rate_kd=.65f;
 c.sync_max=8.0f;c.sync_slew=.5f;c.sync_filter_alpha=.25f;
 c.duty_slew=.04f;
 sequence=MS_IDLE;wait_since=0;memset(g_motion_results,0,sizeof(g_motion_results));
 if(!dual_motor_init(m,&c,l,r))sequence=MS_FAULT;
 mirror(m);
}
void motion_start(MotionPd *m){
 if(motion_busy(m)||m->phase==DM_FAULT||m->phase==DM_STOPPING)return;
 if(dual_motor_start_direction(m,1)){sequence=MS_FORWARD;memset(g_motion_results,0,sizeof(g_motion_results));}
 mirror(m);
}
void motion_stop(MotionPd *m){sequence=MS_CANCELLED;dual_motor_stop(m);mirror(m);}
void motion_trip(MotionPd *m,MotionError e){sequence=MS_FAULT;dual_motor_trip(m,e);mirror(m);}
void motion_tick(MotionPd *m,uint16_t l,uint16_t r,bool ok){
 dual_motor_tick_1ms(m,l,r,ok);
 if(m->phase==DM_FAULT)sequence=MS_FAULT;
 else if(sequence==MS_FORWARD&&m->phase==DM_STOPPED){
  g_motion_results[0][0]=m->actual[0];g_motion_results[0][1]=m->actual[1];
  wait_since=m->now;sequence=MS_WAIT;
 }else if(sequence==MS_WAIT&&m->now-wait_since>=2000U){
  /* Only now clear segment counts/start reverse. Never reset encoder counters. */
  if(dual_motor_start_direction(m,-1))sequence=MS_REVERSE;
 }else if(sequence==MS_REVERSE&&m->phase==DM_STOPPED){
  g_motion_results[1][0]=m->actual[0];g_motion_results[1][1]=m->actual[1];sequence=MS_DONE;
 }
 mirror(m);
}
float motion_pulse_duty(unsigned w,float t,float v,float d,float lag){DualMotorConfig c;(void)lag;dual_motor_default_config(&c);return dual_motor_pd_duty(&c,w,t,v,d);}
const char *motion_action_name(unsigned a){return dual_motor_action_name(a);}
const char *motion_phase_name(MotionPhase p){return dual_motor_phase_name(p);}
const char *motion_error_name(MotionError e){return dual_motor_error_name(e);}
