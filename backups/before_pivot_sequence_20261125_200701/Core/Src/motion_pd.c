/* TEST single-right-wheel body pivot, using the existing positional PD core. */
#include "motion_pd.h"
#include <math.h>
#include <string.h>
volatile uint32_t g_motor_pulses[2];
int32_t g_motion_results[2][2];
static MotionSequenceStage sequence;
static void mirror(const MotionPd *m){g_motor_pulses[0]=m->counts[0];g_motor_pulses[1]=m->counts[1];}
int32_t motion_pivot_counts(float angle){
 if(!isfinite(angle)||angle<=0||angle>360)return 0;
 return (int32_t)lroundf(angle*MOTION_PIVOT_TRACK_MM*MOTION_PIVOT_A_EDGES_PER_TURN*MOTION_PIVOT_CAL_SCALE/(180.0f*MOTION_PIVOT_WHEEL_MM));
}
bool motion_busy(const MotionPd *m){return m->running||sequence==MS_FORWARD;}
MotionSequenceStage motion_sequence_stage(void){return sequence;}
const char *motion_sequence_name(void){static const char *n[]={"IDLE","PIVOT_R90","WAIT_UNUSED","REV_UNUSED","DONE","CANCELLED","FAULT"};return n[sequence];}
uint32_t motion_wait_remaining(const MotionPd *m){(void)m;return 0;}
void motion_init(MotionPd *m,uint16_t l,uint16_t r){
 DualMotorConfig c;dual_motor_default_config(&c);
 c.cruise_target=34.0f;c.goal_counts=motion_pivot_counts(MOTION_PIVOT_ANGLE_DEG);
 c.soft_stop_ms=0;c.sync_enabled=false;c.duty_slew=.04f;
 sequence=MS_IDLE;memset(g_motion_results,0,sizeof(g_motion_results));
 if(!dual_motor_init(m,&c,l,r))sequence=MS_FAULT;
 mirror(m);
}
void motion_start(MotionPd *m){
 if(motion_busy(m)||m->phase==DM_FAULT||m->phase==DM_STOPPING)return;
 if(dual_motor_start_wheels(m,1,0,m->config.goal_counts)){sequence=MS_FORWARD;memset(g_motion_results,0,sizeof(g_motion_results));}
 mirror(m);
}
void motion_stop(MotionPd *m){sequence=MS_CANCELLED;dual_motor_stop(m);mirror(m);}
void motion_trip(MotionPd *m,MotionError e){sequence=MS_FAULT;dual_motor_trip(m,e);mirror(m);}
void motion_tick(MotionPd *m,uint16_t l,uint16_t r,bool ok){
 dual_motor_tick_1ms(m,l,r,ok);
 if(m->phase==DM_FAULT)sequence=MS_FAULT;
 else if(sequence==MS_FORWARD&&m->phase==DM_STOPPED){
  g_motion_results[0][0]=m->actual[0];g_motion_results[0][1]=m->actual[1];sequence=MS_DONE;
 }
 mirror(m);
}
float motion_pulse_duty(unsigned w,float t,float v,float d,float lag){DualMotorConfig c;(void)lag;dual_motor_default_config(&c);return dual_motor_pd_duty(&c,w,t,v,d);}
const char *motion_action_name(unsigned a){(void)a;return "PIVOT_RIGHT_FWD";}
const char *motion_phase_name(MotionPhase p){return dual_motor_phase_name(p);}
const char *motion_error_name(MotionError e){return dual_motor_error_name(e);}
