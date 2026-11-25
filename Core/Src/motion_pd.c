/* LEFT-wheel absolute-angle sequence: 30 -> 120 -> 60 -> 0 degrees.
 * The portable PD core handles each individual movement, never the 5s waits. */
#include "motion_pd.h"
#include <math.h>
#include <string.h>
volatile uint32_t g_motor_pulses[2];
volatile int32_t g_pivot_net_counts[2];
int32_t g_motion_results[2][2];
int32_t g_pivot_stage_results[MOTION_PIVOT_STEPS];
static MotionSequenceStage sequence;
static unsigned step_index;
static uint32_t wait_since;
static int64_t plan_origin[2];
static int32_t segment_goal;
static const unsigned targets[MOTION_PIVOT_STEPS]={30,120,60,0};
int32_t motion_pivot_counts(float angle){
 if(!isfinite(angle)||angle<=0||angle>360)return 0;
 /* Independent left calibration anchors: do not scale the confirmed90 point.
    The right-wheel geometric/calibrated mapping remains unchanged. */
 if(MOTION_PIVOT_ACTIVE_WHEEL==0U&&angle<=120.0f){
  static const float angles[]={0,30,60,90,120};
  static const int32_t counts[]={0,MOTION_PIVOT_LEFT_COUNT_30,MOTION_PIVOT_LEFT_COUNT_60,MOTION_PIVOT_LEFT_COUNT_90,MOTION_PIVOT_LEFT_COUNT_120};
  for(unsigned i=1;i<5;++i)if(angle<=angles[i]){
   float t=(angle-angles[i-1])/(angles[i]-angles[i-1]);
   return (int32_t)lroundf(counts[i-1]+t*(counts[i]-counts[i-1]));
  }
 }
 return (int32_t)lroundf(angle*MOTION_PIVOT_TRACK_MM*MOTION_PIVOT_A_EDGES_PER_TURN*MOTION_PIVOT_CAL_SCALE/(180.0f*MOTION_PIVOT_WHEEL_MM));
}
static void mirror(const MotionPd *m){
 for(unsigned w=0;w<2;++w){
  g_motor_pulses[w]=m->counts[w];int64_t net=m->wheel[w].total-plan_origin[w];
  g_pivot_net_counts[w]=net>INT32_MAX?INT32_MAX:net<INT32_MIN?INT32_MIN:(int32_t)net;
 }
}
bool motion_busy(const MotionPd *m){return m->running||sequence==MS_FORWARD||sequence==MS_REVERSE||sequence==MS_WAIT;}
MotionSequenceStage motion_sequence_stage(void){return sequence;}
const char *motion_sequence_name(void){static const char *n[]={"IDLE","PIVOT_FWD","WAIT_5S","PIVOT_REV","DONE","CANCELLED","FAULT"};return n[sequence];}
unsigned motion_pivot_step(void){return step_index+1;}
unsigned motion_pivot_target_degrees(void){return targets[step_index];}
int32_t motion_pivot_absolute_goal(void){return motion_pivot_counts((float)targets[step_index]);}
int32_t motion_pivot_segment_goal(void){return segment_goal;}
uint32_t motion_wait_remaining(const MotionPd *m){uint32_t elapsed=m->now-wait_since;return sequence==MS_WAIT&&elapsed<MOTION_PIVOT_WAIT_MS?MOTION_PIVOT_WAIT_MS-elapsed:0;}
static void stage_finished(MotionPd *m){
 int64_t net=m->wheel[MOTION_PIVOT_ACTIVE_WHEEL].total-plan_origin[MOTION_PIVOT_ACTIVE_WHEEL];g_pivot_stage_results[step_index]=(int32_t)net;
 g_motion_results[m->move_direction>0?0:1][0]=m->actual[0];
 g_motion_results[m->move_direction>0?0:1][1]=m->actual[1];
 if(step_index+1<MOTION_PIVOT_STEPS){sequence=MS_WAIT;wait_since=m->now;}
 else sequence=MS_DONE;
}
static void begin_stage(MotionPd *m){
 int64_t net=m->wheel[MOTION_PIVOT_ACTIVE_WHEEL].total-plan_origin[MOTION_PIVOT_ACTIVE_WHEEL];
 int64_t difference=(int64_t)motion_pivot_absolute_goal()-net;
 if(difference>INT32_MAX||difference<-(int64_t)INT32_MAX){motion_trip(m,DM_ENCODER);return;}
 segment_goal=(int32_t)(difference<0?-difference:difference);
 if(!segment_goal){stage_finished(m);return;}
 int sign=difference>0?1:-1;
 if(dual_motor_start_wheels(m,sign,MOTION_PIVOT_ACTIVE_WHEEL==0U?segment_goal:0,MOTION_PIVOT_ACTIVE_WHEEL==1U?segment_goal:0))sequence=sign>0?MS_FORWARD:MS_REVERSE;
 else motion_trip(m,DM_CONFIG);
}
void motion_init(MotionPd *m,uint16_t l,uint16_t r){
 DualMotorConfig c;dual_motor_default_config(&c);
 c.cruise_target=34;c.goal_counts=motion_pivot_counts(90);
 c.soft_stop_ms=0;c.sync_enabled=false;c.duty_slew=.04f;
 sequence=MS_IDLE;step_index=0;segment_goal=0;wait_since=0;plan_origin[0]=plan_origin[1]=0;
 memset(g_motion_results,0,sizeof(g_motion_results));memset(g_pivot_stage_results,0,sizeof(g_pivot_stage_results));
 if(!dual_motor_init(m,&c,l,r))sequence=MS_FAULT;
 mirror(m);
}
void motion_start(MotionPd *m){
 if(motion_busy(m)||m->phase==DM_FAULT||m->phase==DM_STOPPING)return;
 step_index=0;wait_since=0;plan_origin[0]=m->wheel[0].total;plan_origin[1]=m->wheel[1].total;
 memset(g_motion_results,0,sizeof(g_motion_results));memset(g_pivot_stage_results,0,sizeof(g_pivot_stage_results));
 begin_stage(m);mirror(m);
}
void motion_stop(MotionPd *m){sequence=MS_CANCELLED;dual_motor_stop(m);mirror(m);}
void motion_trip(MotionPd *m,MotionError e){sequence=MS_FAULT;dual_motor_trip(m,e);mirror(m);}
void motion_tick(MotionPd *m,uint16_t l,uint16_t r,bool ok){
 dual_motor_tick_1ms(m,l,r,ok);
 if(m->phase==DM_FAULT)sequence=MS_FAULT;
 else if((sequence==MS_FORWARD||sequence==MS_REVERSE)&&m->phase==DM_STOPPED)stage_finished(m);
 else if(sequence==MS_WAIT&&m->now-wait_since>=MOTION_PIVOT_WAIT_MS){++step_index;begin_stage(m);}
 mirror(m);
}
float motion_pulse_duty(unsigned w,float t,float v,float d,float lag){DualMotorConfig c;(void)lag;dual_motor_default_config(&c);return dual_motor_pd_duty(&c,w,t,v,d);}
const char *motion_action_name(unsigned a){(void)a;static const char *n[]={"LEFT_TO_30","LEFT_TO_120","LEFT_TO_60","LEFT_TO_0"};return n[step_index];}
const char *motion_phase_name(MotionPhase p){return dual_motor_phase_name(p);}
const char *motion_error_name(MotionError e){return dual_motor_error_name(e);}
