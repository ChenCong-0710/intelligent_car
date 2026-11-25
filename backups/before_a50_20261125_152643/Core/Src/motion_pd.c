#include "motion_pd.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
volatile int32_t g_distance_pulses;
#define PI 3.14159265358979323846f
static const int32_t goals[MOTION_ACTIONS][2]={
 {622,622},{-622,-622},{-215,215},{-430,430},{-645,645},{215,-215},{430,-430},{645,-645}};
static float clip(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
static int signi(int32_t x){return x<0?-1:1;}
static void phase(MotionPd *m,MotionPhase p){m->phase=p;m->since=m->now;m->stopped_since=0;++m->event;}
static void brake_wheel(MotionPd *m,unsigned w){
 if(!m->braked[w])m->at_brake[w]=m->actual[w];
 m->braked[w]=true;m->wheel[w].target_pulses=m->wheel[w].duty=0;
}
static void brake_all(MotionPd *m){brake_wheel(m,0);brake_wheel(m,1);}
void motion_trip(MotionPd *m,MotionError e){
 if(m->phase==MP_FAULT)return;
 brake_all(m);m->running=false;m->enable=false;m->error=e;phase(m,MP_FAULT);
}
void motion_init(MotionPd *m,uint16_t l,uint16_t r){
 memset(m,0,sizeof(*m));g_distance_pulses=0;m->phase=MP_IDLE;
 m->wheel[0].previous_counter=l;m->wheel[1].previous_counter=r;
 for(unsigned w=0;w<2;++w)for(unsigned d=0;d<2;++d)m->lead[w][d]=8;
 m->braked[0]=m->braked[1]=true;
}
static void begin_action(MotionPd *m){
 m->trim_round=0;m->position_warning=false;m->action_since=m->drive_since=m->now;m->learn_pending=false;m->sample_ms=0;m->distance_pulses=m->turn_pulses=0;g_distance_pulses=0;
 for(unsigned w=0;w<2;++w){
  m->goal[w]=goals[m->action][w];m->origin[w]=m->wheel[w].total;m->actual[w]=0;
  m->coast_counts[w]=0;m->braked[w]=false;m->wheel[w].target_pulses=m->wheel[w].duty=0;
  m->wheel[w].sample_sum=0;m->wheel[w].speed_pulses=0;m->wheel[w].target_pulses=0;m->wheel[w].previous_error=0;
  m->wheel[w].derivative=0;
  m->wheel[w].wrong_since=m->wheel[w].stall_since=m->wheel[w].overspeed_since=m->now;
  m->pulse_sum[w]=m->peak_error[w]=0;m->cruise_n[w]=0;
  m->disturbance[w]=m->assessed[w]=m->recovery_valid[w]=false;
  m->disturbed_since[w]=m->good_since[w]=m->max_recovery[w]=0;
 }
 m->enable=true;phase(m,MP_RUN);
}
void motion_start(MotionPd *m){
 if(m->running||m->phase==MP_STOPPING||m->phase==MP_FAULT)return;
 m->running=true;m->error=MP_OK;m->action=0;m->cycle=1;m->position_warnings=0;
 memset(m->result,0,sizeof(m->result));begin_action(m);
}
void motion_stop(MotionPd *m){
 brake_all(m);m->running=false;m->enable=true;phase(m,MP_STOPPING);
}
static void feedback(MotionPd *m,unsigned w,uint16_t counter){
 PdWheel *p=&m->wheel[w];
 uint16_t wrapped=(uint16_t)(counter-p->previous_counter);
 int32_t d=wrapped>32767U?(int32_t)wrapped-65536:(int32_t)wrapped;
 p->previous_counter=counter;
 if(abs(d)>50){motion_trip(m,MP_ENCODER);return;}
 p->total+=d;p->sample_sum+=d;
 int64_t relative=p->total-m->origin[w];
 if(relative>INT32_MAX||relative<INT32_MIN){motion_trip(m,MP_ENCODER);return;}
 m->actual[w]=(int32_t)relative;
}
static void metrics(MotionPd *m,unsigned w){
 PdWheel *p=&m->wheel[w];
 if(m->phase!=MP_RUN||m->braked[w]||m->now-m->drive_since<150U)return;
 float speed=signi(m->goal[w])*p->speed_pulses;
 float error=fabsf(MOTION_TARGET_PULSES-speed);
 ++m->cruise_n[w];m->pulse_sum[w]+=speed;m->peak_error[w]=fmaxf(m->peak_error[w],error);
}
/* Absolute PWM position-form PD; inputs in signed counts / 20ms.
   FF estimates from recorded fixed-PWM run; ground tuning still required. */
float motion_pulse_duty(unsigned wheel,float target,float measured,float error_delta,float lag){
 if(fabsf(target)<0.01f)return 0;
 float sign=target<0?-1.0f:1.0f;
 unsigned dir=target<0?1:0;
 static const float intercept[2][2]={{.045f,.050f},{.0615f,.067f}};
 static const float slope[2][2]={{.0075f,.0074f},{.00711f,.00687f}};
 if(wheel>1)return 0;
 float ff=sign*(intercept[wheel][dir]+slope[wheel][dir]*fabsf(target));
 float u=ff+MOTION_KP*(target-measured)+MOTION_KD*error_delta+
         sign*clip(MOTION_SYNC_KP*lag,-.08f,.08f);
 return sign*clip(sign*u,0,MOTION_MAX_DUTY);
}
static void pd_update(MotionPd *m){
 const float common=MOTION_TARGET_PULSES;
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];
  if(m->phase!=MP_RUN||m->braked[w]){p->target_pulses=p->duty=0;p->previous_error=p->derivative=0;continue;}
  int sign=signi(m->goal[w]);p->target_pulses=sign*common;
  float error=p->target_pulses-p->speed_pulses;
  p->derivative+=.5f*((error-p->previous_error)-p->derivative);p->previous_error=error;
  float mine=sign*m->actual[w];float other=signi(m->goal[1-w])*m->actual[1-w];
  p->duty=motion_pulse_duty(w,p->target_pulses,p->speed_pulses,p->derivative,other-mine);
  metrics(m,w);
 }
}
static bool position_ok(const MotionPd *m){
 if(abs(m->goal[0]-m->actual[0])>8||abs(m->goal[1]-m->actual[1])>8)return false;
 return m->action<2 || abs(abs(m->actual[0])-abs(m->actual[1]))<=6;
}
static void save_result(MotionPd *m){
 MotionResult *r=&m->result[m->action];memset(r,0,sizeof(*r));
 for(unsigned w=0;w<2;++w){
  r->actual[w]=m->actual[w];r->goal[w]=m->goal[w];
  r->coast[w]=m->coast_counts[w];
  r->cruise_samples[w]=m->cruise_n[w];
  r->mean_pulses[w]=m->cruise_n[w]?m->pulse_sum[w]/m->cruise_n[w]:0;
  r->peak_error[w]=m->peak_error[w];r->assessed[w]=m->assessed[w];
  r->recovery_valid[w]=m->recovery_valid[w];r->recovery_ms[w]=m->max_recovery[w];
 }
 r->duration_ms=m->now-m->action_since;r->trims=m->trim_round;r->passed=position_ok(m);
 r->distance_mm=(m->actual[0]+m->actual[1])*0.5f*PI*MOTION_WHEEL_MM/MOTION_CPR;
 r->angle_deg=(m->actual[1]-m->actual[0])*180.0f*MOTION_WHEEL_MM/(MOTION_TRACK_MM*MOTION_CPR);
}
static void learn_brake(MotionPd *m){
 if(!m->learn_pending)return;
 m->learn_pending=false;
 for(unsigned w=0;w<2;++w){
  int sign=signi(m->goal[w]);unsigned dir=sign>0?0U:1U;
  int extra=sign*(m->actual[w]-m->at_brake[w]);
  m->coast_counts[w]=extra;
  /* Learn actual coast pulses per wheel/direction, with bounded smoothing. */
  float learned=0.5f*m->lead[w][dir]+0.5f*(float)(extra>0?extra:0);
  m->lead[w][dir]=(uint16_t)lroundf(clip(learned,1,60));
 }
}
void motion_tick(MotionPd *m,uint16_t left,uint16_t right,bool driver_ok){
 ++m->now;
 feedback(m,0,left);feedback(m,1,right);
 if(!driver_ok)motion_trip(m,MP_DRIVER);
 if(m->phase==MP_FAULT){m->enable=false;return;}
 bool speed_ready=++m->sample_ms>=MOTION_SPEED_MS;
 if(speed_ready){
  m->sample_ms=0;
  for(unsigned w=0;w<2;++w){PdWheel *p=&m->wheel[w];p->speed_pulses=(float)p->sample_sum;p->sample_sum=0;}
 }
 if(m->running && m->phase!=MP_PAUSE && m->now-m->action_since>=MOTION_ACTION_MS){save_result(m);motion_trip(m,MP_TIMEOUT);return;}
 if(m->phase==MP_RUN){
  int32_t progress=(int32_t)(((int64_t)signi(m->goal[0])*m->actual[0]+(int64_t)signi(m->goal[1])*m->actual[1])/2);
  if(m->action<2){m->distance_pulses=progress;g_distance_pulses=progress;}
  else m->turn_pulses=progress;
  if(progress>=abs(m->goal[0])){
   brake_all(m);m->distance_pulses=m->turn_pulses=0;g_distance_pulses=0;phase(m,MP_BRAKE);
  }
 }else if(m->phase==MP_BRAKE){
  bool stopped=fabsf(m->wheel[0].speed_pulses)<1&&fabsf(m->wheel[1].speed_pulses)<1;
  if(stopped){if(!m->stopped_since)m->stopped_since=m->now;}else m->stopped_since=0;
  if(m->now-m->since>=100U && m->stopped_since && m->now-m->stopped_since>=30U){
   learn_brake(m);
   save_result(m);
   if(m->action<2){m->last_straight[0]=m->actual[0];m->last_straight[1]=m->actual[1];}
   /* Endpoint errors are diagnostics, not a reason to reverse or halt testing. */
   m->position_warning=!m->result[m->action].passed;
   if(m->position_warning)++m->position_warnings;
   phase(m,MP_PAUSE);
  }else if(m->now-m->since>=600U){save_result(m);motion_trip(m,MP_NOT_SETTLED);}
 }else if(m->phase==MP_PAUSE){
  if(m->now-m->since>=MOTION_PAUSE_MS){
   if(++m->action==MOTION_ACTIONS){m->action=0;++m->cycle;}
   begin_action(m);
  }
 }else if(m->phase==MP_STOPPING){
  if(m->now-m->since>=100U){m->enable=false;phase(m,MP_STOPPED);}
 }
 if(speed_ready)pd_update(m);
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];bool active=m->phase==MP_RUN&&!m->braked[w];
  bool wrong=active&&p->target_pulses*p->speed_pulses<0&&fabsf(p->speed_pulses)>=2;
  bool stalled=active&&m->now-m->drive_since>=MOTION_STALL_GRACE_MS&&fabsf(p->speed_pulses)<1;
  bool too_fast=active&&fabsf(p->speed_pulses)>45;
  if(!wrong)p->wrong_since=m->now;
  if(!stalled)p->stall_since=m->now;
  if(!too_fast)p->overspeed_since=m->now;
  if(wrong&&m->now-p->wrong_since>=100U){motion_trip(m,MP_DIRECTION);return;}
  if(stalled&&m->now-p->stall_since>=MOTION_STALL_MS){motion_trip(m,MP_STALL);return;}
  if(too_fast&&m->now-p->overspeed_since>=100U){motion_trip(m,MP_OVERSPEED);return;}
 }
}
const char *motion_action_name(unsigned a){static const char *const n[]={"FWD_10CM","REV_10CM","LEFT_30","LEFT_60","LEFT_90","RIGHT_30","RIGHT_60","RIGHT_90"};return a<MOTION_ACTIONS?n[a]:"UNKNOWN";}
const char *motion_phase_name(MotionPhase p){static const char *const n[]={"IDLE","RUN","BRAKE","TRIM","PAUSE_2S","STOPPING","STOPPED","FAULT"};return (unsigned)p<sizeof(n)/sizeof(n[0])?n[p]:"UNKNOWN";}
const char *motion_error_name(MotionError e){static const char *const n[]={"OK","DRIVER","ENCODER","DIRECTION","STALL","ACTION_TIMEOUT","POSITION","NOT_SETTLED","OVERSPEED","WATCHDOG"};return (unsigned)e<sizeof(n)/sizeof(n[0])?n[e]:"UNKNOWN";}
