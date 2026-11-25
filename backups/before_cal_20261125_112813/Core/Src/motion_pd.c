#include "motion_pd.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define PI 3.14159265358979323846f
static const int32_t goals[MOTION_ACTIONS][2]={
 {622,622},{-622,-622},{-215,215},{-430,430},{-645,645},{215,-215},{430,-430},{645,-645}};
static float clip(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
static int signi(int32_t x){return x<0?-1:1;}
static void phase(MotionPd *m,MotionPhase p){m->phase=p;m->since=m->now;m->stopped_since=0;++m->event;}
static void brake_wheel(MotionPd *m,unsigned w){
 if(!m->braked[w])m->at_brake[w]=m->actual[w];
 m->braked[w]=true;m->wheel[w].target=m->wheel[w].duty=0;
}
static void brake_all(MotionPd *m){brake_wheel(m,0);brake_wheel(m,1);}
void motion_trip(MotionPd *m,MotionError e){
 if(m->phase==MP_FAULT)return;
 brake_all(m);m->running=false;m->enable=false;m->error=e;phase(m,MP_FAULT);
}
void motion_init(MotionPd *m,uint16_t l,uint16_t r){
 memset(m,0,sizeof(*m));m->phase=MP_IDLE;
 m->wheel[0].previous_counter=l;m->wheel[1].previous_counter=r;
 for(unsigned w=0;w<2;++w)for(unsigned d=0;d<2;++d)m->lead[w][d]=8;
 m->braked[0]=m->braked[1]=true;
}
static void begin_action(MotionPd *m){
 m->trim_round=0;m->position_warning=false;m->action_since=m->drive_since=m->now;m->learn_pending=true;
 for(unsigned w=0;w<2;++w){
  m->goal[w]=goals[m->action][w];m->origin[w]=m->wheel[w].total;m->actual[w]=0;
  m->coast_counts[w]=0;m->braked[w]=false;m->wheel[w].target=m->wheel[w].duty=0;
  m->wheel[w].derivative=0;m->wheel[w].previous_rpm=m->wheel[w].rpm;
  m->wheel[w].wrong_since=m->wheel[w].stall_since=m->wheel[w].overspeed_since=m->now;
  m->rpm_sum[w]=m->peak_error[w]=0;m->cruise_n[w]=0;
  m->disturbance[w]=m->assessed[w]=m->recovery_valid[w]=false;
  m->disturbed_since[w]=m->good_since[w]=m->max_recovery[w]=0;
 }
 m->enable=true;phase(m,MP_RUN);
}
void motion_start(MotionPd *m){
 if(m->running||m->phase==MP_STOPPING)return;
 m->running=true;m->error=MP_OK;m->action=0;m->cycle=1;m->position_warnings=0;
 memset(m->result,0,sizeof(m->result));begin_action(m);
}
void motion_stop(MotionPd *m){
 brake_all(m);m->running=false;m->enable=true;phase(m,MP_STOPPING);
}
float motion_pd_duty(float target,float measured,float derivative,float lag){
 if(fabsf(target)<0.5f)return 0;
 float sign=target<0?-1.0f:1.0f;
 float ff=sign*(0.14f+0.0040f*fabsf(target));
 /* Positional PD: absolute PWM each evaluation. No integrator or delta-PWM. */
 float u=ff+MOTION_KP*(target-measured)-MOTION_KD*derivative+
         sign*clip(MOTION_SYNC_KP*lag,-0.12f,0.12f);
 return sign*clip(sign*u,0,MOTION_MAX_DUTY);
}
static void feedback(MotionPd *m,unsigned w,uint16_t counter){
 PdWheel *p=&m->wheel[w];
 uint16_t wrapped=(uint16_t)(counter-p->previous_counter);
 int32_t d=wrapped>32767U?(int32_t)wrapped-65536:(int32_t)wrapped;
 p->previous_counter=counter;
 if(abs(d)>50){motion_trip(m,MP_ENCODER);return;}
 p->total+=d;
 p->window_sum+=d-p->window[m->index];p->window[m->index]=(int16_t)d;
 p->short_sum+=d-p->short_window[m->short_index];p->short_window[m->short_index]=(int16_t)d;
 p->rpm=(float)p->window_sum*60000.0f/(MOTION_CPR*m->filled);
 p->short_rpm=(float)p->short_sum*60000.0f/(MOTION_CPR*m->short_filled);
 int64_t relative=p->total-m->origin[w];
 if(relative>INT32_MAX||relative<INT32_MIN){motion_trip(m,MP_ENCODER);return;}
 m->actual[w]=(int32_t)relative;
}
static void metrics(MotionPd *m,unsigned w){
 PdWheel *p=&m->wheel[w];
 if(m->phase!=MP_RUN||m->braked[w]||m->now-m->drive_since<150U||fabsf(p->target)<64.9f)return;
 float speed=signi(m->goal[w])*p->rpm;
 float error=fabsf(MOTION_RPM-speed);
 ++m->cruise_n[w];m->rpm_sum[w]+=speed;m->peak_error[w]=fmaxf(m->peak_error[w],error);
 if(error>10 && !m->disturbance[w]){
  m->disturbance[w]=true;m->disturbed_since[w]=m->now;m->good_since[w]=0;
 }
 if(error<=5){
  if(!m->good_since[w])m->good_since[w]=m->now;
  if(m->now-m->good_since[w]>=100U){
   m->assessed[w]=true;
   if(m->disturbance[w]){
    uint32_t duration=m->now-m->disturbed_since[w];
    if(duration>m->max_recovery[w])m->max_recovery[w]=duration;
    m->recovery_valid[w]=true;m->disturbance[w]=false;
   }
  }
 }else m->good_since[w]=0;
}
static void pd_update(MotionPd *m){
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];
  float raw=(p->rpm-p->previous_rpm)/0.005f;
  p->previous_rpm=p->rpm;p->derivative+=0.2f*(raw-p->derivative);
  if(m->phase!=MP_RUN||m->braked[w]){p->target=p->duty=0;continue;}
  float ramp=clip((float)(m->now-m->drive_since)/100.0f,0,1);
  int sign=signi(m->goal[w]);
  p->target=sign*MOTION_RPM*ramp;
  float lag=0;
  if(m->phase==MP_RUN&&!m->braked[1-w]){
   int32_t mine=signi(m->goal[w])*m->actual[w];
   int32_t other=signi(m->goal[1-w])*m->actual[1-w];lag=(float)(other-mine);
  }
  p->duty=motion_pd_duty(p->target,p->rpm,p->derivative,lag);
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
  r->mean_rpm[w]=m->cruise_n[w]?m->rpm_sum[w]/m->cruise_n[w]:0;
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
 if(m->filled<20)++m->filled;
 if(m->short_filled<5)++m->short_filled;
 feedback(m,0,left);feedback(m,1,right);
 m->index=(uint8_t)((m->index+1)%20);m->short_index=(uint8_t)((m->short_index+1)%5);
 if(!driver_ok)motion_trip(m,MP_DRIVER);
 if(m->phase==MP_FAULT){m->enable=false;return;}
 if(m->running && m->phase!=MP_PAUSE && m->now-m->action_since>=MOTION_ACTION_MS){save_result(m);motion_trip(m,MP_TIMEOUT);return;}
 if(m->phase==MP_RUN){
  for(unsigned w=0;w<2;++w){
   int sign=signi(m->goal[w]);unsigned dir=sign>0?0U:1U;
   if(!m->braked[w] && sign*m->actual[w]>=abs(m->goal[w])-(int)m->lead[w][dir])brake_wheel(m,w);
  }
  if(m->braked[0]&&m->braked[1])phase(m,MP_BRAKE);
 }else if(m->phase==MP_BRAKE){
  bool stopped=fabsf(m->wheel[0].rpm)<=3&&fabsf(m->wheel[1].rpm)<=3;
  if(stopped){if(!m->stopped_since)m->stopped_since=m->now;}else m->stopped_since=0;
  if(m->now-m->since>=100U && m->stopped_since && m->now-m->stopped_since>=30U){
   learn_brake(m);
   save_result(m);
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
 if(m->now%5==0)pd_update(m);
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];bool active=m->phase==MP_RUN&&!m->braked[w];
  bool wrong=active&&p->target*p->rpm<0&&fabsf(p->rpm)>5;
  bool stalled=active&&m->now-m->drive_since>=MOTION_STALL_GRACE_MS&&fabsf(p->rpm)<1;
  bool too_fast=active&&fabsf(p->rpm)>160;
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
