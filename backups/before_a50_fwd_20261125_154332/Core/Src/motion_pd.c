#include "motion_pd.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
volatile uint32_t g_motor_pulses[2];
static float clip(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
static int direction(const MotionPd *m){return m->action==0?1:-1;}
static void phase(MotionPd *m,MotionPhase p){m->phase=p;m->since=m->now;m->stopped_since=0;}
static void brake(MotionPd *m,unsigned w){
 if(!m->braked[w])m->at_brake[w]=m->actual[w];
 m->braked[w]=true;m->wheel[w].duty=m->wheel[w].target_pulses=0;
}
void motion_trip(MotionPd *m,MotionError error){
 if(m->phase==MP_FAULT)return;
 brake(m,0);brake(m,1);m->error=error;m->enable=m->running=false;phase(m,MP_FAULT);
}
void motion_init(MotionPd *m,uint16_t l,uint16_t r){
 memset(m,0,sizeof(*m));m->wheel[0].previous_counter=l;m->wheel[1].previous_counter=r;
 m->braked[0]=m->braked[1]=true;g_motor_pulses[0]=g_motor_pulses[1]=0;
}
static void begin(MotionPd *m){
 m->sample_ms=0;m->drive_since=m->now;m->enable=true;
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];m->origin[w]=p->total;m->actual[w]=m->at_brake[w]=0;g_motor_pulses[w]=0;
  p->sample_sum=0;p->speed_pulses=0;p->target_pulses=0;
  p->previous_error=p->derivative=p->duty=0;m->braked[w]=false;
  p->wrong_since=p->stall_since=p->overspeed_since=m->now;
 }
 phase(m,MP_RUN);
}
void motion_start(MotionPd *m){
 if(m->running||m->phase==MP_STOPPING||m->phase==MP_FAULT)return;
 m->action=0;m->error=MP_OK;m->running=true;
 memset(m->forward_result,0,sizeof(m->forward_result));memset(m->reverse_result,0,sizeof(m->reverse_result));begin(m);
}
void motion_stop(MotionPd *m){brake(m,0);brake(m,1);m->running=false;m->enable=true;phase(m,MP_STOPPING);}
float motion_pulse_duty(unsigned wheel,float target,float measured,float error_delta,float lag){
 if(wheel>1||fabsf(target)<.01f)return 0;
 float sign=target<0?-1.f:1.f;
 /* Previous x4/20ms feedforward re-expressed for A-only x2/50ms.
    Initial estimate only; hardware still needs closed-loop validation. */
 static const float base[2][2]={{.045f,.05f},{.0615f,.067f}};
 static const float slope[2][2]={{.006f,.00592f},{.005688f,.005496f}};
 unsigned d=target<0?1:0;
 float u=sign*(base[wheel][d]+slope[wheel][d]*fabsf(target))+
 MOTION_KP*(target-measured)+MOTION_KD*error_delta+sign*clip(.0005f*lag,-.06f,.06f);
 return sign*clip(sign*u,0,MOTION_MAX_DUTY);
}
static void update_pd(MotionPd *m){
 int sign=direction(m);
 float start=clip((float)(m->now-m->drive_since)/MOTION_START_RAMP_MS,0,1);
 start=start*start*(3.f-2.f*start);
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];
  if(m->phase!=MP_RUN||m->braked[w]){p->duty=p->target_pulses=0;continue;}
  float remaining=fmaxf(0,MOTION_GOAL_PULSES-m->actual[w]);
  float end=clip(remaining/MOTION_SLOW_COUNTS,0,1);end=end*end*(3.f-2.f*end);
  float speed=MOTION_END_SPEED+(MOTION_TARGET_PULSES-MOTION_END_SPEED)*end;
  p->target_pulses=sign*speed*start;
  float error=p->target_pulses-p->speed_pulses;
  p->derivative+=.5f*((error-p->previous_error)-p->derivative);p->previous_error=error;
  float lag=m->braked[1-w]?0.f:(float)(m->actual[1-w]-m->actual[w]);
  /* Absolute output: no PWM accumulation, no integral. */
  float demand=motion_pulse_duty(w,p->target_pulses,p->speed_pulses,p->derivative,lag*end);
  p->duty+=clip(demand-p->duty,-MOTION_DUTY_SLEW,MOTION_DUTY_SLEW);
 }
}
void motion_tick(MotionPd *m,uint16_t left,uint16_t right,bool driver_ok){
 ++m->now;const uint16_t counters[2]={left,right};
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];uint16_t wrap=(uint16_t)(counters[w]-p->previous_counter);
  int32_t delta=wrap>32767U?(int32_t)wrap-65536:(int32_t)wrap;p->previous_counter=counters[w];
  if(abs(delta)>50){motion_trip(m,MP_ENCODER);return;}
  p->total+=delta;p->sample_sum+=delta;
  /* Signed displacement along this segment's command; reverse starts at zero. */
  if(m->phase==MP_RUN||m->phase==MP_BRAKE||m->phase==MP_STOPPING){
   int64_t count=direction(m)*(p->total-m->origin[w]);
   if(count>INT32_MAX||count<INT32_MIN){motion_trip(m,MP_ENCODER);return;}
   m->actual[w]=(int32_t)count;g_motor_pulses[w]=count>0?(uint32_t)count:0;
  }
 }
 if(!driver_ok)motion_trip(m,MP_DRIVER);
 if(m->phase==MP_FAULT)return;
 bool sampled=++m->sample_ms>=MOTION_SPEED_MS;
 if(sampled){m->sample_ms=0;for(unsigned w=0;w<2;++w){m->wheel[w].speed_pulses=(float)m->wheel[w].sample_sum;m->wheel[w].sample_sum=0;}}
 if(m->phase==MP_RUN){
  if(m->now-m->drive_since>=15000U){motion_trip(m,MP_TIMEOUT);return;}
  for(unsigned w=0;w<2;++w)if(!m->braked[w]&&m->actual[w]>=MOTION_GOAL_PULSES)brake(m,w);
  if(m->braked[0]&&m->braked[1])phase(m,MP_BRAKE);
 }else if(m->phase==MP_BRAKE){
  bool stopped=m->wheel[0].speed_pulses==0&&m->wheel[1].speed_pulses==0;
  if(!stopped)m->stopped_since=0;else if(!m->stopped_since)m->stopped_since=m->now;
  if(m->stopped_since&&m->now-m->stopped_since>=100U){
   int32_t *result=m->action==0?m->forward_result:m->reverse_result;
   result[0]=m->actual[0];result[1]=m->actual[1];
   if(m->action==0)phase(m,MP_PAUSE);
   else {m->running=false;phase(m,MP_STOPPED);} /* hold brake, no loop */
  }else if(m->now-m->since>=1000U){motion_trip(m,MP_NOT_SETTLED);return;}
 }else if(m->phase==MP_PAUSE&&m->now-m->since>=MOTION_PAUSE_MS){m->action=1;begin(m);}
 else if(m->phase==MP_STOPPING&&m->now-m->since>=150U){m->enable=false;phase(m,MP_STOPPED);}
 if(sampled)update_pd(m);
 for(unsigned w=0;w<2;++w){
  PdWheel *p=&m->wheel[w];bool active=m->phase==MP_RUN&&!m->braked[w];
  bool wrong=active&&direction(m)*p->speed_pulses<=-2;
  bool stalled=active&&m->now-m->drive_since>=1000U&&fabsf(p->speed_pulses)<1;
  bool fast=active&&fabsf(p->speed_pulses)>125;
  if(!wrong)p->wrong_since=m->now;
  if(!stalled)p->stall_since=m->now;
  if(!fast)p->overspeed_since=m->now;
  if(wrong&&m->now-p->wrong_since>=150U){motion_trip(m,MP_DIRECTION);return;}
  if(stalled&&m->now-p->stall_since>=1000U){motion_trip(m,MP_STALL);return;}
  if(fast&&m->now-p->overspeed_since>=150U){motion_trip(m,MP_OVERSPEED);return;}
 }
}
const char *motion_action_name(unsigned a){return a==0?"FWD_2000":"REV_2000";}
const char *motion_phase_name(MotionPhase p){static const char *n[]={"IDLE","RUN","BRAKE","PAUSE_2S","STOPPING","STOPPED","FAULT"};return p<=MP_FAULT?n[p]:"UNKNOWN";}
const char *motion_error_name(MotionError e){static const char *n[]={"OK","DRIVER","ENCODER","DIRECTION","STALL","TIMEOUT","NOT_SETTLED","OVERSPEED","WATCHDOG"};return e<=MP_WATCHDOG?n[e]:"UNKNOWN";}
