#include "dual_motor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
static float clip(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
static int direction(const DualMotor *m){return m->move_direction;}
static void phase(DualMotor *m,DualMotorPhase p){m->phase=p;m->since=m->now;m->stopped_since=0;}
static void brake(DualMotor *m,unsigned w){
 if(!m->braked[w])m->at_brake[w]=m->actual[w];
 m->decelerating[w]=false;m->braked[w]=true;m->wheel[w].duty=m->wheel[w].target_pulses=0;
}
void dual_motor_trip(DualMotor *m,DualMotorError error){
 if(m->phase==DM_FAULT)return;
 brake(m,0);brake(m,1);m->error=error;m->enable=m->running=false;phase(m,DM_FAULT);
}
bool dual_motor_init(DualMotor *m,const DualMotorConfig *config,uint16_t l,uint16_t r){
 DualMotorConfig selected;if(config)selected=*config;else dual_motor_default_config(&selected);
 memset(m,0,sizeof(*m));m->config=selected;m->move_direction=1;
 m->wheel[0].previous_counter=l;m->wheel[1].previous_counter=r;
 m->braked[0]=m->braked[1]=true;
 if(!dual_motor_config_valid(&selected)){m->phase=DM_FAULT;m->error=DM_CONFIG;return false;}
 return true;
}
static void begin(DualMotor *m){
 m->sync_error=0;m->sync_rate=m->sync_correction=0;
 m->sample_ms=0;m->drive_since=m->now;m->enable=true;
 for(unsigned w=0;w<2;++w){
  DualMotorWheel *p=&m->wheel[w];m->origin[w]=p->total;m->actual[w]=m->at_brake[w]=0;m->counts[w]=0;
  p->sample_sum=0;p->speed_pulses=0;p->target_pulses=0;
  p->previous_error=p->derivative=p->duty=0;m->braked[w]=m->wheel_goal[w]==0;
  m->decelerating[w]=false;m->decel_since[w]=0;m->decel_start_duty[w]=0;m->goal_trigger[w]=0;
  p->wrong_since=p->stall_since=p->overspeed_since=m->now;
 }
 phase(m,DM_RUN);
}
bool dual_motor_start_wheels(DualMotor *m,int sign,int32_t left_goal,int32_t right_goal){
 if((sign!=1&&sign!=-1)||left_goal<0||right_goal<0||(!left_goal&&!right_goal)||m->running||m->phase==DM_STOPPING||m->phase==DM_FAULT)return false;
 m->continuous=false;m->move_direction=sign;m->action=sign>0?0U:1U;m->error=DM_OK;m->running=true;
 m->wheel_goal[0]=left_goal;m->wheel_goal[1]=right_goal;
 memset(m->forward_result,0,sizeof(m->forward_result));begin(m);return true;
}
bool dual_motor_start_continuous(DualMotor *m,int sign){
 if((sign!=1&&sign!=-1)||m->running||m->phase==DM_STOPPING||m->phase==DM_FAULT)return false;
 m->continuous=true;m->move_direction=sign;m->action=sign>0?0U:1U;
 m->error=DM_OK;m->running=true;m->wheel_goal[0]=m->wheel_goal[1]=0;
 memset(m->forward_result,0,sizeof(m->forward_result));begin(m);
 m->braked[0]=m->braked[1]=false;return true;
}
bool dual_motor_start_direction(DualMotor *m,int sign){
 return dual_motor_start_wheels(m,sign,m->config.goal_counts,m->config.goal_counts);
}
void dual_motor_start(DualMotor *m){(void)dual_motor_start_direction(m,1);}
void dual_motor_stop(DualMotor *m){brake(m,0);brake(m,1);m->running=false;m->enable=true;phase(m,DM_STOPPING);}
float dual_motor_pd_duty(const DualMotorConfig *config,unsigned wheel,float target,float measured,float error_delta){
 if(wheel>1||fabsf(target)<.01f)return 0;
 float sign=target<0?-1.f:1.f;
 /* Previous x4/20ms feedforward re-expressed for A-only x2/50ms.
    Initial estimate only; hardware still needs closed-loop validation. */

 unsigned d=target<0?1:0;
 float u=sign*(config->ff_base[wheel][d]+config->ff_slope[wheel][d]*fabsf(target))+
 config->speed_kp*(target-measured)+config->speed_kd*error_delta;
 return sign*clip(sign*u,0,config->max_duty);
}
static void update_pd(DualMotor *m){
 int sign=direction(m);
 float start=clip((float)(m->now-m->drive_since)/m->config.start_ramp_ms,0,1);
 start=start*start*(3.f-2.f*start);
 m->sync_error=m->actual[0]-m->actual[1];
 if(m->config.sync_enabled&&m->phase==DM_RUN&&!m->braked[0]&&!m->braked[1]&&!m->decelerating[0]&&!m->decelerating[1]){
  float rate=sign*(m->wheel[0].speed_pulses-m->wheel[1].speed_pulses);
  m->sync_rate+=m->config.sync_filter_alpha*(rate-m->sync_rate);
  float e=(float)m->sync_error;
  float count_error=copysignf(fmaxf(0,fabsf(e)-m->config.sync_deadband),e);
  /* Positive error: left is ahead. Lower its speed reference, raise right. */
  float demand=clip(m->config.sync_count_kp*count_error+m->config.sync_rate_kd*m->sync_rate,-m->config.sync_max,m->config.sync_max)*start;
  m->sync_correction+=clip(demand-m->sync_correction,-m->config.sync_slew,m->config.sync_slew);
 }else {m->sync_rate=m->sync_correction=0;}
 for(unsigned w=0;w<2;++w){
  DualMotorWheel *p=&m->wheel[w];
  if(m->phase!=DM_RUN||m->braked[w]){p->duty=p->target_pulses=0;continue;}
  if(m->decelerating[w]){p->target_pulses=0;continue;}
  /* The previous low endpoint reference stalled the real car near 1900.
     Maintain the requested speed until the per-wheel count threshold. */
  float common=m->config.cruise_target*start;
  float correction=w==0?-m->sync_correction:m->sync_correction;
  p->target_pulses=sign*fmaxf(0,common+correction);
  float error=p->target_pulses-p->speed_pulses;
  p->derivative+=m->config.derivative_alpha*((error-p->previous_error)-p->derivative);p->previous_error=error;
  /* Absolute output: no PWM accumulation, no integral. */
  float demand=dual_motor_pd_duty(&m->config,w,p->target_pulses,p->speed_pulses,p->derivative);
  p->duty+=clip(demand-p->duty,-m->config.duty_slew,m->config.duty_slew);
 }
}
void dual_motor_tick_1ms(DualMotor *m,uint16_t left,uint16_t right,bool driver_ok){
 ++m->now;const uint16_t counters[2]={left,right};
 for(unsigned w=0;w<2;++w){
  DualMotorWheel *p=&m->wheel[w];uint16_t wrap=(uint16_t)(counters[w]-p->previous_counter);
  int32_t delta=wrap>32767U?(int32_t)wrap-65536:(int32_t)wrap;p->previous_counter=counters[w];
  if(abs(delta)>m->config.max_delta_1ms){dual_motor_trip(m,DM_ENCODER);return;}
  p->total+=delta;p->sample_sum+=delta;
  /* Signed displacement along this segment's command; reverse starts at zero. */
  if(m->phase==DM_RUN||m->phase==DM_BRAKE||m->phase==DM_STOPPING){
   int64_t count=direction(m)*(p->total-m->origin[w]);
   if(count>INT32_MAX||count<INT32_MIN){dual_motor_trip(m,DM_ENCODER);return;}
   m->actual[w]=(int32_t)count;m->counts[w]=count>0?(uint32_t)count:0;
  }
 }
 if(!driver_ok)dual_motor_trip(m,DM_DRIVER);
 if(m->phase==DM_FAULT)return;
 bool sampled=++m->sample_ms>=m->config.speed_ms;
 if(sampled){m->sample_ms=0;for(unsigned w=0;w<2;++w){m->wheel[w].speed_pulses=(float)m->wheel[w].sample_sum;m->wheel[w].sample_sum=0;}}
 if(m->phase==DM_RUN){
  if(m->config.timeout_protection_enabled&&m->now-m->drive_since>=m->config.action_timeout_ms){dual_motor_trip(m,DM_TIMEOUT);return;}
  for(unsigned w=0;w<2;++w){
   if(!m->continuous&&!m->braked[w]&&!m->decelerating[w]&&m->actual[w]>=m->wheel_goal[w]){
    m->goal_trigger[w]=m->actual[w];
    if(m->config.soft_stop_ms){
     m->decelerating[w]=true;m->decel_since[w]=m->now;
     m->decel_start_duty[w]=m->wheel[w].duty;m->wheel[w].target_pulses=0;
    }else brake(m,w);
   }
   if(m->decelerating[w]){
    uint32_t elapsed=m->now-m->decel_since[w];
    if(elapsed>=m->config.soft_stop_ms)brake(m,w);
    else {
     float x=(float)elapsed/m->config.soft_stop_ms;
     float factor=1.f-x*x*(3.f-2.f*x);
     m->wheel[w].duty=m->decel_start_duty[w]*factor;
    }
   }
  }
  if(m->braked[0]&&m->braked[1])phase(m,DM_BRAKE);
 }else if(m->phase==DM_BRAKE){
  bool stopped=m->wheel[0].speed_pulses==0&&m->wheel[1].speed_pulses==0;
  if(!stopped)m->stopped_since=0;else if(!m->stopped_since)m->stopped_since=m->now;
  if((!m->config.settle_require_zero_speed&&m->now-m->since>=m->config.settle_ms)||
     (m->stopped_since&&m->now-m->stopped_since>=m->config.settle_ms)){
   m->forward_result[0]=m->actual[0];m->forward_result[1]=m->actual[1];
   m->running=false;phase(m,DM_STOPPED); /* forward once; retain brake and results */
  }else if(m->config.settle_timeout_enabled&&m->now-m->since>=m->config.brake_timeout_ms){dual_motor_trip(m,DM_NOT_SETTLED);return;}
 }else if(m->phase==DM_STOPPING&&m->now-m->since>=m->config.manual_stop_ms){m->enable=false;phase(m,DM_STOPPED);}
 if(sampled)update_pd(m);
 for(unsigned w=0;w<2;++w){
  DualMotorWheel *p=&m->wheel[w];bool active=m->phase==DM_RUN&&!m->braked[w]&&!m->decelerating[w];
  bool wrong=m->config.direction_protection_enabled&&active&&direction(m)*p->speed_pulses<=-m->config.direction_threshold;
  bool stalled=m->config.stall_protection_enabled&&active&&m->now-m->drive_since>=m->config.stall_grace_ms&&fabsf(p->speed_pulses)<m->config.stall_threshold;
  bool fast=m->config.overspeed_protection_enabled&&active&&fabsf(p->speed_pulses)>m->config.overspeed_threshold;
  if(!wrong)p->wrong_since=m->now;
  if(!stalled)p->stall_since=m->now;
  if(!fast)p->overspeed_since=m->now;
  if(wrong&&m->now-p->wrong_since>=m->config.direction_ms){dual_motor_trip(m,DM_DIRECTION);return;}
  if(stalled&&m->now-p->stall_since>=m->config.stall_ms){dual_motor_trip(m,DM_STALL);return;}
  if(fast&&m->now-p->overspeed_since>=m->config.overspeed_ms){dual_motor_trip(m,DM_OVERSPEED);return;}
 }
}
const char *dual_motor_action_name(unsigned a){return a==0?"FORWARD":"REVERSE";}
const char *dual_motor_phase_name(DualMotorPhase p){static const char *n[]={"IDLE","RUN","BRAKE","PAUSE_2S","STOPPING","STOPPED","FAULT"};return p<=DM_FAULT?n[p]:"UNKNOWN";}
const char *dual_motor_error_name(DualMotorError e){static const char *n[]={"OK","DRIVER","ENCODER","DIRECTION","STALL","TIMEOUT","NOT_SETTLED","OVERSPEED","WATCHDOG","CONFIG"};return e<=DM_CONFIG?n[e]:"UNKNOWN";}

void dual_motor_default_config(DualMotorConfig *c){
 const DualMotorConfig defaults={
 .speed_ms=50,.start_ramp_ms=500,.action_timeout_ms=15000,.soft_stop_ms=0,
 .settle_ms=100,.brake_timeout_ms=1000,.manual_stop_ms=150,
 .stall_grace_ms=1000,.stall_ms=1000,.direction_ms=150,.overspeed_ms=150,
 .goal_counts=2000,.max_delta_1ms=50,.cruise_target=50,
 .duty_slew=.06f,.max_duty=.85f,.speed_kp=.004f,.speed_kd=.0015f,
 .timeout_protection_enabled=true,.overspeed_protection_enabled=true,.settle_timeout_enabled=true,.settle_require_zero_speed=true,
 .stall_protection_enabled=true,.direction_protection_enabled=true,
 .sync_enabled=true,.sync_count_kp=.55f,.sync_rate_kd=.65f,
 .sync_max=12,.sync_slew=2,.sync_deadband=1,.sync_filter_alpha=.4f,.derivative_alpha=.5f,
 .direction_threshold=2,.stall_threshold=1,.overspeed_threshold=125,
 .ff_base={{.045f,.05f},{.0615f,.067f}},.ff_slope={{.006f,.00592f},{.005688f,.005496f}}
 };*c=defaults;
}
bool dual_motor_config_valid(const DualMotorConfig *c){
 if(!c||c->speed_ms==0||c->speed_ms>1000||c->start_ramp_ms==0||c->goal_counts<=0||c->max_delta_1ms<=0||c->max_delta_1ms>32767)return false;
 if(c->soft_stop_ms>5000||c->soft_stop_ms>=c->action_timeout_ms)return false;
 if(c->action_timeout_ms==0||c->settle_ms==0||c->brake_timeout_ms<=c->settle_ms||c->manual_stop_ms==0||c->stall_ms==0||c->direction_ms==0||c->overspeed_ms==0)return false;
 const float values[]={c->cruise_target,c->duty_slew,c->max_duty,c->speed_kp,c->speed_kd,c->sync_count_kp,c->sync_rate_kd,c->sync_max,c->sync_slew,c->sync_deadband,c->sync_filter_alpha,c->derivative_alpha,c->direction_threshold,c->stall_threshold,c->overspeed_threshold};
 for(unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i)if(!isfinite(values[i])||values[i]<0)return false;
 if(c->cruise_target<=0||c->max_duty<=0||c->max_duty>1||c->duty_slew<=0||c->sync_slew<=0||c->sync_max>=c->cruise_target||c->sync_filter_alpha<=0||c->sync_filter_alpha>1||c->derivative_alpha<=0||c->derivative_alpha>1||c->overspeed_threshold<=c->cruise_target)return false;
 for(unsigned w=0;w<2;++w)for(unsigned d=0;d<2;++d)if(!isfinite(c->ff_base[w][d])||c->ff_base[w][d]<0||!isfinite(c->ff_slope[w][d])||c->ff_slope[w][d]<0)return false;
 return true;
}
void dual_motor_get_status(const DualMotor *m,DualMotorStatus *s){
 s->enable=m->enable;s->running=m->running;s->phase=m->phase;s->error=m->error;
 s->direction=m->move_direction;
 s->sync_error=m->sync_error;s->sync_correction=m->sync_correction;
 for(unsigned w=0;w<2;++w){s->decelerating[w]=m->decelerating[w];s->goal_trigger[w]=m->goal_trigger[w];s->goal[w]=m->wheel_goal[w];s->brake[w]=m->braked[w];s->duty[w]=m->wheel[w].duty;s->speed[w]=m->wheel[w].speed_pulses;s->target[w]=m->wheel[w].target_pulses;s->counts[w]=m->counts[w];s->signed_counts[w]=m->actual[w];}
}
