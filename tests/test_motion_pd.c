#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define PI 3.14159265359f
static void pd_tests(void){
 float steady=motion_pd_duty(65,65,0,0);
 assert(fabsf(steady-0.4f)<0.00001f);
 assert(motion_pd_duty(65,40,-200,0)>steady);
 assert(motion_pd_duty(65,85,200,0)<steady);
 assert(motion_pd_duty(-65,-40,200,0)<-steady);
 assert(motion_pd_duty(-65,-85,-200,0)>-steady);
 assert(motion_pd_duty(65,65,0,20)>motion_pd_duty(65,65,0,-20));
 assert(motion_pd_duty(65,250,2000,0)==0);
 for(unsigned i=0;i<1000;++i)assert(motion_pd_duty(65,40,0,0)==motion_pd_duty(65,40,0,0));
 assert(fabsf(motion_pd_duty(65,-10000,-10000,0))<=MOTION_MAX_DUTY);
 assert(motion_pd_duty(0,65,1000,0)==0);
 assert(lroundf(100*840/(PI*43))==622);
 assert(lroundf(132*840*30/(360.0f*43))==215);
 assert(lroundf(132*840*60/(360.0f*43))==430);
 assert(lroundf(132*840*90/(360.0f*43))==645);
}
typedef struct {MotionPd m;float rpm[2],pulses[2];unsigned completed,turns,trims;uint32_t pause_start;unsigned old_action;MotionPhase prev;bool loaded;} Plant;
static void plant_init(Plant *p,bool loaded){memset(p,0,sizeof(*p));p->loaded=loaded;motion_init(&p->m,0,0);p->prev=MP_IDLE;}
static void plant_tick(Plant *p){
 for(unsigned w=0;w<2;++w){
  float u=p->m.wheel[w].duty;
  bool brake=p->m.enable&&(p->m.braked[w]||fabsf(u)<0.0001f);
  float eq=0,tau=brake?0.015f:0.055f;
  if(p->m.enable&&!brake){
   float dead=0.14f+(p->loaded?0.025f:0)+(w?0.006f:0);
   float slope=0.004f+(w?0.00003f:0);
   eq=copysignf(fmaxf(0,(fabsf(u)-dead)/slope),u);
  }
  if(!p->m.enable)tau=0.15f;
  p->rpm[w]+=(eq-p->rpm[w])*(.001f/tau);
  p->pulses[w]+=p->rpm[w]*840/60000.0f;
 }
 uint16_t l=(uint16_t)(int32_t)lroundf(p->pulses[0]);
 uint16_t r=(uint16_t)(int32_t)lroundf(p->pulses[1]);
 motion_tick(&p->m,l,r,true);
 assert(p->m.trim_round<=3);
 for(unsigned w=0;w<2;++w){assert(fabsf(p->m.wheel[w].duty)<=MOTION_MAX_DUTY+0.00001f);assert(fabsf(p->m.wheel[w].target)<=65.001f);}
 if(p->m.phase==MP_RUN && !p->m.braked[0]&&!p->m.braked[1])
  assert(fabsf(fabsf(p->m.wheel[0].target)-fabsf(p->m.wheel[1].target))<0.0001f);
 if(p->m.phase==MP_PAUSE&&p->prev!=MP_PAUSE){
  ++p->completed;
  MotionResult *a=&p->m.result[p->m.action];
  printf("%s cycle %u %s pulses %ld,%ld goal %ld,%ld trim %u mean %.1f,%.1f\n",p->loaded?"GROUND":"AIR",p->m.cycle,motion_action_name(p->m.action),(long)a->actual[0],(long)a->actual[1],(long)a->goal[0],(long)a->goal[1],a->trims,a->mean_rpm[0],a->mean_rpm[1]);
  assert(a->trims==0);
  assert(a->passed==!p->m.position_warning);
  if(p->m.action>=2){++p->turns;if(a->passed)assert(abs(abs(a->actual[0])-abs(a->actual[1]))<=6);}
  p->pause_start=p->m.now;p->old_action=p->m.action;p->trims+=a->trims;
 }
 if(p->prev==MP_PAUSE&&p->m.phase!=MP_PAUSE){
  assert(p->m.now-p->pause_start==2000);
  assert(p->m.action==(p->old_action+1)%8);
 }
 if(p->m.phase==MP_TRIM)assert(p->m.now-p->m.since<=200);
 p->prev=p->m.phase;
}
static void cycle_tests(void){
 for(unsigned loaded=0;loaded<2;++loaded){
  Plant p;plant_init(&p,loaded!=0);for(unsigned i=0;i<100;++i)plant_tick(&p);
  assert(p.m.phase==MP_IDLE&&!p.m.enable);
  motion_start(&p.m);
  for(unsigned i=0;i<60000&&p.completed<16&&p.m.phase!=MP_FAULT;++i)plant_tick(&p);
  printf("RESULT %s phase=%s error=%s complete=%u\n",loaded?"GROUND":"AIR",motion_phase_name(p.m.phase),motion_error_name(p.m.error),p.completed);
  assert(p.m.phase!=MP_FAULT&&p.completed==16&&p.turns==12&&p.m.cycle==2);
 }
}
static void fault_tests(void){
 MotionPd m;motion_init(&m,65530,5);motion_tick(&m,5,65530,true);
 assert(m.wheel[0].total==11&&m.wheel[1].total==-11);
 motion_tick(&m,100,65530,true);assert(m.error==MP_ENCODER&&!m.enable);
 motion_init(&m,0,0);motion_start(&m);motion_tick(&m,0,0,false);assert(m.error==MP_DRIVER&&!m.enable);
 motion_init(&m,0,0);motion_start(&m);
 for(unsigned t=0;t<4200&&m.phase!=MP_FAULT;++t)motion_tick(&m,0,0,true);
 assert(m.error==MP_STALL&&m.now>=3999&&!m.enable);
 motion_init(&m,0,0);motion_start(&m);m.now=m.action_since+4999;motion_tick(&m,0,0,true);assert(m.error==MP_TIMEOUT);
 motion_init(&m,0,0);motion_start(&m);motion_stop(&m);
 assert(m.phase==MP_STOPPING&&!m.running&&m.wheel[0].duty==0);
 for(unsigned i=0;i<100;++i)motion_tick(&m,0,0,true);
 assert(m.phase==MP_STOPPED&&!m.enable);
 motion_start(&m);assert(m.phase==MP_RUN&&m.running&&m.action==0);
 /* Wrong feedback must not be mistaken for valid forward travel. */
 motion_init(&m,0,0);motion_start(&m);
 for(unsigned i=0;i<300&&m.phase!=MP_FAULT;++i)motion_tick(&m,(uint16_t)(-(int)i),(uint16_t)(-(int)i),true);
 assert(m.error==MP_DIRECTION);
}
static void trim_and_no_integral(void){
 MotionPd m;motion_init(&m,0,0);motion_start(&m);
 for(unsigned i=0;i<500;++i)motion_tick(&m,0,0,true);
 float duty=m.wheel[0].duty;
 for(unsigned i=0;i<1000;++i)motion_tick(&m,0,0,true);
 assert(m.phase==MP_RUN&&fabsf(m.wheel[0].duty-duty)<0.000001f);
 assert(fabsf(duty-0.595f)<0.00001f);
 motion_init(&m,650,640);motion_start(&m);
 m.origin[0]=m.origin[1]=0;m.wheel[0].total=650;m.wheel[1].total=640;
 m.actual[0]=650;m.actual[1]=640;m.at_brake[0]=650;m.at_brake[1]=640;
 m.braked[0]=m.braked[1]=true;m.phase=MP_BRAKE;m.since=0;m.learn_pending=false;
 for(unsigned i=0;i<1800&&m.phase!=MP_FAULT;++i){
  motion_tick(&m,650,640,true);
  if(m.phase==MP_TRIM)assert(m.now-m.since<=200);
 }
 assert(m.error==MP_OK&&m.trim_round==0&&m.phase==MP_PAUSE&&m.position_warning&&m.running);
 for(unsigned p=MP_RUN;p<=MP_PAUSE;++p){
  motion_init(&m,0,0);motion_start(&m);m.phase=(MotionPhase)p;motion_stop(&m);
  assert(m.wheel[0].duty==0&&m.wheel[1].duty==0&&!m.running);
  for(unsigned t=0;t<100;++t)motion_tick(&m,0,0,true);
  assert(m.phase==MP_STOPPED&&!m.enable);
 }
}
int main(void){pd_tests();fault_tests();trim_and_no_integral();cycle_tests();puts("PASS: positional PD, disturbance signs, no accumulation, targets, encoder wrap, stop/fault, AIR/GROUND 2-cycle simulation");}
