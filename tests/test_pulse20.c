#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
static MotionPd m;
static double rpm[2],pos[2];
static void plant(void){
 uint16_t cnt[2];
 for(unsigned w=0;w<2;++w){
  float u=m.wheel[w].duty;
  float eq=0;
  if(m.enable&&!m.braked[w])eq=copysignf(fmaxf(0,(fabsf(u)-(w?.065f:.05f))/(w?.002f:.0021f)),u);
  rpm[w]+=(eq-rpm[w])/(m.braked[w]?15.0:55.0);
  pos[w]+=rpm[w]*840.0/60000.0;cnt[w]=(uint16_t)(int32_t)pos[w];
 }
 motion_tick(&m,cnt[0],cnt[1],true);
}
int main(void){
 assert(fabsf(MOTION_TARGET_PULSES-18.0f)<.00001f);
 assert(lroundf(100*840/(3.14159265f*43))==622);
 motion_init(&m,65530,5);
 for(unsigned t=1;t<=19;++t){motion_tick(&m,(uint16_t)(65530+t),(uint16_t)(5-t),true);assert(m.wheel[0].speed_pulses==0);}
 motion_tick(&m,14,(uint16_t)(5-20),true);
 assert(m.wheel[0].speed_pulses==20&&m.wheel[1].speed_pulses==-20);
 assert(m.wheel[0].total==20&&m.wheel[1].total==-20);
 for(unsigned t=0;t<19;++t)motion_tick(&m,14,(uint16_t)(5-20),true);
 assert(m.wheel[0].speed_pulses==20);
 motion_tick(&m,14,(uint16_t)(5-20),true);assert(m.wheel[0].speed_pulses==0);
 float ff=motion_pulse_duty(0,18.2f,18.2f,0,0);
 assert(ff>.17f&&ff<.20f);
 assert(motion_pulse_duty(0,18.2f,10,0,0)>ff);
 assert(motion_pulse_duty(0,18.2f,25,0,0)<ff);
 assert(motion_pulse_duty(0,-18.2f,-10,0,0)<-ff);
 assert(motion_pulse_duty(0,18.2f,18.2f,1,0)>ff);
 for(unsigned i=0;i<1000;++i)assert(motion_pulse_duty(0,18.2f,18.2f,0,0)==ff);
 assert(motion_pulse_duty(0,0,20,20,0)==0);
 motion_init(&m,0,0);motion_start(&m);
 for(unsigned t=0;t<19;++t){motion_tick(&m,0,0,true);assert(m.wheel[0].duty==0);}
 motion_tick(&m,0,0,true);assert(m.wheel[0].duty>0);
 motion_tick(&m,621,621,true);assert(m.phase==MP_FAULT); /* implausible jump retained */
 motion_init(&m,0,0);motion_start(&m);
 for(unsigned t=1;t<=622;++t){motion_tick(&m,t,t,true);if(t<622)assert(g_distance_pulses==(int32_t)t);}
 assert(m.phase==MP_BRAKE&&g_distance_pulses==0&&m.distance_pulses==0);
 for(unsigned t=0;t<150;++t)motion_tick(&m,625,626,true);
 assert(m.phase==MP_PAUSE&&m.last_straight[0]==625&&m.last_straight[1]==626);
 assert(m.result[0].actual[0]==625&&m.result[0].trims==0);
 motion_init(&m,0,0);motion_start(&m);
 unsigned completed=0;MotionPhase prev=m.phase;
 for(unsigned t=0;t<60000&&completed<16;++t){
  plant();assert(m.error==MP_OK);
  for(unsigned w=0;w<2;++w)assert(m.wheel[w].duty*m.goal[w]>=0);
  if(m.phase==MP_PAUSE&&prev!=MP_PAUSE){++completed;assert(m.result[m.action].trims==0);assert(g_distance_pulses==0);}
  prev=m.phase;
 }
 assert(completed==16);motion_stop(&m);for(unsigned t=0;t<100;++t)plant();assert(!m.running&&!m.enable);
 motion_init(&m,0,0);motion_start(&m);motion_tick(&m,0,0,false);assert(m.error==MP_DRIVER&&!m.enable);
 motion_init(&m,0,0);motion_start(&m);for(unsigned t=0;t<4200&&m.running;++t)motion_tick(&m,0,0,true);
 assert(m.error==MP_STALL&&!m.enable);
 puts("PASS: non-overlapping 20ms signed speed, encoder wrap, 18-count target, positional PD, global distance reset, settled stop totals, 16 simulated actions without reversal, stop/driver/stall");
}
