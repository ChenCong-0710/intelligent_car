#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static MotionPd m;static uint16_t l,r;
static void init(void){l=r=0;motion_init(&m,l,r);}
static void step(int dl,int dr){l=(uint16_t)(l+dl);r=(uint16_t)(r+dr);motion_tick(&m,l,r,true);}
int main(void){
 init();for(unsigned i=0;i<49;++i){step(1,-1);assert(m.wheel[0].speed_pulses==0);}step(1,-1);
 assert(m.wheel[0].speed_pulses==50&&m.wheel[1].speed_pulses==-50);
 for(unsigned i=0;i<50;++i)step(0,0);assert(m.wheel[0].speed_pulses==0);
 init();motion_start(&m);float old=0;
 for(unsigned t=1;t<=500;++t){step(0,0);assert(m.wheel[0].duty>=0);assert(m.wheel[0].duty-old<=.06001f);old=m.wheel[0].duty;}
 assert(m.wheel[0].target_pulses==50);
 init();motion_start(&m);
 for(unsigned i=1;i<=2000;++i)step(1,i%2?0:1);
 assert(m.braked[0]&&!m.braked[1]&&m.actual[0]==2000&&m.actual[1]==1000&&m.phase==MP_RUN);
 for(unsigned i=0;i<2000;++i)step(0,i%2?0:1);
 assert(m.phase==MP_BRAKE&&g_motor_pulses[0]==2000&&g_motor_pulses[1]==2000);
 for(unsigned i=0;i<250&&m.phase==MP_BRAKE;++i)step(0,0);
 assert(m.phase==MP_PAUSE&&m.forward_result[0]==2000&&m.forward_result[1]==2000);
 uint32_t pause=m.now;
 for(unsigned i=0;i<1999;++i){step(0,0);assert(m.phase==MP_PAUSE);}
 step(0,0);assert(m.now-pause==2000&&m.phase==MP_RUN&&m.action==1);
 assert(g_motor_pulses[0]==0&&g_motor_pulses[1]==0);
 for(unsigned i=1;i<=2000;++i)step(-1,-1);
 assert(m.phase==MP_BRAKE&&m.actual[0]==2000&&m.actual[1]==2000);
 for(unsigned i=0;i<250&&m.phase==MP_BRAKE;++i)step(0,0);
 assert(m.phase==MP_STOPPED&&!m.running&&m.enable&&m.reverse_result[0]==2000);
 for(unsigned i=0;i<3000;++i)step(0,0);assert(m.phase==MP_STOPPED&&m.action==1);
 init();motion_start(&m);for(unsigned i=0;i<1600;++i)step(1,1);
 assert(m.wheel[0].target_pulses==50);
 for(unsigned i=0;i<350;++i)step(1,1);
 assert(m.wheel[0].target_pulses<25&&m.wheel[0].target_pulses>=20);
 motion_stop(&m);for(unsigned i=0;i<150;++i)step(0,0);assert(!m.enable&&!m.running);
 init();motion_start(&m);motion_tick(&m,0,0,false);assert(m.error==MP_DRIVER);
 init();motion_start(&m);for(unsigned i=0;i<2200&&m.running;++i)step(0,0);assert(m.error==MP_STALL);
 init();motion_start(&m);step(100,0);assert(m.error==MP_ENCODER);
 float ff=motion_pulse_duty(0,50,50,0,0);
 assert(motion_pulse_duty(0,50,40,0,0)>ff);assert(motion_pulse_duty(0,50,60,0,0)<ff);
 assert(motion_pulse_duty(0,-50,-40,0,0)<0);
 puts("PASS: raw A 50ms signed sampling, wrap, soft start/slew, per-wheel 2000 stop, settled 2s pause, independent reverse segment, final stop without loop, soft endpoint, fault/stop, positional PD");
}
