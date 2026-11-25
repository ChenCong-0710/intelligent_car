#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static MotionPd m;static uint16_t l,r;
static void init(void){l=r=0;motion_init(&m,l,r);}
static void step(int dl,int dr){l=(uint16_t)(l+dl);r=(uint16_t)(r+dr);motion_tick(&m,l,r,true);}
int main(void){
 init();for(unsigned i=0;i<49;++i){step(1,-1);assert(m.wheel[0].speed_pulses==0);}step(1,-1);
 assert(m.wheel[0].speed_pulses==50&&m.wheel[1].speed_pulses==-50);
 init();motion_start(&m);
 for(unsigned i=0;i<500;++i)step(0,0);
 for(unsigned i=0;i<100;++i)step(1,0);
 if(MOTION_SYNC_ENABLED){assert(m.sync_error==100&&m.sync_correction>0);assert(m.wheel[0].target_pulses<50&&m.wheel[1].target_pulses>50);}
 init();motion_start(&m);
 for(unsigned i=0;i<500;++i)step(0,0);
 for(unsigned i=0;i<100;++i)step(0,1);
 if(MOTION_SYNC_ENABLED){assert(m.sync_correction<0);assert(m.wheel[0].target_pulses>50&&m.wheel[1].target_pulses<50);}
 init();motion_start(&m);float previous=0;
 for(unsigned i=0;i<500;++i){step(0,0);assert(m.wheel[0].duty-previous<=.06001f);previous=m.wheel[0].duty;}
 assert(m.wheel[0].target_pulses==50);
 init();motion_start(&m);
 for(unsigned t=1;t<=2000;++t){step(1,t%2?0:1);if(t>=500&&t<2000){assert(m.wheel[0].target_pulses>=38&&m.wheel[0].target_pulses<=62);assert(fabsf(m.wheel[0].target_pulses+m.wheel[1].target_pulses-100)<.001f);}}
 assert(m.braked[0]&&!m.braked[1]&&m.actual[0]==2000&&m.actual[1]==1000);
 for(unsigned t=0;t<2000;++t)step(0,t%2?0:1);
 assert(m.phase==MP_BRAKE&&g_motor_pulses[0]==2000&&g_motor_pulses[1]==2000);
 step(3,4);for(unsigned t=0;t<300&&m.phase==MP_BRAKE;++t)step(0,0);
 assert(m.phase==MP_STOPPED&&!m.running&&m.enable);
 assert(m.forward_result[0]==2003&&m.forward_result[1]==2004);
 for(unsigned t=0;t<20000;++t){step(0,0);assert(m.phase==MP_STOPPED&&m.wheel[0].duty==0&&m.wheel[1].duty==0);}
 /* Loaded model with a substantial stiction threshold reproduces why the old
    20-count endpoint reference failed. It must finish without lowering target. */
 init();motion_start(&m);double pos[2]={0},speed[2]={0};unsigned peak=0;double sum=0;unsigned samples=0;
 for(unsigned t=0;t<14000&&m.running;++t){
  uint16_t count[2];
  for(unsigned w=0;w<2;++w){
   float u=m.wheel[w].duty;assert(u>=0);
   double eq=m.enable&&!m.braked[w]?fmax(0,(u-(w?.35:.31))*250):0;
   speed[w]+=(eq-speed[w])/(m.braked[w]?12:80);
   pos[w]+=speed[w]/50.;count[w]=(uint16_t)(unsigned)pos[w];
  }
  motion_tick(&m,count[0],count[1],true);assert(m.error==MP_OK);
  if(m.phase==MP_RUN&&!m.braked[0]&&!m.braked[1]){
   unsigned e=(unsigned)abs(m.actual[0]-m.actual[1]);if(e>peak)peak=e;sum+=e;++samples;
  }
 }
 assert(m.phase==MP_STOPPED&&m.forward_result[0]>=2000&&m.forward_result[1]>=2000);
 printf("SYNC_METRICS enabled=%d peak=%u mean=%.3f\n",MOTION_SYNC_ENABLED,peak,sum/samples);
 printf("Loaded model settled counts %ld,%ld (software only)\n",(long)m.forward_result[0],(long)m.forward_result[1]);
 init();motion_start(&m);for(unsigned i=0;i<2200&&m.running;++i)step(0,0);assert(m.error==MP_STALL);
 init();motion_start(&m);step(100,0);assert(m.error==MP_ENCODER);
 init();motion_start(&m);motion_tick(&m,0,0,false);assert(m.error==MP_DRIVER);
 init();motion_start(&m);motion_stop(&m);for(unsigned i=0;i<150;++i)step(0,0);assert(!m.enable&&!m.running);
 puts("PASS: 50ms raw sampling, soft start, course-reference correction, independent 2000 cutoff, real coast totals, no reverse/restart, loaded model, protection");
}
