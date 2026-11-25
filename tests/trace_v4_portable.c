#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
static MotionPd m;static double position[2],speed[2];
static void emit(FILE *f){
 #define E(x) assert(fwrite(&(x),sizeof(x),1,f)==1)
 E(m.phase);E(m.error);E(m.running);E(m.enable);E(m.braked);E(m.action);E(m.sample_ms);
 E(m.sync_error);E(m.sync_rate);E(m.sync_correction);E(m.now);E(m.since);E(m.drive_since);E(m.stopped_since);
 E(m.origin);E(m.actual);E(m.at_brake);E(m.forward_result);
 for(unsigned w=0;w<2;++w){PdWheel *q=&m.wheel[w];E(q->previous_counter);E(q->total);E(q->sample_sum);E(q->speed_pulses);E(q->target_pulses);E(q->previous_error);E(q->derivative);E(q->duty);E(q->wrong_since);E(q->stall_since);E(q->overspeed_since);}
 uint32_t counts[2]={g_motor_pulses[0],g_motor_pulses[1]};E(counts);
 #undef E
}
int main(int argc,char **argv){
 assert(argc==2);FILE *f=fopen(argv[1],"wb");assert(f);motion_init(&m,0,0);
 for(unsigned t=0;t<50000;++t){
  if(t==10||t==10000||t==16000||t==22000||t==33000||t==41000)motion_start(&m);
  if(t==9000||t==26000)motion_stop(&m);
  if(t==15000||t==32000||t==40000){position[0]=position[1]=65530;speed[0]=speed[1]=0;motion_init(&m,65530,65530);}
  uint16_t c[2];
  for(unsigned w=0;w<2;++w){
   float u=m.wheel[w].duty;double dead=w?.35:.31;
   if(t>=18000&&t<18300&&w==1)dead+=.08;
   double eq=m.enable&&!m.braked[w]?fmax(0,(u-dead)*250):0;
   speed[w]+=(eq-speed[w])/(m.braked[w]?12:80);position[w]+=speed[w]/50;
   c[w]=(uint16_t)(int32_t)position[w];
  }
  if(t==30000)c[0]=(uint16_t)(c[0]+1000); /* encoder discontinuity */
  motion_tick(&m,c[0],c[1],t!=14000);
  emit(f);
 }
 assert(fclose(f)==0);return 0;
}
