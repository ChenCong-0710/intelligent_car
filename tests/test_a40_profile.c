#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static MotionPd m;
static double position[2],speed[2];
static void tick(void){
 uint16_t count[2];
 for(unsigned w=0;w<2;++w){
  float u=m.wheel[w].duty;
  double dead=u>=0?(w?.35:.31):(w?.34:.32);
  double eq=m.enable&&!m.braked[w]?copysign(fmax(0,(fabs(u)-dead)*250),u):0;
  speed[w]+=(eq-speed[w])/(m.braked[w]?12:80);position[w]+=speed[w]/50;
  count[w]=(uint16_t)(int32_t)position[w];
 }
 float previous=m.sync_correction;
 bool active=m.phase==DM_RUN&&!m.braked[0]&&!m.braked[1]&&!m.decelerating[0]&&!m.decelerating[1];
 motion_tick(&m,count[0],count[1],true);assert(m.error==DM_OK);
 bool after=m.phase==DM_RUN&&!m.braked[0]&&!m.braked[1]&&!m.decelerating[0]&&!m.decelerating[1];
 if(active&&after){
  assert(fabsf(m.sync_correction-previous)<=.50001f);
  assert(fabsf(m.sync_correction)<=8.00001f);
  if(m.now-m.drive_since>=500){
   assert(fabsf(fabsf(m.wheel[0].target_pulses)+fabsf(m.wheel[1].target_pulses)-80)<.001f);
   for(unsigned w=0;w<2;++w)assert(fabsf(m.wheel[w].target_pulses)>=31.999f&&fabsf(m.wheel[w].target_pulses)<=48.001f);
  }
 }
}
int main(void){
 motion_init(&m,0,0);
 assert(m.config.cruise_target==40&&m.config.sync_max==8&&m.config.sync_slew==.5f);
 assert(m.config.soft_stop_ms==300&&m.config.speed_ms==50&&m.config.goal_counts==2000);
 motion_start(&m);bool wait_seen=false,reverse_seen=false;uint32_t waiting_at=0;
 for(unsigned t=0;t<30000&&motion_sequence_stage()!=MS_DONE;++t){
  MotionSequenceStage before=motion_sequence_stage();tick();MotionSequenceStage after=motion_sequence_stage();
  if(before==MS_FORWARD&&after==MS_WAIT){wait_seen=true;waiting_at=m.now;}
  if(before==MS_WAIT&&after==MS_REVERSE){reverse_seen=true;assert(m.now-waiting_at==2000);assert(m.counts[0]==0&&m.counts[1]==0);}
 }
 assert(wait_seen&&reverse_seen&&motion_sequence_stage()==MS_DONE&&!motion_busy(&m));
 for(unsigned w=0;w<2;++w){assert(g_motion_results[0][w]>2000);assert(g_motion_results[1][w]>2000);assert(m.goal_trigger[w]>=2000&&m.goal_trigger[w]<=2002);}
 for(unsigned i=0;i<3000;++i)tick();assert(motion_sequence_stage()==MS_DONE);
 printf("PASS: A40 cruise, slow count correction <=0.5/sample, target bounds32..48, loaded roundtrip, 2000 triggers, 2s settled wait, final stop. Model totals forward=%ld,%ld reverse=%ld,%ld\n",(long)g_motion_results[0][0],(long)g_motion_results[0][1],(long)g_motion_results[1][0],(long)g_motion_results[1][1]);
}
