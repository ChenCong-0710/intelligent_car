#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static MotionPd m;
static uint16_t counter[2];
static double position[2],speed[2];
static void init(void){position[0]=position[1]=speed[0]=speed[1]=0;counter[0]=counter[1]=0;motion_init(&m,0,0);}
static void plant(bool ok){
 for(unsigned w=0;w<2;++w){
  float u=m.wheel[w].duty;
  double dead=u>=0?(w?.35:.31):(w?.34:.32);
  double eq=m.enable&&!m.braked[w]?copysign(fmax(0,(fabs(u)-dead)*250),u):0;
  speed[w]+=(eq-speed[w])/(m.braked[w]?12:80);position[w]+=speed[w]/50;
  counter[w]=(uint16_t)(int32_t)position[w];
 }
 motion_tick(&m,counter[0],counter[1],ok);
}
int main(void){
 /* The original V4 API still defaults to forward and immediate endpoint brake. */
 DualMotor core;DualMotorConfig cfg;dual_motor_default_config(&cfg);assert(cfg.soft_stop_ms==0);
 assert(dual_motor_init(&core,&cfg,0,0));assert(!dual_motor_start_direction(&core,0));
 assert(dual_motor_start_direction(&core,-1));assert(!dual_motor_start_direction(&core,1));
 for(unsigned i=0;i<500;++i)dual_motor_tick_1ms(&core,0,0,true);
 for(unsigned i=1;i<=100;++i)dual_motor_tick_1ms(&core,(uint16_t)(-((int)i)),0,true);
 assert(core.actual[0]==100&&core.actual[1]==0&&core.sync_correction>0);
 assert(core.wheel[0].target_pulses<0&&core.wheel[0].target_pulses>core.wheel[1].target_pulses);
 assert(fabsf(core.wheel[0].target_pulses+core.wheel[1].target_pulses+100)<.001f);
 /* Soft brake begins only at threshold, is monotonic, lasts the configured time. */
 cfg.goal_counts=100;cfg.soft_stop_ms=300;assert(dual_motor_init(&core,&cfg,0,0));dual_motor_start(&core);
 for(unsigned i=0;i<500;++i)dual_motor_tick_1ms(&core,0,0,true);
 for(unsigned i=1;i<=100;++i)dual_motor_tick_1ms(&core,i,i,true);
 assert(core.decelerating[0]&&core.decelerating[1]&&!core.braked[0]);
 assert(core.goal_trigger[0]==100&&core.goal_trigger[1]==100);
 float previous=core.wheel[0].duty;uint32_t start=core.now;
 for(unsigned age=1;age<=300;++age){
  uint16_t cnt=(uint16_t)(100+age/50);dual_motor_tick_1ms(&core,cnt,cnt,true);
  assert(fabsf(core.wheel[0].duty)<=fabsf(previous)+.000001f);previous=core.wheel[0].duty;
  if(age<300)assert(!core.braked[0]);
 }
 assert(core.now-start==300&&core.braked[0]&&core.braked[1]&&core.actual[0]==106);
 dual_motor_trip(&core,DM_DRIVER);assert(!core.enable&&!core.decelerating[0]);
 /* Real app sequence: both directions, independent 2000 triggers, retained coast,
    exactly 2s after settled, then count reset, final stop with no automatic loop. */
 init();motion_start(&m);bool waited=false,reversed=false;uint32_t waiting_at=0;int32_t forward[2]={0};
 for(unsigned t=0;t<30000&&motion_sequence_stage()!=MS_DONE;++t){
  MotionSequenceStage before=motion_sequence_stage();plant(true);assert(m.error==DM_OK);
  MotionSequenceStage after=motion_sequence_stage();
  if(before==MS_FORWARD&&after==MS_WAIT){
   waited=true;waiting_at=m.now;
   for(unsigned w=0;w<2;++w){forward[w]=m.actual[w];assert(forward[w]>2000);assert(m.goal_trigger[w]>=2000&&m.goal_trigger[w]<=2002);}
   assert(motion_busy(&m)&&motion_wait_remaining(&m)==2000);
  }
  if(after==MS_WAIT){assert(m.now-waiting_at<2000);assert(m.actual[0]==forward[0]&&m.actual[1]==forward[1]);}
  if(before==MS_WAIT&&after==MS_REVERSE){
   reversed=true;assert(m.now-waiting_at==2000);assert(m.move_direction==-1);
   assert(g_motor_pulses[0]==0&&g_motor_pulses[1]==0);
   assert(m.actual[0]==0&&m.actual[1]==0);
   assert(g_motion_results[0][0]==forward[0]&&g_motion_results[0][1]==forward[1]);
  }
  if(after==MS_REVERSE)for(unsigned w=0;w<2;++w)assert(m.wheel[w].duty<=0);
 }
 assert(waited&&reversed&&motion_sequence_stage()==MS_DONE&&!motion_busy(&m));
 assert(m.braked[0]&&m.braked[1]&&m.enable);
 for(unsigned w=0;w<2;++w){assert(g_motion_results[1][w]>2000);assert(m.goal_trigger[w]>=2000&&m.goal_trigger[w]<=2002);}
 printf("Roundtrip model forward=%ld,%ld reverse=%ld,%ld; thresholds=%ld,%ld (software only)\n",(long)g_motion_results[0][0],(long)g_motion_results[0][1],(long)g_motion_results[1][0],(long)g_motion_results[1][1],(long)m.goal_trigger[0],(long)m.goal_trigger[1]);
 for(unsigned i=0;i<4000;++i)plant(true);assert(motion_sequence_stage()==MS_DONE&&!motion_busy(&m));
 /* User cancellation or driver fault while waiting must suppress reverse. */
 init();motion_start(&m);for(unsigned i=0;i<15000&&motion_sequence_stage()!=MS_WAIT;++i)plant(true);
 assert(motion_sequence_stage()==MS_WAIT);motion_stop(&m);for(unsigned i=0;i<3000;++i)plant(true);
 assert(motion_sequence_stage()==MS_CANCELLED&&m.move_direction==1&&!m.enable);
 init();motion_start(&m);for(unsigned i=0;i<15000&&motion_sequence_stage()!=MS_WAIT;++i)plant(true);
 assert(motion_sequence_stage()==MS_WAIT);plant(false);for(unsigned i=0;i<3000;++i)plant(true);
 assert(motion_sequence_stage()==MS_FAULT&&m.error==DM_DRIVER&&!m.enable&&m.move_direction==1);
 puts("PASS: reverse synchronization sign, 2000-triggered 300ms soft stop, exact settled 2s wait, delayed count reset, signed rollover, no loop, wait cancellation/fault");
}
