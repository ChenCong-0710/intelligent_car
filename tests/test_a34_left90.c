#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static MotionPd m;static uint16_t left;
static void init(void){left=0;motion_init(&m,0,0);motion_start(&m);}
static void step(int dl,bool ok){left=(uint16_t)(left+dl);motion_tick(&m,left,0,ok);}
int main(void){
 assert(MOTION_PIVOT_ACTIVE_WHEEL==0&&MOTION_PIVOT_STEPS==1&&motion_pivot_counts(90)==611);
 init();assert(m.wheel_goal[0]==611&&m.wheel_goal[1]==0&&!m.braked[0]&&m.braked[1]);
 assert(m.config.cruise_target==34&&m.config.soft_stop_ms==0&&!m.config.sync_enabled);
 for(unsigned i=0;i<500;++i){step(0,true);assert(m.braked[1]&&m.wheel[1].duty==0);}
 assert(m.wheel[0].target_pulses==34&&m.wheel[0].duty>0&&m.wheel[1].target_pulses==0);
 for(unsigned i=1;i<=611;++i){step(1,true);assert(m.braked[1]&&m.wheel[1].duty==0);if(i<611)assert(!m.braked[0]);}
 assert(m.phase==DM_BRAKE&&m.goal_trigger[0]==611&&m.wheel[0].duty==0&&!m.decelerating[0]);
 step(2,true);for(unsigned i=0;i<250&&m.phase==DM_BRAKE;++i)step(0,true);
 assert(motion_sequence_stage()==MS_DONE&&g_pivot_net_counts[0]==613&&g_pivot_net_counts[1]==0);
 for(unsigned i=0;i<6000;++i)step(0,true);assert(motion_sequence_stage()==MS_DONE&&!motion_busy(&m));
 init();double pos=0,speed=0;
 for(unsigned t=0;t<14000&&motion_busy(&m);++t){
  float u=m.wheel[0].duty;assert(m.braked[1]&&m.wheel[1].duty==0&&u>=0);
  double eq=m.enable&&!m.braked[0]?fmax(0,(u-.31)*250):0;
  speed+=(eq-speed)/(m.braked[0]?12:80);pos+=speed/50;
  left=(uint16_t)(int32_t)pos;motion_tick(&m,left,0,true);assert(m.error==DM_OK);
 }
 assert(motion_sequence_stage()==MS_DONE&&m.goal_trigger[0]>=611&&g_pivot_net_counts[1]==0);
 printf("Left pivot model threshold=%ld final=%ld, fixed right=%ld (software only)\n",(long)m.goal_trigger[0],(long)m.actual[0],(long)m.actual[1]);
 init();motion_stop(&m);for(unsigned i=0;i<150;++i)step(0,true);assert(!m.enable&&!motion_busy(&m));
 init();step(0,false);assert(m.error==DM_DRIVER&&!m.enable);
 init();for(unsigned i=0;i<2200&&m.running;++i)step(0,true);assert(m.error==DM_STALL);
 puts("PASS: left-only PD34, right always braked, 611 immediate cutoff, real stop total, single action/no wait/reverse, stop/driver/stall");
}
