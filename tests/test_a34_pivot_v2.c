#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static MotionPd m;static uint16_t l,r;
static void init(void){l=r=0;motion_init(&m,0,0);motion_start(&m);}
static void step(int dl,int dr){l=(uint16_t)(l+dl);r=(uint16_t)(r+dr);motion_tick(&m,l,r,true);}
int main(void){
 assert(motion_pivot_counts(90)==611);assert(motion_pivot_counts(45)==305);
 assert(motion_pivot_counts(0)==0&&motion_pivot_counts(-90)==0);
 init();assert(m.config.cruise_target==34&&m.config.soft_stop_ms==0&&!m.config.sync_enabled);
 assert(m.wheel_goal[0]==0&&m.wheel_goal[1]==611&&m.braked[0]&&!m.braked[1]);
 for(unsigned t=0;t<500;++t){step(0,0);assert(m.wheel[0].duty==0&&m.wheel[0].target_pulses==0&&m.braked[0]);}
 assert(m.wheel[1].target_pulses==34&&m.wheel[1].duty>0);
 for(unsigned t=1;t<=611;++t){
  step(0,1);assert(m.braked[0]&&m.wheel[0].duty==0&&m.sync_correction==0);
  if(t<611)assert(!m.braked[1]);
 }
 assert(m.phase==DM_BRAKE&&m.braked[1]&&m.wheel[1].duty==0&&!m.decelerating[1]);
 assert(m.goal_trigger[1]==611&&m.at_brake[1]==611);
 step(0,4);for(unsigned t=0;t<250&&m.phase==DM_BRAKE;++t)step(0,0);
 assert(m.phase==DM_STOPPED&&motion_sequence_stage()==MS_DONE&&g_motor_pulses[1]==615&&g_motor_pulses[0]==0);
 for(unsigned i=0;i<3000;++i)step(0,0);assert(motion_sequence_stage()==MS_DONE&&!motion_busy(&m));
 /* A loaded right wheel completes while the inactive left wheel never trips stall. */
 init();double pos=0,speed=0;
 for(unsigned t=0;t<14000&&motion_busy(&m);++t){
  float u=m.wheel[1].duty;assert(m.braked[0]&&m.wheel[0].duty==0&&u>=0);
  double eq=m.enable&&!m.braked[1]?fmax(0,(u-.35)*250):0;
  speed+=(eq-speed)/(m.braked[1]?12:80);pos+=speed/50.;
  r=(uint16_t)(unsigned)pos;motion_tick(&m,0,r,true);assert(m.error==DM_OK);
 }
 assert(motion_sequence_stage()==MS_DONE&&m.goal_trigger[1]>=611&&m.actual[1]>=611);
 printf("Pivot loaded model threshold=%ld final=%ld (software only)\n",(long)m.goal_trigger[1],(long)m.actual[1]);
 init();motion_stop(&m);for(unsigned i=0;i<150;++i)step(0,0);assert(!m.enable&&!motion_busy(&m));
 init();for(unsigned i=0;i<2200&&m.running;++i)step(0,0);assert(m.error==DM_STALL);
 init();motion_tick(&m,0,0,false);assert(m.error==DM_DRIVER);
 DualMotorConfig c;dual_motor_default_config(&c);DualMotor x;assert(dual_motor_init(&x,&c,0,0));
 assert(!dual_motor_start_wheels(&x,1,0,0)&&!dual_motor_start_wheels(&x,1,-1,611));
 puts("PASS: left wheel always braked, right-only positional PD target34, 611 threshold immediate brake, true final counts, no repeat, inactive-wheel protection, stop/driver/stall");
}
