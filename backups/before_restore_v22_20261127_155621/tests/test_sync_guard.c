#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void tick(CarMotion *c,uint16_t r[2]){car_motion_tick_1ms(c,r[0],r[1],true);assert(c->motor.error==DM_OK);}
int main(void){
 for(int sign=-1;sign<=1;sign+=2)for(unsigned lead=0;lead<2;++lead){
  CarMotion c;uint16_t r[2]={65530,65530};assert(car_motion_init(&c,r[0],r[1]));assert(car_motion_start_edge_scan(&c,sign));
  for(unsigned t=0;t<600;++t)tick(&c,r);
  r[lead]=(uint16_t)(r[lead]+sign*25);tick(&c,r);
  while(c.motor.sample_ms)tick(&c,r);
  assert(c.motor.guard_phase==0&&!c.motor.guard_excess); /* Exactly25 does not arm. */
  r[lead]=(uint16_t)(r[lead]+sign);tick(&c,r);
  while(c.motor.sample_ms)tick(&c,r);
  assert(fabsf(c.motor.wheel[lead].target_pulses)<=10.01f);assert(c.motor.guard_phase==0);
  r[lead]=(uint16_t)(r[lead]+sign*14);tick(&c,r);
  for(unsigned t=0;t<151;++t)tick(&c,r);
  assert(c.motor.guard_phase==1&&c.motor.guard_hold[0]&&c.motor.guard_hold[1]);
  int64_t origin[2]={c.motor.origin[0],c.motor.origin[1]};
  for(unsigned t=0;t<200&&c.motor.guard_phase!=2;++t)tick(&c,r);
  assert(c.motor.guard_phase==2&&c.motor.guard_leader==lead&&c.motor.guard_hold[lead]&&!c.motor.guard_hold[1-lead]);
  unsigned lag=1-lead;
  for(unsigned t=0;t<38;++t){r[lag]=(uint16_t)(r[lag]+sign);tick(&c,r);}
  assert(c.motor.guard_phase==3&&c.motor.actual[lead]==40&&c.motor.actual[lag]==38);
  for(unsigned t=0;t<250&&c.motor.guard_phase;++t)tick(&c,r);
  assert(!c.motor.guard_phase&&c.motor.running&&c.motor.guard_events==1);
  assert(c.motor.origin[0]==origin[0]&&c.motor.origin[1]==origin[1]);
  assert(c.motor.actual[lead]==40&&c.motor.actual[lag]==38&&!car_motion_done(&c));
  assert(car_motion_finish_straight(&c));for(unsigned t=0;t<300&&c.motor.running;++t)tick(&c,r);
  assert(car_motion_done(&c));
 }
 /* During the entire500ms ramp, guard adds no intervention, even with40
    counted edges of error. Its150ms persistence starts at ramp completion. */
 for(int sign=-1;sign<=1;sign+=2){
  CarMotion guarded,baseline;uint16_t rr[2]={0,0};
  assert(car_motion_init(&guarded,0,0)&&car_motion_init(&baseline,0,0));
  assert(car_motion_start_edge_scan(&guarded,sign)&&car_motion_start_edge_scan(&baseline,sign));
  baseline.motor.config.straight_guard_enabled=false;rr[0]=(uint16_t)(sign*40);
  for(unsigned t=1;t<500;++t){
   tick(&guarded,rr);tick(&baseline,rr);
   assert(!guarded.motor.guard_phase&&!guarded.motor.guard_excess);
   for(unsigned w=0;w<2;++w)assert(guarded.motor.wheel[w].target_pulses==baseline.motor.wheel[w].target_pulses);
  }
  assert(guarded.motor.actual[0]==40&&guarded.motor.origin[0]==0);
  tick(&guarded,rr);assert(guarded.motor.guard_excess&&!guarded.motor.guard_phase);
  for(unsigned t=0;t<149;++t)tick(&guarded,rr);
  assert(!guarded.motor.guard_phase);tick(&guarded,rr);assert(guarded.motor.guard_phase==1);
 }
 /* Finite goals remain valid and a driver fault interrupts any correction. */
 CarMotion c;uint16_t r[2]={0,0};assert(car_motion_init(&c,0,0));assert(car_motion_start_straight(&c,1,2000));
 r[0]=40;tick(&c,r);for(unsigned t=0;t<400;++t)tick(&c,r);
 assert(!c.motor.guard_phase&&!c.motor.guard_excess&&c.motor.actual[0]==40);
 for(unsigned t=0;t<450;++t)tick(&c,r);
 assert(c.motor.guard_phase==2&&c.motor.wheel_goal[0]==1990&&c.motor.wheel_goal[1]==1990);
 car_motion_tick_1ms(&c,r[0],r[1],false);assert(c.motor.phase==DM_FAULT&&!c.motor.enable);
 assert(car_motion_init(&c,0,0));assert(car_motion_start_pivot(&c,0,-1,45));assert(!c.motor.config.straight_guard_enabled);
 r[0]=0;r[1]=0;for(unsigned t=0;t<400;++t){if(t<100)--r[0];tick(&c,r);}assert(!c.motor.guard_phase&&c.motor.wheel_goal[0]==291);
 puts("PASS guard: forward/reverse, either leader, >25 yield after start, persistent error brake, catch to2, settle/restart without count reset, external finish, finite goals, driver fault, pivot exclusion");
}
