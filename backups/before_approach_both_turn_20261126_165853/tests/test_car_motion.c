#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void tick(CarMotion *c,int sign,unsigned left,unsigned right){
 for(unsigned i=1;i<=1500&&c->motor.running;++i){
  uint16_t l=(uint16_t)(sign*(int)(i<left?i:left));uint16_t r=(uint16_t)(sign*(int)(i<right?i:right));
  car_motion_tick_1ms(c,l,r,true);
 }
}
int main(void){
 CarMotion c;assert(car_motion_init(&c,0,0));assert(car_motion_pivot_counts(0,30)==175&&car_motion_pivot_counts(1,30)==204);
 assert(car_motion_pivot_counts(0,90)==611&&car_motion_pivot_counts(1,90)==611);
 assert(!car_motion_start_pivot(&c,2,-1,30)&&!car_motion_start_pivot(&c,0,0,30));
 for(unsigned w=0;w<2;++w)for(int sign=-1;sign<=1;sign+=2){
  assert(car_motion_init(&c,0,0));assert(car_motion_start_pivot(&c,w,sign,30));
  assert(c.motor.config.cruise_target==34&&!c.motor.config.sync_enabled&&c.motor.config.soft_stop_ms==0);
  assert(c.motor.braked[1-w]&&c.motor.wheel_goal[1-w]==0);
  unsigned n=(unsigned)c.motor.wheel_goal[w];tick(&c,sign,w==0?n:0,w==1?n:0);
  assert(car_motion_done(&c));float body=(w==1?1.f:-1.f)*sign*30;
  assert(fabsf(car_motion_estimated_angle(&c)-body)<.2f);
 }
 for(int sign=-1;sign<=1;sign+=2){
  assert(car_motion_init(&c,0,0));assert(car_motion_start_straight(&c,sign,200));
  assert(c.motor.config.cruise_target==40&&c.motor.config.sync_enabled&&c.motor.config.soft_stop_ms==300);
  tick(&c,sign,200,200);assert(car_motion_done(&c)&&c.motor.counts[0]==200&&c.motor.counts[1]==200);
 }
 assert(car_motion_init(&c,0,0));assert(car_motion_start_pivot(&c,1,-1,90));car_motion_stop(&c);
 for(unsigned i=0;i<200;++i)car_motion_tick_1ms(&c,0,0,true);assert(!car_motion_done(&c)&&!c.motor.enable);
 assert(car_motion_init(&c,0,0));assert(car_motion_start_pivot(&c,0,-1,90));car_motion_tick_1ms(&c,0,0,false);assert(c.motor.error==DM_DRIVER&&!c.motor.enable);
 /* Restore real STALL and DIRECTION protection for finite and scan actions. */
 for(unsigned kind=0;kind<3;++kind)for(unsigned wrong=0;wrong<2;++wrong){
  assert(car_motion_init(&c,0,0));
  if(kind==0)assert(car_motion_start_straight(&c,-1,450));
  else if(kind==1)assert(car_motion_start_pivot(&c,1,-1,90));
  else assert(car_motion_start_scan(&c));
  for(unsigned t=0;t<2600&&c.motor.running;++t){
   int value=wrong?(int)(t/5):0;if(kind==2)value=-value;
   car_motion_tick_1ms(&c,kind==1?0:(uint16_t)value,(uint16_t)value,true);
  }
  assert(c.motor.error==(wrong?DM_DIRECTION:DM_STALL));
 }
 /* The scan has no 620 cap/15s deadline; the SAME straight synchronization runs. */
 assert(car_motion_init(&c,0,0));assert(car_motion_start_scan(&c));
 for(unsigned t=1;t<=18000;++t){car_motion_tick_1ms(&c,(uint16_t)t,(uint16_t)t,true);assert(c.motor.error==DM_OK);}
 assert(c.motor.running&&c.motor.counts[0]==18000);
 assert(car_motion_finish_straight(&c));
 for(unsigned t=0;t<700;++t)car_motion_tick_1ms(&c,18000,18000,true);
 assert(car_motion_done(&c));
 puts("PASS: forward/reverse40, left/right single-wheel34 both directions, independent angle calibration, real-count finish, drive fault and cancel");
}
