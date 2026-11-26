#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void tick_to_end(CarMotion *c,uint16_t raw[2],unsigned coast){
 unsigned extra[2]={0,0};
 for(unsigned t=0;t<4000&&c->motor.running;++t){
  for(unsigned w=0;w<2;++w){
   if(!c->motor.braked[w]||extra[w]<coast){
    if(c->motor.braked[w])++extra[w];
    raw[w]=(uint16_t)(raw[w]+c->direction*c->frame_sign[w]);
   }
  }
  car_motion_tick_1ms(c,raw[0],raw[1],true);assert(c->motor.error==DM_OK);
 }
 assert(car_motion_done(c));
}
int main(void){
 CarMotion c;assert(car_motion_init(&c,0,0));
 assert(car_motion_pivot_counts(0,30)==175&&car_motion_pivot_counts(1,30)==204);
 assert(car_motion_pivot_counts(0,90)==611&&car_motion_pivot_counts(1,90)==611);
 for(int sign=-1;sign<=1;sign+=2){
  uint16_t raw[2]={0,0};assert(car_motion_init(&c,0,0));assert(car_motion_start_straight(&c,sign,2000));
  assert(c.motor.config.cruise_target==40&&c.motor.config.soft_stop_ms==0);
  assert(c.motor.config.sync_slew==1&&c.motor.config.sync_count_kp==.45f&&c.motor.config.sync_rate_kd==.65f);
  assert(c.motor.config.start_ramp_ms==500&&c.motor.config.speed_ms==50);
  assert(c.motor.wheel_goal[0]==1990&&c.motor.wheel_goal[1]==1990);
  tick_to_end(&c,raw,10);assert(c.motor.counts[0]==2000&&c.motor.counts[1]==2000);
 }
 /* Measured anchors, interpolation, rollover and transition back to straight. */
 assert(car_motion_pivot_counts(0,60)==407&&car_motion_pivot_counts(1,120)==814);
 assert(car_motion_pivot_counts(0,120)==814&&car_motion_pivot_counts(1,121)==0);
 assert(car_motion_pivot_counts(0,15)==88&&car_motion_pivot_counts(1,15)==102);
 for(int sign=-1;sign<=1;sign+=2){
  uint16_t raw[2]={65530,15};assert(car_motion_init(&c,raw[0],raw[1]));
  assert(car_motion_start_pivot(&c,1,sign,30));assert(c.motor.braked[0]);
  for(unsigned t=0;t<2000&&c.motor.running;++t){
   if(!c.motor.braked[1])raw[1]=(uint16_t)(raw[1]+sign);
   car_motion_tick_1ms(&c,raw[0],raw[1],true);assert(c.motor.error==DM_OK);
  }
  assert(car_motion_done(&c)&&fabsf(car_motion_wheel_angle(&c,1)-sign*30)<.3f);
  assert(car_motion_start_straight(&c,1,100));assert(c.motor.actual[0]==0&&c.motor.actual[1]==0);
  for(unsigned t=0;t<10;++t){++raw[0];++raw[1];car_motion_tick_1ms(&c,raw[0],raw[1],true);}
  assert(c.motor.actual[0]==10&&c.motor.actual[1]==10&&c.motor.error==DM_OK);
  car_motion_stop(&c);for(unsigned t=0;t<200;++t)car_motion_tick_1ms(&c,raw[0],raw[1],true);
  assert(!car_motion_done(&c));
 }
 /* Restore/retain legacy single-wheel calibration without a frame artifact. */
 for(unsigned w=0;w<2;++w){uint16_t raw[2]={0,0};assert(car_motion_init(&c,0,0));
  assert(car_motion_start_pivot(&c,w,-1,30));assert(c.motor.braked[1-w]);
  for(unsigned t=0;t<1000&&c.motor.running;++t){if(!c.motor.braked[w])--raw[w];car_motion_tick_1ms(&c,raw[0],raw[1],true);}
  assert(car_motion_done(&c)&&fabsf(fabsf(car_motion_estimated_angle(&c))-30)<.3f);
 }
 /* A 20mm remaining path uses a10-edge lead; ramp is not reset on planning. */
 assert(car_motion_init(&c,0,0));assert(car_motion_start_scan(&c));
 for(unsigned t=1;t<=1000;++t)car_motion_tick_1ms(&c,t,t,true);
 assert(car_motion_finish_after_counts(&c,62));assert(c.motor.wheel_goal[0]==1052&&c.desired_counts==1062);
 assert(c.motor.now-c.motor.drive_since==c.motor.config.start_ramp_ms);
 assert(!car_motion_finish_after_counts(&c,-1));
 car_motion_tick_1ms(&c,1000,1000,false);assert(c.motor.error==DM_DRIVER);
 puts("PASS: imported TEST A40 profile/brake10; calibrated single-wheel forward/reverse turns; encoder rollover and coordinate transitions; single-wheel calibration retained; 20mm planner and hardware stop");
}
