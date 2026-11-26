#include "car_motion.h"
#include <math.h>
#include <string.h>
static const float left_angles[]={0,30,60,90,120};
static const int32_t left_counts[]={0,175,407,611,814};
/* Preserve TEST's wheel-specific angle calibration, not a track-width fudge. */
static float nominal_counts(float angle){return angle*132.f*420.f*(90.f/95.f)/(180.f*43.f);}
int32_t car_motion_pivot_counts(unsigned wheel,float angle){
 if(wheel>1||!isfinite(angle)||angle<=0||angle>180)return 0;
 if(wheel==0&&angle<=120){
  for(unsigned i=1;i<5;++i)if(angle<=left_angles[i]){
   float t=(angle-left_angles[i-1])/(left_angles[i]-left_angles[i-1]);
   return (int32_t)lroundf(left_counts[i-1]+t*(left_counts[i]-left_counts[i-1]));
  }
 }
 return (int32_t)lroundf(nominal_counts(angle));
}
static float degrees_for_counts(unsigned wheel,float counts){
 float sign=counts<0?-1.f:1.f,x=fabsf(counts);
 if(wheel==0&&x<=814){
  for(unsigned i=1;i<5;++i)if(x<=left_counts[i]){
   float t=(x-left_counts[i-1])/(left_counts[i]-left_counts[i-1]);
   return sign*(left_angles[i-1]+t*(left_angles[i]-left_angles[i-1]));
  }
 }
 return sign*x/nominal_counts(1.f);
}
bool car_motion_init(CarMotion *c,uint16_t l,uint16_t r){memset(c,0,sizeof(*c));return dual_motor_init(&c->motor,0,l,r);}
void car_motion_tick_1ms(CarMotion *c,uint16_t l,uint16_t r,bool ok){dual_motor_tick_1ms(&c->motor,l,r,ok);}
static bool ready(const CarMotion *c){return !c->motor.running&&c->motor.phase!=DM_FAULT&&c->motor.phase!=DM_STOPPING;}
bool car_motion_start_straight(CarMotion *c,int sign,int32_t counts){
 if(!ready(c)||(sign!=1&&sign!=-1)||counts<=0)return false;
 DualMotorConfig profile;dual_motor_default_config(&profile);
 /* Explicit application preference: no STALL/DIRECTION auto-stop. */
 profile.stall_protection_enabled=false;profile.direction_protection_enabled=false;
 profile.cruise_target=40;profile.soft_stop_ms=300;
 profile.sync_count_kp=.45f;profile.sync_rate_kd=.65f;profile.sync_max=8;
 profile.sync_slew=.5f;profile.sync_filter_alpha=.25f;profile.duty_slew=.04f;
 profile.goal_counts=counts;c->motor.config=profile;
 if(!dual_motor_start_direction(&c->motor,sign))return false;
 c->kind=CAR_STRAIGHT;c->direction=sign;c->cancelled=false;c->requested_deg=0;return true;
}
bool car_motion_start_pivot(CarMotion *c,unsigned wheel,int sign,float degrees){
 int32_t counts=car_motion_pivot_counts(wheel,degrees);
 if(!ready(c)||(sign!=1&&sign!=-1)||!counts)return false;
 DualMotorConfig profile;dual_motor_default_config(&profile);
 /* Explicit application preference: no STALL/DIRECTION auto-stop. */
 profile.stall_protection_enabled=false;profile.direction_protection_enabled=false;
 profile.cruise_target=34;profile.sync_enabled=false;profile.soft_stop_ms=0;profile.duty_slew=.04f;profile.goal_counts=counts;
 c->motor.config=profile;
 if(!dual_motor_start_wheels(&c->motor,sign,wheel==0?counts:0,wheel==1?counts:0))return false;
 c->kind=wheel==0?CAR_PIVOT_LEFT:CAR_PIVOT_RIGHT;c->pivot_wheel=wheel;
 c->direction=sign;c->cancelled=false;c->requested_deg=degrees;return true;
}
/* End an open-ended traverse with the SAME per-wheel soft-stop path as
   a count goal. Call with encoder/tick IRQs masked, like start functions. */
bool car_motion_finish_straight(CarMotion *c){
 if(c->kind!=CAR_STRAIGHT||c->cancelled||c->motor.phase==DM_FAULT)return false;
 if(car_motion_done(c))return true;
 if(!c->motor.running)return false;
 for(unsigned w=0;w<2;++w)c->motor.wheel_goal[w]=c->motor.actual[w];
 return true;
}
void car_motion_stop(CarMotion *c){c->cancelled=true;dual_motor_stop(&c->motor);}
void car_motion_trip(CarMotion *c,DualMotorError e){dual_motor_trip(&c->motor,e);}
bool car_motion_done(const CarMotion *c){return c->kind!=CAR_NONE&&!c->cancelled&&c->motor.phase==DM_STOPPED&&c->motor.error==DM_OK;}
float car_motion_estimated_angle(const CarMotion *c){
 if(c->kind!=CAR_PIVOT_LEFT&&c->kind!=CAR_PIVOT_RIGHT)return 0;
 float sign=(c->pivot_wheel==1?1.f:-1.f)*c->direction;
 return sign*degrees_for_counts(c->pivot_wheel,(float)c->motor.actual[c->pivot_wheel]);
}
