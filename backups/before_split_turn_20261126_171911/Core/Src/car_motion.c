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
bool car_motion_init(CarMotion *c,uint16_t l,uint16_t r){
 memset(c,0,sizeof(*c));c->frame_sign[0]=c->frame_sign[1]=1;
 c->raw_counter[0]=l;c->raw_counter[1]=r;return dual_motor_init(&c->motor,0,l,r);
}
void car_motion_tick_1ms(CarMotion *c,uint16_t l,uint16_t r,bool ok){
 c->raw_counter[0]=l;c->raw_counter[1]=r;
 dual_motor_tick_1ms(&c->motor,(uint16_t)((int32_t)l*c->frame_sign[0]),
  (uint16_t)((int32_t)r*c->frame_sign[1]),ok);
}
static bool ready(const CarMotion *c){return !c->motor.running&&c->motor.phase!=DM_FAULT&&c->motor.phase!=DM_STOPPING;}
/* Call starts with tick IRQ masked. Rebase samples when changing coordinates;
   no hardware counter resets and no synthetic encoder jump on a new action. */
static void frame(CarMotion *c,int left,int right){
 c->frame_sign[0]=(int8_t)left;c->frame_sign[1]=(int8_t)right;
 for(unsigned w=0;w<2;++w)c->motor.wheel[w].previous_counter=(uint16_t)((int32_t)c->raw_counter[w]*c->frame_sign[w]);
}
static void straight_profile(DualMotorConfig *p){
 dual_motor_default_config(p);
 p->cruise_target=40;p->soft_stop_ms=0;
 p->sync_count_kp=.45f;p->sync_rate_kd=.65f;p->sync_max=8;
 p->sync_slew=1.0f;p->sync_filter_alpha=.25f;p->duty_slew=.04f;
}
bool car_motion_start_straight(CarMotion *c,int sign,int32_t counts){
 if(!ready(c)||(sign!=1&&sign!=-1)||counts<=0)return false;
 straight_profile(&c->motor.config);c->desired_counts=counts;
 c->motor.config.goal_counts=counts>CAR_BRAKE_LEAD_COUNTS?counts-CAR_BRAKE_LEAD_COUNTS:1;
 frame(c,1,1);
 if(!dual_motor_start_direction(&c->motor,sign))return false;
 c->kind=CAR_STRAIGHT;c->direction=sign;c->cancelled=false;c->requested_deg=0;return true;
}
bool car_motion_start_scan(CarMotion *c){
 if(!ready(c))return false;
 straight_profile(&c->motor.config);frame(c,1,1);
 if(!dual_motor_start_continuous(&c->motor,1))return false;
 c->desired_counts=0;c->kind=CAR_STRAIGHT;c->direction=1;c->cancelled=false;c->requested_deg=0;return true;
}
/* When the range reaches about120mm, plan the remaining path to100mm with
   TEST's ten-edge early braking allowance. Preserve the existing ramp age. */
bool car_motion_finish_after_counts(CarMotion *c,int32_t remaining){
 if(c->kind!=CAR_STRAIGHT||!c->motor.running||c->cancelled||remaining<0)return false;
 int32_t mean=(int32_t)(((int64_t)c->motor.actual[0]+c->motor.actual[1])/2);
 int32_t advance=remaining>CAR_BRAKE_LEAD_COUNTS?remaining-CAR_BRAKE_LEAD_COUNTS:0;
 if(mean>INT32_MAX-remaining)return false;
 c->desired_counts=mean+remaining;
 uint32_t age=c->motor.now-c->motor.drive_since;
 if(age>c->motor.config.start_ramp_ms)age=c->motor.config.start_ramp_ms;
 c->motor.drive_since=c->motor.now-age;c->motor.continuous=false;
 for(unsigned w=0;w<2;++w)c->motor.wheel_goal[w]=mean+advance;
 return true;
}
bool car_motion_start_inplace(CarMotion *c,float degrees){
 if(!ready(c)||!isfinite(degrees)||degrees==0||fabsf(degrees)>180)return false;
 /* Both wheels trace half the single-wheel pivot radius. Single-wheel
    empirical calibration is deliberately not reused as an in-place scale. */
 int32_t counts=(int32_t)lroundf(fabsf(degrees)*132.f*420.f/(360.f*43.f));
 if(counts<1)counts=1;
 straight_profile(&c->motor.config);c->motor.config.cruise_target=34;c->motor.config.goal_counts=counts;
 int sign=degrees>0?1:-1;frame(c,-sign,sign);
 /* Virtual forward may be physical reverse: preserve TEST's physical
    per-wheel/per-direction feedforward curves under the coordinate change. */
 for(unsigned w=0;w<2;++w)if(c->frame_sign[w]<0){
  float b=c->motor.config.ff_base[w][0],k=c->motor.config.ff_slope[w][0];
  c->motor.config.ff_base[w][0]=c->motor.config.ff_base[w][1];
  c->motor.config.ff_slope[w][0]=c->motor.config.ff_slope[w][1];
  c->motor.config.ff_base[w][1]=b;c->motor.config.ff_slope[w][1]=k;
 }
 if(!dual_motor_start_wheels(&c->motor,1,counts,counts))return false;
 c->desired_counts=counts;c->kind=CAR_INPLACE;c->direction=sign;c->requested_deg=degrees;c->cancelled=false;return true;
}
float car_motion_output_duty(const CarMotion *c,unsigned w){return w<2?c->motor.wheel[w].duty*c->frame_sign[w]:0;}
void car_motion_get_status(const CarMotion *c,DualMotorStatus *s){
 dual_motor_get_status(&c->motor,s);
 for(unsigned w=0;w<2;++w){s->duty[w]*=c->frame_sign[w];s->speed[w]*=c->frame_sign[w];s->target[w]*=c->frame_sign[w];}
}
bool car_motion_start_pivot(CarMotion *c,unsigned wheel,int sign,float degrees){
 int32_t counts=car_motion_pivot_counts(wheel,degrees);
 if(!ready(c)||(sign!=1&&sign!=-1)||!counts)return false;
 DualMotorConfig profile;dual_motor_default_config(&profile);
 profile.cruise_target=34;profile.sync_enabled=false;profile.soft_stop_ms=0;profile.duty_slew=.04f;profile.goal_counts=counts;
 c->motor.config=profile;frame(c,1,1);
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
 if(c->motor.continuous)c->motor.drive_since=c->motor.now; /* stop-phase timeout origin */
 c->motor.continuous=false;
 for(unsigned w=0;w<2;++w)c->motor.wheel_goal[w]=c->motor.actual[w];
 return true;
}
void car_motion_stop(CarMotion *c){c->cancelled=true;dual_motor_stop(&c->motor);}
void car_motion_trip(CarMotion *c,DualMotorError e){dual_motor_trip(&c->motor,e);}
bool car_motion_done(const CarMotion *c){return c->kind!=CAR_NONE&&!c->cancelled&&c->motor.phase==DM_STOPPED&&c->motor.error==DM_OK;}
float car_motion_estimated_angle(const CarMotion *c){
 if(c->kind==CAR_INPLACE)return c->direction*((float)c->motor.actual[0]+c->motor.actual[1])*.5f*360.f*43.f/(132.f*420.f);
 if(c->kind!=CAR_PIVOT_LEFT&&c->kind!=CAR_PIVOT_RIGHT)return 0;
 float sign=(c->pivot_wheel==1?1.f:-1.f)*c->direction;
 return sign*degrees_for_counts(c->pivot_wheel,(float)c->motor.actual[c->pivot_wheel]);
}
