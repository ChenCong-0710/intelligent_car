#include "car_motion.h"
#include <math.h>
#include <string.h>
/* User-confirmed calibration is the only source for commanded turn angles.
   Intermediate angles interpolate; do not replace with wheelbase geometry. */
static const float calibrated_angles[]={0,30,60,90,120};
static const int32_t calibrated_counts[2][5]={{0,175,407,611,814},{0,204,407,611,814}};
int32_t car_motion_pivot_counts(unsigned wheel,float angle){
 if(wheel>1||!isfinite(angle)||angle<=0||angle>120)return 0;
 for(unsigned i=1;i<5;++i)if(angle<=calibrated_angles[i]){
  float t=(angle-calibrated_angles[i-1])/(calibrated_angles[i]-calibrated_angles[i-1]);
  return (int32_t)lroundf(calibrated_counts[wheel][i-1]+t*(calibrated_counts[wheel][i]-calibrated_counts[wheel][i-1]));
 }
 return 0;
}
static float degrees_for_counts(unsigned wheel,float counts){
 float sign=counts<0?-1.f:1.f,x=fabsf(counts);
 /* Reporting coast beyond the last anchor uses its last calibrated slope;
    commanding beyond the120-degree measured range is not permitted here. */
 for(unsigned i=1;i<5;++i)if(x<=calibrated_counts[wheel][i]||i==4){
  float t=(x-calibrated_counts[wheel][i-1])/(calibrated_counts[wheel][i]-calibrated_counts[wheel][i-1]);
  return sign*(calibrated_angles[i-1]+t*(calibrated_angles[i]-calibrated_angles[i-1]));
 }
 return 0;
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
bool car_motion_start_scan(CarMotion *c){return car_motion_start_scan_direction(c,1);}
bool car_motion_start_scan_direction(CarMotion *c,int sign){
 if(!ready(c)||(sign!=1&&sign!=-1))return false;
 straight_profile(&c->motor.config);frame(c,1,1);
 if(!dual_motor_start_continuous(&c->motor,sign))return false;
 c->desired_counts=0;c->kind=CAR_STRAIGHT;c->direction=sign;c->cancelled=false;c->requested_deg=0;return true;
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
 if(c->kind!=CAR_PIVOT_LEFT&&c->kind!=CAR_PIVOT_RIGHT)return 0;
 float sign=(c->pivot_wheel==1?1.f:-1.f)*c->direction;
 return sign*degrees_for_counts(c->pivot_wheel,(float)c->motor.actual[c->pivot_wheel]);
}

float car_motion_wheel_angle(const CarMotion *c,unsigned w){
 if(w>1||(c->kind!=CAR_PIVOT_LEFT&&c->kind!=CAR_PIVOT_RIGHT))return 0;
 return c->direction*degrees_for_counts(w,(float)c->motor.actual[w]);
}

/* Change only the requested cruise speed; preserve the validated controller. */
bool car_motion_start_edge_scan(CarMotion *c,int sign){
 if(!car_motion_start_scan_direction(c,sign))return false;
 c->motor.config.cruise_target=sign<0?25:20;return true;
}
bool car_motion_start_center_back(CarMotion *c,int32_t counts){
 if(!car_motion_start_straight(c,-1,counts))return false;
 c->motor.config.cruise_target=25;return true;
}

bool car_motion_start_front_traverse(CarMotion *c){
 if(!car_motion_start_scan_direction(c,1))return false;
 c->motor.config.cruise_target=30;return true;
}
