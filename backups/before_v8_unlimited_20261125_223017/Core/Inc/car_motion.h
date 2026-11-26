#ifndef CAR_MOTION_H
#define CAR_MOTION_H
#include "../../portable/dual_motor/dual_motor.h"
typedef enum {CAR_NONE,CAR_STRAIGHT,CAR_PIVOT_LEFT,CAR_PIVOT_RIGHT} CarMotionKind;
typedef struct {DualMotor motor;CarMotionKind kind;unsigned pivot_wheel;int direction;bool cancelled;float requested_deg;} CarMotion;
/* Normalized A-channel BOTH-edge free-running counters, forward positive. */
bool car_motion_init(CarMotion *car,uint16_t left,uint16_t right);
void car_motion_tick_1ms(CarMotion *car,uint16_t left,uint16_t right,bool driver_ok);
bool car_motion_start_straight(CarMotion *car,int direction,int32_t counts);
bool car_motion_start_pivot(CarMotion *car,unsigned moving_wheel,int direction,float body_angle_deg);
bool car_motion_finish_straight(CarMotion *car);
void car_motion_stop(CarMotion *car);
void car_motion_trip(CarMotion *car,DualMotorError error);
bool car_motion_done(const CarMotion *car);
int32_t car_motion_pivot_counts(unsigned wheel,float angle_deg);
float car_motion_estimated_angle(const CarMotion *car);
#endif
