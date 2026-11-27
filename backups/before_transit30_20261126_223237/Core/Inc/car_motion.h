#ifndef CAR_MOTION_H
#define CAR_MOTION_H
#include "../../portable/dual_motor/dual_motor.h"
typedef enum {CAR_NONE,CAR_STRAIGHT,CAR_PIVOT_LEFT,CAR_PIVOT_RIGHT} CarMotionKind;
typedef struct {DualMotor motor;CarMotionKind kind;unsigned pivot_wheel;int direction;bool cancelled;float requested_deg;uint16_t raw_counter[2];int8_t frame_sign[2];int32_t desired_counts;} CarMotion;
/* Raw A-channel BOTH-edge free-running counters, physical forward positive. */
bool car_motion_init(CarMotion *car,uint16_t left,uint16_t right);
void car_motion_tick_1ms(CarMotion *car,uint16_t left,uint16_t right,bool driver_ok);
bool car_motion_start_straight(CarMotion *car,int direction,int32_t counts);
bool car_motion_start_pivot(CarMotion *car,unsigned moving_wheel,int direction,float body_angle_deg);
bool car_motion_start_scan(CarMotion *car);
/* Same validated translation profile, open-ended in either direction. */
bool car_motion_start_scan_direction(CarMotion *car,int direction);
/* Post-approach translation: forward20/reverse25 A edges/50ms; same PD/sync/brake. */
bool car_motion_start_edge_scan(CarMotion *car,int direction);
bool car_motion_start_center_back(CarMotion *car,int32_t counts);
bool car_motion_finish_after_counts(CarMotion *car,int32_t nominal_remaining);
float car_motion_output_duty(const CarMotion *car,unsigned wheel);
void car_motion_get_status(const CarMotion *car,DualMotorStatus *status);
#define CAR_BRAKE_LEAD_COUNTS 10
#define CAR_MM_PER_EDGE (43.0f*3.14159265359f/420.0f)
bool car_motion_finish_straight(CarMotion *car);
void car_motion_stop(CarMotion *car);
void car_motion_trip(CarMotion *car,DualMotorError error);
bool car_motion_done(const CarMotion *car);
int32_t car_motion_pivot_counts(unsigned wheel,float angle_deg);
float car_motion_estimated_angle(const CarMotion *car);
/* Yesterday's calibrated single-wheel BODY-pivot angle equivalent, signed
   physical forward+, reverse-. Not mechanical shaft degrees / IMU heading. */
float car_motion_wheel_angle(const CarMotion *car,unsigned wheel);
#endif
