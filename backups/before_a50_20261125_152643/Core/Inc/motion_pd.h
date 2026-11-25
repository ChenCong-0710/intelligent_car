#ifndef MOTION_PD_H
#define MOTION_PD_H
#include <stdbool.h>
#include <stdint.h>
#define MOTION_ACTIONS 8U
#define MOTION_CPR 840.0f
#define MOTION_SPEED_MS 20U
#define MOTION_TARGET_PULSES 18.0f
#define MOTION_TRACK_MM 132.0f
#define MOTION_WHEEL_MM 43.0f
#define MOTION_KP 0.0200f /* duty / (count per 20ms) */
#define MOTION_KD 0.0080f /* duty / sample error difference */
#define MOTION_SYNC_KP 0.0010f
#define MOTION_MAX_DUTY 0.85f
#define MOTION_STALL_GRACE_MS 2500U
#define MOTION_STALL_MS 1500U
#define MOTION_ACTION_MS 5000U
#define MOTION_TRIM_MS 200U
#define MOTION_PAUSE_MS 2000U
typedef enum { MP_IDLE,MP_RUN,MP_BRAKE,MP_TRIM,MP_PAUSE,MP_STOPPING,MP_STOPPED,MP_FAULT } MotionPhase;
typedef enum { MP_OK,MP_DRIVER,MP_ENCODER,MP_DIRECTION,MP_STALL,MP_TIMEOUT,MP_POSITION,MP_NOT_SETTLED,MP_OVERSPEED,MP_WATCHDOG } MotionError;
typedef struct {
 float derivative,duty;
 float speed_pulses,target_pulses,previous_error;
 int32_t sample_sum;
 uint16_t previous_counter;
 int64_t total;
 uint32_t wrong_since,stall_since,overspeed_since;
} PdWheel;
typedef struct {
 int32_t actual[2],goal[2],coast[2];
 float mean_pulses[2],peak_error[2],distance_mm,angle_deg;
 uint32_t duration_ms,recovery_ms[2],cruise_samples[2];
 bool assessed[2],recovery_valid[2],passed;
 uint8_t trims;
} MotionResult;
typedef struct {
 MotionPhase phase;MotionError error;
 bool running,braked[2],enable,learn_pending,position_warning;
 uint32_t position_warnings;
 uint8_t action,trim_round,index,filled,short_index,short_filled;
 uint32_t now,since,action_since,drive_since,stopped_since,cycle,event;
 uint32_t disturbed_since[2],good_since[2],max_recovery[2],cruise_n[2];
 bool disturbance[2],recovery_valid[2],assessed[2];
 float pulse_sum[2],peak_error[2];
 uint16_t lead[2][2];
 int64_t origin[2];
 int32_t goal[2],actual[2],at_brake[2],coast_counts[2],trim_origin[2];
 int8_t trim_sign[2];
 PdWheel wheel[2];
 MotionResult result[MOTION_ACTIONS];
 uint8_t sample_ms;
 int32_t distance_pulses,turn_pulses,last_straight[2];

} MotionPd;
void motion_init(MotionPd *m,uint16_t left,uint16_t right);
void motion_start(MotionPd *m);
void motion_stop(MotionPd *m);
void motion_trip(MotionPd *m,MotionError error);
/* Exactly 1ms per call. Non-overlapping speed sampling and positional PD every 20ms. */
void motion_tick(MotionPd *m,uint16_t left,uint16_t right,bool driver_ok);
float motion_pulse_duty(unsigned wheel,float target,float measured,float error_delta,float lag);
extern volatile int32_t g_distance_pulses;
const char *motion_action_name(unsigned action);
const char *motion_phase_name(MotionPhase phase);
const char *motion_error_name(MotionError error);
#endif
