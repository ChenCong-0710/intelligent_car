#ifndef MOTION_PD_H
#define MOTION_PD_H
#include <stdbool.h>
#include <stdint.h>
#define MOTION_SPEED_MS 50U
#define MOTION_TARGET_PULSES 50.0f
#define MOTION_GOAL_PULSES 2000
#define MOTION_START_RAMP_MS 500U
#define MOTION_DUTY_SLEW .06f
#define MOTION_MAX_DUTY .85f
#define MOTION_KP .004f
#define MOTION_KD .0015f
/* Units are raw A-channel edges; no wheel circumference or speed conversion. */
typedef enum {MP_IDLE,MP_RUN,MP_BRAKE,MP_PAUSE,MP_STOPPING,MP_STOPPED,MP_FAULT} MotionPhase;
typedef enum {MP_OK,MP_DRIVER,MP_ENCODER,MP_DIRECTION,MP_STALL,MP_TIMEOUT,MP_NOT_SETTLED,MP_OVERSPEED,MP_WATCHDOG} MotionError;
typedef struct {
 uint16_t previous_counter;
 int64_t total;
 int32_t sample_sum;
 float speed_pulses,target_pulses,previous_error,derivative,duty;
 uint32_t wrong_since,stall_since,overspeed_since;
} PdWheel;
typedef struct {
 MotionPhase phase;MotionError error;
 bool running,enable,braked[2];
 unsigned action,sample_ms;
 uint32_t now,since,drive_since,stopped_since;
 int64_t origin[2];
 int32_t actual[2],at_brake[2],forward_result[2];
 PdWheel wheel[2];
} MotionPd;
extern volatile uint32_t g_motor_pulses[2];
void motion_init(MotionPd *m,uint16_t left,uint16_t right);
void motion_start(MotionPd *m);
void motion_stop(MotionPd *m);
void motion_trip(MotionPd *m,MotionError error);
void motion_tick(MotionPd *m,uint16_t left,uint16_t right,bool driver_ok);
float motion_pulse_duty(unsigned wheel,float target,float measured,float error_delta,float lag);
const char *motion_action_name(unsigned action);
const char *motion_phase_name(MotionPhase phase);
const char *motion_error_name(MotionError error);
#endif
