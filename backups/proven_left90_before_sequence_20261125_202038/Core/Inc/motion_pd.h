#ifndef MOTION_PD_H
#define MOTION_PD_H
#include "../../portable/dual_motor/dual_motor.h"
typedef DualMotor MotionPd;
typedef DualMotorWheel PdWheel;
typedef DualMotorPhase MotionPhase;
typedef DualMotorError MotionError;
#define MOTION_SPEED_MS 50U
#define MOTION_TARGET_PULSES 50.0f
#define MOTION_GOAL_PULSES 2000
#define MOTION_START_RAMP_MS 500U
#define MOTION_DUTY_SLEW .06f
#define MOTION_MAX_DUTY .85f
#define MOTION_KP .004f
#define MOTION_KD .0015f
#ifndef MOTION_SYNC_ENABLED
#define MOTION_SYNC_ENABLED 1
#endif
#define MOTION_SYNC_COUNT_KP .55f
#define MOTION_SYNC_RATE_KD .65f
#define MOTION_SYNC_MAX 12.0f
#define MOTION_SYNC_SLEW 2.0f
#define MOTION_SYNC_DEADBAND 1.0f
#define MP_IDLE DM_IDLE
#define MP_RUN DM_RUN
#define MP_BRAKE DM_BRAKE
#define MP_PAUSE DM_PAUSE
#define MP_STOPPING DM_STOPPING
#define MP_STOPPED DM_STOPPED
#define MP_FAULT DM_FAULT
#define MP_OK DM_OK
#define MP_DRIVER DM_DRIVER
#define MP_ENCODER DM_ENCODER
#define MP_DIRECTION DM_DIRECTION
#define MP_STALL DM_STALL
#define MP_TIMEOUT DM_TIMEOUT
#define MP_NOT_SETTLED DM_NOT_SETTLED
#define MP_OVERSPEED DM_OVERSPEED
#define MP_WATCHDOG DM_WATCHDOG
extern volatile uint32_t g_motor_pulses[2];
void motion_init(MotionPd *m,uint16_t left,uint16_t right);
typedef enum {MS_IDLE,MS_FORWARD,MS_WAIT,MS_REVERSE,MS_DONE,MS_CANCELLED,MS_FAULT} MotionSequenceStage;
bool motion_busy(const MotionPd *m);
MotionSequenceStage motion_sequence_stage(void);
const char *motion_sequence_name(void);
uint32_t motion_wait_remaining(const MotionPd *m);
extern int32_t g_motion_results[2][2];
/* Nominal geometry for body pivot about the stationary left wheel.
 * A-only rising+falling=420/turn estimated from previous x4=840/turn. */
#define MOTION_PIVOT_ANGLE_DEG 90.0f
#define MOTION_PIVOT_TRACK_MM 132.0f
#define MOTION_PIVOT_WHEEL_MM 43.0f
#define MOTION_PIVOT_A_EDGES_PER_TURN 420.0f
/* Measured: nominal645 threshold produced about95deg at right target34. */
#define MOTION_PIVOT_RIGHT_CAL_SCALE (90.0f/95.0f)
/* Initial left estimate borrowed from right; left calibration still pending. */
#define MOTION_PIVOT_LEFT_CAL_SCALE (90.0f/95.0f)
#define MOTION_PIVOT_ACTIVE_WHEEL 0U
#define MOTION_PIVOT_CAL_SCALE (MOTION_PIVOT_ACTIVE_WHEEL==0U?MOTION_PIVOT_LEFT_CAL_SCALE:MOTION_PIVOT_RIGHT_CAL_SCALE)
int32_t motion_pivot_counts(float body_angle_deg);
#define MOTION_PIVOT_STEPS 1U
#define MOTION_PIVOT_WAIT_MS 5000U
extern volatile int32_t g_pivot_net_counts[2];
extern int32_t g_pivot_stage_results[MOTION_PIVOT_STEPS];
unsigned motion_pivot_step(void);
unsigned motion_pivot_target_degrees(void);
int32_t motion_pivot_absolute_goal(void);
int32_t motion_pivot_segment_goal(void);
void motion_start(MotionPd *m);
void motion_stop(MotionPd *m);
void motion_trip(MotionPd *m,MotionError error);
void motion_tick(MotionPd *m,uint16_t left,uint16_t right,bool driver_ok);
float motion_pulse_duty(unsigned wheel,float target,float measured,float error_delta,float lag);
const char *motion_action_name(unsigned action);
const char *motion_phase_name(MotionPhase phase);
const char *motion_error_name(MotionError error);
#endif
