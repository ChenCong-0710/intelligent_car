#ifndef DUAL_MOTOR_H
#define DUAL_MOTOR_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* All speeds are raw signed A-channel edges per speed_ms, duty is 0..1.
 * These defaults preserve the validated A50_SYNC_V4 controller. */
typedef struct {
 uint32_t speed_ms,start_ramp_ms,action_timeout_ms;
 uint32_t settle_ms,brake_timeout_ms,manual_stop_ms;
 uint32_t stall_grace_ms,stall_ms,direction_ms,overspeed_ms;
 int32_t goal_counts,max_delta_1ms;
 float cruise_target,duty_slew,max_duty,speed_kp,speed_kd;
 bool sync_enabled;
 float sync_count_kp,sync_rate_kd,sync_max,sync_slew,sync_deadband;
 float sync_filter_alpha,derivative_alpha;
 float direction_threshold,stall_threshold,overspeed_threshold;
 float ff_base[2][2],ff_slope[2][2];
} DualMotorConfig;
typedef enum {DM_IDLE,DM_RUN,DM_BRAKE,DM_PAUSE,DM_STOPPING,DM_STOPPED,DM_FAULT} DualMotorPhase;
typedef enum {DM_OK,DM_DRIVER,DM_ENCODER,DM_DIRECTION,DM_STALL,DM_TIMEOUT,DM_NOT_SETTLED,DM_OVERSPEED,DM_WATCHDOG,DM_CONFIG} DualMotorError;
typedef struct {
 uint16_t previous_counter;
 int64_t total;
 int32_t sample_sum;
 float speed_pulses,target_pulses,previous_error,derivative,duty;
 uint32_t wrong_since,stall_since,overspeed_since;
} DualMotorWheel;
typedef struct {
 DualMotorPhase phase;DualMotorError error;
 bool running,enable,braked[2];
 DualMotorConfig config;
 uint32_t counts[2];
 unsigned action,sample_ms;
 int32_t sync_error;float sync_rate,sync_correction;
 uint32_t now,since,drive_since,stopped_since;
 int64_t origin[2];
 int32_t actual[2],at_brake[2],forward_result[2];
 DualMotorWheel wheel[2];
} DualMotor;
typedef struct {
 bool enable,brake[2],running;
 float duty[2],speed[2],target[2],sync_correction;
 uint32_t counts[2];
 int32_t signed_counts[2],sync_error;
 DualMotorPhase phase;DualMotorError error;
} DualMotorStatus;

/* No allocation, HAL, GPIO, timer, display or global instance dependencies. */
void dual_motor_default_config(DualMotorConfig *config);
bool dual_motor_config_valid(const DualMotorConfig *config);
/* NULL config selects V4 defaults. Invalid config fails closed; init returns false. */
bool dual_motor_init(DualMotor *motor,const DualMotorConfig *config,uint16_t left,uint16_t right);
/* Forward once using config.goal_counts and config.cruise_target. No reverse. */
void dual_motor_start(DualMotor *motor);
void dual_motor_stop(DualMotor *motor);
void dual_motor_trip(DualMotor *motor,DualMotorError error);
/* Call exactly once each 1ms with FREE-RUNNING signed-direction 16-bit counters.
 * Counter rollover is handled internally. Do not reset counters each sample.
 * driver_ok=false is a latched fault. Continue ticks while braking/stopped. */
void dual_motor_tick_1ms(DualMotor *motor,uint16_t left,uint16_t right,bool driver_ok);
/* Caller must protect snapshots/start/stop from a concurrent tick ISR. */
void dual_motor_get_status(const DualMotor *motor,DualMotorStatus *status);
float dual_motor_pd_duty(const DualMotorConfig *config,unsigned wheel,float target,float measured,float error_delta);
const char *dual_motor_action_name(unsigned action);
const char *dual_motor_phase_name(DualMotorPhase phase);
const char *dual_motor_error_name(DualMotorError error);

#ifdef __cplusplus
}
#endif
#endif
