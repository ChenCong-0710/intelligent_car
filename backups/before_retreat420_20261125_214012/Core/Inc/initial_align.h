#ifndef INITIAL_ALIGN_H
#define INITIAL_ALIGN_H
#include <stdbool.h>
#include <stdint.h>
#define ALIGN_POINTS 37U
#define ALIGN_LEFT_US 2400U
#define ALIGN_CENTRE_US 1460U
#define ALIGN_RIGHT_US 500U
#define ALIGN_SIDE_SETTLE_MS 1200U
typedef enum {ALIGN_IDLE,ALIGN_SCAN,ALIGN_FIT,ALIGN_SIDE_SETTLE,ALIGN_TURN,
 ALIGN_BRAKE,ALIGN_LOCK,ALIGN_DONE,ALIGN_FAILED,ALIGN_CANCELLED} AlignPhase;
typedef enum {ALIGN_OK,ALIGN_NO_FACE,ALIGN_TURN_TIMEOUT,ALIGN_ENCODER,ALIGN_STALL,ALIGN_SIDE_NO_ECHO,ALIGN_HARDWARE,ALIGN_MOTOR} AlignError;
typedef struct {bool busy,done,fault;float actual_deg;uint32_t error;} AlignMotionFeedback;
typedef struct {
 AlignPhase phase;AlignError error;
 uint32_t since,started,last_seq,sum,turn_since,motor_error;
 uint16_t range[ALIGN_POINTS],servo_us,side_mm;
 uint8_t index,hits,fit_first,fit_last,moving_wheel;
 int8_t motor_direction;
 bool side_left,edge_left_seen,edge_right_seen,turn_requested;
 uint16_t edge_left_mm,edge_right_mm;uint8_t fit_support,fit_total;
 float edge_normal_deg,centre_y_mm;
 float normal_deg,rho_mm,rms_mm,target_deg,actual_deg;
} InitialAlign;
void align_start(InitialAlign *a,uint32_t now);
bool align_fit(InitialAlign *a);
/* Left-positive angles. Side-left selects wheel1 reverse, side-right wheel0 reverse.
   Geometry target is side probe normal to face, body approximately parallel. */
void align_step(InitialAlign *a,uint32_t now,uint32_t seq,uint32_t trigger_ms,
 uint32_t echo_ms,bool valid,uint16_t mm,const AlignMotionFeedback *motion);
void align_cancel(InitialAlign *a);
const char *align_phase_name(AlignPhase p);
const char *align_error_name(AlignError e);
#endif
