#ifndef INITIAL_ALIGN_H
#define INITIAL_ALIGN_H
#include <stdbool.h>
#include <stdint.h>
#define ALIGN_POINTS 37U
#define ALIGN_APPROACH_TRIGGER_MM 120U
#define ALIGN_APPROACH_TARGET_MM 100U
#define ALIGN_LEFT_US 2400U
#define ALIGN_CENTRE_US 1460U
#define ALIGN_RIGHT_US 500U
#define ALIGN_SIDE_SETTLE_MS 1200U
#define ALIGN_SEGMENT_WAIT_MS 500U
#define ALIGN_RETREAT_COUNTS 150
#define ALIGN_FINAL_BACK_COUNTS 450
#define ALIGN_LINE_SAMPLES 64U
typedef enum {ALIGN_IDLE,ALIGN_SCAN,ALIGN_FIT,ALIGN_SIDE_SETTLE,ALIGN_TURN,
 ALIGN_BRAKE,ALIGN_LOCK,ALIGN_RETREAT,ALIGN_WAIT_FORWARD,ALIGN_FORWARD,ALIGN_SCAN_BRAKE,ALIGN_FINE_TURN,ALIGN_WAIT_FINAL,ALIGN_FINAL_BACK,ALIGN_DONE,ALIGN_FAILED,ALIGN_CANCELLED,ALIGN_HOME,ALIGN_SEEK,ALIGN_APPROACH,ALIGN_APPROACH_STOP,ALIGN_PRE_SCAN_WAIT} AlignPhase;
typedef enum {ALIGN_OK,ALIGN_NO_FACE,ALIGN_TURN_TIMEOUT,ALIGN_ENCODER,ALIGN_STALL,ALIGN_SIDE_NO_ECHO,ALIGN_HARDWARE,ALIGN_MOTOR,ALIGN_RETREAT_TIMEOUT,ALIGN_NO_BOUNDARIES,ALIGN_BAD_LINE,ALIGN_STAGE_TIMEOUT} AlignError;
typedef struct {bool busy,done,fault;float actual_deg;uint32_t error;uint32_t counts[2];} AlignMotionFeedback;
typedef struct {
 AlignPhase phase;AlignError error;
 uint32_t since,started,last_seq,sum,turn_since,motor_error;
 uint16_t range[ALIGN_POINTS],servo_us,side_mm;
 uint8_t index,hits,fit_first,fit_last,moving_wheel;
 int8_t motor_direction;
 bool side_left,edge_left_seen,edge_right_seen,turn_requested,retreat_requested;
 uint16_t edge_left_mm,edge_right_mm;uint8_t fit_support,fit_total;
 float edge_normal_deg,centre_y_mm;
 float normal_deg,rho_mm,rms_mm,target_deg,actual_deg;
 bool forward_requested,scan_stop_requested,fine_requested,final_requested,line_warning;
 bool outside_seen,entry_seen,exit_seen,entry_inferred;
 uint8_t outside_hits,near_hits,exit_hits,line_n,line_support;
 uint32_t outside_pos,near_pos,last_near,exit_pos,edge_entry,edge_exit;
 uint16_t near_range;
 float line_x[ALIGN_LINE_SAMPLES],line_y[ALIGN_LINE_SAMPLES];
 float fine_deg,fine_normal_deg,line_rms,line_slope;
 bool approach_requested,approach_plan_ready,approach_plan_applied;
 bool approach_hard_stop,approach_hard_stop_applied,approach_echo_lost;
 uint8_t approach_invalid;
 uint16_t approach_range_mm,approach_plan_mm;
 int32_t approach_remaining_counts;

} InitialAlign;
void align_start(InitialAlign *a,uint32_t now);
bool align_fit(InitialAlign *a);
bool align_fit_line(InitialAlign *a);
/* Left-positive angles. After approach+scan, both wheels turn in opposite
   directions: side probe normal to face, body approximately parallel. */
void align_step(InitialAlign *a,uint32_t now,uint32_t seq,uint32_t trigger_ms,
 uint32_t echo_ms,bool valid,uint16_t mm,const AlignMotionFeedback *motion);
void align_cancel(InitialAlign *a);
const char *align_phase_name(AlignPhase p);
const char *align_error_name(AlignError e);
#endif
