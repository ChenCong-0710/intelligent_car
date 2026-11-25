#ifndef INITIAL_ALIGN_H
#define INITIAL_ALIGN_H
#include <stdbool.h>
#include <stdint.h>
#define ALIGN_POINTS 37U
/* Physical left/centre/right. Swap LEFT and RIGHT if the horn is mirrored. */
#define ALIGN_LEFT_US 2400U
#define ALIGN_CENTRE_US 1500U
#define ALIGN_RIGHT_US 600U
typedef enum { ALIGN_IDLE, ALIGN_SCAN, ALIGN_FIT, ALIGN_TURN,
    ALIGN_BRAKE, ALIGN_LOCK, ALIGN_DONE, ALIGN_FAILED, ALIGN_CANCELLED } AlignPhase;
typedef enum { ALIGN_OK, ALIGN_NO_FACE, ALIGN_TURN_TIMEOUT,
    ALIGN_ENCODER, ALIGN_STALL, ALIGN_SIDE_NO_ECHO, ALIGN_HARDWARE } AlignError;
typedef struct {
    AlignPhase phase;
    AlignError error;
    uint32_t since, started, last_seq, sum, turn_since, bad_since[2], stall_since[2];
    uint16_t range[ALIGN_POINTS], servo_us, side_mm;
    uint8_t index, hits, fit_first, fit_last;
    bool side_left, edge_left_seen, edge_right_seen;
    float normal_deg, rho_mm, rms_mm, target_deg, actual_deg, start_rad;
    float speed[2];
} InitialAlign;
void align_start(InitialAlign *a, uint32_t now);
/* Standalone geometry solver: range[] are left(+90) to right(-90), zero=invalid. */
bool align_fit(InitialAlign *a);
void align_step(InitialAlign *a, uint32_t now, uint32_t seq, uint32_t trigger_ms,
                uint32_t echo_ms, bool valid, uint16_t mm, float heading_rad,
                const float measured[2]);
const char *align_phase_name(AlignPhase p);
const char *align_error_name(AlignError e);
#endif
