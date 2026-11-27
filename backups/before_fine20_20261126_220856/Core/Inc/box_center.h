#ifndef BOX_CENTER_H
#define BOX_CENTER_H
#include <stdint.h>
#include <stdbool.h>
typedef enum {BC_IDLE,BC_BACK_SEARCH,BC_EDGE_MOVE,BC_BRAKE,BC_TRANSIT,
 BC_CENTER_MOVE,BC_FIRE,BC_DONE,BC_CANCELLED,BC_FAILED} BoxCenterPhase;
typedef enum {BC_CMD_NONE,BC_CMD_FORWARD,BC_CMD_REVERSE,BC_CMD_STOP,BC_CMD_CENTER} BoxCenterCommand;
typedef struct {
 BoxCenterPhase phase;BoxCenterCommand command;
 uint32_t since,last_seq,reference_sum;
 uint16_t reference_mm;
 uint8_t reference_hits,confirm_hits,edge,visits[2];
 int direction;bool expect_outside,origin_set,laser,buzzer;
 int32_t origin[2],position,minimum[2],maximum[2],crossings[2][4],target,remaining;
 const char *error;
} BoxCenter;
/* Physical forward positive, A-channel rising+falling edges; reset once at
   the first confirmed rear crossing, never at direction changes. */
extern int32_t g_box_position;
void box_center_start(BoxCenter *b,uint32_t now,uint32_t sequence);
void box_center_step(BoxCenter *b,uint32_t now,uint32_t sequence,
 uint32_t trigger_ms,uint32_t echo_ms,bool valid,uint16_t mm,
 const int32_t total[2],bool busy,bool done,bool fault,bool cancel);
void box_center_fail(BoxCenter *b,const char *reason);
const char *box_center_phase_name(BoxCenterPhase phase);
#endif
