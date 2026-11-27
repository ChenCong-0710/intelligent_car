#ifndef FOUR_FACES_H
#define FOUR_FACES_H
#include "box_center.h"
#define FOUR_FACE_COUNT 4U
#define FOUR_CORNER_CLEARANCE 450
#define FOUR_CORNER_HALF_DEG 45.0f
typedef enum {FF_IDLE,FF_FACE,FF_EXIT,FF_CLEARANCE,FF_CORNER_WAIT,
 FF_INNER,FF_TURN_GAP,FF_OUTER,FF_TURN_SETTLE,FF_ACQUIRE,FF_ACQUIRE_BRAKE,
 FF_DONE,FF_CANCELLED,FF_FAILED} FourFacesPhase;
typedef enum {FF_CMD_NONE,FF_CMD_FORWARD,FF_CMD_STOP,FF_CMD_CLEARANCE,
 FF_CMD_INNER,FF_CMD_OUTER} FourFacesCommand;
typedef struct {
 int32_t rear_min,rear_max,front_min,front_max,target,stopped;
 float fine_degrees;
} FourFaceResult;
typedef struct {
 FourFacesPhase phase;FourFacesCommand command;
 bool side_left,exit_inside;
 uint8_t hits,face_number,inner_wheel,outer_wheel;
 uint32_t since,last_seq,reference_sum;
 uint16_t new_reference;
 int32_t clearance_origin[2],clearance_counts[2],clearance_mean;
 float corner_wheel_degrees[2];
 FourFaceResult results[FOUR_FACE_COUNT];
 const char *error;
} FourFaces;
extern uint8_t g_laser_shots;
extern int32_t g_corner_progress;
void four_faces_start(FourFaces *t,BoxCenter *b,bool side_left,uint32_t now,uint32_t sequence);
void four_faces_step(FourFaces *t,BoxCenter *b,uint32_t now,uint32_t sequence,
 uint32_t trigger_ms,uint32_t echo_ms,bool valid,uint16_t mm,const int32_t total[2],
 bool busy,bool done,bool fault,bool cancel);
void four_faces_fail(FourFaces *t,BoxCenter *b,const char *error);
const char *four_faces_phase_name(FourFacesPhase p);
bool four_faces_moving(const FourFaces *t);
#endif
