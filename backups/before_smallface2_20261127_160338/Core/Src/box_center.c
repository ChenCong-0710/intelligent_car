#include "box_center.h"
#include "initial_align.h"
#include "car_motion.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
int32_t g_box_position;
static void position(BoxCenter *b,const int32_t total[2]){
 if(!b->origin_set)return;
 /* Single-wheel pivot counts are angular motion, not traverse position. */
 if(b->phase>=BC_FINE_RIGHT&&b->phase<=BC_FINE_SETTLE)return;
 int32_t l=(int32_t)((uint32_t)total[0]-(uint32_t)b->origin[0]);
 int32_t r=(int32_t)((uint32_t)total[1]-(uint32_t)b->origin[1]);
 b->position=(int32_t)(((int64_t)l+r)/2);g_box_position=b->position;
}
void box_center_fail(BoxCenter *b,const char *reason){
 b->phase=BC_FAILED;b->error=reason;b->laser=b->buzzer=false;b->command=BC_CMD_NONE;
}
void box_center_start(BoxCenter *b,uint32_t now,uint32_t sequence){
 memset(b,0,sizeof(*b));b->phase=BC_BACK_SEARCH;b->since=now;b->last_seq=sequence;
 b->direction=-1;b->expect_outside=true;b->command=BC_CMD_REVERSE;b->error="OK";
 g_box_position=0;
}
static void move(BoxCenter *b,uint32_t now,BoxCenterPhase phase,int dir,bool outside){
 b->phase=phase;b->since=now;b->direction=dir;b->expect_outside=outside;b->confirm_hits=0;
 b->command=dir>0?BC_CMD_FORWARD:BC_CMD_REVERSE;
}
static void crossing(BoxCenter *b,uint32_t now,const int32_t total[2]){
 if(!b->origin_set){b->origin[0]=total[0];b->origin[1]=total[1];b->origin_set=true;position(b,total);}
 unsigned e=b->edge,n=b->visits[e];
 if(n>=3)return;
 int32_t p=b->position;b->crossings[e][n]=p;
 if(!n||p<b->minimum[e])b->minimum[e]=p;
 if(!n||p>b->maximum[e])b->maximum[e]=p;
 b->visits[e]=(uint8_t)(n+1);b->phase=BC_BRAKE;b->since=now;
 b->command=BC_CMD_STOP;b->confirm_hits=0;
}
static void sample_face(BoxCenter *b,uint16_t mm){
 float x=b->position*CAR_MM_PER_EDGE;
 if(b->fit_n&&x<=b->fit_x[b->fit_n-1])return;
 /* Compact rather than truncate the traverse when the buffer fills. */
 if(b->fit_n==64){
  for(unsigned k=0;k<32;++k){b->fit_x[k]=b->fit_x[2*k];b->fit_y[k]=b->fit_y[2*k];}
  b->fit_n=32;
 }
 unsigned n=b->fit_n++;b->fit_x[n]=x;b->fit_y[n]=mm;
}
static bool fit_fine(BoxCenter *b){
 InitialAlign line={0};line.side_left=b->side_left;
 line.entry_seen=line.exit_seen=true;line.line_n=b->fit_n;
 if(b->fit_n<2){b->fine_warning=true;return false;}
 line.edge_entry=(uint32_t)lroundf(b->fit_x[0]/CAR_MM_PER_EDGE);
 line.edge_exit=(uint32_t)lroundf(b->fit_x[b->fit_n-1]/CAR_MM_PER_EDGE);
 for(unsigned k=0;k<b->fit_n;++k){line.line_x[k]=b->fit_x[k];line.line_y[k]=b->fit_y[k];}
 /* Reuse the existing slope -> side normal -> body correction algorithm. */
 bool ok=align_fit_line(&line);b->fine_warning=line.line_warning||!ok;
 b->fine_degrees=ok?line.fine_deg:0;b->fine_rms=ok?line.line_rms:0;
 return ok;
}
static void return_to_center(BoxCenter *b,uint32_t now){
 /* Midpoint of the two edge estimates, each from three confirmed crossings. */
 int64_t sum=(int64_t)b->minimum[0]+b->maximum[0]+b->minimum[1]+b->maximum[1];
 b->target=(int32_t)(sum>=0?(sum+2)/4:-((-sum+2)/4));
 b->remaining=b->position-b->target;
 if(b->remaining<=0){box_center_fail(b,"CENTER_PASSED");return;}
 b->phase=BC_CENTER_MOVE;b->since=now;b->command=BC_CMD_CENTER;
}
static void finish_fine_and_sample(BoxCenter *b,uint32_t now,const int32_t total[2]){
 /* Keep the first-pass boundary records and the logical straight coordinate.
    Remove all per-wheel encoder change during the pivot (including coast)
    from the straight-motion origins. Do not count a turn as translation. */
 for(unsigned w=0;w<2;++w){
  uint32_t turn_delta=(uint32_t)total[w]-(uint32_t)b->fine_begin_total[w];
  b->origin[w]=(int32_t)((uint32_t)b->origin[w]+turn_delta);
 }
 b->phase=BC_BRAKE;position(b,total);
 move(b,now,BC_EDGE_MOVE,-1,false); /* Front crossing #2, then forward #3. */
}
void box_center_step(BoxCenter *b,uint32_t now,uint32_t sequence,
 uint32_t trigger_ms,uint32_t echo_ms,bool valid,uint16_t mm,
 const int32_t total[2],bool busy,bool done,bool fault,bool cancel){
 b->buzzer=false; /* User requested silent operation; laser timing retained. */
 if(b->phase==BC_IDLE)return;
 position(b,total); /* Includes braking coast, stopped motion, and all restarts. */
 if(b->phase==BC_DONE||b->phase==BC_CANCELLED||b->phase==BC_FAILED)return;
 if(cancel){b->phase=BC_CANCELLED;b->error="CANCELLED";b->laser=b->buzzer=false;b->command=BC_CMD_NONE;return;}
 if(fault){box_center_fail(b,"MOTOR");return;}
 bool fresh=sequence!=b->last_seq;if(fresh)b->last_seq=sequence;
 if(b->phase==BC_FINE_RIGHT||b->phase==BC_FINE_LEFT){
  if(!busy&&done){
   ++b->fine_completed;b->phase=b->fine_completed==1?BC_FINE_WAIT:BC_FINE_SETTLE;b->since=now;
  }
  return;
 }
 if(b->phase==BC_FINE_WAIT){
  if(now-b->since>=500U){
   b->phase=b->side_left?BC_FINE_LEFT:BC_FINE_RIGHT;b->since=now;
   b->command=b->side_left?BC_CMD_FINE_LEFT:BC_CMD_FINE_RIGHT;
  }
  return;
 }
 if(b->phase==BC_FINE_SETTLE){
  if(now-b->since>=500U)finish_fine_and_sample(b,now,total);
  return;
 }
 if(b->phase==BC_FIRE){
  if(now-b->since>=10000U){b->laser=b->buzzer=false;b->phase=BC_DONE;}
  return;
 }
 if(b->phase==BC_CENTER_MOVE){
  if(!busy&&done){b->phase=BC_FIRE;b->since=now;b->laser=true;b->buzzer=false;}
  return;
 }
 if(b->phase==BC_BRAKE){
  if(busy||!done)return;
  if(b->edge==1&&b->visits[1]==1&&!b->fine_done){
   b->fine_done=true;
   if(fit_fine(b)&&car_motion_pivot_counts(0,fabsf(b->fine_degrees)*.5f)>0&&
      car_motion_pivot_counts(1,fabsf(b->fine_degrees)*.5f)>0){
    b->fine_begin_total[0]=total[0];b->fine_begin_total[1]=total[1];
    b->fine_completed=0;b->phase=b->side_left?BC_FINE_RIGHT:BC_FINE_LEFT;b->since=now;
    b->command=b->side_left?BC_CMD_FINE_RIGHT:BC_CMD_FINE_LEFT;return;
   }
  }
  if(b->visits[b->edge]<3){
   int dir=-b->direction;move(b,now,BC_EDGE_MOVE,dir,!b->expect_outside);
  }else if(b->edge==0){
   b->edge=1;b->transit_inside=false;if(!b->fine_done)b->fit_n=0;move(b,now,BC_TRANSIT,1,true);
  }else {
   return_to_center(b,now);
  }
  return;
 }
 /* Use only completed new echoes triggered after the current motion began.
    Two measurement confirmations, never two reads of the same echo. */
 if(!busy||!fresh||now-echo_ms>100U||(uint32_t)(trigger_ms-b->since)>now-b->since)return;
 bool good=valid&&mm>=40&&mm<=4000;
 bool outside=!good||mm>300;
 if(!b->reference_mm){
  if(good&&!outside){
   b->reference_sum+=mm;
   if(++b->reference_hits==2)b->reference_mm=(uint16_t)((b->reference_sum+1U)/2U);
  }else {b->reference_sum=0;b->reference_hits=0;}
  b->confirm_hits=0;return;
 }
 if(b->phase==BC_TRANSIT&&!b->fine_done&&good&&!outside)sample_face(b,mm);
 bool inside=good&&!outside&&abs((int)mm-(int)b->reference_mm)<=30;
 /* Rear crossing #3 ends outside the face. Arm the FRONT exit only after
    two near-face measurements on the forward traverse, otherwise the same
    rear-side far echoes would be mistaken for the opposite boundary. */
 if(b->phase==BC_TRANSIT&&!b->transit_inside){
  if(inside){if(++b->confirm_hits==2){b->transit_inside=true;b->confirm_hits=0;}}
  else b->confirm_hits=0;
  return;
 }
 bool match=b->expect_outside?outside:inside;
 if(!match){b->confirm_hits=0;return;}
 if(++b->confirm_hits==2)crossing(b,now,total);
}
const char *box_center_phase_name(BoxCenterPhase p){
 static const char *const names[]={"IDLE","BACK_SEARCH","EDGE_MOVE","EDGE_BRAKE","TO_FRONT","CENTER_BACK","FIRE10S","DONE","CANCELLED","FAILED","FINE_RIGHT","FINE_WAIT","FINE_LEFT","FINE_SETTLE"};
 return (unsigned)p<sizeof(names)/sizeof(names[0])?names[p]:"UNKNOWN";
}
