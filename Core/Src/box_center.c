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
 b->fine_enabled=true;b->crossings_required=3; /* Standalone / large-face default. */
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
 if(n>=b->crossings_required)return;
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
static bool fine_angle_allowed(BoxCenter *b){
 b->fine_skipped=fabsf(b->fine_degrees)>BOX_FINE_MAX_DEG;
 return !b->fine_skipped; /* Keep the computed angle; do not clamp it to10. */
}
static void return_to_center(BoxCenter *b,uint32_t now){
 /* Midpoint of the two edge estimates, from the configured number of confirmed crossings. */
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
 if(b->fine_after_rear){
  b->edge=1;b->transit_inside=false;move(b,now,BC_TRANSIT,1,true);
 }else move(b,now,BC_EDGE_MOVE,-1,false); /* Front #2, then #3 only on large faces. */
}
/* Narrow-face stationary scan: calibrated side90 toward centre0, never outward. */
static uint16_t side_scan_pulse(const BoxCenter *b){
 unsigned angle=90U-5U*b->side_scan.index;
 if(b->side_left)return (uint16_t)(ALIGN_CENTRE_US+(ALIGN_LEFT_US-ALIGN_CENTRE_US)*angle/90U);
 return (uint16_t)(ALIGN_CENTRE_US-(ALIGN_CENTRE_US-ALIGN_RIGHT_US)*angle/90U);
}
uint16_t box_center_servo_us(const BoxCenter *b,uint16_t fallback){
 if(b->phase==BC_SIDE_SCAN||b->phase==BC_SIDE_RETURN)return b->side_scan.servo_us;
 return fallback;
}
static void side_scan_begin(BoxCenter *b,uint32_t now){
 b->side_scan.started=true;b->side_scan.index=0;b->side_scan.attempts=b->side_scan.triggers=0;
 b->side_scan.since=now;b->side_scan.servo_us=side_scan_pulse(b);
 b->fine_after_rear=true;b->fit_n=0;b->phase=BC_SIDE_SCAN;b->since=now;
}
static bool fit_side_scan(BoxCenter *b){
 b->fit_n=0;
 for(unsigned k=0;k<b->side_scan.points;++k){
  uint16_t distance=b->side_scan.range[k];if(distance<40U||distance>250U)continue;
  float angle=(90.f-5.f*k)*.0174532925199433f;
  float x=distance*cosf(angle),y=distance*sinf(angle);
  /* Sort Cartesian points before reusing the existing straight-line fit. */
  unsigned n=b->fit_n++,j=n;
  while(j&&b->fit_x[j-1]>x){b->fit_x[j]=b->fit_x[j-1];b->fit_y[j]=b->fit_y[j-1];--j;}
  b->fit_x[j]=x;b->fit_y[j]=y;
 }
 return fit_fine(b);
}
static void side_scan_step(BoxCenter *b,uint32_t now,uint32_t trigger,bool fresh,bool valid,uint16_t mm){
 BoxSideScan *s=&b->side_scan;uint32_t elapsed=now-s->since;
 if(fresh&&s->attempts<4U&&(uint32_t)(trigger-s->since)>=60U&&
    (uint32_t)(trigger-s->since)<=elapsed){
  s->samples[s->attempts++]=valid&&mm>=40U&&mm<=4000U?mm:4000U;
 }
 if(elapsed<300U||s->attempts!=4U)return;
 uint32_t sum=0;uint16_t lo=65535U,hi=0;
 for(unsigned k=0;k<4;++k){uint16_t v=s->samples[k];sum+=v;if(v<lo)lo=v;if(v>hi)hi=v;}
 uint16_t d=(uint16_t)((sum-lo-hi+1U)/2U);s->range[s->index]=d;s->points=s->index+1U;
 if(d>250U||s->index==18U){
  s->outside=d>250U;s->limit=s->index==18U;s->finished=true;
  s->fit_ok=fit_side_scan(b);b->fine_done=true;
  s->servo_us=b->side_left?ALIGN_LEFT_US:ALIGN_RIGHT_US;
  b->phase=BC_SIDE_RETURN;b->since=now;return;
 }
 ++s->index;s->attempts=s->triggers=0;s->since=now;s->servo_us=side_scan_pulse(b);
}
void box_center_step(BoxCenter *b,uint32_t now,uint32_t sequence,
 uint32_t trigger_ms,uint32_t echo_ms,bool valid,uint16_t mm,
 const int32_t total[2],bool busy,bool done,bool fault,bool cancel){
 b->buzzer=b->phase==BC_FIRE; /* Synchronous10s laser/buzzer output. */
 if(b->phase==BC_IDLE)return;
 position(b,total); /* Includes braking coast, stopped motion, and all restarts. */
 if(b->phase==BC_DONE||b->phase==BC_CANCELLED||b->phase==BC_FAILED)return;
 if(cancel){b->phase=BC_CANCELLED;b->error="CANCELLED";b->laser=b->buzzer=false;b->command=BC_CMD_NONE;return;}
 if(fault){box_center_fail(b,"MOTOR");return;}
 bool fresh=sequence!=b->last_seq;if(fresh)b->last_seq=sequence;
 if(b->phase==BC_SIDE_SCAN){
  side_scan_step(b,now,trigger_ms,fresh&&now-echo_ms<=100U,valid,mm);return;
 }
 if(b->phase==BC_SIDE_RETURN){
  if(now-b->since<500U)return; /* Allow the probe to return to calibrated side90. */
  if(b->side_scan.fit_ok&&fine_angle_allowed(b)&&car_motion_pivot_counts(0,fabsf(b->fine_degrees)*.5f)>0&&
     car_motion_pivot_counts(1,fabsf(b->fine_degrees)*.5f)>0){
   b->fine_begin_total[0]=total[0];b->fine_begin_total[1]=total[1];b->fine_completed=0;
   b->phase=b->side_left?BC_FINE_LEFT:BC_FINE_RIGHT;
   b->command=b->side_left?BC_CMD_FINE_LEFT:BC_CMD_FINE_RIGHT;
   b->since=now;
  }else {b->edge=1;b->transit_inside=false;move(b,now,BC_TRANSIT,1,true);}
  return;
 }
 if(b->phase==BC_FINE_RIGHT||b->phase==BC_FINE_LEFT){
  if(!busy&&done){
   ++b->fine_completed;b->phase=b->fine_completed==1?BC_FINE_WAIT:BC_FINE_SETTLE;b->since=now;
  }
  return;
 }
 if(b->phase==BC_FINE_WAIT){
  if(now-b->since>=500U){
   if(b->fine_after_rear){
    b->phase=b->side_left?BC_FINE_RIGHT:BC_FINE_LEFT;
    b->command=b->side_left?BC_CMD_FINE_RIGHT:BC_CMD_FINE_LEFT;
   }else{
    b->phase=b->side_left?BC_FINE_LEFT:BC_FINE_RIGHT;
    b->command=b->side_left?BC_CMD_FINE_LEFT:BC_CMD_FINE_RIGHT;
   }
   b->since=now;
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
  if(!busy&&done){b->phase=BC_FIRE;b->since=now;b->laser=b->buzzer=true;}
  return;
 }
 if(b->phase==BC_BRAKE){
  if(busy||!done)return;
  if(b->edge==0&&b->visits[0]>=b->crossings_required&&b->side_scan.enabled&&!b->side_scan.started){
   side_scan_begin(b,now);return;
  }
  if(b->edge==1&&b->visits[1]==1&&!b->fine_done){
   b->fine_done=true;
   if(b->fine_enabled&&fit_fine(b)&&fine_angle_allowed(b)&&car_motion_pivot_counts(0,fabsf(b->fine_degrees)*.5f)>0&&
      car_motion_pivot_counts(1,fabsf(b->fine_degrees)*.5f)>0){
    b->fine_begin_total[0]=total[0];b->fine_begin_total[1]=total[1];
    b->fine_completed=0;b->phase=b->side_left?BC_FINE_RIGHT:BC_FINE_LEFT;b->since=now;
    b->command=b->side_left?BC_CMD_FINE_RIGHT:BC_CMD_FINE_LEFT;return;
   }
  }
  if(b->visits[b->edge]<b->crossings_required){
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
 if(b->phase==BC_TRANSIT&&b->fine_enabled&&!b->fine_done&&good&&!outside)sample_face(b,mm);
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
 static const char *const names[]={"IDLE","BACK_SEARCH","EDGE_MOVE","EDGE_BRAKE","TO_FRONT","CENTER_BACK","FIRE10S","DONE","CANCELLED","FAILED","FINE_RIGHT","FINE_WAIT","FINE_LEFT","FINE_SETTLE","SIDE_SCAN","SIDE_RETURN"};
 return (unsigned)p<sizeof(names)/sizeof(names[0])?names[p]:"UNKNOWN";
}
