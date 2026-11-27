#include "box_center.h"
#include <string.h>
#include <stdlib.h>
int32_t g_box_position;
static void position(BoxCenter *b,const int32_t total[2]){
 if(!b->origin_set)return;
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
 if(n>=4)return;
 int32_t p=b->position;b->crossings[e][n]=p;
 if(!n||p<b->minimum[e])b->minimum[e]=p;
 if(!n||p>b->maximum[e])b->maximum[e]=p;
 b->visits[e]=(uint8_t)(n+1);b->phase=BC_BRAKE;b->since=now;
 b->command=BC_CMD_STOP;b->confirm_hits=0;
}
void box_center_step(BoxCenter *b,uint32_t now,uint32_t sequence,
 uint32_t trigger_ms,uint32_t echo_ms,bool valid,uint16_t mm,
 const int32_t total[2],bool busy,bool done,bool fault,bool cancel){
 if(b->phase==BC_IDLE)return;
 position(b,total); /* Includes braking coast, stopped motion, and all restarts. */
 if(b->phase==BC_DONE||b->phase==BC_CANCELLED||b->phase==BC_FAILED)return;
 if(cancel){b->phase=BC_CANCELLED;b->error="CANCELLED";b->laser=b->buzzer=false;b->command=BC_CMD_NONE;return;}
 if(fault){box_center_fail(b,"MOTOR");return;}
 bool fresh=sequence!=b->last_seq;if(fresh)b->last_seq=sequence;
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
  if(b->visits[b->edge]<4){
   int dir=-b->direction;move(b,now,BC_EDGE_MOVE,dir,!b->expect_outside);
  }else if(b->edge==0){
   b->edge=1;move(b,now,BC_TRANSIT,1,true);
  }else {
   int64_t sum=(int64_t)b->minimum[0]+b->maximum[0]+b->minimum[1]+b->maximum[1];
   b->target=(int32_t)(sum>=0?(sum+2)/4:-((-sum+2)/4));
   b->remaining=b->position-b->target;
   if(b->remaining<=0){box_center_fail(b,"CENTER_PASSED");return;}
   b->phase=BC_CENTER_MOVE;b->since=now;b->command=BC_CMD_CENTER;
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
 bool inside=good&&!outside&&abs((int)mm-(int)b->reference_mm)<=30;
 bool match=b->expect_outside?outside:inside;
 if(!match){b->confirm_hits=0;return;}
 if(++b->confirm_hits==2)crossing(b,now,total);
}
const char *box_center_phase_name(BoxCenterPhase p){
 static const char *const names[]={"IDLE","BACK_SEARCH","EDGE_MOVE","EDGE_BRAKE","TO_FRONT","CENTER_BACK","FIRE10S","DONE","CANCELLED","FAILED"};
 return (unsigned)p<sizeof(names)/sizeof(names[0])?names[p]:"UNKNOWN";
}
