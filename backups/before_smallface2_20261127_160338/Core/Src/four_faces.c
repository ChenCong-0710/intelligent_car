#include "four_faces.h"
#include <string.h>
#include <stdlib.h>
uint8_t g_laser_shots;
int32_t g_corner_progress;
static void stage(FourFaces *t,FourFacesPhase phase,uint32_t now){
 t->phase=phase;t->since=now;t->hits=0;
}
void four_faces_fail(FourFaces *t,BoxCenter *b,const char *error){
 t->phase=FF_FAILED;t->error=error;t->command=FF_CMD_NONE;
 b->command=BC_CMD_NONE;b->laser=b->buzzer=false;
}
void four_faces_start(FourFaces *t,BoxCenter *b,bool left,uint32_t now,uint32_t seq){
 memset(t,0,sizeof(*t));g_laser_shots=0;g_corner_progress=0;
 t->side_left=left;t->inner_wheel=left?0U:1U;t->outer_wheel=1U-t->inner_wheel;
 t->face_number=1;t->error="OK";t->last_seq=seq;
 stage(t,FF_FACE,now);box_center_start(b,now,seq);b->side_left=left;
}
static void record_face(FourFaces *t,const BoxCenter *b){
 FourFaceResult *r=&t->results[g_laser_shots];
 r->rear_min=b->minimum[0];r->rear_max=b->maximum[0];
 r->front_min=b->minimum[1];r->front_max=b->maximum[1];
 r->target=b->target;r->stopped=b->position;r->fine_degrees=b->fine_degrees;
}
void four_faces_step(FourFaces *t,BoxCenter *b,uint32_t now,uint32_t seq,
 uint32_t trigger,uint32_t echo,bool valid,uint16_t mm,const int32_t total[2],
 bool busy,bool done,bool fault,bool cancel){
 if(t->phase==FF_IDLE)return;
 if(cancel&&t->phase!=FF_FAILED){
  t->phase=FF_CANCELLED;t->error="CANCELLED";t->command=FF_CMD_NONE;
  b->command=BC_CMD_NONE;b->laser=b->buzzer=false;return;
 }
 if(t->phase==FF_DONE||t->phase==FF_CANCELLED||t->phase==FF_FAILED)return;
 if(fault){four_faces_fail(t,b,"MOTOR");return;}
 uint32_t previous_seq=t->last_seq;
 bool fresh=seq!=previous_seq;if(fresh)t->last_seq=seq;
 if(t->phase==FF_FACE){
  box_center_step(b,now,seq,trigger,echo,valid,mm,total,busy,done,fault,cancel);
  if(b->phase==BC_FAILED){four_faces_fail(t,b,b->error);return;}
  if(b->phase==BC_CANCELLED){t->phase=FF_CANCELLED;t->error="CANCELLED";return;}
  if(b->phase==BC_DONE){
   record_face(t,b);++g_laser_shots; /* Count only a completed10s shot. */
   b->laser=b->buzzer=false;b->command=BC_CMD_NONE;
   if(g_laser_shots==FOUR_FACE_COUNT){stage(t,FF_DONE,now);return;}
   stage(t,FF_EXIT,now);t->exit_inside=false;t->exit_samples=0;t->command=FF_CMD_FORWARD;
  }
  return;
 }
 /* Subsequent stages never assert a laser or buzzer output. */
 b->laser=b->buzzer=false;
 if((t->phase==FF_EXIT&&t->exit_samples>0)||t->phase==FF_EXIT_BRAKE||t->phase==FF_CORNER_WAIT){
  for(unsigned w=0;w<2;++w)t->clearance_counts[w]=(int32_t)((uint32_t)total[w]-(uint32_t)t->clearance_origin[w]);
  t->clearance_mean=(int32_t)(((int64_t)t->clearance_counts[0]+t->clearance_counts[1])/2);
  g_corner_progress=t->clearance_mean;
 }
 if(t->phase==FF_EXIT_BRAKE){
  if(!busy&&done)stage(t,FF_CORNER_WAIT,now);
  return;
 }
 if(t->phase==FF_CORNER_WAIT){
  if(now-t->since>=500U){
   t->corner_wheel_degrees[0]=t->corner_wheel_degrees[1]=0;
   stage(t,FF_INNER,now);t->command=FF_CMD_INNER;
  }
  return;
 }
 if(t->phase==FF_INNER){
  if(!busy&&done)stage(t,FF_TURN_GAP,now);
  return;
 }
 if(t->phase==FF_TURN_GAP){
  if(now-t->since>=500U){stage(t,FF_OUTER,now);t->command=FF_CMD_OUTER;}
  return;
 }
 if(t->phase==FF_OUTER){
  if(!busy&&done)stage(t,FF_TURN_SETTLE,now);
  return;
 }
 if(t->phase==FF_TURN_SETTLE){
  if(now-t->since>=500U){
   stage(t,FF_ACQUIRE,now);t->reference_sum=0;t->new_reference=0;
   t->face_number=(uint8_t)(g_laser_shots+1U);t->command=FF_CMD_FORWARD;
  }
  return;
 }
 if(t->phase==FF_ACQUIRE_BRAKE){
  if(!busy&&done){
   stage(t,FF_FACE,now);box_center_start(b,now,seq);b->side_left=t->side_left;
   /* The new face already supplied two fresh side echoes before braking. */
   b->reference_mm=t->new_reference;b->reference_hits=2;b->reference_sum=t->reference_sum;
  }
  return;
 }
 if(!busy||!fresh||now-echo>100U||(uint32_t)(trigger-t->since)>now-t->since)return;
 bool good=valid&&mm>=40U&&mm<=4000U;
 bool outside=!good||mm>300U;
 if(t->phase==FF_EXIT){
  /* After entering the current face, continue forward until thirty consecutive
     completed60ms transactions indicate outside. There is NO pulse clearance. */
  if(!t->exit_inside){
   bool inside=good&&!outside&&abs((int)mm-(int)b->reference_mm)<=30;
   if(!inside){t->hits=0;return;}
   if(++t->hits==2){t->exit_inside=true;t->hits=0;}
   return;
  }
  if(!outside){t->hits=0;t->exit_samples=0;g_corner_progress=0;return;}
  /* A missed result cannot establish a continuous run of outside samples. */
  if(seq!=previous_seq+1U)t->hits=0;
  if(!t->hits){
   for(unsigned w=0;w<2;++w){t->clearance_origin[w]=total[w];t->clearance_counts[w]=0;}
   t->clearance_mean=g_corner_progress=0;
  }
  t->exit_samples=++t->hits;
  if(t->exit_samples<FOUR_EXIT_SAMPLES)return;
  stage(t,FF_EXIT_BRAKE,now);t->command=FF_CMD_STOP;
 }else if(t->phase==FF_ACQUIRE){
  if(good&&!outside){
   t->reference_sum+=mm;
   if(++t->hits==2){
    t->new_reference=(uint16_t)((t->reference_sum+1U)/2U);
    stage(t,FF_ACQUIRE_BRAKE,now);t->command=FF_CMD_STOP;
   }
  }else {t->hits=0;t->reference_sum=0;}
 }
}
bool four_faces_moving(const FourFaces *t){
 return t->phase==FF_EXIT||t->phase==FF_INNER||t->phase==FF_OUTER||t->phase==FF_ACQUIRE;
}
const char *four_faces_phase_name(FourFacesPhase p){
 static const char *const names[]={"IDLE","FACE","EXIT_FACE","EXIT_BRAKE","CORNER_WAIT","INNER_45","TURN_GAP","OUTER_45","TURN_SETTLE","FIND_NEW_FACE","NEW_FACE_BRAKE","ALL_DONE","CANCELLED","FAILED"};
 return (unsigned)p<sizeof(names)/sizeof(names[0])?names[p]:"UNKNOWN";
}
