#include "four_faces.h"
#include "car_motion.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
static unsigned inner_calls,outer_calls,clear_calls,fine_calls;
static void commands(FourFaces *t,BoxCenter *b,CarMotion *c){
 bool ok=false;FourFacesCommand cmd=t->command;
 if(cmd!=FF_CMD_NONE){
  if(cmd==FF_CMD_FORWARD)ok=t->phase==FF_EXIT?car_motion_start_front_traverse(c):car_motion_start_edge_scan(c,1);
  else if(cmd==FF_CMD_STOP){
   if(t->phase==FF_EXIT_BRAKE){assert(t->exit_samples==10&&c->motor.continuous);++clear_calls;}
   ok=car_motion_finish_straight(c);
  }else if(cmd==FF_CMD_INNER){
   assert(t->inner_wheel==(t->side_left?0U:1U));
   ok=car_motion_start_pivot(c,t->inner_wheel,-1,45);
   assert(c->motor.wheel_goal[t->inner_wheel]==(t->inner_wheel?306:291));++inner_calls;
  }else if(cmd==FF_CMD_OUTER){
   assert(t->outer_wheel==(t->side_left?1U:0U));
   ok=car_motion_start_pivot(c,t->outer_wheel,1,45);
   assert(c->motor.wheel_goal[t->outer_wheel]==(t->outer_wheel?306:291));++outer_calls;
  }
  assert(ok);t->command=FF_CMD_NONE;return;
 }
 if(t->phase!=FF_FACE||b->command==BC_CMD_NONE)return;
 BoxCenterCommand bc=b->command;
 if(bc==BC_CMD_FORWARD&&b->phase==BC_TRANSIT)ok=car_motion_start_front_traverse(c);
 else if(bc==BC_CMD_FORWARD||bc==BC_CMD_REVERSE)ok=car_motion_start_edge_scan(c,bc==BC_CMD_FORWARD?1:-1);
 else if(bc==BC_CMD_STOP)ok=car_motion_finish_straight(c);
 else if(bc==BC_CMD_CENTER)ok=car_motion_start_center_back(c,b->remaining);
 else if(bc==BC_CMD_FINE_RIGHT){ok=car_motion_start_pivot(c,1,b->fine_degrees>0?1:-1,fabsf(b->fine_degrees)*.5f);++fine_calls;}
 else if(bc==BC_CMD_FINE_LEFT){ok=car_motion_start_pivot(c,0,b->fine_degrees>0?-1:1,fabsf(b->fine_degrees)*.5f);++fine_calls;}
 assert(ok);b->command=BC_CMD_NONE;
}
static void run(bool left,bool invalid_far,float slope){
 FourFaces t;BoxCenter b;CarMotion c;uint16_t raw[2]={65500,65500};
 assert(car_motion_init(&c,raw[0],raw[1]));four_faces_start(&t,&b,left,0,0);
 assert(g_laser_shots==0&&t.phase==FF_FACE&&b.side_left==left);
 commands(&t,&b,&c);int32_t width=500,physical=250;unsigned coast[2]={0,0};
 unsigned shots_on=0;uint32_t fire_start=0;bool prev_laser=false;
 uint32_t seq=0,trig=0;bool valid=true;uint16_t mm=100;
 inner_calls=outer_calls=clear_calls=fine_calls=0;
 for(uint32_t now=1;now<180000;++now){
  if(now%2==0)for(unsigned w=0;w<2;++w){
   if(c.motor.running&&!c.motor.braked[w]){
    raw[w]=(uint16_t)(raw[w]+c.direction);coast[w]=7;
    if(w==0&&c.kind==CAR_STRAIGHT)physical+=c.direction;
   }else if(coast[w]){
    raw[w]=(uint16_t)(raw[w]+c.direction);--coast[w];
    if(w==0&&c.kind==CAR_STRAIGHT)physical+=c.direction;
   }
  }
  car_motion_tick_1ms(&c,raw[0],raw[1],true);assert(c.motor.error==DM_OK);
  if(now%60==0){
   ++seq;trig=now;bool outside=physical<0||physical>width;
   valid=!outside||!invalid_far;mm=outside?800:(uint16_t)lroundf(100+slope*physical*CAR_MM_PER_EDGE);
  }
  if(now%5)continue;
  int32_t total[2]={c.motor.wheel[0].total,c.motor.wheel[1].total};
  FourFacesPhase before=t.phase;unsigned before_shots=g_laser_shots;
  four_faces_step(&t,&b,now,seq,trig,trig,valid,mm,total,c.motor.running,car_motion_done(&c),false,false);
  assert(t.phase!=FF_FAILED&&t.phase!=FF_CANCELLED&&g_laser_shots<=4&&!b.buzzer);
  if(t.phase==FF_ACQUIRE&&before!=FF_ACQUIRE){
   /* Model the next side after the turn; this tests task sequencing,
      not the physical geometry of the corner or measured yaw accuracy. */
   physical=-80;width=t.face_number%2?500:160;
  }
  if(t.phase==FF_FACE&&before==FF_ACQUIRE_BRAKE){
   assert(t.side_left==left&&b.side_left==left&&b.reference_mm>=40);
   assert(b.visits[0]==0&&b.visits[1]==0&&b.command==BC_CMD_REVERSE);
  }
  if(g_laser_shots!=before_shots){
   assert(g_laser_shots==before_shots+1&&shots_on==g_laser_shots);
   assert(now-fire_start>=10000&&!b.laser);
   assert(t.results[before_shots].rear_min<=t.results[before_shots].rear_max);
   assert(abs(t.results[before_shots].target-t.results[before_shots].stopped)<25);
  }
  commands(&t,&b,&c);
  if(b.laser&&!prev_laser){
   assert(t.phase==FF_FACE&&b.phase==BC_FIRE&&!c.motor.running&&car_motion_done(&c));
   assert(b.visits[0]==3&&b.visits[1]==3);fire_start=now;++shots_on;
  }
  if(t.phase!=FF_FACE)assert(!b.laser);
  prev_laser=b.laser;
  if(t.phase==FF_DONE){
   assert(g_laser_shots==4&&shots_on==4&&inner_calls==3&&outer_calls==3&&clear_calls==3);
   assert(!c.motor.running&&!b.laser&&t.command==FF_CMD_NONE);
   if(slope!=0)assert(fine_calls>0);
   for(unsigned k=1;k<=100;++k)four_faces_step(&t,&b,now+k,seq,trig,trig,true,100,total,false,true,false,false);
   assert(g_laser_shots==4&&t.phase==FF_DONE&&t.command==FF_CMD_NONE);
   printf("PASS four sides: %s, four complete10s shots, three ten-sample exits, calibrated inner/outer45, fine calls=%u, no fifth action\n",left?"LEFT":"RIGHT",fine_calls);
   return;
  }
 }
 fprintf(stderr,"unfinished phase=%s b=%s physical=%ld shots=%u\n",four_faces_phase_name(t.phase),box_center_phase_name(b.phase),(long)physical,g_laser_shots);
 assert(!"Four-face task did not complete");
}
static void fresh_exit_check(void){
 FourFaces t;BoxCenter b;int32_t total[2]={1000,2000};four_faces_start(&t,&b,true,0,0);
 b.phase=BC_DONE;b.reference_mm=100;four_faces_step(&t,&b,100,1,99,100,true,100,total,false,true,false,false);
 assert(g_laser_shots==1&&t.phase==FF_EXIT);t.command=FF_CMD_NONE;
 uint32_t seq=1,now=100;
 for(unsigned k=0;k<2;++k){now+=60;four_faces_step(&t,&b,now,++seq,now-1,now,true,800,total,true,false,false,false);}
 assert(!t.exit_inside&&t.exit_samples==0);
 for(unsigned k=0;k<2;++k){now+=60;four_faces_step(&t,&b,now,++seq,now-1,now,true,100,total,true,false,false,false);}
 assert(t.exit_inside);
 for(unsigned k=1;k<=9;++k){
  now+=60;four_faces_step(&t,&b,now,++seq,now-1,now,true,301,total,true,false,false,false);
  assert(t.phase==FF_EXIT&&t.exit_samples==k&&t.command==FF_CMD_NONE);
  four_faces_step(&t,&b,now+1,seq,now-1,now,true,301,total,true,false,false,false);
  assert(t.exit_samples==k); /* Same result cannot count twice. */
 }
 now+=60;four_faces_step(&t,&b,now,++seq,now-1,now,true,300,total,true,false,false,false);
 assert(t.phase==FF_EXIT&&t.exit_samples==0); /* Exactly300 is not outside. */
 for(unsigned k=0;k<4;++k){now+=60;four_faces_step(&t,&b,now,++seq,now-1,now,true,800,total,true,false,false,false);}
 now+=60;seq+=2;four_faces_step(&t,&b,now,seq,now-1,now,true,800,total,true,false,false,false);
 assert(t.exit_samples==1); /* A missing transaction breaks continuity. */
 for(unsigned k=2;k<=10;++k){
  now+=60;total[0]+=10;total[1]+=10;
  four_faces_step(&t,&b,now,++seq,now-1,now,k!=10,k==10?0:800,total,true,false,false,false);
  if(k<10)assert(t.phase==FF_EXIT&&t.exit_samples==k&&t.command==FF_CMD_NONE);
 }
 assert(t.phase==FF_EXIT_BRAKE&&t.exit_samples==10&&t.command==FF_CMD_STOP);
 assert(g_corner_progress==90); /* Count is diagnostic only, not450. */
 t.command=FF_CMD_NONE;
 now+=5;four_faces_step(&t,&b,now,seq,now-1,now,true,800,total,true,false,false,false);
 assert(t.phase==FF_EXIT_BRAKE);
 now+=5;four_faces_step(&t,&b,now,seq,now-1,now,true,800,total,false,true,false,false);
 assert(t.phase==FF_CORNER_WAIT);
}
static void cancel_fault_check(void){
 int32_t total[2]={0};
 for(unsigned p=FF_EXIT;p<=FF_ACQUIRE_BRAKE;++p){
  FourFaces t;BoxCenter b;four_faces_start(&t,&b,false,0,0);g_laser_shots=1;t.phase=(FourFacesPhase)p;b.laser=true;
  four_faces_step(&t,&b,10,1,9,10,true,100,total,true,false,true,false);
  assert(t.phase==FF_FAILED&&g_laser_shots==1&&!b.laser&&t.command==FF_CMD_NONE);
  four_faces_start(&t,&b,false,0,0);g_laser_shots=1;t.phase=(FourFacesPhase)p;b.laser=true;
  four_faces_step(&t,&b,10,1,9,10,true,100,total,true,false,false,true);
  assert(t.phase==FF_CANCELLED&&g_laser_shots==1&&!b.laser&&t.command==FF_CMD_NONE);
 }
}
int main(void){fresh_exit_check();cancel_fault_check();run(true,false,0);run(false,true,0);run(true,true,.04f);run(false,false,-.04f);}
