#include "box_center.h"
#include "car_motion.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
static void command(BoxCenter *b,CarMotion *c){
 BoxCenterCommand cmd=b->command;if(cmd==BC_CMD_NONE)return;
 bool ok=false;
 if(cmd==BC_CMD_FORWARD&&b->phase==BC_TRANSIT)ok=car_motion_start_front_traverse(c);
 else if(cmd==BC_CMD_FORWARD||cmd==BC_CMD_REVERSE)ok=car_motion_start_edge_scan(c,cmd==BC_CMD_FORWARD?1:-1);
 if(cmd==BC_CMD_STOP)ok=car_motion_finish_straight(c);
 if(cmd==BC_CMD_CENTER)ok=car_motion_start_center_back(c,b->remaining);
 if(cmd==BC_CMD_FINE_RIGHT)ok=car_motion_start_pivot(c,1,b->fine_degrees>0?1:-1,fabsf(b->fine_degrees)*.5f);
 if(cmd==BC_CMD_FINE_LEFT)ok=car_motion_start_pivot(c,0,b->fine_degrees>0?-1:1,fabsf(b->fine_degrees)*.5f);
 assert(ok);
 if(cmd==BC_CMD_FORWARD)assert(c->motor.config.cruise_target==(b->phase==BC_TRANSIT?30:20));
 else if(cmd==BC_CMD_REVERSE||cmd==BC_CMD_CENTER)assert(c->motor.config.cruise_target==25);
 else if(cmd==BC_CMD_FINE_RIGHT||cmd==BC_CMD_FINE_LEFT)assert(c->motor.config.cruise_target==34);
 b->command=BC_CMD_NONE;
}
static void unit_checks(void){
 BoxCenter b;int32_t total[2]={0,0};box_center_start(&b,100,10);b.command=BC_CMD_NONE;
 box_center_step(&b,161,11,160,161,true,100,total,true,false,false,false);
 box_center_step(&b,162,11,160,161,true,100,total,true,false,false,false);
 assert(b.reference_hits==1); /* duplicate is not a second measurement */
 box_center_step(&b,221,12,220,221,true,110,total,true,false,false,false);
 assert(b.reference_mm==105);
 box_center_step(&b,281,13,280,281,true,301,total,true,false,false,false);
 assert(b.visits[0]==0);
 box_center_step(&b,341,14,340,341,false,0,total,true,false,false,false);
 assert(b.visits[0]==1&&b.origin_set&&g_box_position==0);
 b.command=BC_CMD_NONE;total[0]=-12;total[1]=-8;
 box_center_step(&b,342,14,340,341,false,0,total,true,false,false,false);
 assert(g_box_position==-10); /* Coast remains in the same coordinate frame. */
 box_center_step(&b,343,14,340,341,false,0,total,false,true,false,false);
 assert(b.phase==BC_EDGE_MOVE&&b.command==BC_CMD_FORWARD);
 b.command=BC_CMD_NONE;
 /* Late echo from before this new motion may not confirm entry. */
 box_center_step(&b,344,15,280,344,true,105,total,true,false,false,false);
 assert(b.confirm_hits==0);
 box_center_step(&b,404,16,400,404,true,136,total,true,false,false,false);
 assert(b.confirm_hits==0); /* Outside the +/-30mm band. */
 box_center_step(&b,464,17,460,464,true,135,total,true,false,false,false);
 assert(b.confirm_hits==1);
 box_center_step(&b,524,18,520,524,true,75,total,true,false,false,false);
 assert(b.visits[0]==2);
 b.phase=BC_FIRE;b.since=600;b.laser=b.buzzer=true;
 box_center_step(&b,10599,18,520,524,true,100,total,false,true,false,false);
 assert(b.laser&&b.buzzer);
 box_center_step(&b,10600,18,520,524,true,100,total,false,true,false,false);
 assert(b.phase==BC_DONE&&!b.laser&&!b.buzzer);
 box_center_start(&b,0,0);b.phase=BC_FIRE;b.laser=b.buzzer=true;
 box_center_step(&b,10,0,0,0,false,0,total,false,true,false,true);
 assert(b.phase==BC_CANCELLED&&!b.laser&&!b.buzzer);
 box_center_start(&b,0,0);b.phase=BC_FIRE;b.laser=b.buzzer=true;
 box_center_step(&b,10,0,0,0,false,0,total,false,true,true,false);
 assert(b.phase==BC_FAILED&&!b.laser&&!b.buzzer);
 puts("PASS: fresh echo/trigger checks, two-frame confirmation, signed coast, 30mm tolerance, 10s outputs and cancel/fault");
}
static void full_run(int32_t initial,int32_t width,bool missing_outside,float slope,bool side_left){
 BoxCenter b;CarMotion c;assert(car_motion_init(&c,65500,65500));
 uint16_t raw[2]={65500,65500};int32_t physical=initial;unsigned coast[2]={0,0};
 uint32_t seq=0,trigger=0;bool valid=true;uint16_t mm=100;
 box_center_start(&b,0,seq);b.side_left=side_left;command(&b,&c);
 uint32_t fire=0;bool reached=false,saw_right=false,saw_left=false,saw_restart=false;
 for(uint32_t now=1;now<120000;++now){
  for(unsigned w=0;w<2;++w){
   if(now%2==0){
    if(c.motor.running&&!c.motor.braked[w]){
     raw[w]=(uint16_t)(raw[w]+c.direction);if(w==0&&c.kind==CAR_STRAIGHT)physical+=c.direction;coast[w]=7;
    }else if(coast[w]){
     raw[w]=(uint16_t)(raw[w]+c.direction);if(w==0&&c.kind==CAR_STRAIGHT)physical+=c.direction;--coast[w];
    }
   }
  }
  car_motion_tick_1ms(&c,raw[0],raw[1],true);assert(c.motor.error==DM_OK);
  if(now%60==0){++seq;trigger=now;bool out=physical<0||physical>width;
   valid=!out||!missing_outside;mm=out?1000:(uint16_t)lroundf(100+slope*physical*CAR_MM_PER_EDGE);}
  if(now%5==0){
   int32_t total[2]={c.motor.wheel[0].total,c.motor.wheel[1].total};
   box_center_step(&b,now,seq,trigger,trigger,valid,mm,total,c.motor.running,car_motion_done(&c),false,false);
   assert(b.phase!=BC_FAILED);command(&b,&c);
   if(b.phase==BC_FINE_RIGHT){saw_right=true;assert(c.motor.braked[0]);}
   if(b.phase==BC_FINE_LEFT){saw_left=true;assert(c.motor.braked[1]);}
   if(b.phase==BC_BACK_SEARCH&&b.fine_done&&saw_left){saw_restart=true;assert(!b.origin_set);}

   if(b.phase==BC_FIRE){
    assert(!c.motor.running&&car_motion_done(&c)&&b.laser&&b.buzzer);
    if(!reached){fire=now;reached=true;
     if(slope!=0){
      assert(saw_right&&saw_left&&saw_restart&&b.fine_done);
      float expected=(side_left?1.f:-1.f)*atanf(slope)*57.2957795f;
      assert(fabsf(b.fine_degrees-expected)<.5f);
     }else assert(!saw_right&&!saw_left&&b.fine_done);

     assert(b.visits[0]==4&&b.visits[1]==4&&b.crossings[0][0]==0);
     assert(abs(physical-width/2)<80);
     assert(abs(b.position-b.target)<20);
     printf("Center mock: reference=%u rear=[%ld,%ld] front=[%ld,%ld] target=%ld stopped=%ld physical=%ld\n",b.reference_mm,(long)b.minimum[0],(long)b.maximum[0],(long)b.minimum[1],(long)b.maximum[1],(long)b.target,(long)b.position,(long)physical);
    }
   }
   if(b.phase==BC_DONE){assert(reached&&now-fire>=10000&&!b.laser&&!b.buzzer);return;}
  }
 }
 assert(!"Complete center run did not finish");
}
static void no_stall_check(void){
 CarMotion c;assert(car_motion_init(&c,0,0));
 assert(car_motion_start_edge_scan(&c,-1));
 for(unsigned t=0;t<3000;++t)car_motion_tick_1ms(&c,0,0,true);
 assert(c.motor.error==DM_OK&&c.motor.running);
 car_motion_tick_1ms(&c,0,0,false);assert(c.motor.error==DM_DRIVER);
 assert(car_motion_init(&c,0,0));assert(car_motion_start_pivot(&c,1,1,30));
 for(unsigned t=0;t<3000;++t)car_motion_tick_1ms(&c,0,0,true);
 assert(c.motor.error==DM_OK&&c.motor.running);
 puts("PASS: zero encoder no longer trips STALL in reverse or pivot; driver fault retained");
}
int main(void){no_stall_check();unit_checks();full_run(400,800,false,0,false);full_run(650,1300,true,0,true);full_run(400,800,false,.06f,false);full_run(400,800,true,-.06f,true);full_run(400,800,false,.06f,true);full_run(2500,5000,false,.01f,false);puts("PASS: both edges four crossings, no-echo outside, reverse continuous PD, encoder wrap, center braking and stopped firing");}
