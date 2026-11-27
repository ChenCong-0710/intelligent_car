#include "initial_align.h"
#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static AlignMotionFeedback feedback(CarMotion *c){
 AlignMotionFeedback f={c->motor.running,car_motion_done(c),c->motor.phase==DM_FAULT,car_motion_estimated_angle(c),c->motor.error,{c->motor.counts[0],c->motor.counts[1]},{car_motion_wheel_angle(c,0),car_motion_wheel_angle(c,1)}};return f;
}
static void commands(InitialAlign *a,CarMotion *c){
 if((a->phase==ALIGN_APPROACH||a->phase==ALIGN_APPROACH_STOP)&&!a->approach_requested){assert(car_motion_start_scan(c));a->approach_requested=true;}
 if(a->approach_requested&&a->approach_plan_ready&&!a->approach_plan_applied&&!a->approach_hard_stop){assert(car_motion_finish_after_counts(c,a->approach_remaining_counts));a->approach_plan_applied=true;}
 if(a->approach_requested&&a->approach_hard_stop&&!a->approach_hard_stop_applied){assert(car_motion_finish_straight(c));a->approach_hard_stop_applied=true;}
 if(a->phase==ALIGN_TURN&&!a->turn_requested){assert(car_motion_start_pivot(c,1,a->target_deg>0?1:-1,fabsf(a->target_deg)*.5f));a->turn_requested=true;}
 if(a->phase==ALIGN_TURN_LEFT&&!a->left_turn_requested){assert(car_motion_start_pivot(c,0,a->target_deg>0?-1:1,fabsf(a->target_deg)*.5f));a->left_turn_requested=true;}
}
static void step(InitialAlign *a,CarMotion *c,uint32_t now,uint32_t seq,uint32_t trig,uint32_t echo,bool valid,uint16_t mm){
 AlignMotionFeedback f=feedback(c);align_step(a,now,seq,trig,echo,valid,mm,&f);commands(a,c);
}
static void run(float phi,int lo,int hi){
 InitialAlign a;CarMotion c;uint16_t raw[2]={0,0};uint32_t now=0,seq=0;
 assert(car_motion_init(&c,0,0));align_start(&a,now);
 assert(a.phase==ALIGN_HOME&&a.servo_us==1460&&!c.motor.running);
 step(&a,&c,1199,0,0,0,false,0);assert(a.phase==ALIGN_HOME);
 now=1200;step(&a,&c,now,0,0,0,false,0);assert(a.phase==ALIGN_SEEK);
 step(&a,&c,1201,1,1200,1201,true,501);assert(a.phase==ALIGN_SEEK&&!c.motor.running);
 step(&a,&c,1202,2,1201,1202,true,800);assert(a.phase==ALIGN_SEEK&&!c.motor.running);
 step(&a,&c,1203,3,1202,1203,true,500);assert(a.phase==ALIGN_APPROACH&&c.motor.running);
 /* Reinitialize for the same full approach run at250mm. */
 assert(car_motion_init(&c,0,0));align_start(&a,0);a.phase=ALIGN_SEEK;
 now=1260;step(&a,&c,now,++seq+1,now,now,true,250);assert(a.phase==ALIGN_APPROACH&&c.motor.running&&a.servo_us==1460);
 unsigned extra[2]={0,0};uint32_t last_seq=seq+1;bool near_plan=false;
 for(unsigned t=1;t<5000&&a.phase!=ALIGN_SCAN;++t){
  ++now;
  for(unsigned w=0;w<2;++w)if(c.motor.running&&t%2==0){
   if(!c.motor.braked[w])++raw[w];else if(extra[w]<10){++raw[w];++extra[w];}
  }
  car_motion_tick_1ms(&c,raw[0],raw[1],true);assert(c.motor.error==DM_OK);
  float distance=250-((raw[0]+raw[1])*.5f*CAR_MM_PER_EDGE);
  if(t%60==0){last_seq=++seq+1;step(&a,&c,now,last_seq,now-1,now,true,(uint16_t)lroundf(distance));}
  else step(&a,&c,now,last_seq,now-1,now,true,(uint16_t)lroundf(distance));
  if(a.approach_plan_applied){near_plan=true;assert(a.approach_plan_mm<=120&&a.approach_plan_mm>100);}
  if(a.phase!=ALIGN_SCAN)assert(a.servo_us==1460);
 }
 assert(near_plan&&a.phase==ALIGN_SCAN&&car_motion_done(&c));
 assert(a.approach_stop_hits>=2&&a.approach_stop_mm>=95&&a.approach_stop_mm<=105);
 float stopped=250-((raw[0]+raw[1])*.5f*CAR_MM_PER_EDGE);assert(stopped>=95&&stopped<=105);
 printf("Approach mock stop %.1f mm, plan at %u mm\n",stopped,a.approach_plan_mm);
 /* Centre-out scan, four distinct echo transactions per commanded angle. */
 unsigned points=0;
 while(a.phase==ALIGN_SCAN){
  unsigned i=a.index;uint32_t origin=a.since;
  uint16_t mm=(int)i>=lo&&(int)i<=hi?(uint16_t)lroundf(150/cosf((90-5*(int)i-phi)*.01745329252f)):0;
  for(unsigned k=1;k<=4;++k){
   uint32_t trig=origin+60*k;
   step(&a,&c,trig+5,++seq+1,trig,trig+5,mm!=0,mm);
  }
  now=origin+300;step(&a,&c,now,seq+1,0,0,false,0);
  assert(++points<=38);
 }
 assert(a.phase==ALIGN_FIT);step(&a,&c,++now,seq+1,0,0,false,0);
 assert(a.phase==ALIGN_SIDE_SETTLE&&a.servo_us==(a.side_left?2400:500)&&!c.motor.running);
 float target=phi+(a.side_left?-90:90);assert(fabsf(a.target_deg-target)<1);
 step(&a,&c,now+1199,seq+1,0,0,false,0);assert(!c.motor.running);
 now+=1200;step(&a,&c,now,seq+1,0,0,false,0);
 assert(a.phase==ALIGN_TURN&&c.kind==CAR_PIVOT_RIGHT&&c.motor.braked[0]&&!c.motor.braked[1]);
 int sign=target>0?1:-1;assert(c.direction==sign&&c.frame_sign[0]==1&&c.frame_sign[1]==1);
 assert(c.motor.wheel_goal[1]==car_motion_pivot_counts(1,fabsf(a.target_deg)*.5f));
 bool saw_wait=false,saw_left=false;uint32_t stopped_at=0;
 float first_right=0;
 for(unsigned t=0;t<5000&&a.phase!=ALIGN_DONE;++t){
  ++now;for(unsigned w=0;w<2;++w)if(c.motor.running&&!c.motor.braked[w])raw[w]=(uint16_t)(raw[w]+c.direction*c.frame_sign[w]);
  car_motion_tick_1ms(&c,raw[0],raw[1],true);assert(c.motor.error==DM_OK);
  step(&a,&c,now,seq+1,0,0,false,0);
  if(a.phase==ALIGN_TURN_HALF_WAIT){
   if(!saw_wait){stopped_at=now;first_right=a.wheel_degrees[1];}
   saw_wait=true;assert(!c.motor.running&&c.motor.braked[0]&&c.motor.braked[1]);
   assert(a.wheel_degrees[1]==first_right&&fabsf(first_right-sign*fabsf(target)*.5f)<.5f);
  }
  if(a.phase==ALIGN_TURN_LEFT){
   saw_left=true;assert(now-stopped_at>=500&&c.direction==-sign&&c.motor.braked[1]);
   assert(c.motor.wheel_goal[0]==car_motion_pivot_counts(0,fabsf(a.target_deg)*.5f));
  }
 }
 assert(saw_wait&&saw_left);
 assert(fabsf(a.wheel_degrees[1]-target*.5f)<.5f&&fabsf(a.wheel_degrees[0]+target*.5f)<.5f);
 assert(a.phase==ALIGN_DONE&&car_motion_done(&c)&&fabsf(a.actual_deg-target)<.5f);
 int32_t left=c.motor.actual[0],right=c.motor.actual[1];
 step(&a,&c,now+30000,++seq+1,now,now,true,150);
 assert(a.phase==ALIGN_DONE&&!c.motor.running&&c.motor.actual[0]==left&&c.motor.actual[1]==right);
}
static void scan4_checks(void){
 InitialAlign a;AlignMotionFeedback m={0};
 uint16_t samples[4]={90,100,120,220};
 align_start(&a,0);a.phase=ALIGN_SCAN;a.index=18;a.scan_bounded=true;
 a.scan_left_bound=a.scan_right_bound=255;
 for(unsigned k=0;k<4;++k){uint32_t t=60*(k+1);
  align_step(&a,t+5,k+1,t,t+5,true,samples[k],&m);
  /* Duplicate sequence must not count twice. */
  align_step(&a,t+6,k+1,t,t+5,true,samples[k],&m);
 }
 assert(a.scan_attempts==4);align_step(&a,299,4,240,245,true,220,&m);
 assert(a.index==18);align_step(&a,300,4,240,245,true,220,&m);
 assert(a.range[18]==110&&a.index==17&&a.servo_us>1460);
 /* One missing echo is trimmed; two missing echoes mark the outside. */
 for(unsigned k=0;k<4;++k){uint32_t t=360+60*k;
  align_step(&a,t+5,5+k,t,t+5,k!=3,100,&m);
 }
 align_step(&a,600,8,540,545,false,0,&m);
 assert(a.range[17]==100&&a.index==16);
 for(unsigned k=0;k<4;++k){uint32_t t=660+60*k;
  align_step(&a,t+5,9+k,t,t+5,k<2,100,&m);
 }
 align_step(&a,900,12,840,845,false,0,&m);
 assert(a.range[16]>250&&a.scan_left_bound==17&&a.index==18&&a.servo_us==1460&&a.scan_direction==1);
 puts("PASS: four samples, trimmed mean, duplicate rejection, 300ms schedule and missing-echo outside policy");
}
int main(void){
 scan4_checks();
 run(20,10,24);run(-20,12,26);run(0,10,26);run(-30,12,27);run(30,9,24);
 InitialAlign a;CarMotion c;assert(car_motion_init(&c,0,0));align_start(&a,0);
 step(&a,&c,1200,0,0,0,false,0);step(&a,&c,1300,1,1290,1295,true,95);
 assert(a.phase==ALIGN_PRE_SCAN_WAIT&&!c.motor.running&&a.servo_us==1460);
 step(&a,&c,1799,1,0,0,false,0);assert(a.phase==ALIGN_PRE_SCAN_WAIT);
 step(&a,&c,1800,1,0,0,false,0);assert(a.phase==ALIGN_SCAN);
 /* Two genuinely missing echoes brake and return to centre reacquisition. */
 assert(car_motion_init(&c,0,0));align_start(&a,0);a.phase=ALIGN_SEEK;
 step(&a,&c,1300,1,1290,1295,true,200);assert(c.motor.running);
 step(&a,&c,1360,2,1350,1355,false,0);assert(!a.approach_hard_stop);
 step(&a,&c,1420,3,1410,1415,false,0);assert(a.approach_hard_stop&&a.approach_echo_lost);
 for(unsigned t=0;t<250;++t){car_motion_tick_1ms(&c,0,0,true);step(&a,&c,1421+t,3,0,0,false,0);}
 assert(a.phase==ALIGN_PRE_SCAN_WAIT);step(&a,&c,a.since+500,3,0,0,false,0);assert(a.phase==ALIGN_SEEK&&a.servo_us==1460);
 AlignMotionFeedback bad={0};bad.fault=true;bad.error=DM_DRIVER;align_step(&a,3000,0,0,0,false,0,&bad);assert(a.phase==ALIGN_FAILED&&a.error==ALIGN_MOTOR);
 align_start(&a,0);align_cancel(&a);align_step(&a,3000,0,0,0,true,150,0);assert(a.phase==ALIGN_CANCELLED);
 puts("PASS: home0, wait obstacle, straight with fixed servo, TEST brake lead toward100mm, stopped centre-out four-sample scan, side choice, right calibrated half then left calibrated opposite half, 500ms settle gap, signed reporting and terminal DONE; close target/no echo/fault/cancel");
}
