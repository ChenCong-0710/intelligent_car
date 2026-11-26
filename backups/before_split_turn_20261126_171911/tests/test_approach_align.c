#include "initial_align.h"
#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static AlignMotionFeedback feedback(CarMotion *c){
 AlignMotionFeedback f={c->motor.running,car_motion_done(c),c->motor.phase==DM_FAULT,car_motion_estimated_angle(c),c->motor.error,{c->motor.counts[0],c->motor.counts[1]}};return f;
}
static void commands(InitialAlign *a,CarMotion *c){
 if((a->phase==ALIGN_APPROACH||a->phase==ALIGN_APPROACH_STOP)&&!a->approach_requested){assert(car_motion_start_scan(c));a->approach_requested=true;}
 if(a->approach_requested&&a->approach_plan_ready&&!a->approach_plan_applied&&!a->approach_hard_stop){assert(car_motion_finish_after_counts(c,a->approach_remaining_counts));a->approach_plan_applied=true;}
 if(a->approach_requested&&a->approach_hard_stop&&!a->approach_hard_stop_applied){assert(car_motion_finish_straight(c));a->approach_hard_stop_applied=true;}
 if(a->phase==ALIGN_TURN&&!a->turn_requested){assert(car_motion_start_inplace(c,a->target_deg));a->turn_requested=true;}
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
 step(&a,&c,1201,1,1200,1201,false,800);assert(a.phase==ALIGN_SEEK&&!c.motor.running);
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
 float stopped=250-((raw[0]+raw[1])*.5f*CAR_MM_PER_EDGE);assert(stopped>=95&&stopped<=105);
 printf("Approach mock stop %.1f mm, plan at %u mm\n",stopped,a.approach_plan_mm);
 /* Actual37-point scan is performed only after stopping, left to right. */
 uint32_t scan_since=a.since;uint16_t previous=2400;
 for(unsigned i=0;i<37;++i){
  assert(a.index==i&&a.servo_us<=previous);previous=a.servo_us;
  if(i==0)assert(a.servo_us==2400);if(i==18)assert(a.servo_us==1460);if(i==36)assert(a.servo_us==500);
  uint16_t mm=(int)i>=lo&&(int)i<=hi?(uint16_t)lroundf(150/cosf((90-5*(int)i-phi)*.01745329252f)):0;
  uint32_t settle=i==0?1200:120;
  step(&a,&c,scan_since+settle+20,++seq+1,scan_since+settle+10,scan_since+settle+15,mm!=0,mm);
  step(&a,&c,scan_since+settle+80,++seq+1,scan_since+settle+70,scan_since+settle+75,mm!=0,mm);
  now=scan_since+settle+180;step(&a,&c,now,seq+1,0,0,false,0);scan_since=now;
 }
 assert(a.phase==ALIGN_FIT);step(&a,&c,++now,seq+1,0,0,false,0);
 assert(a.phase==ALIGN_SIDE_SETTLE&&a.servo_us==(a.side_left?2400:500)&&!c.motor.running);
 float target=phi+(a.side_left?-90:90);assert(fabsf(a.target_deg-target)<1);
 step(&a,&c,now+1199,seq+1,0,0,false,0);assert(!c.motor.running);
 now+=1200;step(&a,&c,now,seq+1,0,0,false,0);
 assert(a.phase==ALIGN_TURN&&c.kind==CAR_INPLACE&&!c.motor.braked[0]&&!c.motor.braked[1]);
 int sign=target>0?1:-1;assert(c.frame_sign[0]==-sign&&c.frame_sign[1]==sign);
 for(unsigned t=0;t<2500&&a.phase!=ALIGN_DONE;++t){
  ++now;for(unsigned w=0;w<2;++w)if(c.motor.running&&!c.motor.braked[w])raw[w]=(uint16_t)(raw[w]+c.frame_sign[w]);
  car_motion_tick_1ms(&c,raw[0],raw[1],true);assert(c.motor.error==DM_OK);
  step(&a,&c,now,seq+1,0,0,false,0);
 }
 assert(a.phase==ALIGN_DONE&&car_motion_done(&c)&&fabsf(a.actual_deg-target)<.5f);
 int32_t left=c.motor.actual[0],right=c.motor.actual[1];
 step(&a,&c,now+30000,++seq+1,now,now,true,150);
 assert(a.phase==ALIGN_DONE&&!c.motor.running&&c.motor.actual[0]==left&&c.motor.actual[1]==right);
}
int main(void){
 run(20,10,17);run(-20,19,26);run(0,7,13);run(0,23,29);
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
 puts("PASS: home0, wait obstacle, straight with fixed servo, TEST brake lead toward100mm, stopped37-point scan, side choice, BOTH opposite-wheel turn and terminal DONE; close target/no echo/fault/cancel");
}
