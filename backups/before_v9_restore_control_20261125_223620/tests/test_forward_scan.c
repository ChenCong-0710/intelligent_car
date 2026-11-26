#include "initial_align.h"
#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static AlignMotionFeedback fb(CarMotion *c){
 AlignMotionFeedback f={c->motor.running,car_motion_done(c),c->motor.phase==DM_FAULT,car_motion_estimated_angle(c),c->motor.error,{c->motor.counts[0],c->motor.counts[1]}};return f;
}
static void run(bool left,float slope,bool inside){
 InitialAlign a={0};CarMotion c;uint16_t counter[2]={0,0};uint32_t seq=0,now=1000;
 assert(car_motion_init(&c,0,0));a.phase=ALIGN_FORWARD;a.side_left=left;a.side_mm=150;a.since=now;
 assert(car_motion_start_scan(&c));a.forward_requested=true;
 for(unsigned t=0;t<1400&&a.phase==ALIGN_FORWARD;++t){
  ++now;++counter[0];++counter[1];car_motion_tick_1ms(&c,counter[0],counter[1],true);
  bool on=(inside||t>=120)&&t<=840;uint16_t mm=on?(uint16_t)lroundf(150+slope*t*(43*3.14159265f/420)):600;
  bool fresh=t%60==0;if(fresh)++seq;
  AlignMotionFeedback f=fb(&c);align_step(&a,now,seq,now,now,on,mm,&f);
 }
 assert(a.phase==ALIGN_SCAN_BRAKE&&a.entry_seen&&a.exit_seen&&a.line_n>=7);
 assert(car_motion_finish_straight(&c));a.scan_stop_requested=true;
 for(unsigned t=0;t<800&&a.phase==ALIGN_SCAN_BRAKE;++t){
  ++now;if(t<20){++counter[0];++counter[1];}
  car_motion_tick_1ms(&c,counter[0],counter[1],true);AlignMotionFeedback f=fb(&c);
  align_step(&a,now,seq,now,now,false,0,&f);
 }
 assert(fabsf(a.fine_deg-(left?1:-1)*atanf(slope)*57.2957795f)<.5f);
 if(fabsf(slope)<.01f)assert(a.phase==ALIGN_WAIT_FINAL);
 else{
  assert(a.phase==ALIGN_FINE_TURN);int sign=a.fine_deg>0?1:-1;
  assert(car_motion_start_pivot(&c,1,sign,fabsf(a.fine_deg)));a.fine_requested=true;
  assert(c.motor.counts[0]==0&&c.motor.counts[1]==0);
  assert(c.motor.braked[0]&&c.motor.wheel_goal[0]==0&&c.motor.config.cruise_target==34);
  uint16_t fixed=counter[0];
  for(unsigned t=0;t<1000&&a.phase==ALIGN_FINE_TURN;++t){
   ++now;if(c.motor.running&&!c.motor.braked[1])counter[1]=(uint16_t)(counter[1]+sign);
   car_motion_tick_1ms(&c,counter[0],counter[1],true);assert(counter[0]==fixed&&c.motor.braked[0]);
   AlignMotionFeedback f=fb(&c);align_step(&a,now,seq,now,now,false,0,&f);
  }
  assert(a.phase==ALIGN_WAIT_FINAL);
 }
 AlignMotionFeedback waiting=fb(&c);
 align_step(&a,now+499,seq,0,0,false,0,&waiting);assert(a.phase==ALIGN_WAIT_FINAL);
 now+=500;align_step(&a,now,seq,0,0,false,0,&waiting);assert(a.phase==ALIGN_FINAL_BACK);
 assert(car_motion_start_straight(&c,-1,ALIGN_FINAL_BACK_COUNTS));a.final_requested=true;
 assert(c.motor.counts[0]==0&&c.motor.counts[1]==0);
 for(unsigned t=0;t<1500&&a.phase==ALIGN_FINAL_BACK;++t){
  ++now;if(t<470){--counter[0];--counter[1];}
  car_motion_tick_1ms(&c,counter[0],counter[1],true);assert(c.motor.error==DM_OK);
  assert(c.motor.wheel[0].duty<=0&&c.motor.wheel[1].duty<=0);
  AlignMotionFeedback f=fb(&c);align_step(&a,now,seq,now,now,false,0,&f);
 }
 assert(a.phase==ALIGN_DONE&&c.motor.counts[0]==470&&c.motor.counts[1]==470);
 assert(c.motor.wheel_goal[0]==450&&c.motor.wheel_goal[1]==450);
}
int main(void){
 run(true,.08f,false);run(false,.08f,false);run(true,-.08f,false);run(false,-.08f,false);run(true,0,false);
 run(true,.08f,true);run(false,-.08f,true);run(true,0,true);
 InitialAlign a={0};AlignMotionFeedback f={0};a.phase=ALIGN_FORWARD;a.forward_requested=true;f.done=true;
 align_step(&a,100,1,100,100,true,150,&f);assert(a.phase==ALIGN_FORWARD&&a.error==ALIGN_OK);
 /* Starting inside is accepted, marked inferred. Duplicate seq cannot confirm exit. */
 memset(&a,0,sizeof a);memset(&f,0,sizeof f);a.phase=ALIGN_FORWARD;a.forward_requested=true;a.side_mm=150;
 for(unsigned i=1;i<=4;++i){f.counts[0]=f.counts[1]=i*40;align_step(&a,i*60,i,0,i*60,true,150,&f);}
 assert(a.entry_seen&&a.entry_inferred&&!a.outside_seen);
 align_step(&a,400,5,0,400,false,600,&f);align_step(&a,401,5,0,400,false,600,&f);assert(!a.exit_seen);
 align_step(&a,460,6,0,460,false,600,&f);assert(a.exit_seen);
 for(unsigned p=ALIGN_FORWARD;p<=ALIGN_FINAL_BACK;++p){
  memset(&a,0,sizeof a);a.phase=(AlignPhase)p;f.fault=true;f.error=DM_DRIVER;
  align_step(&a,100,0,0,0,false,0,&f);assert(a.phase==ALIGN_FAILED&&a.error==ALIGN_MOTOR);
  a.phase=(AlignPhase)p;align_cancel(&a);align_step(&a,101,0,0,0,false,0,&f);assert(a.phase==ALIGN_CANCELLED);
 }
 /* Large slope, insufficient span, and too many nonplanar points reject the fit. */
 memset(&a,0,sizeof a);a.entry_seen=a.exit_seen=true;a.edge_entry=0;a.edge_exit=700;a.line_n=12;a.side_left=true;
 for(unsigned i=0;i<12;++i){a.line_x[i]=10+i*15;a.line_y[i]=150+.7f*a.line_x[i];}assert(align_fit_line(&a)&&a.line_warning);
 for(unsigned i=0;i<12;++i){a.line_y[i]=150+.08f*a.line_x[i];}
 a.line_y[6]+=40;
 assert(align_fit_line(&a)&&fabsf(a.fine_deg-4.57f)<.3f&&a.line_support<10);
 for(unsigned i=0;i<12;++i){a.line_y[i]=150+(i%2?25:-25);}assert(align_fit_line(&a)&&a.line_warning);
 a.line_n=5;assert(align_fit_line(&a));
 /* Forward has neither a distance threshold nor a replacement time limit.
    Cross the former 620 count limit and the former 15s timeout. */
 CarMotion continuous;assert(car_motion_init(&continuous,0,0));assert(car_motion_start_scan(&continuous));
 for(unsigned t=1;t<=18000;++t){
  car_motion_tick_1ms(&continuous,(uint16_t)t,(uint16_t)t,true);
  assert(continuous.motor.error==DM_OK&&continuous.motor.running&&continuous.motor.continuous);
 }
 assert(continuous.motor.counts[0]==18000&&continuous.motor.wheel_goal[0]==0);
 assert(car_motion_finish_straight(&continuous));
 for(unsigned t=0;t<700;++t)car_motion_tick_1ms(&continuous,18000,18000,true);
 assert(car_motion_done(&continuous)&&!continuous.motor.continuous);
 memset(&a,0,sizeof a);memset(&f,0,sizeof f);a.phase=ALIGN_FORWARD;a.forward_requested=true;
 align_step(&a,20000,0,0,0,false,0,&f);assert(a.phase==ALIGN_FORWARD);
 /* Insufficient data does not dead-end after stopping; skip angle, back450. */
 memset(&a,0,sizeof a);a.phase=ALIGN_SCAN_BRAKE;a.scan_stop_requested=true;
 align_step(&a,100,0,0,0,false,0,&f);
 assert(a.phase==ALIGN_WAIT_FINAL&&a.line_warning&&a.error==ALIGN_BAD_LINE);
 align_step(&a,600,0,0,0,false,0,&f);assert(a.phase==ALIGN_FINAL_BACK);
 puts("PASS: continuous forward past620 and15s, exit-only finish, quality warnings continue, stop before fit, both sides/signs right-wheel-only fine turn, skip tiny correction, final reverse450, outlier/duplicate/hardware-fault/cancel checks");
}
