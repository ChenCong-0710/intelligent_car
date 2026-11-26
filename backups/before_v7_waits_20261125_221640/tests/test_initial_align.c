#include "initial_align.h"
#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define R .0174532925199433f
static void plane(InitialAlign *a,float phi,int lo,int hi){
 memset(a,0,sizeof(*a));
 for(int i=lo;i<=hi;++i)a->range[i]=(uint16_t)lroundf(150/cosf((90-5*i-phi)*R));
}
static AlignMotionFeedback feedback(const CarMotion *m){
 AlignMotionFeedback f={m->motor.running,car_motion_done(m),m->motor.phase==DM_FAULT,car_motion_estimated_angle(m),(uint32_t)m->motor.error,{m->motor.counts[0],m->motor.counts[1]}};return f;
}
static void finish_case(float phi,int lo,int hi,bool left,unsigned wheel,float turn){
 InitialAlign a;CarMotion car;plane(&a,phi,lo,hi);a.phase=ALIGN_FIT;
 assert(car_motion_init(&car,0,0));AlignMotionFeedback f=feedback(&car);uint32_t now=20000;
 align_step(&a,now,0,0,0,false,0,&f);
 assert(a.phase==ALIGN_SIDE_SETTLE&&a.side_left==left&&a.moving_wheel==wheel&&a.motor_direction==-1);
 assert(fabsf(a.target_deg-turn)<1);assert(a.servo_us==(left?2400:500));
 align_step(&a,now+1199,0,0,0,false,0,&f);assert(a.phase==ALIGN_SIDE_SETTLE&&!car.motor.enable);
 now+=1200;align_step(&a,now,0,0,0,false,0,&f);assert(a.phase==ALIGN_TURN&&!a.turn_requested);
 assert(car_motion_start_pivot(&car,a.moving_wheel,a.motor_direction,fabsf(a.target_deg)));a.turn_requested=true;
 assert(car.motor.braked[1-wheel]&&!car.motor.braked[wheel]);
 assert(car.motor.wheel_goal[wheel]==car_motion_pivot_counts(wheel,fabsf(a.target_deg)));
 uint16_t count[2]={0,0};
 for(unsigned t=0;t<3000&&a.phase==ALIGN_TURN;++t){
  ++now;if(car.motor.phase==DM_RUN&&!car.motor.braked[wheel])count[wheel]=(uint16_t)(count[wheel]-1);
  car_motion_tick_1ms(&car,count[0],count[1],true);assert(car.motor.error==DM_OK);
  assert(car.motor.braked[1-wheel]&&car.motor.wheel[1-wheel].duty==0);
  assert(car.motor.wheel[wheel].duty<=0);
  f=feedback(&car);align_step(&a,now,0,0,0,false,0,&f);
 }
 assert(a.phase==ALIGN_BRAKE&&car_motion_done(&car));assert(fabsf(a.actual_deg-turn)<1);
 now+=300;align_step(&a,now,0,0,0,false,0,&f);assert(a.phase==ALIGN_LOCK);
 now+=1250;align_step(&a,now,1,now-30,now-20,true,150,&f);
 assert(a.phase==ALIGN_RETREAT&&!a.retreat_requested);
 /* Old pivot DONE cannot skip this new segment; origins reset per move. */
 align_step(&a,++now,1,0,0,false,0,&f);assert(a.phase==ALIGN_RETREAT);
 assert(car_motion_start_straight(&car,-1,ALIGN_RETREAT_COUNTS));a.retreat_requested=true;
 assert(car.motor.actual[0]==0&&car.motor.actual[1]==0);
 assert(car.motor.config.cruise_target==40&&car.motor.config.sync_enabled&&car.motor.config.soft_stop_ms==300);
 unsigned retreat[2]={0,0};bool crossed[2]={false,false};
 for(unsigned t=0;t<1800&&a.phase==ALIGN_RETREAT;++t){
  ++now;
  for(unsigned w=0;w<2;++w){
   /* Lag one wheel, allow both to coast beyond the goal during soft stopping. */
   if(retreat[w]<ALIGN_RETREAT_COUNTS+20U&&(w==0||t%7!=0)){--count[w];++retreat[w];}
  }
  car_motion_tick_1ms(&car,count[0],count[1],true);
  assert(car.motor.error==DM_OK);
  for(unsigned w=0;w<2;++w){
   assert(car.motor.wheel[w].duty<=0); /* Never reverse to fill a position. */
   if(car.motor.decelerating[w]||car.motor.braked[w]){
    assert(car.motor.goal_trigger[w]>=ALIGN_RETREAT_COUNTS);crossed[w]=true;
   }
  }
  f=feedback(&car);align_step(&a,now,1,0,0,false,0,&f);
 }
 assert(a.phase==ALIGN_FORWARD&&car_motion_done(&car)&&crossed[0]&&crossed[1]);
 assert(car.motor.counts[0]==ALIGN_RETREAT_COUNTS+20U&&car.motor.counts[1]==ALIGN_RETREAT_COUNTS+20U);
 assert(car.motor.wheel_goal[0]==ALIGN_RETREAT_COUNTS&&car.motor.wheel_goal[1]==ALIGN_RETREAT_COUNTS);
 align_step(&a,now+10000,2,now+9990,now+9995,true,140,&f);assert(a.phase==ALIGN_FORWARD);
}
int main(void){
 InitialAlign a;AlignMotionFeedback idle={0};
 /* Feed two fresh echoes per actual scan position, not prefilled fit input. */
 for(unsigned side=0;side<2;++side){
  InitialAlign input;plane(&input,side?-20:20,side?19:10,side?26:17);
  align_start(&a,0);uint32_t now=0,seq=0;
  for(unsigned i=0;i<37;++i){
   uint32_t settle=i==0?1200:120;uint16_t value=input.range[i];
   align_step(&a,now+settle+20,++seq,now+settle+10,now+settle+15,value!=0,value,&idle);
   align_step(&a,now+settle+80,++seq,now+settle+70,now+settle+75,value!=0,value,&idle);
   now+=settle+180;align_step(&a,now,seq,now-110,now-105,false,0,&idle);
  }
  assert(a.phase==ALIGN_FIT);for(unsigned i=0;i<37;++i)assert(a.range[i]==input.range[i]);
  align_step(&a,now+5,seq,0,0,false,0,&idle);
  assert(a.phase==ALIGN_SIDE_SETTLE&&a.side_left==(side==0)&&a.moving_wheel==(side==0?1:0));
 }
 /* All37 calibrated anchors and unchanged left-to-right scan settle timing. */
 align_start(&a,0);uint32_t t=0;uint16_t previous=2400;
 for(unsigned i=0;i<37;++i){
  assert(a.index==i&&a.servo_us<=previous);previous=a.servo_us;
  if(i==0)assert(a.servo_us==2400);if(i==18)assert(a.servo_us==1460);if(i==36)assert(a.servo_us==500);
  t+=i==0?1380:300;align_step(&a,t,0,0,0,false,0,&idle);
 }
 assert(a.phase==ALIGN_FIT);align_step(&a,t+5,0,0,0,false,0,&idle);assert(a.phase==ALIGN_FAILED&&a.error==ALIGN_NO_FACE);
 plane(&a,20,10,17);assert(align_fit(&a));assert(a.side_left&&a.moving_wheel==1&&a.motor_direction==-1&&fabsf(a.target_deg+70)<1);
 assert(a.edge_left_seen&&a.edge_right_seen&&a.fit_first==10&&a.fit_last==17&&fabsf(a.edge_normal_deg-20)<1);
 plane(&a,-20,19,26);assert(align_fit(&a));assert(!a.side_left&&a.moving_wheel==0&&fabsf(a.target_deg-70)<1);
 /* Actual range discontinuities to a valid far background also bound the box. */
 plane(&a,20,10,17);for(unsigned i=0;i<37;++i)if(!a.range[i])a.range[i]=350;
 assert(align_fit(&a)&&a.fit_first==10&&a.fit_last==17);
 /* Offset location is decided by box centre, not by normal sign alone. */
 plane(&a,0,7,13);assert(align_fit(&a)&&a.side_left&&fabsf(a.normal_deg)<1&&fabsf(a.target_deg+90)<1);
 plane(&a,0,23,29);assert(align_fit(&a)&&!a.side_left&&fabsf(a.target_deg-90)<1);
 /* No scan limit/fitted-window edge can pretend to be paired physical edges. */
 plane(&a,20,0,17);assert(!align_fit(&a));
 memset(&a,0,sizeof(a));for(unsigned i=0;i<37;++i)a.range[i]=150;assert(!align_fit(&a));
 memset(&a,0,sizeof(a));for(unsigned i=0;i<37;++i){float c=cosf((90-5*(int)i)*R);if(c>0&&150/c<=400)a.range[i]=(uint16_t)lroundf(150/c);}assert(!align_fit(&a));
 plane(&a,20,10,17);a.range[13]=0;assert(align_fit(&a)&&a.fit_support==7&&a.fit_first==10&&a.fit_last==17);
 memset(&a,0,sizeof(a));a.range[18]=150;assert(!align_fit(&a));
 /* Paper-face ripples: +/-5 mm perpendicular deviation over most points.
    A separate +45 mm spurious echo is excluded, not averaged into the normal. */
 for(unsigned side=0;side<2;++side){
  float phi=side?-20:20;int lo=side?19:8,hi=side?30:19;
  plane(&a,phi,lo,hi);
  for(int i=lo+1;i<hi;++i){
   float c=cosf((90-5*i-phi)*R);
   a.range[i]=(uint16_t)lroundf((150+(i%3-1)*5)/c);
  }
  a.range[lo+5]+=45;
  assert(align_fit(&a));assert(a.side_left==(side==0));
  assert(fabsf(a.normal_deg-phi)<6);
  assert(a.fit_total==12&&a.fit_support>=9&&a.fit_support<a.fit_total);
  assert(a.rms_mm<=8);
 }
 /* Widespread alternating +/-25 mm defects are not a reliable flat face. */
 plane(&a,0,10,25);
 for(int i=11;i<25;++i)a.range[i]+=(i%2?25:-25);
 assert(!align_fit(&a));
 /* Duplicate/stale/servo-in-transit measurements do not become scan hits. */
 align_start(&a,100);align_step(&a,1350,1,1340,1345,true,150,&idle);assert(a.hits==1);
 align_step(&a,1360,1,1340,1345,true,150,&idle);assert(a.hits==1);
 align_step(&a,1370,2,1200,1365,true,150,&idle);assert(a.hits==1);
 align_step(&a,1480,3,1420,1425,true,150,&idle);assert(a.range[0]==150&&a.index==1);
 align_step(&a,1780,3,1420,1425,true,150,&idle);assert(a.range[1]==0);
 finish_case(20,10,17,true,1,-70);finish_case(-20,19,26,false,0,70);
 finish_case(60,4,8,true,1,-30);finish_case(-60,28,32,false,0,30);
 /* Motor completion, not a heading estimate or elapsed delay, ends rotation. */
 plane(&a,20,10,17);a.phase=ALIGN_FIT;align_step(&a,1000,0,0,0,false,0,&idle);
 align_step(&a,2200,0,0,0,false,0,&idle);assert(a.phase==ALIGN_TURN);
 a.turn_requested=true;AlignMotionFeedback busy={true,false,false,-70,0,{0,0}};
 align_step(&a,2300,0,0,0,false,0,&busy);assert(a.phase==ALIGN_TURN);
 align_step(&a,17300,0,0,0,false,0,&busy);assert(a.phase==ALIGN_FAILED&&a.error==ALIGN_TURN_TIMEOUT);
 plane(&a,20,10,17);a.phase=ALIGN_TURN;AlignMotionFeedback fault={false,false,true,0,DM_STALL,{0,0}};
 align_step(&a,100,0,0,0,false,0,&fault);assert(a.error==ALIGN_MOTOR&&a.motor_error==DM_STALL);
 memset(&a,0,sizeof(a));a.phase=ALIGN_LOCK;a.since=100;align_step(&a,2700,0,0,0,false,0,&idle);assert(a.error==ALIGN_SIDE_NO_ECHO);
 memset(&a,0,sizeof(a));a.phase=ALIGN_RETREAT;a.since=100;
 align_step(&a,15100,0,0,0,false,0,&idle);assert(a.phase==ALIGN_FAILED&&a.error==ALIGN_RETREAT_TIMEOUT);
 memset(&a,0,sizeof(a));a.phase=ALIGN_RETREAT;a.retreat_requested=true;
 align_step(&a,100,0,0,0,false,0,&fault);assert(a.error==ALIGN_MOTOR);
 a.phase=ALIGN_RETREAT;align_cancel(&a);align_step(&a,999,0,0,0,false,0,&idle);assert(a.phase==ALIGN_CANCELLED);
 align_start(&a,0);align_cancel(&a);align_step(&a,99999,0,0,0,false,0,&idle);assert(a.phase==ALIGN_CANCELLED);
 puts("PASS: retreat150 after lock / per-segment origin / allow coast / no reversal / completion timeout cancel fault, paper ripples / isolated outlier / reject widespread defects, paired discontinuities/edge normal/plane fit, offset side choice, no fake scan edges, calibrated37-point scan, side servo settled BEFORE move, left->right reverse/right->left reverse, completion/fault/cancel/echo freshness");
}
