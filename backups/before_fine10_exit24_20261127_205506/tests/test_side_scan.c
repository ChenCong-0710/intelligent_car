#include "box_center.h"
#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static uint32_t seq;static int32_t total[2];
static void start(BoxCenter *b,bool left){
 seq=0;total[0]=total[1]=0;box_center_start(b,0,0);b->side_left=left;b->fine_enabled=false;
 b->crossings_required=2;b->side_scan.enabled=true;b->reference_mm=100;
 b->phase=BC_BRAKE;b->edge=0;b->visits[0]=2;b->origin_set=true;b->command=BC_CMD_NONE;
 box_center_step(b,100,0,0,0,true,100,total,false,true,false,false);
 assert(b->phase==BC_SIDE_SCAN&&b->visits[1]==0&&b->command==BC_CMD_NONE);
 assert(box_center_servo_us(b,0)==(left?2400:500));
}
static void point(BoxCenter *b,const uint16_t v[4],bool valid){
 uint32_t since=b->side_scan.since;
 for(unsigned k=0;k<4;++k){
  uint32_t t=since+60*(k+1);++seq;
  box_center_step(b,t,seq,t,t,valid,v[k],total,false,true,false,false);
  assert(b->side_scan.attempts==k+1);
  box_center_step(b,t+1,seq,t,t,valid,v[k],total,false,true,false,false);
  assert(b->side_scan.attempts==k+1); /* Duplicate echo never adds a sample. */
  assert(b->phase==BC_SIDE_SCAN);
 }
 box_center_step(b,since+300,seq,since+240,since+240,valid,v[3],total,false,true,false,false);
}
int main(void){
 BoxCenter b;start(&b,true);
 box_center_step(&b,160,++seq,99,160,true,100,total,false,true,false,false);assert(!b.side_scan.attempts);
 uint16_t trim[4]={98,100,102,4000};point(&b,trim,true);assert(b.side_scan.range[0]==101&&b.side_scan.index==1);
 uint16_t far[4]={0,0,0,0};point(&b,far,false);assert(b.phase==BC_SIDE_RETURN&&b.side_scan.outside&&!b.side_scan.fit_ok&&b.fine_warning);
 assert(box_center_servo_us(&b,0)==2400);uint32_t now=b.since;
 box_center_step(&b,now+499,seq,now,now,true,100,total,false,true,false,false);assert(b.phase==BC_SIDE_RETURN);
 box_center_step(&b,now+500,seq,now,now,true,100,total,false,true,false,false);assert(b.phase==BC_TRANSIT&&b.command==BC_CMD_FORWARD);
 for(int side=0;side<2;++side)for(int sign=-1;sign<=1;sign+=2){
  start(&b,side!=0);float m=sign*.1f;
  for(unsigned i=0;i<=10;++i){
   float a=(90.f-5*i)*.01745329252f;float d=100/(sinf(a)-m*cosf(a));
   uint16_t v[4];for(unsigned k=0;k<4;++k)v[k]=i==10?800:(uint16_t)lroundf(d);
   point(&b,v,true);
  }
  assert(b.phase==BC_SIDE_RETURN&&b.side_scan.fit_ok&&b.fit_n==10);
  float expected=(b.side_left?1.f:-1.f)*atanf(m)*57.2957795f;
  assert(fabsf(b.fine_degrees-expected)<1.f);
  now=b.since+500;box_center_step(&b,now,seq,now,now,true,100,total,false,true,false,false);
  assert(b.phase==(b.side_left?BC_FINE_LEFT:BC_FINE_RIGHT));
  unsigned inner=b.side_left?0:1;b.command=BC_CMD_NONE;total[inner]+=50;
  box_center_step(&b,now+10,seq,now,now,true,100,total,false,true,false,false);assert(b.phase==BC_FINE_WAIT);
  box_center_step(&b,now+510,seq,now,now,true,100,total,false,true,false,false);
  assert(b.phase==(b.side_left?BC_FINE_RIGHT:BC_FINE_LEFT));b.command=BC_CMD_NONE;total[1-inner]-=50;
  box_center_step(&b,now+520,seq,now,now,true,100,total,false,true,false,false);assert(b.phase==BC_FINE_SETTLE);
  box_center_step(&b,now+1020,seq,now,now,true,100,total,false,true,false,false);
  assert(b.phase==BC_TRANSIT&&b.command==BC_CMD_FORWARD&&b.visits[0]==2&&b.visits[1]==0);
  assert(b.position==0&&b.origin[inner]==50&&b.origin[1-inner]==-50);
 }
 start(&b,false);uint16_t near[4]={250,250,250,250};
 for(unsigned i=0;i<19;++i){assert(b.side_scan.index==i);point(&b,near,true);}
 assert(b.phase==BC_SIDE_RETURN&&b.side_scan.limit&&!b.side_scan.outside&&b.side_scan.points==19);
 assert(box_center_servo_us(&b,0)==500); /*0 is the scan limit; always return to side90.*/
 start(&b,true);box_center_step(&b,110,seq,0,0,false,0,total,false,true,false,true);assert(b.phase==BC_CANCELLED);
 puts("PASS rear side scan: rear2 trigger, stale/duplicate rejection, four-sample trim, no-echo boundary, 0-degree limit, calibrated return, signed Cartesian fit, inner/outer order, retained counts and forward continuation");
}
