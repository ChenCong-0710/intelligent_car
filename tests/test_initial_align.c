#include "initial_align.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define R 0.01745329252f
static void plane(InitialAlign *a,float phi,int lo,int hi) {
 memset(a,0,sizeof(*a));
 for(int i=lo;i<=hi;++i)a->range[i]=(uint16_t)lroundf(150/cosf((90-5*i-phi)*R));
}
int main(void) {
 InitialAlign a;float v[2]={0,0};
 plane(&a,0,14,22);assert(align_fit(&a));assert(fabsf(a.normal_deg)<1);
 assert(fabsf(fabsf(a.target_deg)-90)<1);assert(a.edge_left_seen&&a.edge_right_seen);
 plane(&a,20,10,17);assert(align_fit(&a));assert(fabsf(a.normal_deg-20)<1);
 assert(a.side_left&&fabsf(a.target_deg+70)<1);
 plane(&a,-20,19,26);assert(align_fit(&a));assert(fabsf(a.normal_deg+20)<1);
 assert(!a.side_left&&fabsf(a.target_deg-70)<1);
 plane(&a,0,0,36);assert(align_fit(&a));assert(!a.edge_left_seen&&!a.edge_right_seen);
 memset(&a,0,sizeof(a));assert(!align_fit(&a));a.range[18]=150;assert(!align_fit(&a));
 plane(&a,0,16,20);assert(align_fit(&a));
 plane(&a,25,10,16);
 for(int i=17;i<=23;++i)a.range[i]=(uint16_t)lroundf(150/cosf((90-5*i+25)*R));
 assert(align_fit(&a));assert(fabsf(a.normal_deg)>15);
 plane(&a,20,10,17);a.range[13]=0;assert(align_fit(&a));
 align_start(&a,100);
 align_step(&a,1350,1,1340,1345,true,150,0,v);assert(a.hits==1);
 align_step(&a,1360,1,1340,1345,true,150,0,v);assert(a.hits==1);
 align_step(&a,1370,2,1200,1365,true,150,0,v);assert(a.hits==1);
 align_step(&a,1480,3,1420,1425,true,150,0,v);assert(a.range[0]==150&&a.index==1);
 align_step(&a,1780,3,1420,1425,true,150,0,v);assert(a.range[1]==0);
 plane(&a,20,10,17);a.phase=ALIGN_FIT;
 align_step(&a,20000,0,0,0,false,0,0,v);assert(a.phase==ALIGN_TURN);
 align_step(&a,20005,0,0,0,false,0,0,v);assert(a.speed[0]>0&&a.speed[1]<0);
 assert(fabsf(a.speed[0]+a.speed[1])<0.001f);
 align_step(&a,20100,0,0,0,false,0,-69*R,v);assert(a.phase==ALIGN_BRAKE);
 align_step(&a,20400,0,0,0,false,0,-70*R,v);assert(a.phase==ALIGN_LOCK);
 assert(a.servo_us==ALIGN_LEFT_US);
 align_step(&a,21700,1,21650,21655,true,150,-70*R,v);assert(a.phase==ALIGN_DONE);
 align_step(&a,30000,2,29950,29955,true,150,-70*R,v);assert(a.phase==ALIGN_DONE);
 plane(&a,-20,19,26);a.phase=ALIGN_FIT;align_step(&a,20000,0,0,0,false,0,0,v);
 align_step(&a,20005,0,0,0,false,0,0,v);assert(a.speed[0]<0&&a.speed[1]>0);
 align_step(&a,36000,0,0,0,false,0,0,v);assert(a.error==ALIGN_TURN_TIMEOUT);
 plane(&a,20,10,17);a.phase=ALIGN_FIT;align_step(&a,20000,0,0,0,false,0,0,v);
 v[0]=-40;v[1]=40;align_step(&a,20005,0,0,0,false,0,0,v);assert(a.phase==ALIGN_TURN);
 align_step(&a,20200,0,0,0,false,0,0,v);assert(a.error==ALIGN_ENCODER);
 memset(&a,0,sizeof(a));a.phase=ALIGN_LOCK;a.since=100;
 align_step(&a,2700,0,0,0,false,0,0,v);assert(a.error==ALIGN_SIDE_NO_ECHO);
 memset(&a,0,sizeof(a));a.phase=ALIGN_CANCELLED;
 align_step(&a,9000,0,0,0,false,0,0,v);assert(a.phase==ALIGN_CANCELLED&&a.speed[0]==0);
 puts("PASS: geometry, corner splitting, dropout, stale/duplicate echo, signed turn, lock, timeout, encoder fault, cancel");
}
