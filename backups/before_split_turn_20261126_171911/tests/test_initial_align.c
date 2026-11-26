#include "initial_align.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define RAD .0174532925199433f
static void plane(InitialAlign *a,float phi,int lo,int hi){
 memset(a,0,sizeof(*a));for(int i=lo;i<=hi;++i)a->range[i]=(uint16_t)lroundf(150/cosf((90-5*i-phi)*RAD));
}
int main(void){
 InitialAlign a;plane(&a,20,10,17);assert(align_fit(&a)&&a.side_left&&fabsf(a.target_deg+70)<1&&a.moving_wheel==2&&a.motor_direction==-1);
 plane(&a,-20,19,26);assert(align_fit(&a)&&!a.side_left&&fabsf(a.target_deg-70)<1&&a.moving_wheel==2&&a.motor_direction==1);
 plane(&a,0,7,13);assert(align_fit(&a)&&a.side_left&&fabsf(a.target_deg+90)<1);
 plane(&a,0,23,29);assert(align_fit(&a)&&!a.side_left&&fabsf(a.target_deg-90)<1);
 plane(&a,20,10,17);a.range[13]=0;assert(align_fit(&a));
 plane(&a,20,10,17);for(unsigned i=0;i<37;++i)if(!a.range[i])a.range[i]=350;assert(align_fit(&a));
 memset(&a,0,sizeof a);assert(!align_fit(&a));a.range[18]=150;assert(!align_fit(&a));
 plane(&a,20,0,17);assert(!align_fit(&a));
 {const uint16_t recorded[37]={0,0,0,0,157,153,157,139,133,121,127,149,135,147,160,132,163,128,143,157,168,167,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};memset(&a,0,sizeof a);memcpy(a.range,recorded,sizeof recorded);assert(align_fit(&a));}
 {const uint16_t recorded[37]={0,0,0,0,170,139,139,146,160,139,121,121,130,135,135,135,132,146,173,160,157,0,0,0,0,0,0,0,0,0,0,0,0,0,0,305,305};memset(&a,0,sizeof a);memcpy(a.range,recorded,sizeof recorded);assert(align_fit(&a));}
 {const uint16_t recorded[37]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,138,155,135,135,135,141,153,132,132,158,128,173,154,0,0,0,0,0,0,0,308};memset(&a,0,sizeof a);memcpy(a.range,recorded,sizeof recorded);assert(align_fit(&a));}
 puts("PASS: existing edge/normal/side geometry and real NO_FACE replay retained, both-wheel request selected");
}
