#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static MotionPd m;
static double pos[2];
static void init(void){motion_init(&m,0,0);pos[0]=pos[1]=0;motion_cal_start(&m);}
static void tick(float rpm){
 for(unsigned w=0;w<2;++w)if(m.phase==MP_RUN&&!m.braked[w])pos[w]+=copysign(rpm,m.wheel[w].duty==0?m.goal[w]:m.wheel[w].duty)*840.0/60000.0;
 motion_tick(&m,(uint16_t)(int32_t)pos[0],(uint16_t)(int32_t)pos[1],true);
}
int main(void){
 init();
 for(unsigned t=0;t<240000&&m.running;++t){
  tick(60);
  assert(m.error==MP_OK);assert(m.cal_count<=CAL_RESULTS);
  if(m.phase==MP_RUN){unsigned w=m.cal_group/2;assert(m.braked[1-w]);assert(fabsf(m.wheel[1-w].duty)<.00001);assert(fabsf(m.wheel[w].duty)<=.40001);}
 }
 assert(!m.running&&!m.enable&&m.phase==MP_STOPPED&&m.cal_count==32);
 for(unsigned i=0;i<32;++i){CalResult *r=&m.cal_result[i];assert(r->valid&&r->samples==3000);assert(fabsf(r->mean_rpm-60)<.1);assert(r->group==i/8&&r->step==i%8);}
 init();for(unsigned t=0;t<500;++t)tick(60);motion_stop(&m);for(unsigned t=0;t<100;++t)tick(0);
 assert(!m.enable&&!m.running&&m.phase==MP_STOPPED&&m.cal_count==0);
 init();for(unsigned t=0;t<1000&&m.phase==MP_RUN;++t)tick(120);
 assert(m.cal_count==1&&m.cal_result[0].capped&&!m.cal_result[0].valid);
 for(unsigned t=0;t<2300;++t)tick(0);assert(m.cal_group==1&&m.cal_step==0);
 init();for(unsigned t=0;t<40000&&m.running;++t)tick(0);assert(m.error==MP_STALL&&!m.enable);
 init();motion_tick(&m,1000,0,true);assert(m.error==MP_ENCODER&&!m.enable);
 init();motion_tick(&m,0,0,false);assert(m.error==MP_DRIVER&&!m.enable);
 init();for(unsigned t=0;t<200&&m.running;++t)motion_tick(&m,(uint16_t)(-(int)t),0,true);
 assert(m.error==MP_DIRECTION&&!m.enable);
 puts("PASS: calibration sequence, 3s count statistics, one-wheel output, stop, speed cap, stall, encoder, driver, direction");
}
