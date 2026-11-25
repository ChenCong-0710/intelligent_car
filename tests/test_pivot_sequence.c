#include "motion_pd.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static MotionPd m;static uint16_t right;
static void init(void){right=0;motion_init(&m,0,0);motion_start(&m);}
static void step(int delta,bool ok){right=(uint16_t)(right+delta);motion_tick(&m,0,right,ok);}
static void until_wait(void){
 for(unsigned t=0;t<2000&&motion_sequence_stage()!=MS_WAIT;++t)step(m.phase==DM_RUN?m.move_direction:0,true);
 assert(motion_sequence_stage()==MS_WAIT);
}
int main(void){
 assert(motion_pivot_counts(30)==204&&motion_pivot_counts(120)==814&&motion_pivot_counts(60)==407&&motion_pivot_counts(0)==0);
 init();unsigned pauses=0;uint32_t wait_at=0;bool coast[4]={0};int32_t expected[]={207,817,404,-3};
 for(unsigned t=0;t<30000&&motion_sequence_stage()!=MS_DONE;++t){
  MotionSequenceStage before=motion_sequence_stage();unsigned old=motion_pivot_step()-1;MotionPhase oldphase=m.phase;
  int delta=0;
  if(m.phase==DM_RUN)delta=m.move_direction;
  else if(m.phase==DM_BRAKE&&!coast[old]){coast[old]=true;delta=3*m.move_direction;}
  step(delta,true);assert(m.error==DM_OK);assert(m.braked[0]&&m.wheel[0].duty==0);
  if(oldphase==DM_RUN&&m.phase==DM_BRAKE){
   assert(g_pivot_net_counts[1]==motion_pivot_absolute_goal());
   assert(m.goal_trigger[1]==m.wheel_goal[1]&&m.wheel[1].duty==0);
  }
  MotionSequenceStage after=motion_sequence_stage();
  if(before!=MS_WAIT&&after==MS_WAIT){
   ++pauses;wait_at=m.now;assert(motion_wait_remaining(&m)==5000);assert(motion_busy(&m));
   assert(g_pivot_stage_results[old]==expected[old]);
  }
  if(before==MS_WAIT&&after==MS_WAIT)assert(m.now-wait_at<5000);
  if(before==MS_WAIT&&after!=MS_WAIT){
   assert(m.now-wait_at==5000);unsigned current=motion_pivot_step()-1;
   int32_t needed=motion_pivot_absolute_goal()-expected[current-1];
   assert(m.wheel_goal[1]==(needed<0?-needed:needed));
   assert(m.move_direction==(current==1?1:-1));
  }
 }
 assert(pauses==3&&motion_sequence_stage()==MS_DONE&&!motion_busy(&m));
 for(unsigned i=0;i<4;++i)assert(g_pivot_stage_results[i]==expected[i]);
 assert(g_pivot_net_counts[1]==-3&&m.braked[0]&&m.braked[1]);
 for(unsigned t=0;t<6000;++t)step(0,true);assert(motion_sequence_stage()==MS_DONE);
 init();until_wait();motion_stop(&m);for(unsigned t=0;t<6000;++t)step(0,true);assert(motion_sequence_stage()==MS_CANCELLED&&!m.running&&!m.enable);
 init();until_wait();step(0,false);for(unsigned t=0;t<6000;++t)step(0,true);assert(motion_sequence_stage()==MS_FAULT&&!m.enable);
 /* A complete asymmetric-load/signed encoder model checks repeated reversals. */
 init();double pos=0,speed=0;
 for(unsigned t=0;t<90000&&motion_sequence_stage()!=MS_DONE;++t){
  float u=m.wheel[1].duty;assert(m.braked[0]&&m.wheel[0].duty==0);
  double dead=u>=0?.35:.34;
  double eq=m.enable&&!m.braked[1]?copysign(fmax(0,(fabs(u)-dead)*250),u):0;
  speed+=(eq-speed)/(m.braked[1]?12:80);pos+=speed/50;
  right=(uint16_t)(int32_t)pos;motion_tick(&m,0,right,true);assert(m.error==DM_OK);
 }
 assert(motion_sequence_stage()==MS_DONE&&!motion_busy(&m)&&g_pivot_net_counts[0]==0);
 assert(abs(g_pivot_net_counts[1])<=15);
 printf("Sequence model net stopped positions: %ld,%ld,%ld,%ld (software only)\n",(long)g_pivot_stage_results[0],(long)g_pivot_stage_results[1],(long)g_pivot_stage_results[2],(long)g_pivot_stage_results[3]);
 puts("PASS: absolute30/120/60/0, actual-coast-based segment goals, 3 settled waits exactly5s, fixed-left, reverse/brake, no post-stop correction/loop, cancel/fault in wait");
}
