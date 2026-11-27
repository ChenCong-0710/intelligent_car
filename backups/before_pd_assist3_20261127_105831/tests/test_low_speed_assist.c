#include "car_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void init(DualMotor *m){
 DualMotorConfig c;dual_motor_default_config(&c);c.sync_enabled=false;c.cruise_target=20;
 assert(c.speed_kp==.004f&&c.speed_kd==.0015f);
 assert(dual_motor_init(m,&c,0,0));assert(dual_motor_start_continuous(m,1));
}
static void until(DualMotor *m,unsigned t,uint16_t l,uint16_t r){
 while(m->now<t){dual_motor_tick_1ms(m,l,r,true);assert(m->error==DM_OK);}
}
static void gain(const DualMotor *m,unsigned w,bool boosted){
 if(m->wheel[w].kp_boost!=boosted)fprintf(stderr,"gain mismatch time=%u wheel=%u speed=%.0f since=%u expected=%u actual=%u\n",m->now,w,m->wheel[w].speed_pulses,m->wheel[w].low_speed_since,boosted,m->wheel[w].kp_boost);
 assert(m->wheel[w].kp_boost==boosted);
 assert(fabsf(m->wheel[w].effective_kp-(boosted?.02f:.004f))<1e-7f);
}
int main(void){
 DualMotor m;init(&m);
 until(&m,500,0,0);gain(&m,0,false);gain(&m,1,false);
 until(&m,501,0,0);gain(&m,0,true);gain(&m,1,true);
 until(&m,549,0,0);dual_motor_tick_1ms(&m,9,8,true);
 assert(m.wheel[0].speed_pulses==9&&m.wheel[1].speed_pulses==8);
 gain(&m,0,false);gain(&m,1,true); /* Exactly8 holds an already boosted gain. */
 until(&m,599,9,8);dual_motor_tick_1ms(&m,18,17,true);gain(&m,1,false);
 until(&m,1150,18,17);gain(&m,0,false); /* First zero-speed window arrives at650. */
 until(&m,1151,18,17);gain(&m,0,true);
 DualMotorStatus s;dual_motor_get_status(&m,&s);
 assert(s.kp_boost[0]&&s.low_speed_ms[0]==501&&fabsf(s.effective_kp[0]-.02f)<1e-7f);
 dual_motor_stop(&m);gain(&m,0,false);gain(&m,1,false);
 until(&m,1301,18,17);assert(dual_motor_start_continuous(&m,-1));
 gain(&m,0,false);gain(&m,1,false); /* New action resets both gain timers. */
 unsigned begin=m.now;until(&m,begin+501,18,17);gain(&m,0,true);gain(&m,1,true);
 unsigned sample=((m.now-begin)/50+1)*50+begin;
 until(&m,sample-1,18,17);dual_motor_tick_1ms(&m,9,8,true);
 assert(m.wheel[0].speed_pulses==-9&&m.wheel[1].speed_pulses==-9);
 gain(&m,0,false);gain(&m,1,false); /* Reverse uses absolute speed. */
 dual_motor_tick_1ms(&m,9,8,false);assert(m.error==DM_DRIVER);gain(&m,0,false);gain(&m,1,false);

 init(&m);uint16_t raw=0;
 for(unsigned t=1;t<=1200;++t){if(t%50==0)raw+=8;dual_motor_tick_1ms(&m,raw,raw,true);gain(&m,0,false);gain(&m,1,false);}
 /* A single8 sample interrupts the continuous <8 interval. */
 for(unsigned t=1201;t<=1700;++t)dual_motor_tick_1ms(&m,raw,raw,true);
 gain(&m,0,false);until(&m,1751,raw,raw);gain(&m,0,true);

 DualMotorConfig c;dual_motor_default_config(&c);c.sync_enabled=false;
 assert(dual_motor_init(&m,&c,0,0));assert(dual_motor_start_wheels(&m,1,0,100000));
 until(&m,501,0,0);gain(&m,0,false);gain(&m,1,true); /* Held wheel receives no boost. */
 assert(m.braked[0]);
 assert(dual_motor_init(&m,&c,0,0));assert(dual_motor_start_wheels(&m,1,2,100000));
 dual_motor_tick_1ms(&m,2,0,true);until(&m,501,2,0);gain(&m,0,false);gain(&m,1,true);

 /* Verify the effective gain actually enters the position-PD output, with
    slew/limit disabled for this numeric check only. */
 init(&m);m.config.duty_slew=1;m.config.max_duty=1;m.config.speed_kd=0;
 until(&m,500,0,0);
 float base=dual_motor_pd_duty(&m.config,0,20,0,0);assert(fabsf(m.wheel[0].duty-base)<1e-6f);
 until(&m,550,0,0);
 DualMotorConfig boosted=m.config;boosted.speed_kp=.02f;
 float expected=dual_motor_pd_duty(&boosted,0,20,0,0);
 assert(fabsf(m.wheel[0].duty-expected)<1e-6f&&m.wheel[0].duty>base);
 puts("PASS: P/D restored; per-wheel <8 for >500ms boosts P5x; >8 restores immediately; equality8, interruptions, reverse, brake/fault/restart and actual PWM path");
}
