#include "../portable/dual_motor/dual_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
 DualMotor a,b;DualMotorConfig ca,cb;DualMotorStatus out;
 dual_motor_default_config(&ca);cb=ca;cb.goal_counts=100;cb.cruise_target=40;
 assert(dual_motor_init(&a,&ca,65530,65530));assert(dual_motor_init(&b,&cb,0,0));
 dual_motor_start(&a);dual_motor_start(&b);
 for(unsigned i=1;i<=100;++i){dual_motor_tick_1ms(&a,(uint16_t)(65530+i),(uint16_t)(65530+i),true);dual_motor_tick_1ms(&b,i,i,true);}
 assert(a.phase==DM_RUN&&b.phase==DM_BRAKE);assert(a.counts[0]==100&&b.counts[0]==100);
 assert(a.config.goal_counts==2000&&b.config.goal_counts==100);
 dual_motor_get_status(&b,&out);assert(out.brake[0]&&out.brake[1]&&out.duty[0]==0&&out.counts[0]==100);
 ca.speed_ms=0;assert(!dual_motor_init(&a,&ca,0,0));assert(!a.enable&&a.phase==DM_FAULT);dual_motor_start(&a);assert(!a.enable&&!a.running);
 dual_motor_default_config(&ca);ca.speed_kp=NAN;assert(!dual_motor_config_valid(&ca));
 assert(dual_motor_init(&a,0,0,0));assert(a.config.sync_count_kp==.55f);
 puts("PASS: portable API, independent instances/configuration, snapshot, invalid-config fail-closed");
}
