/* Integration template only. Implement board_* functions in your hardware layer.
 * Do NOT compile this template into TEST alongside main.c.
 * This template is syntax-checked, but not flashed or a tested board driver. */
#include "../dual_motor.h"

extern uint16_t board_left_encoder_counter(void);
extern uint16_t board_right_encoder_counter(void);
extern bool board_driver_ok(void);
extern void board_apply_motor_output(const DualMotorStatus *output);
extern void board_hard_disable_motors(void);
extern uint32_t board_enter_critical(void);
extern void board_exit_critical(uint32_t previous);
static DualMotor car;

/* Call after encoders are initialized, before enabling the 1ms control timer. */
bool car_init(void){
 DualMotorConfig config;
 dual_motor_default_config(&config); /* V4 tuning; goal2000 and target50/50ms */
 bool ok=dual_motor_init(&car,&config,board_left_encoder_counter(),board_right_encoder_counter());
 board_hard_disable_motors();return ok;
}
/* Call from a real 1ms timer ISR; never use HAL_Delay here. */
void car_tick_1ms(void){
 DualMotorStatus output;
 dual_motor_tick_1ms(&car,board_left_encoder_counter(),board_right_encoder_counter(),board_driver_ok());
 dual_motor_get_status(&car,&output);
 /* enable=false: disable H bridge. brake=true: hardware short brake.
    Otherwise apply signed duty in the same drive/brake PWM scheme as TEST. */
 board_apply_motor_output(&output);
}
void car_start(void){
 uint32_t saved=board_enter_critical();dual_motor_start(&car);board_exit_critical(saved);
}
void car_stop(void){
 uint32_t saved=board_enter_critical();dual_motor_stop(&car);board_exit_critical(saved);
}
void car_read_status(DualMotorStatus *output){
 uint32_t saved=board_enter_critical();dual_motor_get_status(&car,output);board_exit_critical(saved);
}
