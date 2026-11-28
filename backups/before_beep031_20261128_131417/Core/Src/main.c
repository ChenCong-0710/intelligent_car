#include "main.h"
#include "lcd_font.h"
#include "initial_align.h"
#include "car_motion.h"
#include "box_center.h"
#include "four_faces.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#define PWM_PERIOD 799U
#define MOTOR_SIGN_L 1
#define MOTOR_SIGN_R 1
static TIM_HandleTypeDef motor_pwm,enc_left,enc_right,us_trig_timer,us_echo_timer;
InitialAlign g_align;
CarMotion g_car;
BoxCenter g_box_center;
FourFaces g_four_faces;
static volatile uint16_t a_edge_counter[2];
static volatile bool a_capture_fault,cancel_requested;
static volatile uint32_t irq_max_cycles;
static uint32_t key_low_ms,last_step,us_trigger_ms,us_echo_ms,us_seq;
static uint16_t us_mm;
static bool us_valid,us_pending;
/* Completed transaction is immutable until the next result. A new trigger
   must not invalidate or relabel the preceding result before the task reads it. */
static uint32_t us_result_trigger_ms,us_period_ms,us_result_period_ms;
static uint16_t us_pulse_us;
static const char *us_reason="INIT";

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void car_emergency_off(void){
 if(RCC->AHB2ENR&RCC_AHB2ENR_GPIOCEN)GPIOC->BSRR=(uint32_t)(GPIO_PIN_8|GPIO_PIN_9)<<16;
 if(RCC->AHB2ENR&RCC_AHB2ENR_GPIODEN)GPIOD->BSRR=(uint32_t)GPIO_PIN_2<<16;
 if(RCC->APB2ENR&RCC_APB2ENR_TIM1EN)TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=0;
}
static void motor_off(void){car_emergency_off();}
static void gpio(GPIO_TypeDef *port,uint32_t pin,uint32_t mode,uint32_t pull,uint32_t af) {
    GPIO_InitTypeDef g={0};g.Pin=pin;g.Mode=mode;g.Pull=pull;
    g.Speed=GPIO_SPEED_FREQ_HIGH;g.Alternate=af;HAL_GPIO_Init(port,&g);
}
static void require_ok(HAL_StatusTypeDef r) {if(r!=HAL_OK) Error_Handler();}
static void motor_hardware_init(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  SET_BIT(PWR->CR3,PWR_CR3_UCPD_DBDIS);
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};
  TIM_IC_InitTypeDef ic = {0};
  TIM_HandleTypeDef *encoders[2] = {&enc_left, &enc_right};

  /* All four bridge inputs start low; PD2 stays disabled during setup. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &gpio);
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &gpio);

  /* Buttons: PD0 start/stop, PD1 target, PD15 air/ground. Active low. */
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &gpio);
  /* DRV8833 nFAULT is low active. */
  gpio.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOC, &gpio);

  motor_pwm.Instance = TIM1;
  motor_pwm.Init.Prescaler = 0U;
  motor_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
  motor_pwm.Init.Period = PWM_PERIOD;
  motor_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  motor_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&motor_pwm) != HAL_OK) Error_Handler();
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  const uint32_t channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
  for (unsigned i = 0U; i < 4U; ++i) {
    if (HAL_TIM_PWM_ConfigChannel(&motor_pwm, &oc, channels[i]) != HAL_OK) Error_Handler();
  }
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOE, &gpio);
  for (unsigned i = 0U; i < 4U; ++i) {
    if (HAL_TIM_PWM_Start(&motor_pwm, channels[i]) != HAL_OK) Error_Handler();
  }

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &gpio);
  enc_left.Instance = TIM4;
  enc_right.Instance = TIM3;
  ic.ICPolarity=TIM_ICPOLARITY_BOTHEDGE;ic.ICSelection=TIM_ICSELECTION_DIRECTTI;
  ic.ICPrescaler=TIM_ICPSC_DIV1;ic.ICFilter=6U;
  for (unsigned i = 0U; i < 2U; ++i) {
    TIM_HandleTypeDef *timer = encoders[i];
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 65535U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if(HAL_TIM_IC_Init(timer)!=HAL_OK)Error_Handler();
    if(HAL_TIM_IC_ConfigChannel(timer,&ic,TIM_CHANNEL_1)!=HAL_OK)Error_Handler();
    IRQn_Type irq=i==0?TIM4_IRQn:TIM3_IRQn;
    HAL_NVIC_SetPriority(irq,1,0);HAL_NVIC_EnableIRQ(irq);
    if(HAL_TIM_IC_Start_IT(timer,TIM_CHANNEL_1)!=HAL_OK)Error_Handler();
  }
  motor_off();
}

static void output_apply(void){
    if(!g_car.motor.enable){motor_off();return;}
    const int signs[2]={MOTOR_SIGN_L,MOTOR_SIGN_R};
    uint32_t value[4];
    for(unsigned w=0;w<2;++w){
        float d=car_motion_output_duty(&g_car,w)*signs[w];
        /* Drive/brake PWM: both inputs high at zero effort. Braked wheels hold
           independently while the other wheel reaches its encoder endpoint. */
        uint32_t active=(uint32_t)lroundf(fminf(fabsf(d),g_car.motor.config.max_duty)*800.0f);
        value[w*2]=800;value[w*2+1]=800;
        if(!g_car.motor.braked[w]){
            if(d>=0)value[w*2+1]=800-active;
            else value[w*2]=800-active;
        }
    }
    TIM1->CCR1=value[0];TIM1->CCR2=value[1];TIM1->CCR3=value[2];TIM1->CCR4=value[3];
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
}
static void a_edge_irq(TIM_TypeDef *tim,GPIO_TypeDef *port,uint32_t apin,uint32_t bpin,unsigned wheel){
 uint32_t status=tim->SR;
 if(status&TIM_SR_CC1IF){
  (void)tim->CCR1;
  if(status&TIM_SR_CC1OF){tim->SR=~TIM_SR_CC1OF;a_capture_fault=true;return;}
  uint32_t input=port->IDR;
  bool a=(input&apin)!=0,b=(input&bpin)!=0;
  a_edge_counter[wheel]=(uint16_t)(a_edge_counter[wheel]+(a==b?1:-1));
 }
}
void TIM4_IRQHandler(void){a_edge_irq(TIM4,GPIOB,GPIO_PIN_6,GPIO_PIN_7,0);}
void TIM3_IRQHandler(void){a_edge_irq(TIM3,GPIOA,GPIO_PIN_6,GPIO_PIN_7,1);}
static void sample_timer_init(void){
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->PSC=15;TIM6->ARR=999;TIM6->EGR=TIM_EGR_UG;TIM6->SR=0;TIM6->DIER=TIM_DIER_UIE;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,2,0);HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1=TIM_CR1_CEN;
}
void TIM6_DAC_IRQHandler(void){
 if(!(TIM6->SR&TIM_SR_UIF))return;
 TIM6->SR=~TIM_SR_UIF;uint32_t begin=DWT->CYCCNT;
 if(!(GPIOD->IDR&GPIO_PIN_0)){if(key_low_ms<1000)++key_low_ms;}else key_low_ms=0;
 if(key_low_ms>=20&&!cancel_requested){cancel_requested=true;car_motion_stop(&g_car);}
 car_motion_tick_1ms(&g_car,a_edge_counter[0],a_edge_counter[1],(GPIOC->IDR&GPIO_PIN_4)!=0);
 if(a_capture_fault)car_motion_trip(&g_car,DM_ENCODER);
 output_apply();uint32_t elapsed=DWT->CYCCNT-begin;if(elapsed>irq_max_cycles)irq_max_cycles=elapsed;
}
typedef struct {DualMotorStatus status;int32_t total[2];bool done;float actual_deg,wheel_degrees[2];uint32_t irq_us;} MotorView;
static MotorView motor_snapshot(void){
 MotorView v;uint32_t saved=__get_PRIMASK();__disable_irq();
 car_motion_get_status(&g_car,&v.status);v.done=car_motion_done(&g_car);
 v.actual_deg=car_motion_estimated_angle(&g_car);
 for(unsigned w=0;w<2;++w)v.wheel_degrees[w]=car_motion_wheel_angle(&g_car,w);
 for(unsigned w=0;w<2;++w)v.total[w]=g_car.motor.wheel[w].total;
 v.irq_us=irq_max_cycles/16U;
 __set_PRIMASK(saved);return v;
}
static void ultrasonic_init(void) {
    /* 1. PB9 -> TIM17_CH1 (AF1) 硬件 45 us 单脉冲触发器 */
    __HAL_RCC_TIM17_CLK_ENABLE();
    us_trig_timer.Instance = TIM17;
    us_trig_timer.Init.Prescaler = 15; /* 1 MHz (1 us) */
    us_trig_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    us_trig_timer.Init.Period = 49; /* 5 us 准备 + 45 us 高电平脉冲 */
    us_trig_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    us_trig_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    require_ok(HAL_TIM_PWM_Init(&us_trig_timer));
    require_ok(HAL_TIM_OnePulse_Init(&us_trig_timer, TIM_OPMODE_SINGLE));
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM2;
    oc.Pulse = 5;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    require_ok(HAL_TIM_PWM_ConfigChannel(&us_trig_timer, &oc, TIM_CHANNEL_1));
    gpio(GPIOB, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF1_TIM17);
    SET_BIT(TIM17->CCER, TIM_CCER_CC1E);
    SET_BIT(TIM17->BDTR, TIM_BDTR_MOE);

    /* 2. PC6 -> TIM8_CH1 (AF4) 硬件双边沿 PWM 输入捕获 */
    __HAL_RCC_TIM8_CLK_ENABLE();
    gpio(GPIOC, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_AF4_TIM8);
    us_echo_timer.Instance = TIM8;
    us_echo_timer.Init.Prescaler = 15; /* 1 MHz (1 us) */
    us_echo_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    us_echo_timer.Init.Period = 65535;
    us_echo_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    require_ok(HAL_TIM_IC_Init(&us_echo_timer));
    TIM_IC_InitTypeDef ic = {0};
    ic.ICPolarity = TIM_ICPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter = 4;
    require_ok(HAL_TIM_IC_ConfigChannel(&us_echo_timer, &ic, TIM_CHANNEL_1));
    ic.ICPolarity = TIM_ICPOLARITY_FALLING;
    ic.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    require_ok(HAL_TIM_IC_ConfigChannel(&us_echo_timer, &ic, TIM_CHANNEL_2));
    TIM_SlaveConfigTypeDef slave = {0};
    slave.SlaveMode = TIM_SLAVEMODE_RESET;
    slave.InputTrigger = TIM_TS_TI1FP1;
    slave.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    slave.TriggerFilter = 4;
    require_ok(HAL_TIM_SlaveConfigSynchro(&us_echo_timer, &slave));
    require_ok(HAL_TIM_IC_Start(&us_echo_timer, TIM_CHANNEL_1));
    require_ok(HAL_TIM_IC_Start(&us_echo_timer, TIM_CHANNEL_2));
}

static void servo_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    gpio(GPIOC,GPIO_PIN_7,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,0);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
    gpio(GPIOA,GPIO_PIN_15,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_AF1_TIM2);
    TIM2->PSC=15;TIM2->ARR=19999;TIM2->CCR1=0;
    TIM2->CCMR1=TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE;
    TIM2->CCER=TIM_CCER_CC1E;TIM2->EGR=TIM_EGR_UG;TIM2->CR1=TIM_CR1_CEN;
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
}
static void ultrasound(uint32_t now) {
    if(TIM8->SR&TIM_SR_CC2IF) {
        uint32_t sr=TIM8->SR;uint16_t pulse=(uint16_t)TIM8->CCR2;
        TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
        if(us_pending && now-us_trigger_ms<=25U) {
            ++us_seq;us_echo_ms=now;us_pending=false;
            us_result_trigger_ms=us_trigger_ms;us_result_period_ms=us_period_ms;
            us_pulse_us=pulse;
            us_valid=!(sr&TIM_SR_CC2OF)&&pulse>=234U&&pulse<=23324U;
            us_mm=(uint16_t)(((uint32_t)pulse*343U+1000U)/2000U);
            us_reason=(sr&TIM_SR_CC2OF)?"OVERCAP":pulse<234U?"SHORT":pulse>23324U?"LONG":"OK";
        }
    }
    if(us_pending && now-us_trigger_ms>25U) {
        us_pending=false;us_valid=false;us_mm=0;us_pulse_us=0;++us_seq;us_echo_ms=now;
        us_result_trigger_ms=us_trigger_ms;us_result_period_ms=us_period_ms;us_reason="TIMEOUT";
    }
    bool side_scan=g_box_center.phase==BC_SIDE_SCAN;
    bool scan=g_align.phase==ALIGN_SCAN;
    bool due=side_scan?(g_box_center.side_scan.triggers<4U&&
        now-g_box_center.side_scan.since>=60U*(1U+g_box_center.side_scan.triggers)):
        scan?(g_align.scan_trigger_count<4U &&
        now-g_align.since>=60U*(1U+g_align.scan_trigger_count)):true;
    if(due && now-us_trigger_ms>=60U) {
        if(scan)++g_align.scan_trigger_count;
        if(side_scan)++g_box_center.side_scan.triggers;
        us_period_ms=now-us_trigger_ms;us_trigger_ms=now;us_pending=true;
        TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
        TIM17->CNT=0;SET_BIT(TIM17->CR1,TIM_CR1_CEN);
    }
}

#define LCD_CS_LOW()    (GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16U)
#define LCD_CS_HIGH()   (GPIOD->BSRR = GPIO_PIN_11)
#define LCD_DC_CMD()    (GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U)
#define LCD_DC_DATA()   (GPIOD->BSRR = GPIO_PIN_12)
#define LCD_W 128U
#define LCD_H 160U
#define LCD_X_OFFSET 2U
#define LCD_Y_OFFSET 1U /* Original ST7735S panel GRAM offsets. */
#define LCD_BG 0x0000U
#define LCD_WHITE 0xFFFFU
#define LCD_GREEN 0x07E0U
#define LCD_YELLOW 0xFFE0U
#define LCD_RED 0xF800U

static void lcd_spi_byte(uint8_t data)
{
  while ((SPI1->SR & SPI_SR_TXE) == 0U) {}
  *(__IO uint8_t *)&SPI1->DR = data;
}

static void lcd_spi_idle(void)
{
  while ((SPI1->SR & SPI_SR_TXE) == 0U) {}
  while ((SPI1->SR & SPI_SR_BSY) != 0U) {}
}

static void lcd_cmd(uint8_t cmd)
{
  LCD_DC_CMD(); LCD_CS_LOW();
  lcd_spi_byte(cmd); lcd_spi_idle();
  LCD_CS_HIGH(); LCD_DC_DATA();
}

static void lcd_data(uint8_t data)
{
  LCD_DC_DATA(); LCD_CS_LOW();
  lcd_spi_byte(data); lcd_spi_idle();
  LCD_CS_HIGH();
}

static void lcd_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint16_t x0 = x + LCD_X_OFFSET, x1 = x0 + w - 1U;
  uint16_t y0 = (uint16_t)y + LCD_Y_OFFSET, y1 = y0 + h - 1U;
  lcd_cmd(0x2AU); LCD_CS_LOW();
  lcd_spi_byte((uint8_t)(x0 >> 8)); lcd_spi_byte((uint8_t)x0);
  lcd_spi_byte((uint8_t)(x1 >> 8)); lcd_spi_byte((uint8_t)x1);
  lcd_spi_idle(); LCD_CS_HIGH();
  lcd_cmd(0x2BU); LCD_CS_LOW();
  lcd_spi_byte((uint8_t)(y0 >> 8)); lcd_spi_byte((uint8_t)y0);
  lcd_spi_byte((uint8_t)(y1 >> 8)); lcd_spi_byte((uint8_t)y1);
  lcd_spi_idle(); LCD_CS_HIGH();
  lcd_cmd(0x2CU);
}

static void lcd_fill(uint16_t color)
{
  lcd_window(0U, 0U, LCD_W, LCD_H);
  LCD_CS_LOW();
  for (uint32_t p = 0; p < (uint32_t)LCD_W * LCD_H; ++p)
  {
    lcd_spi_byte((uint8_t)(color >> 8));
    lcd_spi_byte((uint8_t)color);
  }
  lcd_spi_idle(); LCD_CS_HIGH();
}

/* Original 6x12 font, eight rows fit the old128x160 display. */
static void lcd_text(uint16_t x,uint16_t y,const char *text,uint16_t color){
 for(const char *s=text;*s&&x+6U<=LCD_W&&y+12U<=LCD_H;++s,x+=6U){
  uint8_t glyph=(*s>=' '&&*s<='~')?(uint8_t)(*s-' '):0;
  lcd_window(x,y,6,12);LCD_CS_LOW();
  for(uint8_t row=0;row<12;++row){uint8_t bits=k_font_6x12[glyph][row];
   for(uint8_t col=0;col<6;++col){
    uint16_t pixel=(bits&(1U<<col))?color:LCD_BG;
    lcd_spi_byte((uint8_t)(pixel>>8));lcd_spi_byte((uint8_t)pixel);
   }
  }
  lcd_spi_idle();LCD_CS_HIGH();
 }
}
static void Lcd_Init(void)
{
  GPIO_InitTypeDef pin = {0};
  static const uint8_t gamma_pos[16] =
    {0x04U,0x22U,0x07U,0x0AU,0x2EU,0x30U,0x25U,0x2AU,0x28U,0x26U,0x2EU,0x3AU,0x00U,0x01U,0x03U,0x13U};
  static const uint8_t gamma_neg[16] =
    {0x04U,0x16U,0x06U,0x0DU,0x2DU,0x26U,0x23U,0x27U,0x27U,0x25U,0x2DU,0x3BU,0x00U,0x01U,0x04U,0x13U};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  pin.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_NOPULL;
  pin.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &pin);
  pin.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  pin.Mode = GPIO_MODE_AF_PP;
  pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  pin.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &pin);
  SPI1->CR1 = 0U;
  SPI1->CR2 = (7U << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
  SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_MSTR;
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(60U);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(120U);
  lcd_cmd(0x11U); HAL_Delay(120U);
  lcd_cmd(0xB1U); lcd_data(0x05U); lcd_data(0x3CU); lcd_data(0x3CU);
  lcd_cmd(0xB2U); lcd_data(0x05U); lcd_data(0x3CU); lcd_data(0x3CU);
  lcd_cmd(0xB3U); lcd_data(0x05U); lcd_data(0x3CU); lcd_data(0x3CU);
  lcd_data(0x05U); lcd_data(0x3CU); lcd_data(0x3CU);
  lcd_cmd(0xB4U); lcd_data(0x03U);
  lcd_cmd(0xC0U); lcd_data(0x28U); lcd_data(0x08U); lcd_data(0x04U);
  lcd_cmd(0xC1U); lcd_data(0xC0U);
  lcd_cmd(0xC2U); lcd_data(0x0DU); lcd_data(0x00U);
  lcd_cmd(0xC3U); lcd_data(0x8DU); lcd_data(0x2AU);
  lcd_cmd(0xC4U); lcd_data(0x8DU); lcd_data(0xEEU);
  lcd_cmd(0xC5U); lcd_data(0x1AU);
  lcd_cmd(0x36U); lcd_data(0xC0U); /* Rotate180 relative to V17, keeping pixel format/colours unchanged. */
  lcd_cmd(0x3AU); lcd_data(0x05U); /* ST7735 RGB565. */
  lcd_cmd(0x20U); /* Explicit inversion OFF: black background, white text. */
  lcd_cmd(0x13U); /* Normal display mode. */
  lcd_cmd(0xE0U); for (uint8_t i = 0U; i < 16U; ++i) lcd_data(gamma_pos[i]);
  lcd_cmd(0xE1U); for (uint8_t i = 0U; i < 16U; ++i) lcd_data(gamma_neg[i]);
  lcd_cmd(0x29U);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  lcd_fill(LCD_BG);
  /* Current motion/task strings are rendered incrementally below. */
}


#define BT_SIZE 2048U
static char bt_data[BT_SIZE];static volatile unsigned bt_head,bt_tail;static uint32_t bt_drop;
static void bt_init(void){
    __HAL_RCC_USART1_CLK_ENABLE();MODIFY_REG(RCC->CCIPR,RCC_CCIPR_USART1SEL,0U);
    GPIO_InitTypeDef p={0};p.Pin=GPIO_PIN_9|GPIO_PIN_10;p.Mode=GPIO_MODE_AF_PP;p.Pull=GPIO_PULLUP;
    p.Speed=GPIO_SPEED_FREQ_HIGH;p.Alternate=GPIO_AF7_USART1;HAL_GPIO_Init(GPIOA,&p);
    USART1->CR1=0;USART1->CR2=0;USART1->CR3=0;
    USART1->BRR=(HAL_RCC_GetPCLK2Freq()+57600U)/115200U;
    USART1->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF;
    USART1->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
    HAL_NVIC_SetPriority(USART1_IRQn,3,0);HAL_NVIC_EnableIRQ(USART1_IRQn);
}
static void bt_queue(const char *s,size_t n){
    unsigned used=(bt_head+BT_SIZE-bt_tail)%BT_SIZE;
    if(n>BT_SIZE-used-1U){++bt_drop;return;}
    for(size_t i=0;i<n;++i){bt_data[bt_head]=s[i];__DMB();bt_head=(bt_head+1U)%BT_SIZE;}
    USART1->CR1|=USART_CR1_TXEIE_TXFNFIE;
}
void USART1_IRQHandler(void){
    if(USART1->ISR&USART_ISR_TXE_TXFNF){
        if(bt_head!=bt_tail){USART1->TDR=(uint8_t)bt_data[bt_tail];bt_tail=(bt_tail+1U)%BT_SIZE;}
        else USART1->CR1&=~USART_CR1_TXEIE_TXFNFIE;
    }
}
static void bt_pump(void){
    if(USART1->ISR&USART_ISR_RXNE_RXFNE)(void)USART1->RDR;
    if(USART1->ISR&(USART_ISR_ORE|USART_ISR_NE|USART_ISR_FE))USART1->ICR=USART_ICR_ORECF|USART_ICR_NECF|USART_ICR_FECF;
}
static long x10(float v){return isfinite(v)?lroundf(v*10):0;}
static const char *task_phase(void){
 if(g_four_faces.phase!=FF_IDLE&&g_four_faces.phase!=FF_FACE)return four_faces_phase_name(g_four_faces.phase);
 return g_box_center.phase==BC_IDLE?align_phase_name(g_align.phase):box_center_phase_name(g_box_center.phase);
}
static const char *task_error(void){
 if(g_four_faces.phase==FF_FAILED||g_four_faces.phase==FF_CANCELLED)return g_four_faces.error;
 return g_box_center.phase==BC_IDLE?align_error_name(g_align.error):g_box_center.error;
}
static void four_faces_telemetry(uint32_t now){
 static uint32_t last;static FourFacesPhase previous=FF_IDLE;
 FourFaces *t=&g_four_faces;if(t->phase==FF_IDLE)return;
 if(now-last<1000U&&previous==t->phase)return;
 last=now;previous=t->phase;char line[384];
 int n=snprintf(line,sizeof(line),
 "F,fw=BOX_BUZZER_V30,ms=%lu,phase=%s,face=%u,shots=%u,side=%s,clearL=%ld,clearR=%ld,clear=%ld,outN=%u,need=%u,inner=%u,outer=%u,turn10=%d,Ldeg10=%ld,Rdeg10=%ld,ref=%u,error=%s\r\n",
 (unsigned long)now,four_faces_phase_name(t->phase),t->face_number,g_laser_shots,t->side_left?"LEFT":"RIGHT",
 (long)t->clearance_counts[0],(long)t->clearance_counts[1],(long)t->clearance_mean,t->exit_samples,FOUR_EXIT_SAMPLES,t->inner_wheel,t->outer_wheel,
 t->side_left?900:-900,x10(t->corner_wheel_degrees[0]),x10(t->corner_wheel_degrees[1]),t->new_reference,t->error);
 if(n>0&&(size_t)n<sizeof(line))bt_queue(line,(size_t)n);else ++bt_drop;
}
static const char *stage_action(void){
 if(g_four_faces.phase!=FF_IDLE&&g_four_faces.phase!=FF_FACE)return four_faces_phase_name(g_four_faces.phase);
 if(g_box_center.phase!=BC_IDLE)return box_center_phase_name(g_box_center.phase);
 switch(g_align.phase){
 case ALIGN_HOME:case ALIGN_SEEK:return "FRONT_HOME";
 case ALIGN_APPROACH:case ALIGN_APPROACH_STOP:return "APPROACH100";
 case ALIGN_PRE_SCAN_WAIT:return "STOP_BEFORE_SCAN";
 case ALIGN_TURN:return g_align.moving_wheel?"RIGHT_HALF_TURN":"LEFT_HALF_TURN";
 case ALIGN_TURN_HALF_WAIT:return "TURN_STOP_WAIT";
 case ALIGN_TURN_LEFT:return g_align.moving_wheel?"RIGHT_HALF_TURN":"LEFT_HALF_TURN";
 default:return "SCAN_AND_ALIGN";
 }
}
static void sonar_telemetry(uint32_t now,const MotorView *v){
 static uint32_t previous;
 if(!us_seq||us_seq==previous)return;
 previous=us_seq;const char *wait="IDLE";BoxCenter *b=&g_box_center;
 unsigned hits=b->confirm_hits;uint16_t reference=b->reference_mm;
 if(g_four_faces.phase==FF_EXIT){wait=g_four_faces.exit_inside?"OUT24":"IN";hits=g_four_faces.hits;}
 else if(g_four_faces.phase==FF_ACQUIRE){wait="NEW_FACE";hits=g_four_faces.hits;reference=0;}
 else if(g_four_faces.phase==FF_EXIT_BRAKE)wait="BRAKE";
 else if(g_four_faces.phase==FF_ACQUIRE_BRAKE)wait="BRAKE";
 else if(b->phase==BC_BACK_SEARCH||b->phase==BC_EDGE_MOVE||b->phase==BC_TRANSIT)
  wait=!b->reference_mm?"REF":(b->phase==BC_TRANSIT&&!b->transit_inside)?"IN":b->expect_outside?"OUT":"IN";
 else if(b->phase==BC_BRAKE)wait="BRAKE";
 else if(g_box_center.phase==BC_SIDE_SCAN)wait="SIDE_SCAN";
 else if(g_box_center.phase==BC_SIDE_RETURN)wait="SIDE_RETURN";
 else if(g_align.phase==ALIGN_SCAN)wait="SCAN";
 char line[448];int n=snprintf(line,sizeof(line),
 "U,fw=BOX_BUZZER_V30,seq=%lu,ms=%lu,trig=%lu,dt=%lu,echoUs=%u,mm=%u,ok=%u,why=%s,wait=%s,hits=%u,ref=%u,pos=%ld,brL=%u,brR=%u,pL=%ld,pR=%ld,uL=%ld,uR=%ld,kpL=%ld,kpR=%ld,kdL=%ld,kdR=%ld,boostL=%u,boostR=%u,lowL=%lu,lowR=%lu,drop=%lu\r\n",
 (unsigned long)us_seq,(unsigned long)now,(unsigned long)us_result_trigger_ms,(unsigned long)us_result_period_ms,
 us_pulse_us,us_mm,us_valid,us_reason,wait,hits,reference,(long)((g_four_faces.phase==FF_EXIT||g_four_faces.phase==FF_EXIT_BRAKE)?g_corner_progress:g_box_position),
 v->status.brake[0],v->status.brake[1],lroundf(v->status.speed[0]),lroundf(v->status.speed[1]),
 lroundf(v->status.duty[0]*1000),lroundf(v->status.duty[1]*1000),
 lroundf(v->status.effective_kp[0]*1000000.f),lroundf(v->status.effective_kp[1]*1000000.f),
 lroundf(v->status.effective_kd[0]*1000000.f),lroundf(v->status.effective_kd[1]*1000000.f),v->status.kp_boost[0],v->status.kp_boost[1],
 (unsigned long)v->status.low_speed_ms[0],(unsigned long)v->status.low_speed_ms[1],(unsigned long)bt_drop);
 if(n>0&&(size_t)n<sizeof(line))bt_queue(line,(size_t)n);else ++bt_drop;
}
static void center_telemetry(uint32_t now){
 static uint32_t last,id;static BoxCenterPhase previous=BC_IDLE;
 BoxCenter *b=&g_box_center;if(b->phase==BC_IDLE)return;
 if(now-last<500U&&previous==b->phase)return;
 last=now;previous=b->phase;char line[768];
 int n=snprintf(line,sizeof(line),
 "C,fw=BOX_BUZZER_V30,id=%lu,ms=%lu,phase=%s,pos=%ld,refMm=%u,refHits=%u,edge=%u,visitsRear=%u,visitsFront=%u,edgeNeed=%u,face=%u,shots=%u,armed=%u,rearMin=%ld,rearMax=%ld,frontMin=%ld,frontMax=%ld,target=%ld,dir=%d,laser=%u,buzzer=%u,error=%s,fine10=%ld,fineRms10=%ld,fineN=%u,fineDone=%u,fineWarn=%u,fineSkip=%u,fineL10=%ld,fineR10=%ld,r0=%ld,r1=%ld,r2=%ld,r3=%ld,f0=%ld,f1=%ld,f2=%ld,f3=%ld,sideIdx=%u,sideN=%u,sideUs=%u,sideOut=%u,sideLimit=%u,sideFit=%u\r\n",
 (unsigned long)id++,(unsigned long)now,box_center_phase_name(b->phase),(long)b->position,b->reference_mm,b->reference_hits,b->edge,
 b->visits[0],b->visits[1],b->crossings_required,g_four_faces.face_number,g_laser_shots,b->transit_inside,(long)b->minimum[0],(long)b->maximum[0],(long)b->minimum[1],(long)b->maximum[1],(long)b->target,b->direction,b->laser,b->buzzer,b->error,x10(b->fine_degrees),x10(b->fine_rms),b->fit_n,b->fine_done,b->fine_warning,b->fine_skipped,x10(b->fine_wheel[0]),x10(b->fine_wheel[1]),
 (long)b->crossings[0][0],(long)b->crossings[0][1],(long)b->crossings[0][2],(long)b->crossings[0][3],
 (long)b->crossings[1][0],(long)b->crossings[1][1],(long)b->crossings[1][2],(long)b->crossings[1][3],b->side_scan.index,b->side_scan.points,b->side_scan.servo_us,b->side_scan.outside,b->side_scan.limit,b->side_scan.fit_ok);
 if(n>0&&(size_t)n<sizeof(line))bt_queue(line,(size_t)n);else ++bt_drop;
}
static void telemetry(uint32_t now,const MotorView *v){
 four_faces_telemetry(now);center_telemetry(now);
 static unsigned sent_points;
 /* Log every accepted face sample, not only the 200ms status snapshot. */
 while(sent_points<g_align.line_n){
  unsigned i=sent_points++;char sample[128];
  int n=snprintf(sample,sizeof(sample),"L,fw=BOX_BUZZER_V30,i=%u,x10=%ld,range10=%ld\r\n",i,x10(g_align.line_x[i]),x10(g_align.line_y[i]));
  if(n>0&&(size_t)n<sizeof(sample))bt_queue(sample,(size_t)n);else ++bt_drop;
 }
 static uint32_t last,seq;static AlignPhase previous=(AlignPhase)255;char line[1024];
 if(now-last<200&&g_align.phase==previous)return;
 last=now;previous=g_align.phase;
 int n=snprintf(line,sizeof(line),
 "M,fw=BOX_BUZZER_V30,seq=%lu,ms=%lu,phase=%s,action=%s,error=%s,motorError=%s,scan=%u,servo=%u,range=%u,valid=%u,side=%s,normal10=%ld,target10=%ld,actual10=%ld,wheelLdeg10=%ld,wheelRdeg10=%ld,edgeL=%u,edgeR=%u,edgeLmm=%u,edgeRmm=%u,edgeNormal10=%ld,centreY10=%ld,moveWheel=%u,dir=%d,refL10=%ld,refR10=%ld,pulseL=%ld,pulseR=%ld,pwmL1000=%ld,pwmR1000=%ld,countL=%lu,countR=%lu,goalL=%ld,goalR=%ld,fitSupport=%u,fitTotal=%u,fitRms10=%ld,entry=%lu,entryInferred=%u,exit=%lu,lineN=%u,lineSupport=%u,lineRms10=%ld,fine10=%ld,fineNormal10=%ld,lineWarn=%u,front=%u,planMm=%u,stopMm=%u,stopHits=%u,remain=%ld,approachPlan=%u,echoLost=%u,txdrop=%lu,irqUs=%lu\r\n",
 (unsigned long)seq++,(unsigned long)now,task_phase(),stage_action(),task_error(),dual_motor_error_name(v->status.error),
 g_align.index,box_center_servo_us(&g_box_center,g_align.servo_us),us_mm,us_valid,g_align.side_left?"LEFT":"RIGHT",x10(g_align.normal_deg),x10(g_align.target_deg),x10(g_align.actual_deg),x10(g_align.wheel_degrees[0]),x10(g_align.wheel_degrees[1]),
 g_align.fit_first,g_align.fit_last,g_align.edge_left_mm,g_align.edge_right_mm,x10(g_align.edge_normal_deg),x10(g_align.centre_y_mm),g_align.moving_wheel,g_align.motor_direction,
 x10(v->status.target[0]),x10(v->status.target[1]),lroundf(v->status.speed[0]),lroundf(v->status.speed[1]),lroundf(v->status.duty[0]*1000),lroundf(v->status.duty[1]*1000),
 (unsigned long)v->status.counts[0],(unsigned long)v->status.counts[1],(long)v->status.goal[0],(long)v->status.goal[1],g_align.fit_support,g_align.fit_total,x10(g_align.rms_mm),(unsigned long)g_align.edge_entry,g_align.entry_inferred,(unsigned long)g_align.edge_exit,g_align.line_n,g_align.line_support,x10(g_align.line_rms),x10(g_align.fine_deg),x10(g_align.fine_normal_deg),g_align.line_warning,g_align.approach_range_mm,g_align.approach_plan_mm,g_align.approach_stop_mm,g_align.approach_stop_hits,(long)g_align.approach_remaining_counts,g_align.approach_plan_applied,g_align.approach_echo_lost,(unsigned long)bt_drop,(unsigned long)v->irq_us);
 if(n>0&&(size_t)n<sizeof(line))bt_queue(line,(size_t)n);else ++bt_drop;
}
/* Dump the actual settled scan averages BEFORE fitting; live range telemetry
   may be from a different servo position. One bounded frame, no RX commands. */
static void scan_telemetry(void){
 char line[384];int n=snprintf(line,sizeof(line),"S,fw=BOX_BUZZER_V30");
 for(unsigned i=0;i<ALIGN_POINTS&&n>0&&(size_t)n<sizeof(line);++i){
  int k=snprintf(line+n,sizeof(line)-(size_t)n,",p%u=%u",i,g_align.range[i]);
  if(k<0||(size_t)k>=sizeof(line)-(size_t)n){++bt_drop;return;}n+=k;
 }
 if(n>0&&(size_t)n+2<sizeof(line)){line[n++]='\r';line[n++]='\n';bt_queue(line,(size_t)n);}
}
static void angle_text(char *out,const char *name,float degrees){
 long tenth=x10(degrees);
 if(tenth<-9999||tenth>9999){snprintf(out,19,"%s OVF",name);return;}
 int v=(int)tenth,mag=v<0?-v:v;
 snprintf(out,19,"%s %c%d.%d deg",name,v<0?'-':'+',mag/10,mag%10);
}
static void screen_service(uint32_t now,const MotorView *v){
 static char lines[8][19];static uint8_t row,col;static uint32_t updated;
 if(row==0&&col==0){
  if(now-updated<150)return;
  updated=now;
  if(us_mm&&now-us_echo_ms<=300U)snprintf(lines[0],19,"DIST %u mm",us_mm);
  else snprintf(lines[0],19,"DIST -- mm");
  snprintf(lines[1],19,"L %lu",(unsigned long)v->status.counts[0]);
  snprintf(lines[2],19,"R %lu",(unsigned long)v->status.counts[1]);
  angle_text(lines[3],"TURN",g_align.target_deg);
  angle_text(lines[4],"L DEG",g_align.wheel_degrees[0]);
  angle_text(lines[5],"R DEG",g_align.wheel_degrees[1]);
  if(g_box_center.phase!=BC_IDLE){
   snprintf(lines[3],19,"POS %+ld",(long)g_box_position);
   snprintf(lines[4],19,"REAR %u/%u",g_box_center.visits[0],g_box_center.crossings_required);
   if(g_box_center.phase>=BC_FINE_RIGHT)angle_text(lines[3],"FINE",g_box_center.fine_degrees);
   snprintf(lines[5],19,"FRONT %u/%u",g_box_center.visits[1],g_box_center.crossings_required);
   if(g_box_center.phase==BC_SIDE_SCAN)snprintf(lines[3],19,"SCAN %c%u deg",g_box_center.side_left?'+':'-',90U-5U*g_box_center.side_scan.index);
  }
  if(g_four_faces.phase!=FF_IDLE){
   if(g_four_faces.phase!=FF_FACE){
    if(g_four_faces.phase==FF_EXIT||g_four_faces.phase==FF_EXIT_BRAKE||g_four_faces.phase==FF_CORNER_WAIT)
     snprintf(lines[3],19,"OUT %u/%u",g_four_faces.exit_samples,FOUR_EXIT_SAMPLES);
    else angle_text(lines[3],"TURN",g_four_faces.side_left?90.f:-90.f);
    angle_text(lines[4],"L DEG",g_four_faces.corner_wheel_degrees[0]);
    angle_text(lines[5],"R DEG",g_four_faces.corner_wheel_degrees[1]);
   }
  }
  snprintf(lines[6],19,"ERR");const char *reason=align_error_name(g_align.error);
  if(v->status.error!=DM_OK)reason=dual_motor_error_name(v->status.error);
  else if(g_align.phase==ALIGN_CANCELLED)reason="CANCELLED";
  if(g_box_center.phase!=BC_IDLE&&g_align.phase!=ALIGN_CANCELLED&&v->status.error==DM_OK)reason=g_box_center.phase==BC_FAILED?g_box_center.error:box_center_phase_name(g_box_center.phase);
  if(g_four_faces.phase!=FF_IDLE){snprintf(lines[6],19,"SHOT %u/4 ERR",g_laser_shots);
   reason=(g_four_faces.phase==FF_FAILED||g_four_faces.phase==FF_CANCELLED)?g_four_faces.error:task_phase();}
  snprintf(lines[7],19,"%s",reason);
  for(unsigned i=0;i<8;++i){size_t n=strlen(lines[i]);while(n<18)lines[i][n++]=' ';lines[i][18]=0;}
 }
 char ch[2]={lines[row][col],0};
 uint16_t color=(g_align.error||v->status.error||g_align.phase==ALIGN_CANCELLED||g_box_center.phase==BC_FAILED||g_four_faces.phase==FF_FAILED)?LCD_RED:LCD_WHITE;
 lcd_text((uint16_t)(4+col*6),(uint16_t)(4+row*18),ch,color);
 if(++col==18){col=0;if(++row==8)row=0;}
}
int main(void){
 HAL_Init();SystemClock_Config();MX_GPIO_Init();motor_hardware_init();
 gpio(GPIOC,GPIO_PIN_8|GPIO_PIN_9,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,0);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);
 gpio(GPIOD,GPIO_PIN_8,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,0);HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
 ultrasonic_init();Lcd_Init();servo_init();bt_init();
 if(!car_motion_init(&g_car,a_edge_counter[0],a_edge_counter[1]))Error_Handler();
 align_start(&g_align,HAL_GetTick());
 if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)){
  car_motion_trip(&g_car,DM_WATCHDOG);g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;g_align.servo_us=ALIGN_CENTRE_US;
 }
 __HAL_RCC_CLEAR_RESET_FLAGS();sample_timer_init();
 IWDG->KR=0x5555;IWDG->PR=4;IWDG->RLR=1250;IWDG->KR=0xCCCC;IWDG->KR=0xAAAA;
 for(;;){
  uint32_t now=HAL_GetTick();ultrasound(now);bt_pump();MotorView view=motor_snapshot();
  if(now-last_step>=5){
   last_step=now;
   if(cancel_requested)align_cancel(&g_align);
   AlignMotionFeedback motion={view.status.running,view.done,view.status.phase==DM_FAULT,view.actual_deg,(uint32_t)view.status.error,{view.status.counts[0],view.status.counts[1]},{view.wheel_degrees[0],view.wheel_degrees[1]}};
   if(g_align.phase==ALIGN_FIT)scan_telemetry();
   AlignPhase before_phase=g_align.phase;uint32_t before_since=g_align.since;
   align_step(&g_align,now,us_seq,us_result_trigger_ms,us_echo_ms,us_valid,us_mm,&motion);
   if(g_align.phase==ALIGN_SCAN&&(before_phase!=ALIGN_SCAN||before_since!=g_align.since)){
    us_pending=false;us_trigger_ms=now;
    TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
   }
   if((g_align.phase==ALIGN_APPROACH||g_align.phase==ALIGN_APPROACH_STOP)&&!g_align.approach_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();bool started=car_motion_start_scan(&g_car);__set_PRIMASK(saved);
    if(started)g_align.approach_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.approach_requested&&g_align.approach_plan_ready&&!g_align.approach_plan_applied&&!g_align.approach_hard_stop&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool planned=car_motion_finish_after_counts(&g_car,g_align.approach_remaining_counts);__set_PRIMASK(saved);
    if(planned)g_align.approach_plan_applied=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.approach_requested&&g_align.approach_hard_stop&&!g_align.approach_hard_stop_applied&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();bool stopped=car_motion_finish_straight(&g_car);__set_PRIMASK(saved);
    if(stopped)g_align.approach_hard_stop_applied=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.phase==ALIGN_TURN&&!g_align.turn_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();bool started=car_motion_start_pivot(&g_car,g_align.moving_wheel,g_align.motor_direction,fabsf(g_align.target_deg)*.5f);__set_PRIMASK(saved);
    if(started)g_align.turn_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.phase==ALIGN_TURN_LEFT&&!g_align.left_turn_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool started=car_motion_start_pivot(&g_car,g_align.moving_wheel,g_align.motor_direction,fabsf(g_align.target_deg)*.5f);__set_PRIMASK(saved);
    if(started)g_align.left_turn_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if((g_align.phase==ALIGN_FAILED||g_align.phase==ALIGN_CANCELLED)&&g_car.motor.running){
    uint32_t saved=__get_PRIMASK();__disable_irq();car_motion_stop(&g_car);__set_PRIMASK(saved);
   }
   if(g_align.phase==ALIGN_DONE&&g_four_faces.phase==FF_IDLE&&!cancel_requested){
    four_faces_start(&g_four_faces,&g_box_center,g_align.side_left,now,us_seq);
   }
   if(g_box_center.phase==BC_FINE_RIGHT)g_box_center.fine_wheel[1]=view.wheel_degrees[1];
   if(g_box_center.phase==BC_FINE_LEFT)g_box_center.fine_wheel[0]=view.wheel_degrees[0];
   if(g_four_faces.phase==FF_INNER)g_four_faces.corner_wheel_degrees[g_four_faces.inner_wheel]=view.wheel_degrees[g_four_faces.inner_wheel];
   if(g_four_faces.phase==FF_OUTER)g_four_faces.corner_wheel_degrees[g_four_faces.outer_wheel]=view.wheel_degrees[g_four_faces.outer_wheel];
   uint32_t old_side_since=g_box_center.side_scan.since;BoxCenterPhase old_box_phase=g_box_center.phase;
   four_faces_step(&g_four_faces,&g_box_center,now,us_seq,us_result_trigger_ms,us_echo_ms,us_valid,us_mm,
    view.total,view.status.running,view.done,view.status.phase==DM_FAULT,cancel_requested);
   if(g_box_center.phase==BC_SIDE_SCAN&&(old_box_phase!=BC_SIDE_SCAN||old_side_since!=g_box_center.side_scan.since)){
    us_pending=false;us_trigger_ms=now;TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
   }
   FourFacesCommand corner_command=g_four_faces.command;
   if(corner_command!=FF_CMD_NONE&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();bool ok=false;
    if(corner_command==FF_CMD_FORWARD)ok=g_four_faces.phase==FF_EXIT?car_motion_start_front_traverse(&g_car):car_motion_start_edge_scan(&g_car,1);
    else if(corner_command==FF_CMD_STOP)ok=car_motion_finish_straight(&g_car);
    else if(corner_command==FF_CMD_INNER)ok=car_motion_start_pivot(&g_car,g_four_faces.inner_wheel,-1,FOUR_CORNER_HALF_DEG);
    else if(corner_command==FF_CMD_OUTER)ok=car_motion_start_pivot(&g_car,g_four_faces.outer_wheel,1,FOUR_CORNER_HALF_DEG);
    __set_PRIMASK(saved);g_four_faces.command=FF_CMD_NONE;
    if(!ok)four_faces_fail(&g_four_faces,&g_box_center,"MOTION_START");
   }
   BoxCenterCommand command=g_box_center.command;
   if(g_four_faces.phase==FF_FACE&&command!=BC_CMD_NONE&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();bool ok=false;
    if(command==BC_CMD_FORWARD&&g_box_center.phase==BC_TRANSIT)
     ok=car_motion_start_front_traverse(&g_car);
    else if(command==BC_CMD_FORWARD||command==BC_CMD_REVERSE)
     ok=car_motion_start_edge_scan(&g_car,command==BC_CMD_FORWARD?1:-1);
    else if(command==BC_CMD_STOP)ok=car_motion_finish_straight(&g_car);
    else if(command==BC_CMD_CENTER)ok=car_motion_start_center_back(&g_car,g_box_center.remaining);
    else if(command==BC_CMD_FINE_RIGHT)ok=car_motion_start_pivot(&g_car,1,g_box_center.fine_degrees>0?1:-1,fabsf(g_box_center.fine_degrees)*.5f);
    else if(command==BC_CMD_FINE_LEFT)ok=car_motion_start_pivot(&g_car,0,g_box_center.fine_degrees>0?-1:1,fabsf(g_box_center.fine_degrees)*.5f);
    __set_PRIMASK(saved);g_box_center.command=BC_CMD_NONE;
    if(!ok){box_center_fail(&g_box_center,"MOTION_START");four_faces_fail(&g_four_faces,&g_box_center,"MOTION_START");}
   }
   if((g_box_center.phase==BC_FAILED||g_four_faces.phase==FF_FAILED)&&g_car.motor.running){
    uint32_t saved=__get_PRIMASK();__disable_irq();car_motion_stop(&g_car);__set_PRIMASK(saved);
   }
   bool firing=g_box_center.laser&&!cancel_requested&&
    g_car.motor.phase==DM_STOPPED&&!g_car.motor.running;
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,firing?GPIO_PIN_SET:GPIO_PIN_RESET);
   bool buzzing=!cancel_requested&&g_car.motor.phase==DM_STOPPED&&!g_car.motor.running&&
    (firing||(g_four_faces.phase==FF_SIGNAL&&g_box_center.buzzer));
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,buzzing?GPIO_PIN_SET:GPIO_PIN_RESET);
   MotorView trace_view=motor_snapshot();sonar_telemetry(now,&trace_view);
   TIM2->CCR1=now-g_align.started>=10?box_center_servo_us(&g_box_center,g_align.servo_us):0;
   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,(g_align.phase==ALIGN_TURN||g_align.phase==ALIGN_TURN_LEFT||g_align.phase==ALIGN_APPROACH||g_align.phase==ALIGN_APPROACH_STOP||g_box_center.phase==BC_BACK_SEARCH||g_box_center.phase==BC_EDGE_MOVE||g_box_center.phase==BC_TRANSIT||g_box_center.phase==BC_CENTER_MOVE||g_box_center.phase==BC_FINE_RIGHT||g_box_center.phase==BC_FINE_LEFT||four_faces_moving(&g_four_faces))?GPIO_PIN_RESET:GPIO_PIN_SET);
  }
  view=motor_snapshot(); /* Starts reset segment counts before status/rendering. */
  telemetry(now,&view);screen_service(now,&view);bt_pump();IWDG->KR=0xAAAA;
 }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  motor_off();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
