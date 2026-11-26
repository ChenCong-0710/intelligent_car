#include "main.h"
#include "lcd_font.h"
#include "initial_align.h"
#include "car_motion.h"
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
static volatile uint16_t a_edge_counter[2];
static volatile bool a_capture_fault,cancel_requested;
static volatile uint32_t irq_max_cycles;
static uint32_t key_low_ms,last_step,us_trigger_ms,us_echo_ms,us_seq;
static uint16_t us_mm;
static bool us_valid,us_pending;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void car_emergency_off(void){
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
        float d=g_car.motor.wheel[w].duty*signs[w];
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
typedef struct {DualMotorStatus status;bool done;float actual_deg;uint32_t irq_us;} MotorView;
static MotorView motor_snapshot(void){
 MotorView v;uint32_t saved=__get_PRIMASK();__disable_irq();
 dual_motor_get_status(&g_car.motor,&v.status);v.done=car_motion_done(&g_car);
 v.actual_deg=car_motion_estimated_angle(&g_car);v.irq_us=irq_max_cycles/16U;
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
        if(us_pending && now-us_trigger_ms<=25U && !(sr&TIM_SR_CC2OF)) {
            ++us_seq;us_echo_ms=now;us_pending=false;
            us_valid=pulse>=234U && pulse<=2333U; /* 40..400 mm */
            us_mm=(uint16_t)(((uint32_t)pulse*343U+1000U)/2000U);
        }
    }
    if(us_pending && now-us_trigger_ms>25U) {us_pending=false;us_valid=false;us_mm=0;++us_seq;us_echo_ms=now;}
    if(now-us_trigger_ms>=60U) {
        us_trigger_ms=now;us_pending=true;us_valid=false;
        TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
        TIM17->CNT=0;SET_BIT(TIM17->CR1,TIM_CR1_CEN);
    }
}
#define LCD_CS_LOW()    (GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16U)
#define LCD_CS_HIGH()   (GPIOD->BSRR = GPIO_PIN_11)
#define LCD_DC_CMD()    (GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U)
#define LCD_DC_DATA()   (GPIOD->BSRR = GPIO_PIN_12)
#define LCD_W 240U
#define LCD_H 280U
#define LCD_Y_OFFSET 20U /* 240x280 glass in 240x320 ST7789 GRAM */
#define LCD_BG 0x0821U
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
  uint16_t x0 = x, x1 = x0 + w - 1U;
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

static void lcd_text2(uint16_t x, uint16_t y, const char *text, uint16_t color)
{
  for (const char *s = text; *s && x <= LCD_W - 12U; ++s, x += 12U)
  {
    uint8_t glyph = (*s >= ' ' && *s <= '~') ? (uint8_t)(*s - ' ') : 0U;
    lcd_window(x, y, 12U, 24U);
    LCD_CS_LOW();
    for (uint8_t row = 0; row < 12U; ++row)
    {
      uint8_t bits = k_font_6x12[glyph][row];
      for (uint8_t repeat_y = 0; repeat_y < 2U; ++repeat_y)
        for (uint8_t col = 0; col < 6U; ++col)
        {
          uint16_t pixel = (bits & (1U << col)) ? color : LCD_BG;
          for (uint8_t repeat_x = 0; repeat_x < 2U; ++repeat_x)
          {
            lcd_spi_byte((uint8_t)(pixel >> 8));
            lcd_spi_byte((uint8_t)pixel);
          }
        }
    }
    lcd_spi_idle(); LCD_CS_HIGH();
  }
}

static void Lcd_Init(void)
{
  GPIO_InitTypeDef pin = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();
  /* Core-board FPC: PB3=SCL PB5=SDA PD11=CS PD12=DC PD13=BL.
   * Module RESET has its own RC circuit and is not connected to PD10. */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  pin.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
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
  /* 16 MHz HSI / 2 = 8 MHz SPI, within the module's serial timing. */
  SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE |
              SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
  HAL_Delay(150U); /* wait for the module's RC reset and power rails */
  lcd_cmd(0x01U); HAL_Delay(150U); /* software reset */
  lcd_cmd(0x11U); HAL_Delay(120U); /* sleep out */
  lcd_cmd(0x36U); lcd_data(0x08U); /* portrait, BGR order */
  lcd_cmd(0x3AU); lcd_data(0x55U); /* RGB565, 16-bit per pixel */
  lcd_cmd(0xB2U); lcd_data(0x0CU); lcd_data(0x0CU);
  lcd_data(0x00U); lcd_data(0x33U); lcd_data(0x33U);
  lcd_cmd(0xB7U); lcd_data(0x35U);
  lcd_cmd(0xBBU); lcd_data(0x19U);
  lcd_cmd(0xC0U); lcd_data(0x2CU);
  lcd_cmd(0xC2U); lcd_data(0x01U);
  lcd_cmd(0xC3U); lcd_data(0x12U);
  lcd_cmd(0xC4U); lcd_data(0x20U);
  lcd_cmd(0xC6U); lcd_data(0x0FU);
  lcd_cmd(0xD0U); lcd_data(0xA4U); lcd_data(0xA1U);
  lcd_cmd(0x21U); /* inversion on for this IPS panel */
  lcd_cmd(0x13U); /* normal display mode */
  lcd_cmd(0x29U); HAL_Delay(20U); /* display on */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  lcd_fill(LCD_BG);
  /* Large, minimal status is drawn by screen_service. */
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
static const char *stage_action(void){
 switch(g_align.phase){
 case ALIGN_RETREAT:return "RETREAT150";
 case ALIGN_FORWARD:case ALIGN_SCAN_BRAKE:return "SCAN_FORWARD";
 case ALIGN_FINE_TURN:return "RIGHT_FINE";
 case ALIGN_FINAL_BACK:return "BACK450";
 default:return "ALIGN_SEQUENCE";
 }
}
static void telemetry(uint32_t now,const MotorView *v){
 static unsigned sent_points;
 /* Log every accepted face sample, not only the 200ms status snapshot. */
 while(sent_points<g_align.line_n){
  unsigned i=sent_points++;char sample[128];
  int n=snprintf(sample,sizeof(sample),"L,fw=BOX_ALIGN_PD_V9,i=%u,x10=%ld,range10=%ld\r\n",i,x10(g_align.line_x[i]),x10(g_align.line_y[i]));
  if(n>0&&(size_t)n<sizeof(sample))bt_queue(sample,(size_t)n);else ++bt_drop;
 }
 static uint32_t last,seq;static AlignPhase previous=(AlignPhase)255;char line[1024];
 if(now-last<200&&g_align.phase==previous)return;
 last=now;previous=g_align.phase;
 int n=snprintf(line,sizeof(line),
 "M,fw=BOX_ALIGN_PD_V9,seq=%lu,ms=%lu,phase=%s,action=%s,error=%s,motorError=%s,scan=%u,servo=%u,range=%u,valid=%u,side=%s,normal10=%ld,target10=%ld,actual10=%ld,edgeL=%u,edgeR=%u,edgeLmm=%u,edgeRmm=%u,edgeNormal10=%ld,centreY10=%ld,moveWheel=%u,dir=%d,refL10=%ld,refR10=%ld,pulseL=%ld,pulseR=%ld,pwmL1000=%ld,pwmR1000=%ld,countL=%lu,countR=%lu,goalL=%ld,goalR=%ld,fitSupport=%u,fitTotal=%u,fitRms10=%ld,entry=%lu,entryInferred=%u,exit=%lu,lineN=%u,lineSupport=%u,lineRms10=%ld,fine10=%ld,fineNormal10=%ld,lineWarn=%u,txdrop=%lu,irqUs=%lu\r\n",
 (unsigned long)seq++,(unsigned long)now,align_phase_name(g_align.phase),stage_action(),align_error_name(g_align.error),dual_motor_error_name(v->status.error),
 g_align.index,g_align.servo_us,us_mm,us_valid,g_align.side_left?"LEFT":"RIGHT",x10(g_align.normal_deg),x10(g_align.target_deg),x10(g_align.actual_deg),
 g_align.fit_first,g_align.fit_last,g_align.edge_left_mm,g_align.edge_right_mm,x10(g_align.edge_normal_deg),x10(g_align.centre_y_mm),g_align.moving_wheel,g_align.motor_direction,
 x10(v->status.target[0]),x10(v->status.target[1]),lroundf(v->status.speed[0]),lroundf(v->status.speed[1]),lroundf(v->status.duty[0]*1000),lroundf(v->status.duty[1]*1000),
 (unsigned long)v->status.counts[0],(unsigned long)v->status.counts[1],(long)v->status.goal[0],(long)v->status.goal[1],g_align.fit_support,g_align.fit_total,x10(g_align.rms_mm),(unsigned long)g_align.edge_entry,g_align.entry_inferred,(unsigned long)g_align.edge_exit,g_align.line_n,g_align.line_support,x10(g_align.line_rms),x10(g_align.fine_deg),x10(g_align.fine_normal_deg),g_align.line_warning,(unsigned long)bt_drop,(unsigned long)v->irq_us);
 if(n>0&&(size_t)n<sizeof(line))bt_queue(line,(size_t)n);else ++bt_drop;
}
/* Dump the actual settled scan averages BEFORE fitting; live range telemetry
   may be from a different servo position. One bounded frame, no RX commands. */
static void scan_telemetry(void){
 char line[384];int n=snprintf(line,sizeof(line),"S,fw=BOX_ALIGN_PD_V9");
 for(unsigned i=0;i<ALIGN_POINTS&&n>0&&(size_t)n<sizeof(line);++i){
  int k=snprintf(line+n,sizeof(line)-(size_t)n,",p%u=%u",i,g_align.range[i]);
  if(k<0||(size_t)k>=sizeof(line)-(size_t)n){++bt_drop;return;}n+=k;
 }
 if(n>0&&(size_t)n+2<sizeof(line)){line[n++]='\r';line[n++]='\n';bt_queue(line,(size_t)n);}
}
static void screen_service(uint32_t now,const MotorView *v){
 /* 2x font, 18 columns = 216px. One glyph per loop keeps echo polling alive. */
 static char lines[5][19];static uint8_t row,col;static uint32_t updated;
 static const uint16_t y[5]={24,80,122,186,220};
 if(row==0&&col==0){
  if(now-updated<150)return;
  updated=now;
  if(us_mm&&now-us_echo_ms<=300U)snprintf(lines[0],19,"DIST %u mm",us_mm);
  else snprintf(lines[0],19,"DIST -- mm");
  /* Core creates a new origin on EVERY start, never resets free-running
     hardware counters. Final reverse counts are retained in DONE. */
  snprintf(lines[1],19,"L %lu",(unsigned long)v->status.counts[0]);
  snprintf(lines[2],19,"R %lu",(unsigned long)v->status.counts[1]);
  snprintf(lines[3],19,"ERR");
  const char *reason=align_error_name(g_align.error);
  if(v->status.error!=DM_OK)reason=dual_motor_error_name(v->status.error);
  else if(g_align.phase==ALIGN_CANCELLED)reason="CANCELLED";
  snprintf(lines[4],19,"%s",reason);
  for(unsigned i=0;i<5;++i){size_t n=strlen(lines[i]);while(n<18)lines[i][n++]=' ';lines[i][18]=0;}
 }
 char ch[2]={lines[row][col],0};
 uint16_t color=(g_align.error||v->status.error||g_align.phase==ALIGN_CANCELLED)?LCD_RED:LCD_WHITE;
 lcd_text2((uint16_t)(12+col*12),y[row],ch,color);
 if(++col==18){col=0;if(++row==5)row=0;}
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
   AlignMotionFeedback motion={view.status.running,view.done,view.status.phase==DM_FAULT,view.actual_deg,(uint32_t)view.status.error,{view.status.counts[0],view.status.counts[1]}};
   if(g_align.phase==ALIGN_FIT)scan_telemetry();
   align_step(&g_align,now,us_seq,us_trigger_ms,us_echo_ms,us_valid,us_mm,&motion);
   if(g_align.phase==ALIGN_TURN&&!g_align.turn_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool started=car_motion_start_pivot(&g_car,g_align.moving_wheel,g_align.motor_direction,fabsf(g_align.target_deg));
    __set_PRIMASK(saved);
    if(started)g_align.turn_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.phase==ALIGN_RETREAT&&!g_align.retreat_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool started=car_motion_start_straight(&g_car,-1,ALIGN_RETREAT_COUNTS);
    __set_PRIMASK(saved);
    if(started)g_align.retreat_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.phase==ALIGN_FORWARD&&!g_align.forward_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool started=car_motion_start_scan(&g_car);
    __set_PRIMASK(saved);
    if(started)g_align.forward_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.phase==ALIGN_SCAN_BRAKE&&!g_align.scan_stop_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool stopped=car_motion_finish_straight(&g_car);
    __set_PRIMASK(saved);
    if(stopped)g_align.scan_stop_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.phase==ALIGN_FINE_TURN&&!g_align.fine_requested&&!cancel_requested){
    g_align.moving_wheel=1;g_align.motor_direction=g_align.fine_deg>0?1:-1;
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool started=car_motion_start_pivot(&g_car,1,g_align.motor_direction,fabsf(g_align.fine_deg));
    __set_PRIMASK(saved);
    if(started)g_align.fine_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if(g_align.phase==ALIGN_FINAL_BACK&&!g_align.final_requested&&!cancel_requested){
    uint32_t saved=__get_PRIMASK();__disable_irq();
    bool started=car_motion_start_straight(&g_car,-1,ALIGN_FINAL_BACK_COUNTS);
    __set_PRIMASK(saved);
    if(started)g_align.final_requested=true;
    else {g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;}
   }
   if((g_align.phase==ALIGN_FAILED||g_align.phase==ALIGN_CANCELLED)&&g_car.motor.running){
    uint32_t saved=__get_PRIMASK();__disable_irq();car_motion_stop(&g_car);__set_PRIMASK(saved);
   }
   TIM2->CCR1=now-g_align.started>=10?g_align.servo_us:0;
   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,(g_align.phase==ALIGN_TURN||g_align.phase==ALIGN_RETREAT)?GPIO_PIN_RESET:GPIO_PIN_SET);
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
