#include "main.h"
#include "lcd_font.h"
#include "initial_align.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define CONTROL_MS 10U
#define COUNTS_PER_REV 840.0f
#define WHEEL_MM 43.0f
/* Effective left/right contact spacing: inherited project value, calibrate on floor. */
#define TRACK_MM 130.0f
#define PWM_PERIOD 799U
#define MAX_DUTY 0.75f
#define KP 0.0030f
#define KI 0.0120f
#define I_LIMIT 0.60f
#define MOTOR_SIGN_L 1
#define MOTOR_SIGN_R 1
#define PI 3.14159265359f
typedef struct { float rpm,duty,integral,previous_rpm; uint16_t previous_count; } Wheel;
static TIM_HandleTypeDef motor_pwm,enc_left,enc_right,us_trig_timer,us_echo_timer;
static Wheel wheels[2];
/* Visible in debugger; left-positive angles, physical left/centre/right servo mapping. */
InitialAlign g_align;
static uint32_t last_control_ms,us_trigger_ms,us_echo_ms,us_seq,last_step;
static uint16_t us_mm;
static bool us_valid,us_pending;
static float heading_rad,command_rpm[2];
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static float clampf(float x,float lo,float hi) {return x<lo?lo:(x>hi?hi:x);}
static void motor_off(void) {
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
    TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=0;
    for(unsigned i=0;i<2;++i) {wheels[i].duty=wheels[i].integral=command_rpm[i]=0;}
}
static void gpio(GPIO_TypeDef *port,uint32_t pin,uint32_t mode,uint32_t pull,uint32_t af) {
    GPIO_InitTypeDef g={0};g.Pin=pin;g.Mode=mode;g.Pull=pull;
    g.Speed=GPIO_SPEED_FREQ_HIGH;g.Alternate=af;HAL_GPIO_Init(port,&g);
}
static void require_ok(HAL_StatusTypeDef r) {if(r!=HAL_OK) Error_Handler();}
static void motor_hardware_init(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};
  TIM_Encoder_InitTypeDef ec = {0};
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
  ec.EncoderMode = TIM_ENCODERMODE_TI12;
  ec.IC1Polarity = TIM_ICPOLARITY_FALLING;
  ec.IC2Polarity = TIM_ICPOLARITY_RISING;
  ec.IC1Selection = ec.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  ec.IC1Prescaler = ec.IC2Prescaler = TIM_ICPSC_DIV1;
  ec.IC1Filter = ec.IC2Filter = 6U;
  for (unsigned i = 0U; i < 2U; ++i) {
    TIM_HandleTypeDef *timer = encoders[i];
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 65535U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Encoder_Init(timer, &ec) != HAL_OK) Error_Handler();
    if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
  }
  wheels[0].previous_count = (uint16_t)TIM4->CNT;
  wheels[1].previous_count = (uint16_t)TIM3->CNT;
  motor_off();
}

static void apply_pwm(float left, float right)
{
  /* Coast PWM: IN1=PWM, IN2=0, with optional motor sign reversal. */
  float command[2] = {left * MOTOR_SIGN_L, right * MOTOR_SIGN_R};
  uint16_t ccr[4] = {0U, 0U, 0U, 0U};
  for (unsigned i = 0U; i < 2U; ++i) {
    float d = clampf(command[i], -MAX_DUTY, MAX_DUTY);
    uint16_t value = (uint16_t)((d < 0.0f ? -d : d) * (PWM_PERIOD + 1U) + 0.5f);
    ccr[2U * i + (d < 0.0f ? 1U : 0U)] = value;
  }
  TIM1->CCR1 = ccr[0]; TIM1->CCR2 = ccr[1];
  TIM1->CCR3 = ccr[2]; TIM1->CCR4 = ccr[3];
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
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
    if(us_pending && now-us_trigger_ms>25U) {us_pending=false;us_valid=false;}
    if(now-us_trigger_ms>=60U) {
        us_trigger_ms=now;us_pending=true;us_valid=false;
        TIM8->SR=~(TIM_SR_CC2IF|TIM_SR_CC2OF);
        TIM17->CNT=0;SET_BIT(TIM17->CR1,TIM_CR1_CEN);
    }
}
static void speed_control(uint32_t now) {
    uint32_t elapsed=now-last_control_ms;if(elapsed<CONTROL_MS)return;
    last_control_ms=now;float dt=elapsed*0.001f;
    if(elapsed>50U) {motor_off();g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;return;}
    uint16_t counts[2]={(uint16_t)TIM4->CNT,(uint16_t)TIM3->CNT};
    float step[2];
    for(unsigned i=0;i<2;++i) {
        int16_t delta=(int16_t)(counts[i]-wheels[i].previous_count);
        wheels[i].previous_count=counts[i];
        step[i]=(float)delta*PI*WHEEL_MM/COUNTS_PER_REV;
        float rpm=(float)delta*60.0f/(COUNTS_PER_REV*dt);
        wheels[i].rpm+=0.4f*(rpm-wheels[i].rpm);
    }
    heading_rad+=(step[1]-step[0])/TRACK_MM;
    if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4)==GPIO_PIN_RESET) {
        motor_off();g_align.phase=ALIGN_FAILED;g_align.error=ALIGN_HARDWARE;return;
    }
    if(g_align.phase!=ALIGN_TURN) {motor_off();return;}
    for(unsigned i=0;i<2;++i) {
        float requested=g_align.speed[i]*60.0f/(PI*WHEEL_MM);
        command_rpm[i]+=clampf(requested-command_rpm[i],-200*dt,200*dt);
        float target=command_rpm[i],error=target-wheels[i].rpm;
        if(fabsf(target)<0.5f) {wheels[i].integral=wheels[i].duty=0;continue;}
        float ff=copysignf(0.14f+fabsf(target)*0.0040f,target);
        float ci=clampf(wheels[i].integral+KI*error*dt,-I_LIMIT,I_LIMIT);
        float u=ff+KP*error+ci,limited=clampf(u,-MAX_DUTY,MAX_DUTY);
        if((u-limited)*error<=0)wheels[i].integral=ci;
        wheels[i].duty=clampf(ff+KP*error+wheels[i].integral,-MAX_DUTY,MAX_DUTY);
        /* Never drive a wheel in the opposite direction to its turn target. */
        if(wheels[i].duty*target<0)wheels[i].duty=0;
    }
    apply_pwm(wheels[0].duty,wheels[1].duty);
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

static void lcd_text(uint16_t x, uint16_t y, const char *text, uint16_t color)
{
  for (const char *s = text; *s && x <= LCD_W - 6U; ++s, x += 6U)
  {
    uint8_t glyph = (*s >= ' ' && *s <= '~') ? (uint8_t)(*s - ' ') : 0U;
    lcd_window(x, y, 6U, 12U);
    LCD_CS_LOW();
    for (uint8_t row = 0; row < 12U; ++row)
    {
      uint8_t bits = k_font_6x12[glyph][row];
      for (uint8_t col = 0; col < 6U; ++col)
      {
        uint16_t pixel = (bits & (1U << col)) ? color : LCD_BG;
        lcd_spi_byte((uint8_t)(pixel >> 8)); lcd_spi_byte((uint8_t)pixel);
      }
    }
    lcd_spi_idle(); LCD_CS_HIGH();
  }
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
  lcd_text2(12U, 10U, "INITIAL ALIGN", LCD_WHITE);
  lcd_text(12U, 262U, "PD0 STOP   RESET SCAN", LCD_GREEN);
}


static void screen_service(uint32_t now) {
    static char lines[12][37];static uint8_t row,col;static uint32_t updated;
    if(row==0 && col==0) {
        if(now-updated<250U)return;
        updated=now;
        snprintf(lines[0],37,"%-35s",align_phase_name(g_align.phase));
        snprintf(lines[1],37,"SCAN %2u/37  SERVO %4u us",g_align.index,g_align.servo_us);
        snprintf(lines[2],37,"RANGE %3u mm VALID %u",us_mm,(unsigned)us_valid);
        snprintf(lines[3],37,"NORMAL %+4d deg",(int)g_align.normal_deg);
        snprintf(lines[4],37,"SIDE %s",g_align.side_left?"LEFT":"RIGHT");
        snprintf(lines[5],37,"TARGET %+4d  ACTUAL %+4d deg",(int)g_align.target_deg,(int)g_align.actual_deg);
        snprintf(lines[6],37,"FIT %u pts RMS %u/10 mm",g_align.fit_last-g_align.fit_first+1,(unsigned)(g_align.rms_mm*10));
        snprintf(lines[7],37,"L %+4d   R %+4d mm/s",(int)(wheels[0].rpm*PI*WHEEL_MM/60),(int)(wheels[1].rpm*PI*WHEEL_MM/60));
        snprintf(lines[8],37,"EDGES L%u R%u",g_align.edge_left_seen,g_align.edge_right_seen);
        snprintf(lines[9],37,"SIDE DIST %3u mm",g_align.side_mm);
        snprintf(lines[10],37,"RESULT %s",align_error_name(g_align.error));
        snprintf(lines[11],37,"STEP1 ONLY - NO LASER");
        for(unsigned r=0;r<12;++r) {size_t n=0;while(n<36&&lines[r][n])++n;while(n<36)lines[r][n++]=' ';lines[r][36]=0;}
    }
    char ch[2]={lines[row][col],0};
    lcd_text((uint16_t)(12+col*6),(uint16_t)(48+row*17),ch,g_align.error?LCD_RED:LCD_WHITE);
    if(++col==36){col=0;if(++row==12)row=0;}
}
int main(void) {
    HAL_Init();SystemClock_Config();MX_GPIO_Init();motor_hardware_init();
    /* Laser/buzzer/park outputs start low and are not used by this test. */
    gpio(GPIOC,GPIO_PIN_8|GPIO_PIN_9,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,0);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);
    gpio(GPIOD,GPIO_PIN_8,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,0);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
    ultrasonic_init();Lcd_Init();servo_init();
    last_control_ms=HAL_GetTick();align_start(&g_align,last_control_ms);
    /* Independent watchdog: fail stopped if the main loop locks. */
    IWDG->KR=0x5555;IWDG->PR=4;IWDG->RLR=1250;IWDG->KR=0xCCCC;IWDG->KR=0xAAAA;
    for(;;) {
        uint32_t now=HAL_GetTick();ultrasound(now);speed_control(now);
        if(now-last_step>=5U) {
            last_step=now;
            float measured[2]={wheels[0].rpm*PI*WHEEL_MM/60,wheels[1].rpm*PI*WHEEL_MM/60};
            AlignPhase before=g_align.phase;
            align_step(&g_align,now,us_seq,us_trigger_ms,us_echo_ms,us_valid,us_mm,heading_rad,measured);
            if(before==ALIGN_FIT) {last_control_ms=HAL_GetTick();last_step=last_control_ms;}
            if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0)==GPIO_PIN_RESET) {
                g_align.phase=ALIGN_CANCELLED;motor_off();
            }
            if(g_align.phase!=ALIGN_TURN) {
                motor_off();
                if(g_align.phase==ALIGN_BRAKE) {
                    TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=PWM_PERIOD+1U;
                    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
                }
            }
            TIM2->CCR1=now-g_align.started>=10U?g_align.servo_us:0;
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,g_align.phase==ALIGN_TURN?GPIO_PIN_RESET:GPIO_PIN_SET);
        }
        screen_service(now);IWDG->KR=0xAAAA;
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
