#include "main.h"
#include "lcd_font.h"
#include "motion_pd.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#define PWM_PERIOD 799U
#define MOTOR_SIGN_L 1
#define MOTOR_SIGN_R 1
#define ENCODER_SIGN_L 1
#define ENCODER_SIGN_R 1
static TIM_HandleTypeDef motor_pwm,enc_left,enc_right;
MotionPd g_motion;
static volatile uint16_t a_edge_counter[2];
static volatile bool a_capture_fault;
static volatile bool stop_key_latched;
static volatile uint32_t key_low_ms,key_max_ms,key_edges,key_stops;
static volatile uint32_t irq_max_cycles;
static bool ground_label;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void pd_emergency_off(void){
    if(RCC->AHB2ENR&RCC_AHB2ENR_GPIODEN)GPIOD->BSRR=(uint32_t)GPIO_PIN_2<<16;
    if(RCC->AHB2ENR&RCC_AHB2ENR_GPIOCEN)GPIOC->BSRR=(uint32_t)(GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9)<<16;
    if(RCC->APB2ENR&RCC_APB2ENR_TIM1EN)TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=0;
}
static void motor_off(void){pd_emergency_off();}
static void output_apply(void){
    if(!g_motion.enable){motor_off();return;}
    const int signs[2]={MOTOR_SIGN_L,MOTOR_SIGN_R};
    uint32_t value[4];
    for(unsigned w=0;w<2;++w){
        float d=g_motion.wheel[w].duty*signs[w];
        /* Drive/brake PWM: both inputs high at zero effort. Braked wheels hold
           independently while the other wheel reaches its encoder endpoint. */
        uint32_t active=(uint32_t)lroundf(fminf(fabsf(d),MOTION_MAX_DUTY)*800.0f);
        value[w*2]=800;value[w*2+1]=800;
        if(!g_motion.braked[w]){
            if(d>=0)value[w*2+1]=800-active;
            else value[w*2]=800-active;
        }
    }
    TIM1->CCR1=value[0];TIM1->CCR2=value[1];TIM1->CCR3=value[2];TIM1->CCR4=value[3];
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
}
void TIM6_DAC_IRQHandler(void){
    if(!(TIM6->SR&TIM_SR_UIF))return;
    TIM6->SR=~TIM_SR_UIF;
    uint32_t begin=DWT->CYCCNT;
    if(!(GPIOD->IDR&GPIO_PIN_0)){
        if(!key_low_ms)++key_edges;
        if(key_low_ms<60000U)++key_low_ms;
        if(key_low_ms>key_max_ms)key_max_ms=key_low_ms;
    }else key_low_ms=0;
    if(key_low_ms>=20U&&motion_busy(&g_motion)){motion_stop(&g_motion);stop_key_latched=true;++key_stops;}
    uint16_t l=(uint16_t)((int32_t)a_edge_counter[0]*ENCODER_SIGN_L);
    uint16_t r=(uint16_t)((int32_t)a_edge_counter[1]*ENCODER_SIGN_R);
    motion_tick(&g_motion,l,r,(GPIOC->IDR&GPIO_PIN_4)!=0);
    if(a_capture_fault)motion_trip(&g_motion,MP_ENCODER);
    output_apply();
    GPIOD->BSRR=motion_busy(&g_motion)&&g_motion.phase!=MP_PAUSE?(uint32_t)GPIO_PIN_8<<16:GPIO_PIN_8;
    uint32_t elapsed=DWT->CYCCNT-begin;
    if(elapsed>irq_max_cycles)irq_max_cycles=elapsed;
}
static void sample_timer_init(void){
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->PSC=15;TIM6->ARR=999;TIM6->EGR=TIM_EGR_UG;TIM6->SR=0;TIM6->DIER=TIM_DIER_UIE;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,2,0);HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1=TIM_CR1_CEN;
}
typedef struct {
 MotionPhase phase;MotionError error;uint32_t ms,irq_us,key_low,key_stops;
 int32_t sync_error;float sync_correction;
 const char *sequence;uint32_t wait_ms;
 bool decelerating[2];int32_t trigger[2],results[2][2];
 unsigned step,target_deg;int32_t absolute_goal,segment_goal,net[2];
 unsigned action;float duty[2],pulses[2],ref_pulses[2];int32_t actual[2];
} MotionView;
static MotionView snapshot(void){
 MotionView v;uint32_t saved=__get_PRIMASK();__disable_irq();
 v.phase=g_motion.phase;v.error=g_motion.error;v.ms=g_motion.now;v.action=g_motion.action;
 v.step=motion_pivot_step();v.target_deg=motion_pivot_target_degrees();
 v.absolute_goal=motion_pivot_absolute_goal();v.segment_goal=motion_pivot_segment_goal();
 for(unsigned w=0;w<2;++w)v.net[w]=g_pivot_net_counts[w];
 v.sequence=motion_sequence_name();v.wait_ms=motion_wait_remaining(&g_motion);
 memcpy(v.results,g_motion_results,sizeof(v.results));
 for(unsigned w=0;w<2;++w){v.decelerating[w]=g_motion.decelerating[w];v.trigger[w]=g_motion.goal_trigger[w];}
 v.sync_error=g_motion.sync_error;v.sync_correction=g_motion.sync_correction;
 v.irq_us=irq_max_cycles/16U;v.key_low=key_low_ms;v.key_stops=key_stops;
 for(unsigned w=0;w<2;++w){v.duty[w]=g_motion.wheel[w].duty;v.pulses[w]=g_motion.wheel[w].speed_pulses;v.ref_pulses[w]=g_motion.wheel[w].target_pulses;v.actual[w]=(int32_t)g_motor_pulses[w];}
 __set_PRIMASK(saved);return v;
}
/* A rising/falling capture interrupt; B is read only to determine direction.
   Equality corresponds to the previous inverted-A encoder convention. */
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
static void buttons(uint32_t now){
    static bool raw[3],stable[3],press_stopped;
    static uint32_t changed[3];const uint16_t pins[3]={GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_15};
    for(unsigned i=0;i<3;++i){
        bool down=HAL_GPIO_ReadPin(GPIOD,pins[i])==GPIO_PIN_RESET;
        if(down!=raw[i]){raw[i]=down;changed[i]=now;}
        if(raw[i]==stable[i]||now-changed[i]<25U)continue;
        stable[i]=raw[i];
        if(i==0){
            uint32_t saved=__get_PRIMASK();__disable_irq();
            if(stable[i])press_stopped=stop_key_latched||motion_busy(&g_motion)||g_motion.phase==MP_STOPPING;
            else{
                if(!press_stopped)motion_start(&g_motion);
                stop_key_latched=false;
            }
            __set_PRIMASK(saved);
        }else if(stable[i]){
            if(i==2&&!motion_busy(&g_motion))ground_label=!ground_label;
        }
    }
}
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
  ic.ICPolarity=TIM_ICPOLARITY_BOTHEDGE;
  ic.ICSelection=TIM_ICSELECTION_DIRECTTI;
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
static void telemetry(const MotionView *v){
 static uint32_t last,seq;char line[560];
 if(v->ms-last<100U)return;
 last=v->ms;
 int n=snprintf(line,sizeof(line),
 "M,fw=A34_LEFT_SEQ_V1,seq=%lu,ms=%lu,phase=%s,action=%s,error=%s,seqstage=%s,step=%u,activeWheel=0,targetDeg=%u,absGoal=%ld,segmentGoal=%ld,netL=%ld,netR=%ld,refL=%ld,refR=%ld,pulseL=%ld,pulseR=%ld,pwmL1000=%ld,pwmR1000=%ld,countL=%ld,countR=%ld,triggerL=%ld,triggerR=%ld,wait=%lu,txdrop=%lu,keylow=%lu,keystops=%lu,irqUs=%lu\r\n",
 (unsigned long)seq++,(unsigned long)v->ms,motion_phase_name(v->phase),motion_action_name(v->action),motion_error_name(v->error),v->sequence,v->step,v->target_deg,
 (long)v->absolute_goal,(long)v->segment_goal,(long)v->net[0],(long)v->net[1],lroundf(v->ref_pulses[0]),lroundf(v->ref_pulses[1]),lroundf(v->pulses[0]),lroundf(v->pulses[1]),
 lroundf(v->duty[0]*1000),lroundf(v->duty[1]*1000),(long)v->actual[0],(long)v->actual[1],(long)v->trigger[0],(long)v->trigger[1],(unsigned long)v->wait_ms,(unsigned long)bt_drop,(unsigned long)v->key_low,(unsigned long)v->key_stops,(unsigned long)v->irq_us);
 if(n>0&&(size_t)n<sizeof(line))bt_queue(line,(size_t)n);
}
static void screen_service(const MotionView *v){
 static char rows[8][19];static unsigned row,col;static uint32_t last;
 if(!row&&!col){
  if(v->ms-last<150U)return;
  last=v->ms;
  snprintf(rows[0],19,"A EDGES /50ms");
  snprintf(rows[1],19,"L %+4ld R %+4ld",lroundf(v->pulses[0]),lroundf(v->pulses[1]));
  snprintf(rows[2],19,"PWM %%");
  snprintf(rows[3],19,"L %+4ld R %+4ld",lroundf(v->duty[0]*100),lroundf(v->duty[1]*100));
  snprintf(rows[4],19,"L GOAL %hu",(unsigned short)v->absolute_goal);
  snprintf(rows[5],19,"L %8ld",(long)v->net[0]);
  snprintf(rows[6],19,"R %8ld",(long)v->net[1]);
  snprintf(rows[7],19,"TO %hu DEG %hhuS",(unsigned short)v->target_deg,(unsigned char)((v->wait_ms+999U)/1000U));
  for(unsigned r=0;r<8;++r){size_t n=strlen(rows[r]);while(n<18)rows[r][n++]=' ';rows[r][18]=0;}
 }
 char ch[2]={rows[row][col],0};
 lcd_text2((uint16_t)(12+col*12),(uint16_t)(12+row*33),ch,LCD_WHITE);
 if(++col==18){col=0;if(++row==8)row=0;}
}
int main(void){
    HAL_Init();SystemClock_Config();MX_GPIO_Init();motor_hardware_init();
    GPIO_InitTypeDef p={0};p.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;p.Mode=GPIO_MODE_OUTPUT_PP;p.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC,&p);HAL_GPIO_WritePin(GPIOC,p.Pin,GPIO_PIN_RESET);
    p.Pin=GPIO_PIN_8;HAL_GPIO_Init(GPIOD,&p);HAL_GPIO_WritePin(GPIOD,p.Pin,GPIO_PIN_SET);
    Lcd_Init();bt_init();
    motion_init(&g_motion,(uint16_t)(a_edge_counter[0]*ENCODER_SIGN_L),(uint16_t)(a_edge_counter[1]*ENCODER_SIGN_R));
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))motion_trip(&g_motion,MP_WATCHDOG);
    __HAL_RCC_CLEAR_RESET_FLAGS();sample_timer_init();
    IWDG->KR=0x5555;IWDG->PR=4;IWDG->RLR=1250;IWDG->KR=0xCCCC;IWDG->KR=0xAAAA;
    for(;;){
        MotionView v=snapshot();buttons(v.ms);telemetry(&v);bt_pump();screen_service(&v);bt_pump();IWDG->KR=0xAAAA;
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
void Error_Handler(void) {
    pd_emergency_off();
    __disable_irq();
    while(1) {}
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
