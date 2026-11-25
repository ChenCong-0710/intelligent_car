#include "main.h"
#include "lcd_font.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
/* 16 MHz HSI, TIM2 1 MHz, 50 Hz PWM, same wiring as wheel test. */
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U
#define STEP_MS 80U
static int angle_deg=0, direction=1;
static uint16_t left_us=2400,centre_us=1460,right_us=500,command_us=1460;
static bool paused=false,pwm_off=false;
static uint32_t next_step,power_since;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void gpio(GPIO_TypeDef *port,uint32_t pins,uint32_t mode,uint32_t af) {
 GPIO_InitTypeDef g={0};g.Pin=pins;g.Mode=mode;g.Pull=GPIO_NOPULL;
 g.Speed=GPIO_SPEED_FREQ_HIGH;g.Alternate=af;HAL_GPIO_Init(port,&g);
}
static void hardware_init(void) {
 __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
 __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOE_CLK_ENABLE();
 /* Motor bridge and laser remain disabled throughout this calibration. */
 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2|GPIO_PIN_8,GPIO_PIN_RESET);
 gpio(GPIOD,GPIO_PIN_2|GPIO_PIN_8,GPIO_MODE_OUTPUT_PP,0);
 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_PIN_RESET);
 gpio(GPIOC,GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_MODE_OUTPUT_PP,0);
 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14,GPIO_PIN_RESET);
 gpio(GPIOE,GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14,GPIO_MODE_OUTPUT_PP,0);
 GPIO_InitTypeDef keys={0};keys.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_15;
 keys.Mode=GPIO_MODE_INPUT;keys.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOD,&keys);
 __HAL_RCC_TIM2_CLK_ENABLE();gpio(GPIOA,GPIO_PIN_15,GPIO_MODE_AF_PP,GPIO_AF1_TIM2);
 TIM2->PSC=15;TIM2->ARR=19999;TIM2->CCR1=0;
 TIM2->CCMR1=TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE;
 TIM2->CCER=TIM_CCER_CC1E;TIM2->EGR=TIM_EGR_UG;TIM2->CR1=TIM_CR1_CEN;
}
static uint16_t pulse_for_angle(int a) {
 int p=a>=0?(int)centre_us+((int)left_us-centre_us)*a/90:
                      (int)centre_us+((int)right_us-centre_us)*(-a)/90;
 return (uint16_t)p;
}
/* Keys have 25ms debounce; no key autorepeat at mechanical endpoints. */
static void keys_service(uint32_t now) {
 static bool raw[3],stable[3],long_done;
 static uint32_t changed[3],pressed;
 const uint16_t pins[3]={GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_15};
 for(unsigned i=0;i<3;++i) {
  bool down=HAL_GPIO_ReadPin(GPIOD,pins[i])==GPIO_PIN_RESET;
  if(down!=raw[i]) {raw[i]=down;changed[i]=now;}
  if(stable[i]!=raw[i] && now-changed[i]>=25U) {
   stable[i]=raw[i];
   if(i==0) {
    if(stable[i]) {pressed=now;long_done=false;}
    else if(!long_done) {
     if(pwm_off) {pwm_off=false;paused=true;power_since=now;HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);}
     else paused=!paused;
     next_step=now+STEP_MS;
    }
   } else if(stable[i] && paused && !pwm_off) {
    /* Trim an actual calibration anchor, not a fictitious feedback angle. */
    uint16_t *anchor=angle_deg>=45?&left_us:(angle_deg<=-45?&right_us:&centre_us);
    angle_deg=angle_deg>=45?90:(angle_deg<=-45?-90:0);
    int p=(int)*anchor+(i==1?10:-10);
    if(p>=SERVO_MIN_US && p<=SERVO_MAX_US)*anchor=(uint16_t)p;
    command_us=pulse_for_angle(angle_deg);
   }
  }
 }
 if(stable[0] && !long_done && now-pressed>=1000U) {
  long_done=true;paused=true;pwm_off=true;TIM2->CCR1=0;
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
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
  lcd_text2(12U, 10U, "SERVO CALIB", LCD_WHITE);
  lcd_text(12U, 262U, "PD0 HOLD  PD1 +  PD15 -", LCD_GREEN);
}



static void screen_service(uint32_t now) {
 static char lines[12][37];static unsigned row,col;static uint32_t last;
 if(!row&&!col) {
  if(now-last<150U)return;
  last=now;
  snprintf(lines[0],37,"%s",pwm_off?"PWM OFF":(paused?"HOLD / TRIM":"AUTO SLOW SWEEP"));
  snprintf(lines[1],37,"CMD ANGLE: %+4d deg",angle_deg);
  snprintf(lines[2],37,"PWM: %4u us / 50 Hz",command_us);
  snprintf(lines[3],37,"%s",angle_deg==90?"LEFT 90 END":(angle_deg==-90?"RIGHT 90 END":(angle_deg==0?"CENTRE 0":"MOVING")));
  snprintf(lines[4],37,"LEFT +90: %4u us",left_us);
  snprintf(lines[5],37,"CENTRE 0: %4u us",centre_us);
  snprintf(lines[6],37,"RIGHT -90: %4u us",right_us);
  snprintf(lines[7],37,"ANGLE IS COMMAND, NOT FEEDBACK");
  snprintf(lines[8],37,"PD0: PAUSE / RESUME");
  snprintf(lines[9],37,"HOLD PD0 1s: PWM OFF");
  snprintf(lines[10],37,"PAUSED: PD1 +10us PD15 -10us");
  snprintf(lines[11],37,"TRIM RAM ONLY; RESET DEFAULTS");
  for(unsigned r=0;r<12;++r) {unsigned n=0;while(n<36&&lines[r][n])++n;while(n<36)lines[r][n++]=' ';lines[r][36]=0;}
 }
 char ch[2]={lines[row][col],0};
 lcd_text((uint16_t)(12+col*6),(uint16_t)(48+row*17),ch,LCD_WHITE);
 if(++col==36) {col=0;if(++row==12)row=0;}
}
int main(void) {
 HAL_Init();SystemClock_Config();MX_GPIO_Init();hardware_init();Lcd_Init();
 power_since=HAL_GetTick();HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
 next_step=power_since+2000U;
 for(;;) {
  uint32_t now=HAL_GetTick();keys_service(now);
  if(!paused&&!pwm_off&&(int32_t)(now-next_step)>=0) {
   angle_deg+=direction;
   if(angle_deg>=90) {angle_deg=90;direction=-1;next_step=now+3000U;}
   else if(angle_deg<=-90) {angle_deg=-90;direction=1;next_step=now+3000U;}
   else if(angle_deg==0)next_step=now+1500U;
   else next_step=now+STEP_MS;
   command_us=pulse_for_angle(angle_deg);
  }
  TIM2->CCR1=(!pwm_off&&now-power_since>=10U)?command_us:0;
  screen_service(now);
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
  TIM2->CCR1=0;
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
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
