#include "main.h"
#include "lcd_font.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CONTROL_MS 50U
#define SCREEN_MS 250U
#define COUNTS_PER_REV 840.0f
#define PWM_PERIOD 799U /* HSI16 / 800 = 20 kHz */
#define MAX_DUTY 0.95f
#define KP 0.0030f          /* duty / rpm */
#define KI 0.0120f          /* duty / rpm / s */
#define KD 0.0f            /* noisy encoder derivative is disabled initially */
#define I_LIMIT 0.60f
#define ENCODER_SIGN_L 1
#define ENCODER_SIGN_R 1
#define MOTOR_SIGN_L 1
#define MOTOR_SIGN_R 1

typedef enum { TEST_AIR, TEST_GROUND } TestMode;
typedef enum { MOTOR_STOPPED, MOTOR_RUNNING, MOTOR_FAULT } MotorState;
typedef struct {
  float rpm;
  float duty;
  float integral;
  float previous_rpm;
  uint16_t previous_count;
  uint16_t wrong_ms;
  uint16_t stall_ms;
} Wheel;
typedef struct {
  bool raw;
  bool stable;
  uint32_t changed_ms;
} Key;

static TIM_HandleTypeDef motor_pwm;
static TIM_HandleTypeDef enc_left, enc_right;
static Wheel wheels[2];
static Key keys[3];
static TestMode test_mode = TEST_AIR;
static MotorState motor_state = MOTOR_STOPPED;
static const uint16_t targets_rpm[] = {40U, 60U, 80U, 100U};
static uint8_t target_index = 1U;
static uint32_t last_control_ms;
static uint32_t last_screen_ms;
static bool screen_dirty = true;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static float clampf(float x, float lo, float hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

static void motor_off(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  if (motor_pwm.Instance == TIM1) {
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0U;
  }
  for (unsigned i = 0U; i < 2U; ++i) {
    wheels[i].duty = 0.0f;
    wheels[i].integral = 0.0f;
    wheels[i].wrong_ms = wheels[i].stall_ms = 0U;
  }
}

static void motor_fault(void)
{
  motor_off();
  motor_state = MOTOR_FAULT;
  screen_dirty = true;
}

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

static void speed_control(uint32_t now)
{
  uint32_t elapsed = now - last_control_ms;
  if (elapsed < CONTROL_MS) return;
  last_control_ms = now;
  if (elapsed > 200U) {
    if (motor_state == MOTOR_RUNNING) motor_fault();
    return;
  }
  float dt = (float)elapsed * 0.001f;
  uint16_t counts[2] = {(uint16_t)TIM4->CNT, (uint16_t)TIM3->CNT};
  const int signs[2] = {ENCODER_SIGN_L, ENCODER_SIGN_R};
  for (unsigned i = 0U; i < 2U; ++i) {
    int16_t delta = (int16_t)(counts[i] - wheels[i].previous_count);
    wheels[i].previous_count = counts[i];
    float measured = (float)(delta * signs[i]) * 60.0f / (COUNTS_PER_REV * dt);
    wheels[i].rpm += 0.4f * (measured - wheels[i].rpm);
  }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) {
    motor_fault();
    return;
  }
  if (motor_state != MOTOR_RUNNING) return;
  for (unsigned i = 0U; i < 2U; ++i) {
    Wheel *w = &wheels[i];
    float target = (float)targets_rpm[target_index];
    float error = target - w->rpm;
    float feedforward = 0.14f + target * 0.0040f;
    float candidate_i = clampf(w->integral + KI * error * dt, -I_LIMIT, I_LIMIT);
    float derivative = -(w->rpm - w->previous_rpm) / dt;
    float unsaturated = feedforward + KP * error + candidate_i + KD * derivative;
    float saturated = clampf(unsaturated, 0.0f, MAX_DUTY);
    if ((unsaturated - saturated) * error <= 0.0f) w->integral = candidate_i;
    w->duty = clampf(feedforward + KP * error + w->integral + KD * derivative,
                     0.0f, MAX_DUTY);
    w->previous_rpm = w->rpm;
    w->wrong_ms = w->rpm < -5.0f ? (uint16_t)(w->wrong_ms + elapsed) : 0U;
    w->stall_ms = (w->duty > 0.35f && w->rpm < 4.0f)
                    ? (uint16_t)(w->stall_ms + elapsed) : 0U;
    if (w->wrong_ms >= 150U || w->stall_ms >= 800U) {
      motor_fault();
      return;
    }
  }
  apply_pwm(wheels[0].duty, wheels[1].duty);
}

static void buttons_service(uint32_t now)
{
  const GPIO_PinState values[3] = {
    HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0),
    HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1),
    HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)
  };
  for (unsigned i = 0U; i < 3U; ++i) {
    bool pressed = values[i] == GPIO_PIN_RESET;
    if (pressed != keys[i].raw) {
      keys[i].raw = pressed;
      keys[i].changed_ms = now;
    }
    if (keys[i].stable == keys[i].raw || now - keys[i].changed_ms < 30U) continue;
    keys[i].stable = keys[i].raw;
    if (!keys[i].stable) continue;
    if (i == 0U) {
      if (motor_state == MOTOR_RUNNING) {
        motor_off(); motor_state = MOTOR_STOPPED;
      } else if (motor_state == MOTOR_FAULT) {
        motor_state = MOTOR_STOPPED;
      } else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET) {
        wheels[0].integral = wheels[1].integral = 0.0f;
        wheels[0].wrong_ms = wheels[1].wrong_ms = 0U;
        wheels[0].stall_ms = wheels[1].stall_ms = 0U;
        motor_state = MOTOR_RUNNING;
        last_control_ms = now;
      } else motor_fault();
    } else if (i == 1U && motor_state != MOTOR_RUNNING) {
      target_index = (uint8_t)((target_index + 1U) % 4U);
    } else if (i == 2U && motor_state != MOTOR_RUNNING) {
      test_mode = test_mode == TEST_AIR ? TEST_GROUND : TEST_AIR;
    }
    screen_dirty = true;
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
  lcd_text2(20U, 18U, "WHEEL SPEED PID", LCD_WHITE);
  lcd_text(20U, 248U, "PD0 RUN/STOP  PD1 RPM  PD15 MODE", LCD_GREEN);
}

static void motor_screen(void)
{
  char line[22];
  const char *state = motor_state == MOTOR_RUNNING ? "RUN" :
                      motor_state == MOTOR_FAULT ? "FAULT" : "STOP";
  uint16_t state_color = motor_state == MOTOR_RUNNING ? LCD_GREEN :
                         motor_state == MOTOR_FAULT ? LCD_RED : LCD_YELLOW;
  (void)snprintf(line, sizeof(line), "MODE: %-6s ", test_mode == TEST_AIR ? "AIR" : "GROUND");
  lcd_text2(20U, 50U, line, LCD_WHITE);
  (void)snprintf(line, sizeof(line), "STATE: %-6s", state);
  lcd_text2(20U, 78U, line, state_color);
  (void)snprintf(line, sizeof(line), "SET: %3u RPM  ", (unsigned)targets_rpm[target_index]);
  lcd_text2(20U, 106U, line, LCD_WHITE);
  (void)snprintf(line, sizeof(line), "L: %4d RPM   ",
                 (int)(wheels[0].rpm + (wheels[0].rpm >= 0 ? 0.5f : -0.5f)));
  lcd_text2(20U, 135U, line, LCD_GREEN);
  (void)snprintf(line, sizeof(line), "PWM: %2u%% %s  ",
                 (unsigned)(wheels[0].duty * 100.0f + 0.5f),
                 motor_state == MOTOR_RUNNING && wheels[0].duty >= MAX_DUTY - 0.01f &&
                 wheels[0].rpm < (float)targets_rpm[target_index] - 5.0f ? "MAX" : "   ");
  lcd_text2(20U, 162U, line, LCD_WHITE);
  (void)snprintf(line, sizeof(line), "R: %4d RPM   ",
                 (int)(wheels[1].rpm + (wheels[1].rpm >= 0 ? 0.5f : -0.5f)));
  lcd_text2(20U, 190U, line, LCD_GREEN);
  (void)snprintf(line, sizeof(line), "PWM: %2u%% %s  ",
                 (unsigned)(wheels[1].duty * 100.0f + 0.5f),
                 motor_state == MOTOR_RUNNING && wheels[1].duty >= MAX_DUTY - 0.01f &&
                 wheels[1].rpm < (float)targets_rpm[target_index] - 5.0f ? "MAX" : "   ");
  lcd_text2(20U, 217U, line, LCD_WHITE);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  motor_hardware_init();
  Lcd_Init();
  last_control_ms = HAL_GetTick();
  last_screen_ms = last_control_ms;
  motor_screen();
  for (;;) {
    uint32_t now = HAL_GetTick();
    buttons_service(now);
    speed_control(now);
    if (screen_dirty || now - last_screen_ms >= SCREEN_MS) {
      last_screen_ms = now;
      screen_dirty = false;
      motor_screen();
    }
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
