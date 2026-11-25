/**
 * @file    board.c
 * @brief   STM32G474VET6 板级支持包 (BSP) 硬件驱动实现
 * @details
 * 已与嘉立创 EDA 主扩展板实时网表 (docs/live-main-netlist.json)、CubeMX G47x 复用数据库
 * 以及核心板原理图逐脚核对：
 *   - PA0..3       : ADC1_IN1..4 四路红外模拟输入（TIM6 1 kHz 硬件触发 + DMA 5× 块均值 → 200 Hz）
 *   - PE9/11/13/14 : TIM1_CH1..4 (GPIO_AF2_TIM1) 20 kHz 双 H 桥四路同步 PWM（注意 PE14 的 AF6 是 BKIN2！）
 *   - PB6/7        : 左轮正交编码器 TIM4_CH1/2 (GPIO_AF2_TIM4，必须先关闭 G4 UCPD 死电下拉)
 *   - PA6/7        : 右轮正交编码器 TIM3_CH1/2 (GPIO_AF2_TIM3，主板焊盘已做镜像换相)
 *   - PB9          : 超声触发 MCU_US_TRIG -> TIM17_CH1 (GPIO_AF1_TIM17) 硬件 45 us 单脉冲发生器
 *   - PC6          : 超声回波 MCU_US_ECHO -> TIM8_CH1 (GPIO_AF4_TIM8) 硬件 1 us 双边沿脉宽捕获
 *   - PA15 / PC7   : 云台舵机 PWM TIM2_CH1 (GPIO_AF1_TIM2, 50 Hz) + PC7 舵机电源/与门防倒灌使能
 *   - PD2 / PC4    : PD2 电机总使能 (高有效)；PC4 DRV8833 nFAULT 故障诊断输入 (低有效)
 *   - PD8/PC9/PC8  : PD8 停车红灯、PC9 蜂鸣器、PC8 激光驱动 (均为高电平有效)
 *   - PA9 / PA10   : USART1_TX / RX (GPIO_AF7_USART1) 接 HC-04 蓝牙模块与调试串口
 *   - DRV_nSLEEP 与 IR_ENABLE 由扩展板硬件 POR_OK 控制，不占用 MCU 引脚。
 */

#include "board.h"
#include "bridge_logic.h"
#include "lcd_st7735.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* BOARD_SWAP_LR（左右通道对调）统一定义在 board.h，并随上电日志 PROBE_BUILD 打印；
 * 本文件只在采样（TIM3/TIM4）与 H 桥映射（values[]）两处使用它。 */

/* ============================================================================
 * 1. 片上外设句柄与硬件缓冲区定义
 * ============================================================================ */

static ADC_HandleTypeDef adc;
static DMA_HandleTypeDef adc_dma;
static TIM_HandleTypeDef pwm, servo_timer, sample_timer, encoder_left, encoder_right;
static TIM_HandleTypeDef us_trig_timer, us_echo_timer;
static UART_HandleTypeDef uart;

/**
 * @brief ADC1 + DMA 双缓冲 5× 过采样配置
 * @details
 * TIM6 每 1 ms (1 kHz) 触发一次 ADC1 扫描 4 路红外通道。
 * DMA 环形缓冲区共 40 个 uint16_t：前 20 个为前半缓冲 (5 次扫描 × 4 通道)，后 20 个为后半缓冲。
 * 因此每 5.0 ms (200 Hz) 触发一次 DMA 半满或全满中断，在中断内对每通道 5 次采样求均值。
 */
#define ADC_OVERSAMPLE 5U
#define DMA_HALF_WORDS (ADC_OVERSAMPLE * 4U)
static _Alignas(4) uint16_t dma_samples[DMA_HALF_WORDS * 2U];
static volatile uint16_t latest_adc[4];
static volatile uint32_t adc_time, adc_frame_seq;
static volatile bool adc_ready, adc_error;
static uint32_t consumed_frame_seq, last_control_ms;

/* 编码器、超声波、电机换向保护与舵机上电时序状态变量 */
static uint16_t previous_count[2];
static uint16_t latest_obstacle_mm;
static uint32_t obstacle_time, last_us_trig_ms;
static bool obstacle_ready;
static bool us_waiting_for_echo;
static uint32_t us_result_seq, us_result_trigger_ms;
static uint16_t us_result_mm;
static int8_t motor_direction[2];
static uint32_t reverse_deadline[2];
static bool reversing[2];
static bool servo_powered;
static uint32_t servo_power_since;

/* USART1 非阻塞环形收发缓冲区（防止串口打印阻塞 200 Hz 控制环） */
#define UART_TX_SIZE 2048U
#define UART_RX_SIZE 256U
static char tx_data[UART_TX_SIZE], rx_data[UART_RX_SIZE];
static volatile uint16_t tx_head, tx_tail, rx_head, rx_tail;
static volatile uint32_t uart_dropped;
static volatile bool rx_overflow;
static uint8_t bt_probe_status;
static uint16_t pwm_period = 4000U;
static uint32_t pwm_freq_hz = 20000U;
static float adc_vref_volts = 3.300f;
static uint8_t adc_vref_mode; /* 0=回退默认, 1=外部VREF+, 2=内部2.90V_VREFBUF */

/* ============================================================================
 * 2. 片内 Flash 非易失性参数与标定存储定义 (0x0807F800)
 * ============================================================================ */

/**
 * @brief 使用 STM32G474 512 KB Flash 的最后一个 2 KB 页 (0x0807F800) 保存标定与参数
 * @note  链接脚本 STM32G474VETX_FLASH.ld 已将代码区限制在前 508 KB，确保不会覆盖此区域
 */
#define FLASH_STORAGE_ADDR 0x0807F800UL
#define FLASH_STORAGE_MAGIC 0x4C464734UL
/* v7 = M2 侧视横扫：LfProbeConfig 新增 11 个 probe_* 参数 (scan_mode, side_* , fit_rms_mm, beam_half_deg, pan_x_mm) */
#define FLASH_STORAGE_VERSION 7UL

typedef struct {
    _Alignas(8) uint32_t magic; /**< 魔数标识 "LFG4" (0x4C464734) */
    uint32_t version;           /**< 数据结构版本号，结构体字段变更时自动升级 */
    LfCalibration cal;          /**< 四路红外黑白标定数据 */
    LfConfig cfg;               /**< 全部 PID 参数、速度环参数与左右轮独立死区标定值 */
    uint32_t crc32;             /**< 前述所有字节的 IEEE 802.3 CRC-32 校验值 */
    uint32_t reserved;          /**< 补齐 8 字节 (64-bit doubleword) 编程对齐 */
} FlashRecord;

/** @brief 计算软件 CRC-32 校验和，用于验证 Flash 存储数据完整性 */
static uint32_t crc32_compute(const void *data, size_t length) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
    return ~crc;
}

/** @brief 外设初始化断言：一旦 HAL 初始化失败，立即切断电机并死循环等待看门狗或人工排查 */
static void require_ok(HAL_StatusTypeDef result) {
    if (result != HAL_OK) {
        board_emergency_off();
        __disable_irq();
        for (;;) {}
    }
}

/* ============================================================================
 * 3. 时钟树、GPIO 与各硬件外设初始化函数
 * ============================================================================ */

/**
 * @brief 配置系统主时钟为 80 MHz (16 MHz HSI / 4 * 40 / 2 = 80 MHz)
 * @note  在初始化任何 GPIO 前，第一时间调用 HAL_PWREx_DisableUCPDDeadBattery()，
 *        彻底断开 PB6 (左轮编码器 TIM4_CH1) 内部的 5.1 kΩ UCPD 死电下拉电阻！
 */
static void clock_init(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    /* 关键硬件修复：关闭 G4 PB6/PB4 的 UCPD 死电下拉，防止左轮编码器脉冲被削幅 */
    HAL_PWREx_DisableUCPDDeadBattery();
    require_ok(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1));
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = RCC_PLLM_DIV4;
    osc.PLL.PLLN = 40;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    require_ok(HAL_RCC_OscConfig(&osc));
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    require_ok(HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2));
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    periph.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    require_ok(HAL_RCCEx_PeriphCLKConfig(&periph));
}

/** @brief 通用 GPIO 引脚配置辅助函数 */
static void gpio(GPIO_TypeDef *port, uint32_t pins, uint32_t mode, uint32_t pull, uint32_t af) {
    GPIO_InitTypeDef pin = {.Pin = pins, .Mode = mode, .Pull = pull,
                            .Speed = GPIO_SPEED_FREQ_HIGH, .Alternate = af};
    HAL_GPIO_Init(port, &pin);
}

/**
 * @brief 初始化数字输入输出 GPIO 引脚
 * @details
 * 先将所有输出锁存器 (ODR) 置为安全关断电平，再切换引脚方向为推挽输出，确保上电瞬间：
 *   - PD2 (MCU_MOTOR_EN) = 0  : 电机驱动禁止
 *   - PD8 (MCU_PARK_LAMP) = 0 : 停车灯熄灭
 *   - PC7 (MCU_SERVO_PWR_EN)=0: 云台舵机电源关闭
 *   - PC8 (MCU_LASER_EN) = 0  : 激光器关闭
 *   - PC9 (MCU_BUZZER) = 0    : 蜂鸣器静音
 *   - PC0 (CORE_LED) = 1      : 核心板指示灯熄灭（低电平点亮）
 */
static void gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOE_CLK_ENABLE();
    /* 先写入安全无效电平，再将引脚配置为输出模式，杜绝上电毛刺动作 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 | GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    gpio(GPIOD, GPIO_PIN_2 | GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);
    gpio(GPIOC, GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);
    gpio(GPIOA, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);
    gpio(GPIOD, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_NOPULL, 0); /* 循迹键 PD0：分压直采 KEY_TRACK_N，常态 3.185V 高电平、按下 0V 低电平有效 */
    gpio(GPIOD, GPIO_PIN_1 | GPIO_PIN_15, GPIO_MODE_INPUT, GPIO_PULLUP, 0); /* 探测键 PD1、核心板键 PD15：低有效 */
    gpio(GPIOC, GPIO_PIN_4 | GPIO_PIN_10, GPIO_MODE_INPUT, GPIO_PULLUP, 0); /* PC4 DRV_nFAULT、PC10 BT_STATUS */
}

/**
 * @brief 初始化 TIM1 四路 20 kHz PWM 用于驱动 DRV8833 双 H 桥
 * @details
 * 引脚映射（必须使用 GPIO_AF2_TIM1）：
 *   - PE9  : TIM1_CH1 -> MCU_AIN1 (左桥 IN1)
 *   - PE11 : TIM1_CH2 -> MCU_AIN2 (左桥 IN2)
 *   - PE13 : TIM1_CH3 -> MCU_BIN1 (右桥 IN1)
 *   - PE14 : TIM1_CH4 -> MCU_BIN2 (右桥 IN2)
 * 计数频率 80 MHz，ARR = 3999，对应 PWM 周期 = 4000 个时钟 = 50 us (20 kHz)。
 */
static void pwm_init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    pwm.Instance = TIM1;
    pwm.Init.Prescaler = 0;
    pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm.Init.Period = 3999; /* 80 MHz / 4000 = 20 kHz */
    pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    require_ok(HAL_TIM_PWM_Init(&pwm));
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    for (unsigned i = 0; i < 4; ++i) require_ok(HAL_TIM_PWM_ConfigChannel(&pwm, &oc, channels[i]));
    gpio(GPIOE, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14,
         GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF2_TIM1);
    for (unsigned i = 0; i < 4; ++i) require_ok(HAL_TIM_PWM_Start(&pwm, channels[i]));
}

void board_set_pwm_hz(uint32_t freq_hz) {
    if (freq_hz < 4000U) freq_hz = 4000U;
    if (freq_hz > 60000U) freq_hz = 60000U;
    if (freq_hz == pwm_freq_hz) return;
    uint32_t new_period = (80000000UL + freq_hz / 2U) / freq_hz;
    if (new_period < 1333U) new_period = 1333U;
    if (new_period > 20000U) new_period = 20000U;
    pwm_freq_hz = freq_hz;
    pwm_period = (uint16_t)new_period;
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    TIM1->ARR = pwm_period - 1U;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0U;
    if (TIM1->CNT >= pwm_period) TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    __set_PRIMASK(saved);
}

uint32_t board_get_pwm_hz(void) { return pwm_freq_hz; }

/**
 * @brief 初始化 TIM2_CH1 (PA15, AF1) 产生 50 Hz 云台舵机控制信号
 * @details
 * 预分频 79 将 80 MHz 分频为 1 MHz (每计数 1 us)，ARR = 19999 对应 20 ms (50 Hz)。
 * 默认 CCR1 = 0 不发波，仅在 PC7 开启舵机电源且经过 5 ms 稳压延时后才输出脉宽，防止信号倒灌。
 */
static void servo_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    servo_timer.Instance = TIM2;
    servo_timer.Init.Prescaler = 79; /* 80 MHz / 80 = 1 MHz (1 us 分辨率) */
    servo_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    servo_timer.Init.Period = 19999; /* 20000 us = 50 Hz */
    servo_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    servo_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    require_ok(HAL_TIM_PWM_Init(&servo_timer));
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    require_ok(HAL_TIM_PWM_ConfigChannel(&servo_timer, &oc, TIM_CHANNEL_1));
    gpio(GPIOA, GPIO_PIN_15, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF1_TIM2);
    require_ok(HAL_TIM_PWM_Start(&servo_timer, TIM_CHANNEL_1));
}

/**
 * @brief 初始化超声波硬件触发 (TIM17, PB9) 与硬件回波脉宽捕获 (TIM8, PC6)
 * @details
 * 采用纯硬件外设协同完成超声波测距，零 CPU 延时阻塞、零中断开销：
 *   1. PB9 (TIM17_CH1, AF1) 配置为单脉冲模式 (One-Pulse Mode)：
 *      每次软件置位 TIM17->CR1.CEN，硬件自动延时 5 us 后输出精确 45 us 高电平触发脉冲并自动停机。
 *   2. PC6 (TIM8_CH1, AF4) 配置为硬件 PWM 输入捕获模式：
 *      - TI1FP1 (上升沿) 触发从模式复位 (TIM_SLAVEMODE_RESET)，硬件自动将 TIM8->CNT 清零；
 *      - TI1FP2 (下降沿) 触发通道 2 捕获，将高电平持续时间 (单位 us) 直接锁存至 TIM8->CCR2！
 */
static void ultrasonic_init(void) {
    /* 1. PB9 -> TIM17_CH1 (AF1) 硬件 45 us 单脉冲触发器 */
    __HAL_RCC_TIM17_CLK_ENABLE();
    us_trig_timer.Instance = TIM17;
    us_trig_timer.Init.Prescaler = 79; /* 1 MHz (1 us) */
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
    us_echo_timer.Init.Prescaler = 79; /* 1 MHz (1 us) */
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

/** @brief 配置单个正交编码器定时器（四倍频 TI12 模式 + ICxF=6，仅反相 IC1 极性使物理前进为正计数） */
static void encoder_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance) {
    timer->Instance = instance;
    timer->Init.Prescaler = 0;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 65535;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    /* 实车手推前进时 TIM3/TIM4 原均向下计数，故反转 IC1 极性令左右轮正向前进输出正脉冲/正速度 */
    enc.IC1Polarity = TIM_ICPOLARITY_FALLING;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = enc.IC2Filter = 6;
    require_ok(HAL_TIM_Encoder_Init(timer, &enc));
    require_ok(HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL));
}

/** @brief 初始化左右轮正交编码器（左轮 PB6/PB7 TIM4，右轮 PA6/PA7 TIM3） */
static void encoders_init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE(); __HAL_RCC_TIM4_CLK_ENABLE();
    gpio(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM4);
    gpio(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3);
    encoder_init(&encoder_left, TIM4);
    encoder_init(&encoder_right, TIM3);
}

/** @brief 临时单次软件触发读取 STM32 内部 VREFINT (~1.21V)，用于检测核心板 VREF+ 是否悬空或已接外部 3.3V */
static uint16_t adc_read_vrefint_avg(void) {
    HAL_ADC_DeInit(&adc);
    adc.Instance = ADC1;
    adc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    adc.Init.Resolution = ADC_RESOLUTION_12B;
    adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc.Init.GainCompensation = 0;
    adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    adc.Init.LowPowerAutoWait = DISABLE;
    adc.Init.ContinuousConvMode = DISABLE;
    adc.Init.NbrOfConversion = 1;
    adc.Init.DiscontinuousConvMode = DISABLE;
    adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc.Init.DMAContinuousRequests = DISABLE;
    adc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    adc.Init.OversamplingMode = DISABLE;
    require_ok(HAL_ADC_Init(&adc));
    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel = ADC_CHANNEL_VREFINT;
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    require_ok(HAL_ADC_ConfigChannel(&adc, &ch));
    require_ok(HAL_ADCEx_Calibration_Start(&adc, ADC_SINGLE_ENDED));
    HAL_Delay(2U);
    uint32_t sum = 0U;
    for (unsigned i = 0U; i < 8U; ++i) {
        require_ok(HAL_ADC_Start(&adc));
        if (HAL_ADC_PollForConversion(&adc, 10U) == HAL_OK) {
            sum += HAL_ADC_GetValue(&adc);
        }
        HAL_ADC_Stop(&adc);
    }
    return (uint16_t)((sum + 4U) / 8U);
}

/**
 * @brief 初始化 ADC1 四路红外扫描 + DMA1_Channel1 循环双缓冲 + TIM6 1 kHz 硬件触发
 * @details
 * 核心板原理图显示 VREF+ 仅经 J5 连往 J2-5，而主板 J_C2-5 悬空：
 * 若重置默认值 (VREFBUF HIZ=0, ENVR=0) 将使 VREF+ 被内部开关拉向 VSSA 导致四路读数漂高并满偏 4095！
 * 本初始化先将 VREFBUF 置为高阻 (HIZ=1) 实测 VREFINT：
 *   - 若检测到外部已接 3.3V (实测 VREF+ ∈ [2.75V, 3.60V])，保持外部参考；
 *   - 若 VREF+ 悬空或低于 2.75V，自动安全开启芯片内部 2.90V VREFBUF 并等待板载 20uF 电容充稳，
 *     再重新校准并反算精确 VREF+ 电压同步给 LCD 与 diag 显示。
 */
static void adc_init(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE(); __HAL_RCC_DMA1_CLK_ENABLE(); __HAL_RCC_DMAMUX1_CLK_ENABLE();
    gpio(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_MODE_ANALOG, GPIO_NOPULL, 0);

    /* 1. 先断开内部下拉开关 (HIZ=1, ENVR=0) 探测核心板是否外接了 VREF+ */
    HAL_SYSCFG_DisableVREFBUF();
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE);
    HAL_Delay(15U);
    uint16_t cal_vrefint = *VREFINT_CAL_ADDR;
    if (cal_vrefint < 1000U || cal_vrefint > 2500U) cal_vrefint = 1657U;
    uint16_t raw_vref_ext = adc_read_vrefint_avg();
    float v_ext = raw_vref_ext > 100U ? (3.000f * (float)cal_vrefint / (float)raw_vref_ext) : 0.0f;

    if (raw_vref_ext >= 1200U && raw_vref_ext <= 1680U && v_ext >= 3.05f && v_ext <= 3.60f) {
        adc_vref_volts = v_ext;
        adc_vref_mode = 1U;
    } else {
        /* 2. 核心板 J5/J2-5 悬空时：自动启用内部 2.90V 基准并等待 C19+C20(20uF) 充稳 */
        HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE2);
        HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
        SET_BIT(VREFBUF->CSR, VREFBUF_CSR_ENVR);
        uint32_t t0 = HAL_GetTick();
        while ((VREFBUF->CSR & VREFBUF_CSR_VRR) == 0U && (HAL_GetTick() - t0) < 120U) {}
        HAL_Delay(10U);
        uint16_t raw_vref_int = adc_read_vrefint_avg();
        float v_int = raw_vref_int > 100U ? (3.000f * (float)cal_vrefint / (float)raw_vref_int) : 0.0f;
        if (raw_vref_int >= 1200U && raw_vref_int <= 2600U && v_int >= 2.00f && v_int <= 3.30f) {
            adc_vref_volts = v_int;
            adc_vref_mode = 2U;
        } else {
            adc_vref_volts = 2.900f;
            adc_vref_mode = 0U;
        }
    }
    lcd_st7735_set_vref(adc_vref_volts);

    HAL_ADC_DeInit(&adc);
    adc_dma.Instance = DMA1_Channel1;
    adc_dma.Init.Request = DMA_REQUEST_ADC1;
    adc_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    adc_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    adc_dma.Init.MemInc = DMA_MINC_ENABLE;
    adc_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    adc_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    adc_dma.Init.Mode = DMA_CIRCULAR;
    adc_dma.Init.Priority = DMA_PRIORITY_HIGH;
    require_ok(HAL_DMA_Init(&adc_dma));
    __HAL_LINKDMA(&adc, DMA_Handle, adc_dma);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    adc.Instance = ADC1;
    adc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; /* 20 MHz */
    adc.Init.Resolution = ADC_RESOLUTION_12B;
    adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc.Init.GainCompensation = 0;
    adc.Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    adc.Init.ContinuousConvMode = DISABLE;
    adc.Init.DiscontinuousConvMode = DISABLE;
    adc.Init.NbrOfConversion = 4;
    adc.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
    adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc.Init.DMAContinuousRequests = ENABLE;
    adc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    adc.Init.OversamplingMode = DISABLE;
    require_ok(HAL_ADC_Init(&adc));
    const uint32_t channels[] = {ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4};
    const uint32_t ranks[] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4};
    for (unsigned i = 0; i < 4; ++i) {
        ADC_ChannelConfTypeDef channel = {0};
        channel.Channel = channels[i]; channel.Rank = ranks[i];
        channel.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
        channel.SingleDiff = ADC_SINGLE_ENDED; channel.OffsetNumber = ADC_OFFSET_NONE;
        require_ok(HAL_ADC_ConfigChannel(&adc, &channel));
    }
    require_ok(HAL_ADCEx_Calibration_Start(&adc, ADC_SINGLE_ENDED));
    __HAL_RCC_TIM6_CLK_ENABLE();
    sample_timer.Instance = TIM6;
    sample_timer.Init.Prescaler = 79;
    sample_timer.Init.Period = 999; /* 1 kHz 触发频率 -> 每 5 次扫描 (5 ms) 产生一次 200 Hz 控制节拍 */
    sample_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    require_ok(HAL_TIM_Base_Init(&sample_timer));
    TIM_MasterConfigTypeDef trigger = {0};
    trigger.MasterOutputTrigger = TIM_TRGO_UPDATE;
    trigger.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    require_ok(HAL_TIMEx_MasterConfigSynchronization(&sample_timer, &trigger));
    require_ok(HAL_ADC_Start_DMA(&adc, (uint32_t *)dma_samples, DMA_HALF_WORDS * 2U));
    require_ok(HAL_TIM_Base_Start(&sample_timer));
}

/** @brief 按指定波特率配置 USART1 (PA9 TX / PA10 RX, 8N1) */
static void uart_apply_baud(uint32_t baud) {
    uart.Instance = USART1;
    uart.Init.BaudRate = baud;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    require_ok(HAL_UART_Init(&uart));
}

/** @brief 清空硬件接收寄存器与错误标志（轮询探测与中断切换前使用） */
static void uart_flush_rx(void) {
    USART1->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_IDLECF;
    while (USART1->ISR & USART_ISR_RXNE_RXFNE) {
        (void)USART1->RDR;
    }
}

/**
 * @brief 向 HC-04 蓝牙模块发送裸 AT 指令（不附带 \r\n）并轮询收集短响应
 * @note  仅在 USART1 中断使能前调用，保证上电探测期间无环形缓冲区竞争
 */
static bool bt_at_probe(const char *cmd, char *resp, size_t cap, uint32_t timeout_ms) {
    if (cap == 0U) return false;
    resp[0] = '\0';
    uart_flush_rx();
    for (const char *p = cmd; *p != '\0'; ++p) {
        uint32_t tx_guard = HAL_GetTick();
        while ((USART1->ISR & USART_ISR_TXE_TXFNF) == 0U) {
            if (HAL_GetTick() - tx_guard > 20U) return false;
        }
        USART1->TDR = (uint8_t)(*p);
    }
    uint32_t tc_guard = HAL_GetTick();
    while ((USART1->ISR & USART_ISR_TC) == 0U) {
        if (HAL_GetTick() - tc_guard > 30U) break;
    }

    size_t len = 0;
    uint32_t start = HAL_GetTick();
    uint32_t last_rx = start;
    while (HAL_GetTick() - start < timeout_ms) {
        uint32_t isr = USART1->ISR;
        if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
            USART1->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF;
        }
        if (isr & USART_ISR_RXNE_RXFNE) {
            char ch = (char)USART1->RDR;
            if (len + 1U < cap) {
                resp[len++] = ch;
                resp[len] = '\0';
            }
            last_rx = HAL_GetTick();
        } else if (len > 0U && strstr(resp, "OK") != NULL && (HAL_GetTick() - last_rx >= 25U)) {
            break;
        }
    }
    return len > 0U;
}

/** @brief 先 9600（无高频乱码污染）后 115200 探测 HC-04，并在不同波特率切换间强制静默 500 ms */
static uint8_t hc04_try_baud_sequence(const char *probe_cmd) {
    char resp[48];
    /* 1. 优先用出厂默认 9600 试探（避免开机先发 115200 脉冲污染 9600 AT 接收缓冲区） */
    uart_apply_baud(9600U);
    HAL_Delay(25U);
    if (bt_at_probe(probe_cmd, resp, sizeof(resp), 180U) && strstr(resp, "OK") != NULL) {
        HAL_Delay(40U);
        const char *baud_cmd = (strcmp(probe_cmd, "AT\r\n") == 0) ? "AT+BAUD=115200\r\n" : "AT+BAUD=115200";
        if (bt_at_probe(baud_cmd, resp, sizeof(resp), 220U) && strstr(resp, "OK") != NULL) {
            /* HC-04 改波特率后自动重启 (~250 ms)，切回 115200 并等待 >=350 ms 稳定 */
            uart_apply_baud(BOARD_UART_BAUD);
            HAL_Delay(380U);
            if (bt_at_probe(probe_cmd, resp, sizeof(resp), 180U) && strstr(resp, "OK") != NULL) {
                return 2U;
            }
        }
        /* 若未确认切换至 115200，安全回退并验证 9600 */
        uart_apply_baud(9600U);
        HAL_Delay(40U);
        if (bt_at_probe(probe_cmd, resp, sizeof(resp), 180U) && strstr(resp, "OK") != NULL) {
            return 3U;
        }
    }
    /* 2. 切换波特率前强制纯静默 >=500 ms，让已处于 115200 的模块彻底清空 AT 解析器超时 */
    HAL_Delay(500U);
    uart_apply_baud(BOARD_UART_BAUD);
    HAL_Delay(25U);
    if (bt_at_probe(probe_cmd, resp, sizeof(resp), 150U) && strstr(resp, "OK") != NULL) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 上电自适配 HC-04 蓝牙模块波特率（解决出厂默认 9600 与固件 115200 失配隐患）
 * @details
 * 在开启 IWDG 看门狗前执行：
 *   1. 等待上电满 400 ms（超过手册 >300 ms 启动静默要求）；
 *   2. 依次在 BT_KEY=LOW 与 BT_KEY=HIGH 下先探 9600 再静默 500 ms 探 115200；
 *   3. 若裸 "AT" 均无应答，兜底再探带回车的 "AT\r\n"（兼容个别要求 CRLF 的固件批次）；
 *   4. 探测结束后强制恢复 BT_KEY=LOW 透明传输态，并清空硬件 RX。
 */
static void hc04_auto_config(void) {
    uint32_t elapsed = HAL_GetTick();
    if (elapsed < 400U) HAL_Delay(400U - elapsed);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(20U);
    bt_probe_status = hc04_try_baud_sequence("AT");
    if (bt_probe_status == 0U) {
        HAL_Delay(500U);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(30U);
        bt_probe_status = hc04_try_baud_sequence("AT");
        if (bt_probe_status == 0U) {
            HAL_Delay(500U);
            bt_probe_status = hc04_try_baud_sequence("AT\r\n");
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(20U);
        if (bt_probe_status == 0U) {
            HAL_Delay(500U);
            bt_probe_status = hc04_try_baud_sequence("AT\r\n");
        }
    }
    if (bt_probe_status == 0U) uart_apply_baud(BOARD_UART_BAUD);
    uart_flush_rx();
}

/** @brief 初始化 USART1 (PA9 TX / PA10 RX 上拉, 默认 115200 8N1) 并执行 HC-04 自适配 */
static void uart_init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF7_USART1);
    gpio(GPIOA, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1);
    hc04_auto_config();
    tx_head = tx_tail = rx_head = rx_tail = 0;
    rx_overflow = false;
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    SET_BIT(USART1->CR1, USART_CR1_RXNEIE_RXFNEIE);
}

/* ============================================================================
 * 4. 板级公共接口实现（初始化、节拍同步、传感器采样、执行器更新、Flash 存取）
 * ============================================================================ */

void board_init(void) {
    HAL_Init(); clock_init(); gpio_init();
    board_emergency_off();
    pwm_init(); servo_init(); ultrasonic_init(); encoders_init();
    /* 在看门狗启动前先完成 LCD 复位亮屏，其 300ms 延时天然覆盖 HC-04 上电静默窗口 */
    lcd_st7735_init();
    uart_init(); adc_init();
    HAL_Delay(15);
    last_control_ms = HAL_GetTick();
    consumed_frame_seq = adc_frame_seq;
    /* 启动独立看门狗 IWDG：标称超时约 0.5 s；一旦程序跑飞复位，PD2 立即由外部下拉电阻关断电机 */
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 3; /* /32 分频 */
    IWDG->RLR = 499;
    while (IWDG->SR != 0) {}
    IWDG->KR = 0xAAAAU;
}

void board_watchdog_feed(void) { IWDG->KR = 0xAAAAU; }
uint32_t board_millis(void) { return HAL_GetTick(); }
void board_idle(void) { __WFI(); }

bool board_control_ready(uint32_t *now_ms, float *dt_s) {
    uint32_t now = HAL_GetTick();
    uint32_t seq = adc_frame_seq;
    uint32_t diff = seq - consumed_frame_seq;
    if (diff > 0U) {
        consumed_frame_seq = seq;
        last_control_ms = now;
        *now_ms = now;
        /* 由硬件 TIM6 + DMA 保证每帧精确对应 5.0 ms (0.005 s)，消除 SysTick 1ms 取整抖动 */
        *dt_s = diff <= 3U ? (float)diff * .005f : .015f;
        return true;
    }
    if (now - last_control_ms >= BOARD_CONTROL_MS * 2U) {
        /* 当 ADC DMA 异常停止时的兜底触发，确保 lf_step 仍被调用并立即触发 LF_ADC_STALE 保护 */
        float dt = (now - last_control_ms) * .001f;
        last_control_ms = now;
        *now_ms = now;
        *dt_s = dt;
        return true;
    }
    return false;
}

/* 每个触发只发布一次结果。无回波在下次触发之前结算，0 序号留给尚未有结果的上电状态。 */
static void ultrasonic_complete(uint16_t mm) {
    us_result_trigger_ms = last_us_trig_ms;
    us_result_mm = mm;
    if (++us_result_seq == 0U) ++us_result_seq;
    us_waiting_for_echo = false;
}

void board_sample(LfInput *sample) {
    uint32_t now = HAL_GetTick();
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    for (unsigned i = 0; i < 4; ++i) sample->raw[i] = latest_adc[i];
    sample->adc_ok = adc_ready && !adc_error && now - adc_time <= 20;
    __set_PRIMASK(saved);
    sample->driver_ok = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET;

    /* 读取 16 位正交编码器计数器并做模 65536 有符号差分，自动处理上下溢出回绕 */
#if BOARD_SWAP_LR
    /* 物理左轮接在右编码器口 (TIM3)，物理右轮接在左编码器口 (TIM4) */
    uint16_t counts[] = {(uint16_t)TIM3->CNT, (uint16_t)TIM4->CNT};
#else
    uint16_t counts[] = {(uint16_t)TIM4->CNT, (uint16_t)TIM3->CNT};
#endif
    for (unsigned i = 0; i < 2; ++i) {
        uint16_t wrapped = (uint16_t)(counts[i] - previous_count[i]);
        int32_t delta = wrapped > 32767 ? (int32_t)wrapped - 65536 : wrapped;
        sample->encoder_delta[i] = (int16_t)delta;
        previous_count[i] = counts[i];
    }

    /* 读取 TIM8_CH2 捕获到的超声波回波下降沿，按声速 343 m/s 换算为毫米距离。
     * CC2IF 是 rc_w0 标志位，**必须软件写 0 清**：本工程没有 TIM8 中断，HAL_TIM_IRQHandler 里那句
     * __HAL_TIM_CLEAR_FLAG 永远不会执行，所以这里不清就没人清。
     * 不清的后果（2026-11-23 定位）：第一次捕获之后 CC2IF 恒为 1，于是每 5 ms 都把同一个 CCR2
     * 当成"新回波"去刷新 obstacle_time → obstacle_valid 永远为真，旧距离被无限重放。
     * 于是"丢回波"在侧扫里永远不会以 d==0 出现，零回波型边界（LF_SIDE_EDGE_ZERO_FRAMES）永不触发，
     * 只剩跳变型边界可用——是多次 PROBE_SIDE_FAIL,edges=0 的成因之一。
     * 清掉之后 obstacle_valid 的语义恢复为"220 ms 内真的收到过一次回波"。 */
    uint32_t echo_flags = TIM8->SR;
    if (echo_flags & TIM_SR_CC2IF) {
        TIM8->SR = ~TIM_SR_CC2IF;                  /* 只清 CC2IF；其余位是 rc_w0，写 1 无副作用 */
        uint16_t pulse_us = (uint16_t)TIM8->CCR2;
        if (pulse_us >= 80U && pulse_us <= 18000U) {
            latest_obstacle_mm = (uint16_t)(((uint32_t)pulse_us * 343U + 1000U) / 2000U);
            obstacle_time = now;
            obstacle_ready = true;
            /* 从本次触发起必须既见过上升沿又见过下降沿；忽略跨周期的残留脉冲。 */
            if (us_waiting_for_echo && (echo_flags & TIM_SR_CC1IF)) {
                ultrasonic_complete(latest_obstacle_mm);
            }
        }
    }
    /* 每 LF_SIDE_PING_MS 启动一次 TIM17 硬件单脉冲触发 (45 us)，完全不阻塞 CPU。
     * 40 ms 为当前试验频率（25 Hz），短于 HC-SR04 常见资料建议的 60 ms 周期。 */
    if (now - last_us_trig_ms >= LF_SIDE_PING_MS) {
        if (us_waiting_for_echo) ultrasonic_complete(0U);
        last_us_trig_ms = now;
        TIM8->SR = ~(TIM_SR_CC1IF | TIM_SR_CC2IF);
        us_waiting_for_echo = true;
        TIM17->CNT = 0;
        SET_BIT(TIM17->CR1, TIM_CR1_CEN);
    }
    sample->obstacle_valid = obstacle_ready && (now - obstacle_time <= 220U);
    sample->obstacle_mm = sample->obstacle_valid ? latest_obstacle_mm : 0U;
    sample->us_active_trigger_ms = last_us_trig_ms;
    sample->us_result_seq = us_result_seq;
    sample->us_trigger_ms = us_result_trigger_ms;
    sample->us_result_mm = us_result_mm;
}

void board_emergency_off(void) {
    /* 可在任意时刻（包括外设初始化前和 HardFault 中）安全调用，直接操作寄存器切断一切功率输出 */
    if (RCC->AHB2ENR & RCC_AHB2ENR_GPIODEN) GPIOD->BSRR = (GPIO_PIN_2 | GPIO_PIN_8) << 16;
    if (RCC->AHB2ENR & RCC_AHB2ENR_GPIOCEN) GPIOC->BSRR = (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9) << 16;
    if (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;
    if (RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN) TIM2->CCR1 = 0;
    servo_powered = false;
}

void board_apply(const LfOutput *out, const LfConfig *cfg) {
    uint32_t now = HAL_GetTick();
    /* 1. 更新声光与激光外设 GPIO */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, out->park_lamp ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, out->buzzer ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, out->laser ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 2. 更新云台舵机电源与 PWM（遵循硬件规范：先开 PC7 电源延时 5 ms 后再发 PWM，关断时先停波防倒灌） */
    if (out->servo_enable) {
        if (!servo_powered) {
            servo_powered = true;
            servo_power_since = now;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
            TIM2->CCR1 = 0;
        } else if (now - servo_power_since >= 5U) {
            uint16_t us = out->servo_us < LF_SERVO_US_MIN ? LF_SERVO_US_MIN
                        : (out->servo_us > LF_SERVO_US_MAX ? LF_SERVO_US_MAX : out->servo_us);
            TIM2->CCR1 = us;
        }
    } else {
        servo_powered = false;
        TIM2->CCR1 = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    /* 3. 若电机未使能、ADC 错误或 DRV8833 报错 (PC4=LOW)，立即拉低 PD2 关断电机 H 桥 */
    if (!out->enable || adc_error || HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) {
        if (RCC->AHB2ENR & RCC_AHB2ENR_GPIODEN) GPIOD->BSRR = GPIO_PIN_2 << 16;
        if (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;
        memset(motor_direction, 0, sizeof(motor_direction));
        memset(reversing, 0, sizeof(reversing));
        return;
    }

    /* 4. 计算左右 H 桥比较值，并在正反转切换之间插入 20 ms 主动制动保护间隔，减小反接浪涌电流 */
    BridgeCompare values[2];
    for (unsigned i = 0; i < 2; ++i) {
        float duty = out->duty[i] * cfg->motor_sign[i];
        int8_t direction = duty > .005f ? 1 : duty < -.005f ? -1 : 0;
        bool brake = out->brake;
        if (direction && motor_direction[i] && direction != motor_direction[i]) {
            if (!reversing[i]) { reversing[i] = true; reverse_deadline[i] = now + 20; }
            if ((int32_t)(now - reverse_deadline[i]) < 0) brake = true;
            else { reversing[i] = false; motor_direction[i] = direction; }
        } else {
            reversing[i] = false;
            if (direction) motor_direction[i] = direction;
        }
        bool brake_pwm = cfg->hw.enable ? !cfg->hw.coast_mode : cfg->drive_brake;
#if BOARD_SWAP_LR
        /* 控制左 (i=0) -> 右桥 values[1] -> TIM1_CH3/4；控制右 (i=1) -> 左桥 values[0] */
        values[1U - i] = bridge_compare(duty, true, brake, brake_pwm, pwm_period);
#else
        values[i] = bridge_compare(duty, true, brake, brake_pwm, pwm_period);
#endif
    }

    /* 5. 利用 TIM1 UDIS 位原子更新四路 CCR 预装载寄存器，确保同一 PWM 周期同步生效 */
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    SET_BIT(TIM1->CR1, TIM_CR1_UDIS);
    TIM1->CCR1 = values[0].in1; TIM1->CCR2 = values[0].in2;
    TIM1->CCR3 = values[1].in1; TIM1->CCR4 = values[1].in2;
    CLEAR_BIT(TIM1->CR1, TIM_CR1_UDIS);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    __set_PRIMASK(saved);
}

bool board_flash_load(LfCalibration *cal, LfConfig *cfg) {
    const FlashRecord *rec = (const FlashRecord *)FLASH_STORAGE_ADDR;
    if (rec->magic != FLASH_STORAGE_MAGIC || rec->version != FLASH_STORAGE_VERSION) return false;
    uint32_t expected = crc32_compute(rec, offsetof(FlashRecord, crc32));
    if (rec->crc32 != expected || !lf_config_valid(&rec->cfg)) return false;
    if (cal) *cal = rec->cal;
    if (cfg) *cfg = rec->cfg;
    return true;
}

static bool flash_write_record(const FlashRecord *rec) {
    board_emergency_off();
    board_watchdog_feed();
    HAL_FLASH_Unlock();
    FLASH->SR = FLASH->SR;
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(FLASH_OPTR_DBANK)
    erase.Banks = (FLASH->OPTR & FLASH_OPTR_DBANK) ? FLASH_BANK_2 : FLASH_BANK_1;
#else
    erase.Banks = FLASH_BANK_1;
#endif
    erase.Page = 127U;
    erase.NbPages = 1U;
    uint32_t page_error = 0;
    bool ok = (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK);
    if (ok && rec) {
        const uint64_t *words = (const uint64_t *)rec;
        size_t count = sizeof(FlashRecord) / sizeof(uint64_t);
        for (size_t i = 0; i < count; ++i) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  FLASH_STORAGE_ADDR + i * 8U, words[i]) != HAL_OK) {
                ok = false;
                break;
            }
        }
    }
    HAL_FLASH_Lock();
    board_watchdog_feed();
    return ok;
}

bool board_flash_save(const LfCalibration *cal, const LfConfig *cfg) {
    if (!cal || !cfg || !lf_config_valid(cfg)) return false;
    FlashRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = FLASH_STORAGE_MAGIC;
    rec.version = FLASH_STORAGE_VERSION;
    rec.cal = *cal;
    rec.cfg = *cfg;
    rec.crc32 = crc32_compute(&rec, offsetof(FlashRecord, crc32));
    return flash_write_record(&rec);
}

bool board_flash_clear(void) { return flash_write_record(NULL); }
bool board_track_pressed(void) { return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET; }
bool board_probe_pressed(void) { return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET; }
bool board_core_pressed(void) { return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_RESET; }
void board_led(bool on) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

void board_log(const char *format, ...) {
    char data[400];
    va_list args; va_start(args, format);
    int length = vsnprintf(data, sizeof(data), format, args); va_end(args);
    if (length <= 0) return;
    if ((size_t)length >= sizeof(data)) length = sizeof(data) - 1;
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    unsigned free_bytes = (tx_tail + UART_TX_SIZE - tx_head - 1) % UART_TX_SIZE;
    if ((unsigned)length > free_bytes) ++uart_dropped;
    else {
        for (int i = 0; i < length; ++i) { tx_data[tx_head] = data[i]; tx_head = (tx_head + 1) % UART_TX_SIZE; }
        SET_BIT(USART1->CR1, USART_CR1_TXEIE_TXFNFIE);
    }
    __set_PRIMASK(saved);
}

int board_uart_getchar(void) {
    if (rx_overflow) {
        uint32_t saved = __get_PRIMASK(); __disable_irq();
        rx_overflow = false; rx_tail = rx_head;
        __set_PRIMASK(saved);
        return -2; /* 发生接收溢出时整行丢弃，绝不执行被截断的残缺命令 */
    }
    if (rx_head == rx_tail) return -1;
    unsigned char value = (unsigned char)rx_data[rx_tail];
    rx_tail = (rx_tail + 1) % UART_RX_SIZE;
    return value;
}

uint32_t board_uart_drops(void) { return uart_dropped; }
uint8_t board_bt_probe_status(void) { return bt_probe_status; }
bool board_bt_status_pin(void) { return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET; }
void board_lcd_update(const LfController *c) { lcd_st7735_service(c, HAL_GetTick()); }
void board_lcd_flip(bool flip_180) { lcd_st7735_set_flip(flip_180); }
float board_adc_vref_v(void) { return adc_vref_volts; }
uint8_t board_adc_vref_mode(void) { return adc_vref_mode; }

void board_read_io(BoardIoSnapshot *io) {
    if (!io) return;
    io->key_track_pin = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_SET;
    io->key_probe_pin = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_SET;
    io->key_core_pin = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET;
    io->drv_nfault_pin = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET;
    io->bt_status_pin = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET;
    io->us_echo_pin = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET;
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    for (unsigned i = 0; i < LF_CHANNELS; ++i) io->raw_adc[i] = latest_adc[i];
    __set_PRIMASK(saved);
    io->enc_left_cnt = (uint16_t)TIM4->CNT;
    io->enc_right_cnt = (uint16_t)TIM3->CNT;
}

/* ============================================================================
 * 5. 硬件中断服务函数与异常安全保护句柄
 * ============================================================================ */

/** @brief DMA 半满/全满回调：对 5 次连续 1 kHz 采样求平均，更新 200 Hz 控制帧序号 */
static void publish_adc(unsigned offset) {
    for (unsigned ch = 0; ch < 4; ++ch) {
        uint32_t sum = 0;
        for (unsigned s = 0; s < ADC_OVERSAMPLE; ++s) sum += dma_samples[offset + s * 4U + ch];
        latest_adc[ch] = (uint16_t)((sum + (ADC_OVERSAMPLE / 2U)) / ADC_OVERSAMPLE);
    }
    adc_time = HAL_GetTick();
    adc_ready = true;
    ++adc_frame_seq;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *handle) { if (handle == &adc) publish_adc(0); }
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *handle) { if (handle == &adc) publish_adc(DMA_HALF_WORDS); }
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle) { if (handle == &adc) { adc_error = true; board_emergency_off(); } }
void DMA1_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&adc_dma); }

void USART1_IRQHandler(void) {
    uint32_t status = USART1->ISR;
    if (status & USART_ISR_RXNE_RXFNE) {
        char value = (char)USART1->RDR;
        uint16_t next = (rx_head + 1) % UART_RX_SIZE;
        if (next == rx_tail) { rx_overflow = true; ++uart_dropped; }
        else { rx_data[rx_head] = value; rx_head = next; }
    }
    if (status & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
        USART1->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF;
        rx_overflow = true;
    }
    if ((status & USART_ISR_TXE_TXFNF) && (USART1->CR1 & USART_CR1_TXEIE_TXFNFIE)) {
        if (tx_tail == tx_head) CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE_TXFNFIE);
        else { USART1->TDR = (uint8_t)tx_data[tx_tail]; tx_tail = (tx_tail + 1) % UART_TX_SIZE; }
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }
void HardFault_Handler(void) { board_emergency_off(); for (;;) {} }
void MemManage_Handler(void) { board_emergency_off(); for (;;) {} }
void BusFault_Handler(void) { board_emergency_off(); for (;;) {} }
void UsageFault_Handler(void) { board_emergency_off(); for (;;) {} }
