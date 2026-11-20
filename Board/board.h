/**
 * @file    board.h
 * @brief   STM32G474VET6 板级支持包 (BSP) 硬件抽象层接口
 * @details
 * 本文件封装了智能小车所有片上外设与扩展板硬件资源的底层操作接口，使上层控制算法
 * (Control/line_follower.c) 与具体寄存器完全解耦：
 *   - 80 MHz 系统时钟 (HSI + PLL) 与 PB6 UCPD 死电下拉关闭
 *   - TIM6 (1 kHz) 触发 ADC1 四通道扫描 + DMA1_Channel1 双缓冲 5× 块均值 (200 Hz 硬件同步节拍)
 *   - TIM1 (20 kHz, AF2) 四通道同步预装载 PWM 驱动 DRV8833 双 H 桥
 *   - TIM4 (PB6/PB7, AF2) 左轮正交编码器与 TIM3 (PA6/PA7, AF2) 右轮正交编码器
 *   - TIM17 (PB9, AF1) 45 us 硬件单脉冲超声触发与 TIM8 (PC6, AF4) 1 us 硬件双边沿回波捕获
 *   - TIM2 (PA15, AF1) 50 Hz 云台舵机 PWM 与 PC7 舵机独立电源防倒灌控制
 *   - PD8 停车灯、PC9 蜂鸣器、PC8 激光、PD2 电机总使能、PC4 驱动故障、三路按键与 USART1 蓝牙串口
 *   - 片内 Flash 顶部页 (0x0807F800) 带 CRC32 校验的掉电校准与参数非易失性存储
 */

#ifndef LINE_BOARD_H
#define LINE_BOARD_H
#include "line_follower.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 控制环标称周期：5 ms (对应 200 Hz 控制频率) */
#define BOARD_CONTROL_MS 5U

/** @brief 电机 H 桥 PWM 载波频率：20 kHz (超出人耳听觉范围，电流纹波小) */
#define BOARD_PWM_HZ 20000U

/** @brief 蓝牙调试串口 (USART1: PA9 TX / PA10 RX) 波特率：115200 bps */
#define BOARD_UART_BAUD 115200U

/** @brief 初始化系统时钟、GPIO、TIM1/2/3/4/6/8/17、ADC1+DMA、USART1 与独立看门狗 */
void board_init(void);

/** @brief 获取系统上电以来的毫秒滴答计数 (HAL_GetTick) */
uint32_t board_millis(void);

/**
 * @brief  检查 200 Hz (5.0 ms) 硬件同步控制节拍是否就绪
 * @param  now_ms 输出当前毫秒时间戳
 * @param  dt_s   输出精确控制周期 (正常为 0.005 s，由 TIM6 + ADC DMA 半满/全满中断推进)
 * @return true 新的一帧 5× 均值传感器数据已就绪，可执行一次 lf_step()
 */
bool board_control_ready(uint32_t *now_ms, float *dt_s);

/** @brief 原子读取最新 5× 均值红外 ADC、双轮编码器增量、驱动故障状态与超声波距离 */
void board_sample(LfInput *sample);

/** @brief 将控制器输出应用到电机 H 桥、换向保护互锁、停车灯、蜂鸣器、激光与云台舵机 */
void board_apply(const LfOutput *out, const LfConfig *cfg);

/** @brief 紧急安全关断：立即拉低 PD2 电机使能、清零 PWM、关闭激光/舵机电源与蜂鸣器 */
void board_emergency_off(void);

/** @brief 从片内 Flash (0x0807F800) 读取并校验 CRC32 后恢复红外标定与控制参数 */
bool board_flash_load(LfCalibration *cal, LfConfig *cfg);

/** @brief 将当前红外标定、电机死区标定与控制参数计算 CRC32 后写入片内 Flash */
bool board_flash_save(const LfCalibration *cal, const LfConfig *cfg);

/** @brief 擦除片内 Flash 参数存储区 */
bool board_flash_clear(void);

/** @brief 读取主扩展板“循迹键” (PD0，分压直采 KEY_TRACK_N，低电平有效：松开 3.185V / 按下 0V) */
bool board_track_pressed(void);

/** @brief 读取主扩展板“探测键” (PD1，低电平有效) */
bool board_probe_pressed(void);

/** @brief 读取 G474 核心板板载用户按键 (PD15，低电平有效) */
bool board_core_pressed(void);

/** @brief 物理 I/O 引脚与编码器/ADC 原始电气状态快照（用于串口 `io` 故障快速定位） */
typedef struct {
    bool key_track_pin;          /**< PD0  引脚实测电平 (true=HIGH ~3.185V, false=LOW 0V) */
    bool key_probe_pin;          /**< PD1  引脚实测电平 (true=HIGH 3.3V, false=LOW 0V) */
    bool key_core_pin;           /**< PD15 引脚实测电平 (true=HIGH 3.3V, false=LOW 0V) */
    bool drv_nfault_pin;         /**< PC4  DRV8833 nFAULT (true=HIGH 正常, false=LOW 故障) */
    bool bt_status_pin;          /**< PC10 蓝牙 BT_STATUS (true=HIGH, false=LOW) */
    bool us_echo_pin;            /**< PC6  超声波 ECHO 实时引脚电平 */
    uint16_t raw_adc[LF_CHANNELS]; /**< 4 路红外 ADC 实时 12-bit 采样均值 */
    uint16_t enc_left_cnt;       /**< 左轮编码器 TIM4->CNT 当前硬件计数值 */
    uint16_t enc_right_cnt;      /**< 右轮编码器 TIM3->CNT 当前硬件计数值 */
} BoardIoSnapshot;

/** @brief 无副作用读取全部关键输入 GPIO 原始电平、ADC 与正交编码器计数值 */
void board_read_io(BoardIoSnapshot *io);

/** @brief 控制 G474 核心板状态指示灯 (PC0，低电平点亮) */
void board_led(bool on);

/** @brief 非阻塞格式化打印日志到 USART1 蓝牙/调试串口环形发送缓冲区 */
void board_log(const char *format, ...);

/** @brief 从 USART1 环形接收缓冲区非阻塞读取 1 字节 (-1=无数据，-2=发生溢出需丢弃当前行) */
int board_uart_getchar(void);

/** @brief 获取串口缓冲区累计丢包次数 */
uint32_t board_uart_drops(void);

/**
 * @brief 获取上电时 HC-04 蓝牙模块波特率自适应探测结果
 * @return 0=未检测到 AT 响应(可能开机时蓝牙已直连透明传输，或接有线 USB-TTL)，
 *         1=已在 115200 波特率握手成功，
 *         2=已自动将出厂 9600 永久升级至 115200，
 *         3=升级未生效并安全回退至 9600 波特率
 */
uint8_t board_bt_probe_status(void);

/** @brief 读取 HC-04 状态引脚 (PC10 BT_STATUS 当前逻辑电平：true=高电平，false=低电平) */
bool board_bt_status_pin(void);

/** @brief 非阻塞分片刷新 1.8 寸 ST7735S LCD 屏（显示 4 路红外、PID_E 与实际左右轮速） */
void board_lcd_update(const LfController *c);

/** @brief 切换 1.8 寸 ST7735S 竖屏正/倒方向 (false=默认正竖屏 0xC0, true=旋转180° 0x00) */
void board_lcd_flip(bool flip_180);

/** @brief 动态设置 TIM1 电机 PWM 载波频率 (Hz，4000~60000；用于 1:1 匹配硬件实测三角波频率) */
void board_set_pwm_hz(uint32_t freq_hz);

/** @brief 读取当前 TIM1 电机 PWM 载波频率 (Hz) */
uint32_t board_get_pwm_hz(void);

/** @brief 获取当前实际使用的 ADC 参考电压 VREF+ (V)：若外部未接 3.3V 则自动启用芯片内部 2.90V VREFBUF */
float board_adc_vref_v(void);

/** @brief 获取当前 VREF+ 来源模式 (0=回退值, 1=外部3.3V参考, 2=自动启用内部2.90V_VREFBUF) */
uint8_t board_adc_vref_mode(void);

/** @brief 喂独立看门狗 (IWDG 超时约 0.5 s，主循环卡死时自动复位并关断电机) */
void board_watchdog_feed(void);

/** @brief 进入 CPU 休眠等待中断 (__WFI)，降低空闲功耗 */
void board_idle(void);

#endif
