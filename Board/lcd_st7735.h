/**
 * @file    lcd_st7735.h
 * @brief   1.8 寸 TFT LCD (ST7735S, ZJY180S0800TG01, 128x160 竖屏) 硬件 SPI1 非阻塞遥测显示驱动
 * @details
 * 引脚映射（已与主扩展板实时网表 docs/live-main-netlist.json 中 J_LCD 逐脚核对）：
 *   - PB3  : SPI1_SCK  (GPIO_AF5_SPI1) -> J_LCD Pin 3 (SCL)
 *   - PB5  : SPI1_MOSI (GPIO_AF5_SPI1) -> J_LCD Pin 4 (SDA)
 *   - PD10 : GPIO 输出                  -> J_LCD Pin 5 (RES, 低有效复位)
 *   - PD12 : GPIO 输出                  -> J_LCD Pin 6 (DC,  0=命令 / 1=数据)
 *   - PD11 : GPIO 输出                  -> J_LCD Pin 7 (CS,  低有效片选)
 *   - PD13 : GPIO 输出                  -> J_LCD Pin 8 (BL,  高电平点亮背光)
 *   - 电源 : J_LCD Pin 1=GND, Pin 2=MCU_3V3
 *
 * 实时性保证：
 *   SPI1 工作在 10 MHz (80 MHz / 8)；刷新采用“脏数据对比 + 轮询单槽位增量刷新”，
 *   每次调用 lcd_st7735_service() 至多刷新 1 个字段 (< 1.5 ms)，绝不拖累 200 Hz (5 ms) 控制闭环。
 */

#ifndef LCD_ST7735_H
#define LCD_ST7735_H

#include "line_follower.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化 SPI1、控制引脚与 1.8 寸 ST7735S 液晶屏 (128x160 竖屏)，绘制静态测量面板框架 */
void lcd_st7735_init(void);

/**
 * @brief 运行时切换竖屏正/倒方向 (false=默认正竖屏 0xC0，true=旋转180度 0x00) 并重绘静态底框
 */
void lcd_st7735_set_flip(bool flip_180);

/** @brief 设置 ADC 实际参考电压 (V)，确保屏幕显示真实的物理红外电压与进度条比例 */
void lcd_st7735_set_vref(float vref_v);

/**
 * @brief 主循环空闲期调用的分片非阻塞刷新函数
 * @param c      控制器只读指针，读取四路红外电压、硬件等效 PID_E 与实测左右轮速
 * @param now_ms 当前毫秒时间戳 (HAL_GetTick)
 */
void lcd_st7735_service(const LfController *c, uint32_t now_ms);

#endif
