/**
 * @file    bridge_logic.h
 * @brief   DRV8833 双 H 桥驱动芯片的 PWM 逻辑映射接口
 * @details
 * 本文件负责将控制器计算出的“有符号归一化占空比 (-1.0 ~ +1.0)”转换为
 * STM32 高级定时器 TIM1 四个通道的捕获比较寄存器值 (CCR)。
 *
 * 硬件连接关系：
 *   G474 TIM1_CH1..4 (PE9/PE11/PE13/PE14, AF2)
 *     → U4 (SN74AHCT157B 端)
 *     → U5 (DRV8833 双 H 桥驱动器)
 *     → 左右 N20 直流减速电机
 */

#ifndef BRIDGE_LOGIC_H
#define BRIDGE_LOGIC_H
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 单侧 H 桥两个输入引脚 (IN1, IN2) 的定时器比较值结构体
 */
typedef struct {
    uint16_t in1; /**< H 桥输入 1 的 PWM 比较计数值 (0 ~ period) */
    uint16_t in2; /**< H 桥输入 2 的 PWM 比较计数值 (0 ~ period) */
} BridgeCompare;

/**
 * @brief  将电机控制命令映射为 DRV8833 的 IN1/IN2 定时器比较值
 * @param  duty        有符号目标占空比，范围 [-1.0, +1.0]；正值表示前进，负值表示后退
 * @param  enable      电机使能标志；为 false 时强制输出 (0, 0) 高阻滑行
 * @param  brake       主动短接制动标志；为 true 时强制输出 (period, period) 即 IN1=1, IN2=1 电磁制动
 * @param  drive_brake 调制方式选择：
 *                     - true  (推荐): 驱动/制动模式 (慢衰减 Slow Decay)，线性度高、过弯内轮减速快
 *                     - false       : 驱动/滑行模式 (快衰减 Fast Decay)，与纯硬件模拟模式一致
 * @param  period      PWM 定时器满量程计数值（本工程 20 kHz 对应 period = 4000）
 * @return BridgeCompare 计算得到的 IN1 和 IN2 比较寄存器值
 */
BridgeCompare bridge_compare(float duty, bool enable, bool brake, bool drive_brake, uint16_t period);

#endif
