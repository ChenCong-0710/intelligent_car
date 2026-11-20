/**
 * @file    bridge_logic.c
 * @brief   DRV8833 双 H 桥驱动芯片的 PWM 调制与衰减模式映射实现
 * @details
 * 根据 TI DRV8833 数据手册真值表：
 *   - IN1 = 0, IN2 = 0 : 滑行 (Coast / 快衰减)，H 桥高阻关断，电机靠机械摩擦减速
 *   - IN1 = 1, IN2 = 0 : 正向驱动 (Forward Drive)
 *   - IN1 = 0, IN2 = 1 : 反向驱动 (Reverse Drive)
 *   - IN1 = 1, IN2 = 1 : 制动 (Brake / 慢衰减)，H 桥低侧双管导通短接电机电枢，产生反电动势制动扭矩
 *
 * 两种 PWM 调制方式对比：
 *   1. 驱动/制动模式 (drive_brake = true, 慢衰减)：
 *      - 正转时：IN1 恒为 1 (CCR = period)，IN2 输出反相占空比 (CCR = period - active)
 *      - 在 PWM 关断周期内，引脚状态为 (1, 1) 电枢短接制动，电流回路阻抗低，占空比与转速呈良好线性关系，
 *        且在连续 S 弯内轮减小占空比时能提供即时电磁减速力。
 *   2. 驱动/滑行模式 (drive_brake = false, 快衰减)：
 *      - 正转时：IN1 输出正相占空比 (CCR = active)，IN2 恒为 0 (CCR = 0)
 *      - 在 PWM 关断周期内，引脚状态为 (0, 0) 高阻滑行，用于与主扩展板纯模拟电路模式做对照试验。
 */

#include "bridge_logic.h"
#include <math.h>

BridgeCompare bridge_compare(float duty, bool enable, bool brake, bool drive_brake, uint16_t period) {
    /* 1. 未使能或输入占空比非法 (NaN/Inf) 时，立即输出 (0, 0) 关断桥臂 */
    if (!enable || !isfinite(duty)) return (BridgeCompare){0, 0};

    /* 2. 主动电磁制动请求：输出 (period, period)，即 IN1=1, IN2=1 短接电枢快速刹停 */
    if (brake) return (BridgeCompare){period, period};

    /* 3. 将占空比幅值限制在 [0.0, 1.0]，并四舍五入换算为定时器计数值 active */
    float a = fminf(1.0f, fabsf(duty));
    uint16_t active = (uint16_t)(a * period + 0.5f);

    /* 4. 驱动/制动模式 (慢衰减)：导通期为 (1,0) 或 (0,1)，续流期为 (1,1) */
    if (drive_brake) {
        return duty >= 0 ? (BridgeCompare){period, (uint16_t)(period - active)} :
                           (BridgeCompare){(uint16_t)(period - active), period};
    }

    /* 5. 驱动/滑行模式 (快衰减)：导通期为 (1,0) 或 (0,1)，续流期为 (0,0) */
    return duty >= 0 ? (BridgeCompare){active, 0} : (BridgeCompare){0, active};
}
