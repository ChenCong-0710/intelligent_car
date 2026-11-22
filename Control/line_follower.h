/**
 * @file    line_follower.h
 * @brief   智能小车核心控制层：快速软件循迹、双轮独立死区与分段曲线前馈速度环、里程计、超声避障停车与四面打靶状态机
 * @details
 * 本模块是与具体单片机硬件解耦的纯算法与状态机核心，既可在 STM32G474 上以 200 Hz (5 ms) 实时运行，
 * 也可在电脑主机端 (MinGW GCC) 直接编译运行自动化单元测试。
 *
 * 主要功能模块：
 *   1. 四路反射式红外传感器归一化、黑白校准、边缘单调外推与全黑横线姿态保持
 *   2. 三次方渐进非线性位置环 PID + 弯道自适应减速 + 失线快速旋回重捕获
 *   3. 4 拍 (20 ms) 滑动窗口正交编码器测速 + 双轮里程计 (位移 s、航向角 θ、平面坐标 x/y)
 *   4. 左右轮独立死区/斜率 + 15 点实测分段线性插值前馈补偿
 *   5. 5 轮空载 (mcal air) 与 5 轮地面带载自动往返 (mcal ground) 电机特性标定状态机 (LF_MOTOR_CAL)
 *   6. < 15 cm 超声波主动电枢短接制动停车 (LF_OBSTACLE_STOP) 与 1.0 s 声光提示
 *   7. 3 分钟限时顺时针四面障碍物瞄准与 10 s 激光发射状态机 (LF_PROBING)
 */

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H
#include <stdbool.h>
#include <stdint.h>

/** @brief 红外循迹传感器通道数量（左外 L2、左内 L1、右内 R1、右外 R2） */
#define LF_CHANNELS 4

/** @brief 编码器测速滑动窗口长度（4 拍 × 5 ms = 20 ms 窗口，降低低速脉冲量化噪声 4 倍） */
#define LF_ENC_WINDOW 4U

/** @brief 电机死区与全量程转速标定的占空比阶梯数量（0.06 ~ 0.95 共 15 档，低速 3% 步长测死区，中高占空比覆盖至 95%） */
#define LF_MCAL_STEPS 15U

/** @brief 云台步进扫掠寻找障碍物平面法线(垂直位置)时允许的最大离散采样点数 */
#define LF_PROBE_SCAN_POINTS 41U

/** @brief 云台舵机 1° 对应的近似脉宽 (us/deg，SG90 类舵机约 10 us/deg)，用于把扫掠脉宽换算成平面法线的几何角度 */
#define LF_PROBE_SCAN_US_PER_DEG 10.0f

/**
 * @brief 云台舵机允许输出的脉宽上下限 (us)
 * @note  旧范围 900~2100 只有 ±60°，不足以让云台侧视 ±90°（侧视横扫法需要 2400us）。
 *        SG90 实测可走 0~180°（约 600~2400us），故放宽；机械顶死时可用 `probe_side_us` 单独调小，
 *        不影响侧视法的中点精度（扫描与发射同角即可，见方案 §3.3）。
 */
#define LF_SERVO_US_MIN 600U
#define LF_SERVO_US_MAX 2400U

/**
 * @brief 顶层运行状态机枚举
 */
typedef enum {
    LF_STOPPED,       /**< 0: 停止待机状态（电机禁能，允许修改参数、启动校准或发车） */
    LF_CALIBRATING,   /**< 1: 八秒红外传感器黑白扫描校准状态 */
    LF_ARMING,        /**< 2: 按下循迹键后的 2 秒起跑倒计时状态（符合任务书硬性要求） */
    LF_TRACKING,      /**< 3: 正常闭环高速循迹状态 */
    LF_LOST_BRAKE,    /**< 4: 丢线后的短时制动过渡状态 */
    LF_SEARCHING,     /**< 5: 丢线后按最后有效偏航方向高速旋回找线状态 */
    LF_STOPPING,      /**< 6: 收到停止指令后的短时主动电磁制动停机状态 */
    LF_FAULT,         /**< 7: 故障保护锁定状态（立即切断驱动，需人工 stop 清故障） */
    LF_OBSTACLE_STOP, /**< 8: 检测到前方 <15cm 障碍物后的主动短接制动保持状态（亮停车灯+1s长鸣） */
    LF_PROBING,       /**< 9: 提高部分（2）顺时针四面障碍物探测与 10s 激光打靶状态 */
    LF_MOTOR_CAL      /**< 10: 左右轮电机死区与全量程转速特性 5 轮自动测试标定状态 */
} LfState;

/**
 * @brief 故障诊断码枚举
 */
typedef enum {
    LF_OK,                /**< 0: 正常无故障 */
    LF_BAD_CALIBRATION,   /**< 1: 红外校准黑白跨度不足 (<200 LSB) 或采样点数不足 */
    LF_ADC_STALE,         /**< 2: ADC 采样超时未更新或 DMA 硬件错误 */
    LF_DRIVER_FAULT,      /**< 3: DRV8833 驱动芯片触发过流/过热保护 (DRV_nFAULT 拉低) */
    LF_LINE_LOST,         /**< 4: 丢线搜索超时仍未找回黑线 */
    LF_AMBIGUOUS_LINE,    /**< 5: 四路红外持续全黑超过设定时间阈值 (ambiguous_ms) */
    LF_ENCODER_DIRECTION, /**< 6: 编码器反馈方向与目标轮速方向相反（A/B 相或电机线序接反） */
    LF_MOTOR_STALL,       /**< 7: 电机堵转或编码器无脉冲反馈（给了较大占空比但轮速持续接近 0） */
    LF_BAD_TIMING,        /**< 8: 控制周期 dt 异常（小于 1 ms 或大于 20 ms） */
    LF_BAD_CONFIG         /**< 9: 控制参数配置越界或存在逻辑冲突 */
} LfFault;

/**
 * @brief 四面顺时针目标探测 (LF_PROBING) 的子阶段枚举
 */
typedef enum {
    LF_PROBE_IDLE,          /**< 0: 未处于探测任务 */
    LF_PROBE_AIM,           /**< 1: 停车状态，水平云台舵机左右扫掠，寻找超声距离极小值以对准平面中心 */
    LF_PROBE_LASER,         /**< 2: 对准平面中心后，保持主动制动+停车灯，同步开启激光与蜂鸣器持续 10 秒 */
    LF_PROBE_TURN_OUT,      /**< 3: 依托编码器里程计原地转体 90°（顺时针绕行时左转），使车头平行于当前障碍物平面 */
    LF_PROBE_DRIVE_EDGE1,   /**< 4: 沿当前平面外侧直线行驶越过拐角（带航向角纠偏） */
    LF_PROBE_TURN_CORNER,   /**< 5: 原地转体 90°（顺时针绕行时右转），使车头平行于下一个侧面 */
    LF_PROBE_DRIVE_EDGE2,   /**< 6: 沿下一侧面外侧直线行驶至中心垂线处（带航向角纠偏） */
    LF_PROBE_TURN_IN,       /**< 7: 原地转体 90°（顺时针绕行时右转），使车头垂直正对下一个平面 */
    LF_PROBE_APPROACH,      /**< 8: 向前缓慢逼近新平面至超声距离 <= obstacle_stop_mm 处刹停，准备下一面瞄准 */
    LF_PROBE_COMPLETE_BEEPS /**< 9: 全部平面完成后，保持停止并连续发出 3 组“两短一长”蜂鸣器鸣叫 */
} LfProbePhase;

/**
 * @brief 四面顺时针障碍物探测与激光打靶 (LF_PROBING) 可调参数结构体
 * @details 所有参数均可通过蓝牙串口 `set probe_*` 实时微调，无需反复重烧固件。
 */
typedef struct {
    bool clockwise;           /**< true = 顺时针绕行四面（任务书默认）；false = 逆时针绕行 */
    uint8_t faces;            /**< 需完成的目标平面数量 (1 ~ 4，默认 4；单面调试可设 1) */
    uint16_t aim_ms;          /**< 云台扫掠寻优总时长 (ms，默认 1200 ms，含前 150 ms 归起点稳定) */
    uint16_t sweep_us;        /**< 云台相对 servo_center_us 的单侧扫掠半幅 (us，默认 600 us -> 900..2100 us) */
    uint16_t scan_step_us;    /**< 步进扫掠的相邻采样点脉宽间隔 (us，默认 75 us；越小越密但单点驻留时间越短) */
    uint16_t laser_ms;        /**< 单面激光+蜂鸣器同步发射持续时间 (ms，默认 10000 ms；调试可缩短) */
    uint16_t aim_min_mm;      /**< 瞄准有效最小超声距离门槛 (mm，默认 40 mm) */
    uint16_t aim_max_mm;      /**< 瞄准有效最大超声距离门槛 (mm，默认 400 mm) */
    uint16_t phase_timeout_ms;/**< 单个运动子阶段的安全超时保护 (ms，默认 4500 ms) */
    float turn_speed_mm_s;    /**< 原地转向单轮线速度 (mm/s，默认 90 mm/s) */
    float drive_speed_mm_s;   /**< 绕边直行线速度 (mm/s，默认 120 mm/s) */
    float approach_speed_mm_s;/**< 逼近下一平面的线速度 (mm/s，默认 85 mm/s) */
    float turn_deg;           /**< 每次 90° 转弯目标角度 (deg，默认 90.0°，可微调补偿地面打滑) */
    float edge1_mm;           /**< 阶段4 离面越角直行距离 (mm，默认 220 mm) */
    float edge2_mm;           /**< 阶段6 沿侧面对中直行距离 (mm，默认 220 mm) */
    float approach_max_mm;    /**< 阶段8 逼近下一面的最大前移安全距离 (mm，默认 260 mm) */
} LfProbeConfig;

/**
 * @brief 电机标定模式枚举（空载架空测试 vs 地面带载往返实测）
 */
typedef enum {
    LF_MCAL_AIR = 0,    /**< 空载标定（车轮架空）：扫描 0.06 ~ 0.95 共 15 档，仅作机械阻尼体检参考 */
    LF_MCAL_GROUND = 1  /**< 地面标定（车放地面）：含轮胎滚动与万向轮阻力，作为实车循迹主前馈模型（推荐优先使用） */
} LfMcalMode;

/**
 * @brief 1:1 仿真主扩展板纯硬件模拟 PID 电路 (U6/U7/U9/U10/U4) 的全部可调硬件等效参数
 * @details
 * 开启 `hw.enable = true` 后，软件严格按主板当前硬件电路方程运行（不做归一化和非线性外推），
 * 让软件中调试出的每一项参数都能 1:1 直接映射到物理电路的电位器、跳线帽或改焊阻容值。
 */
typedef struct {
    bool enable;         /**< true = 启用“硬件模拟 PID 数字孪生仿真模式”；false = 全功能软件循迹模式 */
    bool i_enable;       /**< 积分支路使能，1:1 对应主板 JP_I 跳线帽 (false=拔掉仅PD，true=插上经R47求和) */
    bool coast_mode;     /**< 驱动调制方式，1:1 对应 U4 将硬件模式 IN2 接地 (true=0%滑行快衰减，false=主动制动慢衰减) */
    bool netlist_mode;   /**< true = 按 U6A 真实节点网表 (R39..R46 + 实测 PWM_5V/PWM_MID) 求解；false = 对称权重公式 */
    bool r45_to_5v;      /**< U6A R45 偏置接法 (false=原板接 PWM_MID，true=改焊翘起接 PWM_5V) */
    float w_outer;       /**< netlist_mode=false 时的外侧差分权重，默认 1/3 (对应 50k/150k) */
    float w_inner;       /**< netlist_mode=false 时的内侧差分权重，默认 1/6 (对应 50k/300k) */
    float adc_vref_v;    /**< MCU ADC 实际参考电压 VREF+ (V，板上运行自动同步实测值 ~2.897 V) */
    float v_5v;          /**< 实测主扩展板模拟电源 PWM_5V (V，默认 5.000 V) */
    float v_mid;         /**< 实测主扩展板虚地基准 PWM_MID (V，默认 2.500 V) */
    float e_trim_v;      /**< U6A 静态中心微调偏移 (V，默认 0.000 V，用于补偿电阻公差) */
    float r39_kohm;      /**< U6A 左外 L2 输入电阻 R39 (kΩ，原板 150，改焊组 B 47) */
    float r41_kohm;      /**< U6A 左内 L1 输入电阻 R41 (kΩ，原板 300，改焊组 B 110) */
    float r40_kohm;      /**< U6A 右外 R2 输入电阻 R40 (kΩ，原板 150，改焊组 B 100) */
    float r42_kohm;      /**< U6A 右内 R1 输入电阻 R42 (kΩ，原板 300，改焊组 B 120) */
    float r43_kohm;      /**< U6A 同相基准偏置电阻 R43->PWM_MID (kΩ，默认 100) */
    float r45_kohm;      /**< U6A 偏置电阻 R45 (kΩ，原板 100->PWM_MID，改焊组 B 120->PWM_5V) */
    float r_fb_kohm;     /**< U6A 反馈并联电阻 R44||R46 (kΩ，默认 50) */
    float r_ir_src_kohm; /**< 红外子板输出串阻 (R033..R333，默认 netlist 下 1.0 kΩ；旧模式 0) */
    float kp_stage;      /**< U6B 比例级增益 = (R49+RV_P)/R48，默认 2.5 (物理可调范围 1.0 ~ 11.0) */
    float r60_kohm;      /**< U7A 求和器 P 支路输入电阻 R60 (kΩ，默认 20.0；反馈 R62=20kΩ) */
    float kd_stage_ms;   /**< U6C 微分级增益时间常数 = (R55+RV_D)*C50 (ms，C50=470nF 时为 2.21~49.21 ms) */
    float r61_kohm;      /**< U7A 求和器 D 支路输入电阻 R61 (kΩ，默认 20.0；>=500 等效断开 R61) */
    float d_tau1_ms;     /**< U6C 输入高频截止时间常数 τ1 = R54*C50 (ms，22k*100n=2.20 或 4.7k*470n=2.21) */
    float d_tau2_ms;     /**< U6C 反馈高频滤波时间常数 τ2 = (R55+RV_D)*C60 (ms，10nF 下约 0.05~1.05 ms) */
    float ti_ms;         /**< U6D 积分时间常数 = (R56+RV_I)*C61 (ms，默认 600 ms，对应 100k~1.1MΩ * 1uF) */
    float v_base;        /**< 基准速度电压 V_BASE (V，默认 2.50 V 对应 D0≈48.5%；JP_V 2-3 可低至 1.384 V) */
    float ana_atten;     /**< R74~77 分压衰减系数 = R76/(R74+R76)，默认 2.0/(1.2+2.0) = 0.625 */
    float speed_tau_ms;  /**< R74||R76 与 C62 滤波时间常数 (ms，默认 0.75k*330nF = 0.2475 ms) */
    float tri_min_v;     /**< U9/U10 实测三角波谷底电压 V_tri_min (V，理论 0.351 V) */
    float tri_max_v;     /**< U9/U10 实测三角波波峰电压 V_tri_max (V，理论 2.851 V) */
    float tri_freq_hz;   /**< U9/U10 实测三角波/PWM 载波频率 (Hz，理论约 18.5k~22.6k，hw_emu 自动同步 TIM1) */
} LfHwPidConfig;

/**
 * @brief 可调控制参数与硬件标定配置结构体
 */
typedef struct {
    /* --- 位置环（外环）参数 --- */
    float line_kp;           /**< 位置比例增益 (mm/s)，将归一化位置误差转化为左右轮差速 */
    float line_ki;           /**< 位置积分增益 (mm/s²)，默认设为 0 以避免弯道积分饱和 */
    float line_kd;           /**< 位置微分增益 (mm)，提供入弯/出弯横摆阻尼，抑制连续弯过冲 */
    float line_k3;           /**< 三次方渐进非线性比例系数：pos_eff = pos * (1 + k3 * pos²)，强化外侧大偏差纠偏 */
    float derivative_tau_s;  /**< 位置微分一阶低通滤波时间常数 (s)，滤除高频采样毛刺 */

    /* --- 速度规划与加减速限制参数 --- */
    float cruise_mm_s;       /**< 直线巡航基速 (mm/s) */
    float curve_min_mm_s;    /**< 急弯自动减速的最低基速下限 (mm/s) */
    float max_wheel_mm_s;    /**< 单轮最大允许目标线速度 (mm/s) */
    float max_turn_mm_s;     /**< 位置环输出的最大转向差速幅值 (mm/s) */
    float acceleration_mm_s2;/**< 车轮加速时的最大目标速度变化率 (mm/s²)；减速时自动放宽至 3.5 倍以加快 S 弯换向 */
    float duty_slew_s;       /**< PWM 占空比每秒最大变化率 (1/s) */

    /* --- 速度环（内环）与左右轮独立死区/增益补偿参数 --- */
    float speed_kp;          /**< 编码器速度环比例系数 (duty / (mm/s)) */
    float speed_ki;          /**< 编码器速度环积分系数 (duty / mm) */
    float free_speed_mm_s;   /**< 通用满占空比空载轮速参考值 (mm/s) */
    float feedforward_deadband; /**< 通用电机起转死区占空比参考值 (0 ~ 0.4) */
    float wheel_deadband[2]; /**< [0]=左轮, [1]=右轮 独立实测死区占空比（消除左右电机静摩擦/滚动摩擦死区不一致） */
    float wheel_free_speed[2]; /**< [0]=左轮, [1]=右轮 独立实测满占空比等效转速 (mm/s，消除左右电机斜率不一致) */
    float max_duty;          /**< 最大允许输出 PWM 占空比上限 (0.1 ~ 0.95，默认 0.95) */
    float integral_limit;    /**< 速度环积分项限幅值，防止积分饱和 (Anti-windup) */

    /* --- 车体几何与传感器安装参数 --- */
    float encoder_counts_per_rev; /**< 车轮每转一圈的编码器四倍频总脉冲数（7 PPR × 30:1 × 4 = 840） */
    float wheel_diameter_mm;      /**< 驱动轮直径 (mm，默认 43 mm) */
    float track_width_mm;         /**< 左右驱动轮接地中心间距（轮距 b，默认 130 mm） */
    float sensor_positions[LF_CHANNELS]; /**< 四路探头横向归一化坐标，默认 {-1.0, -0.35, +0.35, +1.0} */

    /* --- 状态机时间与距离阈值参数 --- */
    uint16_t calibration_min_span; /**< 红外校准黑白最小有效压差门槛 (ADC LSB，默认 200) */
    uint16_t calibration_ms;       /**< 红外扫描校准持续时间 (ms，默认 8000 ms) */
    uint16_t arm_delay_ms;         /**< 单击循迹键后的起跑延时 (ms，默认 2000 ms) */
    uint16_t lost_confirm_ms;      /**< 全白失线确认消抖时间 (ms，默认 10 ms) */
    uint16_t lost_brake_ms;        /**< 失线后进入旋回搜索前的短时制动时间 (ms，设为 0 则立即旋回) */
    uint16_t search_ms;            /**< 失线旋回搜索的最大允许时间 (ms，默认 350 ms) */
    uint16_t stop_brake_ms;        /**< 停机指令的主动电磁制动持续时间 (ms，默认 100 ms) */
    uint16_t ambiguous_ms;         /**< 四路全黑（起终点横线/十字交叉）平顺保持最大允许时长 (ms，默认 80 ms) */
    uint16_t obstacle_stop_mm;     /**< 超声波自动停车距离阈值 (mm，默认 150 mm 即 15 cm) */
    uint16_t servo_center_us;      /**< 水平云台舵机正中脉宽 (us，默认 1500 us) */

    /* --- 极性与功能开关 --- */
    int8_t encoder_sign[2];        /**< 左右编码器计数方向修正系数 (+1 或 -1) */
    int8_t motor_sign[2];          /**< 左右电机驱动方向修正系数 (+1 或 -1) */
    uint8_t channel_order[LF_CHANNELS]; /**< 四路红外物理通道映射顺序，默认 {0, 1, 2, 3} */
    bool speed_loop;               /**< 是否启用编码器速度 PI 闭环 (true=前馈+PI闭环，false=仅实测前馈开环) */
    bool allow_reverse;            /**< 急弯时是否允许内侧车轮反转 (true=允许反转缩小转弯半径) */
    bool drive_brake;              /**< PWM 调制模式 (true=驱动/制动慢衰减，false=驱动/滑行快衰减) */
    bool obstacle_enable;          /**< 是否启用超声波 <15cm 自动制动停车功能 */
    LfHwPidConfig hw;              /**< 纯硬件模拟 PID 等效仿真参数（1:1 映射主板电阻/电位器/跳线） */
    LfProbeConfig probe;           /**< 四面顺时针障碍物探测与 10s 激光打靶任务可调参数 */
} LfConfig;

/**
 * @brief 每个控制周期 (5 ms) 由硬件层采集并传入控制器的传感器输入结构体
 */
typedef struct {
    uint16_t raw[LF_CHANNELS]; /**< 四路红外 ADC 12位采样值 (0 ~ 4095，已做 5 次硬件块均值) */
    int16_t encoder_delta[2];  /**< 本周期 (5 ms) 内左右轮编码器新增脉冲数 ([0]=左, [1]=右) */
    uint16_t obstacle_mm;      /**< 超声波测得的前方障碍物距离 (mm) */
    bool adc_ok;               /**< ADC 采样新鲜且无 DMA 错误标志 */
    bool driver_ok;            /**< DRV8833 驱动芯片正常标志 (PC4 DRV_nFAULT 为高电平) */
    bool obstacle_valid;       /**< 超声波回波距离有效标志 */
} LfInput;

/**
 * @brief 四路红外传感器的黑白基准标定值结构体
 */
typedef struct {
    uint16_t black[LF_CHANNELS]; /**< 各通道在黑色胶带上的标定 ADC 值（反射弱，电压低） */
    uint16_t white[LF_CHANNELS]; /**< 各通道在白色底面上的标定 ADC 值（反射强，电压高） */
    bool valid;                  /**< 四路标定数据是否全部通过跨度校验 */
} LfCalibration;

/**
 * @brief 控制器在每个周期 (5 ms) 输出给硬件执行机构的指令结构体
 */
typedef struct {
    bool enable;        /**< 电机总使能信号 (控制 PD2 MCU_MOTOR_EN) */
    bool brake;         /**< H 桥四管主动电枢短接制动标志 (true 时输出 1111 锁停) */
    float duty[2];      /**< 左右电机有符号占空比指令 ([0]=左轮, [1]=右轮, 范围 -1.0 ~ +1.0) */
    bool park_lamp;     /**< 停车指示灯开关 (控制 PD8 MCU_PARK_LAMP) */
    bool buzzer;        /**< 蜂鸣器开关 (控制 PC9 MCU_BUZZER) */
    bool laser;         /**< 激光发射器开关 (控制 PC8 MCU_LASER_EN) */
    bool servo_enable;  /**< 云台舵机独立 5V 电源与门控开关 (控制 PC7 MCU_SERVO_PWR_EN) */
    uint16_t servo_us;  /**< 云台舵机目标脉宽 (控制 PA15 TIM2_CH1，单位 us，900 ~ 2100) */
} LfOutput;

/**
 * @brief 滑动窗口编码器测速与双轮航位推算里程计结构体
 */
typedef struct {
    int16_t window[LF_ENC_WINDOW][2]; /**< 最近 4 拍 (20 ms) 的左右轮编码器增量环形缓冲区 */
    float dt_window[LF_ENC_WINDOW];   /**< 最近 4 拍对应的采样周期 dt (s) */
    uint8_t index;                    /**< 环形缓冲区当前写入下标 */
    uint8_t count;                    /**< 环形缓冲区已填充的有效拍数 (1 ~ 4) */
    float wheel_mm[2];                /**< 左右轮累计行驶弧长 ([0]=左轮, [1]=右轮, 单位 mm) */
    float distance_mm;                /**< 车体中心累计行驶总路程 s (mm) */
    float heading_rad;                /**< 车体累计偏航角 θ (rad，左转为正、右转为负) */
    float x_mm, y_mm;                 /**< 车体在地面坐标系下的推算位置 (mm) */
} LfOdometry;

/**
 * @brief 智能小车控制器主状态结构体（包含全部配置、标定、状态与实时中间量）
 */
typedef struct {
    LfConfig cfg;        /**< 当前生效的控制参数配置 */
    LfCalibration cal;   /**< 当前生效的四路红外黑白标定数据 */
    LfOdometry odo;      /**< 编码器滑窗测速与里程计状态 */
    LfState state;       /**< 当前顶层运行状态 */
    LfFault fault;       /**< 当前故障码 */
    LfProbePhase probe_phase; /**< 四面打靶任务的当前子阶段 */
    LfOutput out;        /**< 当前周期的硬件控制输出 */

    /* --- 状态机时间戳与计数器 --- */
    uint32_t state_since;     /**< 进入当前顶层状态的毫秒时间戳 */
    uint32_t white_since;     /**< 开始检测到全白丢线的毫秒时间戳 */
    uint32_t ambiguous_since; /**< 开始检测到四路全黑的毫秒时间戳 */
    uint32_t phase_since;     /**< 进入当前子阶段（打靶或电机标定阶梯）的毫秒时间戳 */
    uint32_t samples;         /**< 红外校准期间已采集的样本帧数 */
    uint32_t wrong_direction_ms[2]; /**< 左右轮编码器反向持续时间计数 (ms) */
    uint32_t stall_ms[2];     /**< 左右轮堵转持续时间计数 (ms) */

    /* --- 红外传感器信号处理中间量 --- */
    uint16_t raw[LF_CHANNELS];     /**< 经过通道重映射后的四路原始 ADC 值 */
    uint16_t minimum[LF_CHANNELS]; /**< 校准期间记录的各通道最小值（黑线基准） */
    uint16_t maximum[LF_CHANNELS]; /**< 校准期间记录的各通道最大值（白底基准） */
    float darkness[LF_CHANNELS];   /**< 归一化后的各通道见黑强度 (0.0=纯白, 1.0=纯黑) */
    float filtered_raw[LF_CHANNELS]; /**< 一阶低通滤波后的各通道 ADC 值 */
    float position;      /**< 归一化黑线横向位置 (-1.0=最左, 0.0=正中, +1.0=最右) */
    float last_position; /**< 上一拍的横向位置，用于计算位置微分 */
    float derivative;    /**< 滤波后的位置变化率 d(pos)/dt (1/s) */
    float line_integral; /**< 位置环积分累加项 */
    float confidence;    /**< 四路见黑强度之和 (sum of darkness)，表征黑线信号置信度 */

    /* --- 速度环与电机控制中间量 --- */
    float measured_mm_s[2];  /**< 编码器滑窗滤波后的实测左右轮线速度 (mm/s) */
    float target_mm_s[2];    /**< 经过加减速斜率限制后的左右轮目标线速度 (mm/s) */
    float speed_integral[2]; /**< 左右轮速度环 PI 积分项 */
    float applied_duty[2];   /**< 经过占空比斜率限制后的实际输出占空比 */
    float last_turn;         /**< 丢线前最后一次有效偏航方向记忆（正=偏右，负=偏左） */

    /* --- 四面打靶与超声避障状态量 --- */
    float phase_start_mm;    /**< 当前打靶子阶段起始时的累计里程 (mm) */
    float phase_start_rad;   /**< 当前打靶子阶段起始时的累计航向角 (rad) */
    uint16_t best_servo_us;  /**< 云台扫掠寻优找到的正对平面中心舵机脉宽 (us) */
    uint16_t best_obstacle_mm; /**< 云台扫掠寻优记录的最小垂直距离 (mm) */
    uint8_t probe_face;      /**< 当前正在探测的障碍物平面序号 (0 ~ 3 对应第 1 ~ 4 面) */
    uint8_t obstacle_hits;   /**< 连续检测到 <15cm 障碍物的次数（用于消抖） */
    uint16_t last_obstacle_mm; /**< 最近一拍超声波测距值 (mm，供蓝牙遥测与诊断实时读取) */
    bool last_obstacle_valid;  /**< 最近一拍超声波测距有效标志 */

    /* --- 云台步进扫掠 + 平面法线(垂直位置)拟合状态量 --- */
    uint16_t scan_range_mm[LF_PROBE_SCAN_POINTS]; /**< 本轮扫掠各离散角度的平均超声距离 (mm，0 = 该点无有效回波) */
    uint32_t scan_sum_mm[LF_PROBE_SCAN_POINTS];   /**< 本轮扫掠各离散角度的回波距离累加和 (mm) */
    uint8_t scan_hits[LF_PROBE_SCAN_POINTS];      /**< 本轮扫掠各离散角度的有效回波累计帧数 */
    uint8_t scan_points;                          /**< 本轮扫掠实际使用的离散采样点数量 (3 ~ LF_PROBE_SCAN_POINTS) */
    uint8_t scan_index;                           /**< 当前正在驻留测量的离散采样点序号 */

    /* --- 左右轮电机死区与全量程转速自动标定 (LF_MOTOR_CAL：地面带载为主、空载作参考) --- */
    LfMcalMode mcal_mode;                   /**< 当前正在扫描的模式：LF_MCAL_AIR 或 LF_MCAL_GROUND */
    LfMcalMode mcal_active_mode;            /**< 当前激活用于循迹前馈的标定数据源（优先采用 GROUND） */
    float mcal_sum_speed[LF_MCAL_STEPS][2]; /**< 各占空比阶梯在多轮测试中的累计实测轮速和 */
    float mcal_curve_speed[LF_MCAL_STEPS][2]; /**< 当前生效的 15 点实测 (占空比->转速) 前馈查找表 */
    float mcal_air_curve[LF_MCAL_STEPS][2];   /**< 空载参考曲线（仅机械对比参考，不覆盖已测地面曲线） */
    float mcal_ground_curve[LF_MCAL_STEPS][2];/**< 地面带载曲线（含轮胎与万向轮摩擦，实车循迹主用） */
    float mcal_air_db[2], mcal_air_fs[2];     /**< 空载参考拟合死区与满速 */
    float mcal_ground_db[2], mcal_ground_fs[2]; /**< 地面带载拟合死区与满速 */
    uint16_t mcal_samples[LF_MCAL_STEPS];   /**< 各占空比阶梯累计采样的有效帧数 */
    uint16_t mcal_pass_samples;             /**< 当前单轮单步内已采样的帧数 */
    float mcal_pass_sum[2];                 /**< 当前单轮单步内左右轮速度累加值 */
    float mcal_last_duty;       /**< 刚完成测试的阶梯占空比（供蓝牙上位机实时回传） */
    float mcal_last_speed[2];   /**< 刚完成测试的单步左右轮平均转速 (mm/s，供蓝牙上位机实时回传) */
    uint8_t mcal_pass;          /**< 当前正在执行的电机标定轮次 (0 ~ mcal_total_passes-1) */
    uint8_t mcal_total_passes;  /**< 电机标定设定的总重复轮次数 (1 ~ 10，默认 5 轮求平均) */
    uint8_t mcal_step;          /**< 当前正在执行的占空比阶梯序号 (0 ~ LF_MCAL_STEPS) */
    uint8_t mcal_max_steps;     /**< 当前模式下扫描的最大阶梯数（空载与地面带载均为 15 档到 0.95） */
    uint8_t mcal_last_pass;     /**< 刚完成采样的轮次号 */
    uint8_t mcal_last_step;     /**< 刚完成采样的阶梯号 */
    bool mcal_returning;        /**< 地面标定模式下每轮结束正在自动倒车返回起始点标志 */
    bool mcal_return_done;      /**< 地面标定模式下当前轮倒车归位已完成/超时锁存，避免超时后重复重入倒车 */
    bool mcal_step_pause;       /**< 地面标定模式下每步之间的短时刹车停顿标志 */
    bool mcal_step_ready;       /**< 单步阶梯测试数据就绪标志（通知主循环通过蓝牙打印该步数据） */
    bool mcal_done_event;       /**< 全部轮次电机死区标定完成事件标志（通知主循环打印拟合报告） */
    bool mcal_valid;            /**< 当前激活前馈曲线有效标志 */
    bool mcal_air_valid;        /**< 空载参考曲线有效标志 */
    bool mcal_ground_valid;     /**< 地面带载主曲线有效标志 */

    /* --- 硬件模拟 PID 数字孪生中间量与 LCD 实时监测输出 --- */
    float hw_weight_p;          /**< U6A 同相求和节点 WEIGHT_P (U6.3) 电压 (V) */
    float hw_pid_e;             /**< U6A 模拟加权输出端 PID_E (U6.1) 等效物理电压 (V，中点 v_mid) */
    float hw_v_p;               /**< U6B 比例支路相对 v_mid 输出偏移 V_P - V_MID (V) */
    float hw_v_d;               /**< U6C 微分支路相对 v_mid 输出偏移 V_D - V_MID (V) */
    float hw_pid_c;             /**< U7A 模拟 PID 求和端 PID_C (U7.1) 等效物理电压 (V，中点 v_mid) */
    float hw_v_i;               /**< U6D 模拟积分器输出偏移电压 (V，未启用 JP_I 时为 0) */
    float hw_ana_speed[2];      /**< U10.7 / U10.9 输入端滤波后 ANA_SPEED_L / ANA_SPEED_R 电压 (V) */
    float hw_d_lp1, hw_d_lp2;   /**< U6C 带通微分器双极点滤波内部状态 */
    bool hw_d_ready;            /**< 硬件仿真微分状态初始化标志 */

    /* --- 内部滤波器就绪与消抖标志 --- */
    bool filter_ready;       /**< ADC 一阶低通滤波器已初始化标志 */
    bool derivative_ready;   /**< 位置微分器已初始化标志 */
    bool white_pending;      /**< 正在进行全白丢线消抖确认标志 */
    bool ambiguous_pending;  /**< 正在进行四路全黑消抖确认标志 */
} LfController;

/** @brief 获取编译期默认配置参数 */
LfConfig lf_default_config(void);

/** @brief 校验配置参数是否全部合法且满足组合约束 */
bool lf_config_valid(const LfConfig *cfg);

/** @brief 初始化控制器结构体至安全待机状态 */
void lf_init(LfController *c);

/** @brief 清零编码器滑窗与里程计累计位移/航向角 */
void lf_odometry_reset(LfController *c);

/** @brief 启动 8 秒红外传感器黑白扫描校准 */
void lf_calibrate(LfController *c, uint32_t now_ms);

/**
 * @brief 启动左右轮电机死区与全量程转速特性多次自动测试标定
 * @param c      控制器指针
 * @param mode   标定模式：LF_MCAL_AIR (车轮架空空载测试) 或 LF_MCAL_GROUND (地面带载自动往返测试)
 * @param passes 重复扫描测试轮次数 (1 ~ 10，推荐默认 5 轮以消除单次偶然误差)
 * @param now_ms 当前毫秒时间戳
 * @return true 启动成功；false 当前非停止状态或存在未清除故障
 */
bool lf_calibrate_motors(LfController *c, LfMcalMode mode, uint8_t passes, uint32_t now_ms);

/** @brief 获取电机标定第 step 档 (0 ~ 14) 对应的测试占空比 (0.06 ~ 0.95) */
float lf_mcal_step_duty(uint8_t step);

/** @brief 根据左右轮实测曲线和死区/斜率计算目标轮速对应的前馈占空比（供测试与上位机诊断） */
float lf_wheel_feedforward(const LfController *c, unsigned wheel, float target_mm_s);

/** @brief 切换当前激活用于循迹前馈的电机标定数据集（推荐始终保持 GROUND，仅对比时临时切 AIR） */
bool lf_select_mcal_mode(LfController *c, LfMcalMode mode);

/**
 * @brief 按主扩展板硬件加权电路 U6A 计算模拟 PID 入口电压 PID_E (V)
 * @param c         控制器指针（使用重映射后的 raw[0..3] 与当前配置 cfg.hw）
 * @param e_ana_out 可选输出：相对 v_mid 的净误差 e = PID_E - v_mid
 * @return PID_E 物理电压 (V，限幅在 OPA4376 轨电压 0.05 ~ v_5v-0.05 V)
 */
float lf_compute_analog_pid_e(const LfController *c, float *e_ana_out);

/**
 * @brief 一键应用主板 U6A/U6C 硬件电路预设 (group_b=false 原板 150k/300k，true=改焊选组B)
 */
void lf_hw_apply_preset(LfHwPidConfig *hw, bool group_b);

/**
 * @brief 静态计算当前红外读数对应各模拟节点稳态电压与比较器占空比（供 diag 静止与万用表对点校验）
 */
void lf_hw_eval_static(const LfController *c, float *weight_p_v, float *pid_e_v,
                       float *vp_off_v, float *pid_c_v, float spd_v[2], float duty_out[2]);

/** @brief 启动循迹（进入 2 秒倒计时 LF_ARMING，随后自动开始闭环循迹） */
bool lf_start(LfController *c, uint32_t now_ms);

/** @brief 启动顺时针四面障碍物目标探测与激光打靶任务 */
bool lf_start_probe(LfController *c, uint32_t now_ms);

/** @brief 请求停止（若正在运动则先执行短时主动电磁制动，再关断驱动） */
void lf_stop(LfController *c, uint32_t now_ms);

/** @brief 触发安全故障保护，立即关断电机与激光并锁定故障码 */
void lf_trip(LfController *c, LfFault why);

/** @brief 200 Hz (5 ms) 周期调用的核心控制步进函数 */
void lf_step(LfController *c, const LfInput *in, uint32_t now_ms, float dt_s);

/** @brief 将状态枚举转换为可读字符串 */
const char *lf_state_name(LfState state);

/** @brief 将故障码枚举转换为可读字符串 */
const char *lf_fault_name(LfFault fault);

/** @brief 将四面打靶子阶段枚举转换为可读字符串 */
const char *lf_probe_phase_name(LfProbePhase phase);

#endif
