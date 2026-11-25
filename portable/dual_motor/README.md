# V4 双电机直行控制：可移植封装

这份封装保留已认可的 A50_SYNC_V4 默认控制行为：A相双沿原始计数，B相判方向；每50ms测一次速度；位置式PD无积分；运行中累计计数差同步；500ms柔和起步；各轮到2000分别制动；只前进一次，停稳后保持制动，不自动后退、不做停车后补齐。

## 文件与依赖

- `dual_motor.h`：公共API、参数、实例与状态类型。
- `dual_motor.c`：普通C控制核心，只依赖stdint/stdbool/math/stdlib/string，不包含HAL或寄存器。
- `examples/board_example.c`：需要你实现board_*函数的移植模板，不是可直接烧录的驱动。

复制两个核心文件到目标工程，加入源文件列表与头文件搜索路径。使用C99或更新标准；GCC链接时加`-lm`。没有动态内存分配。每台车创建一个`DualMotor`实例。控制核心没有全局运行状态，可在一个工程中创建多个独立实例。调用方必须传入有效的结构体指针。

## 三层架构

```text
应用层：按键/任务流程/屏幕/蓝牙
       start、stop、get_status
                 ↓
控制核心：dual_motor.c
  1ms：编码器差分、计数累计、终点/保护判断
  50ms：速度采样、起步曲线、轮程同步、每轮速度PD
                 ↓
硬件适配：编码器计数输入 → 核心 → duty/brake/enable输出
  TEST中：TIM3/TIM4捕获A双沿；TIM6调度；TIM1输出PWM；PD2使能
```

核心不主动读取硬件，也不直接操作电机。定时器中断先读取输入、调用核心，再应用输出。这样更换MCU、引脚、电机驱动或屏幕时，控制公式仍可保留。

## 最小调用方式

```c
#include "dual_motor.h"
static DualMotor car;
DualMotorConfig config;

dual_motor_default_config(&config);
/* 如需修改行程，可在初始化前设置 config.goal_counts。 */
if (!dual_motor_init(&car, &config, left_counter, right_counter)) {
    /* 不允许驱动电机，先处理无效配置。 */
}

/* 用户请求开始时调用，和tick中断互斥。 */
dual_motor_start(&car);

/* 固定1ms定时器中断内调用，持续到停车之后。 */
dual_motor_tick_1ms(&car, left_counter, right_counter, driver_ok);
DualMotorStatus output;
dual_motor_get_status(&car, &output);
/* 将output.enable、output.brake[]、output.duty[]写入驱动器。 */
```

`dual_motor_stop()`是用户中止，先制动，150ms后关闭使能。正常到2000停车则保持使能和制动，以保持已验证的V4行为。`dual_motor_trip()`用于捕获溢出、看门狗等外部故障；它只修改输出状态，调用后还要立即应用输出。CPU异常时的硬件紧急断使能应由板级代码实现，不能依靠核心继续运行。

## 编码器输入契约

输入是两轮各自独立、一直累加的16位计数器，前进递增，后退递减；允许65535到0回绕。核心每1ms取差分，并识别有符号增量。不要每50ms清零硬件/软件编码器计数器，不要把每50ms的增量直接作为函数输入。

在TEST里，A相上升/下降沿中断更新软件计数器，B相只判方向；TIM6每1ms读取软件计数器。移植可以保留该方式，也可提供等价的定时器硬件编码器计数，但计数倍率、方向和输入滤波必须一致。A相双沿是A单沿的两倍，却是原A/B四倍频的一半，不能再人为乘2。

芯片和传感器换了以后，第一步应架空验证方向和每转计数，再测试同步。若计数倍率变化，目标50、目标2000、PD/同步参数、前馈和保护阈值都需要重新核对。普通无方向的脉冲数不能直接替代这里的有符号计数器，否则反向/回绕判断失效。

## 控制结构与公式

### 1. 分开的速度计数与行程计数

每1ms计算增量`delta`，累计到速度窗口；每50ms：

```text
speed = 这50ms内有符号A相增量之和
速度窗口清零
```

本段行程使用相对起点的净计数，直到停车仍记录制动余量；速度窗口清零不会清除行程。`status.counts[0/1]`是屏幕用的非负计数，`signed_counts`保留方向。累计值超过2000不会被截成2000。停稳后冻结本段结果，下次start重新从0开始。

### 2. 柔和起步

```text
x = clamp(启动时间/500ms, 0, 1)
start = x*x*(3 - 2*x)
common = 50 * start
```

### 3. 外层轮程同步

```text
e = 左累计 - 右累计
rate = 滤波后的(左50ms计数 - 右50ms计数)
u_sync = 限幅并限制变化率[0.55 * deadband(e, 1) + 0.65 * rate]
左目标 = common - u_sync
右目标 = common + u_sync
```

左轮领先时降低左轮目标、提高右轮目标。速度差项用于阻尼，避免追赶过头。修正最大±12计数/50ms，每次变化最多2计数/50ms；起步阶段同步强度也渐增。任一轮到2000并制动后，取消同步，让未完成轮继续到目标。没有同步积分项。

### 4. 内层每轮速度位置式PD

```text
e_speed = target - measured
D = 0.5 * (e_speed - previous_error) + 0.5 * previous_D
requested_PWM = 方向前馈 + 0.004 * e_speed + 0.0015 * D
```

每次重新求绝对PWM，再做0～85%限幅和每50ms最大0.06占空比的变化限制。记录上一PWM用于限制输出变化率，不是增量式PI；没有误差积分。D的单位是每次采样的误差差值，没有再除以50ms，因此修改采样周期必须重新核对增益。

### 5. 终点与保护

每1ms分别检查两轮本段计数，到2000的轮立即制动，不等另一轮。两轮制动、速度窗口为0并持续100ms后进入STOPPED。停稳前继续统计惯性带来的额外计数，所以2000是制动阈值，不保证机械最终刚好2000。

驱动故障、编码器突跳、反向反馈、堵转、超速、动作超时和停车不稳定均保留。FAULT被锁存，普通start不会清除故障，处理原因后再初始化；不要在运行中反复初始化来清故障。默认入口dual_motor_start仍保持前进一次；新增dual_motor_start_direction(m, -1)支持后退，前后都按本段正向进度计数。暂不提供原地旋转、找中心或激光动作。

## 硬件输出契约

1. `enable=false`：断开H桥使能。
2. `enable=true && brake[w]=true`：该轮硬件短路制动。
3. 其余情况：按有符号`duty[w]`输出驱动力；本版正常前进不会命令反转。

TEST使用驱动/制动PWM，零驱动力时两桥输入均高；不同驱动芯片必须按其实际真值表适配。把相同占空比改成驱动/滑行PWM可能改变控制表现与前馈，不属于等价移植。两轮电机和编码器方向应统一成“前进为正”。

## 定时、并发与参数

- `dual_motor_tick_1ms()`必须固定1ms调用，不能由不稳定的主循环替代，不能漏调用后拿一次调用冒充多毫秒。
- 不要在控制中断里刷新整屏、阻塞串口或HAL_Delay。
- start/stop/get_status与tick若处于不同中断/线程，调用方需做短临界区。模板中提供board_enter_critical/exit_critical接口。
- 每个实例保存一份config副本；配置在init前修改，运行中不要直接改实例内部字段。
- `speed_ms`默认50；虽然可配置，修改窗口会改变所有“脉冲/窗口”参数含义，不能只改一个数并期望同样表现。
- 默认前馈和参数来自本车V4。函数可移植不代表换一台车无需标定。

## TEST中的兼容层与构建

`Core/Inc/motion_pd.h`和`Core/Src/motion_pd.c`现在是兼容适配与TEST动作编排层：保留原motion_init/start/stop/tick函数名，并把实例计数镜像到原全局变量g_motor_pulses，main.c因此可继续使用原屏幕和驱动代码。新项目直接使用dual_motor API，不需要复制这个全局变量或兼容层。

CubeIDE已加入portable/dual_motor源目录并排除examples；当前Debug/makefile和objects.list也已登记核心对象。更推荐明确的构建脚本：

```text
python tools/build_a50_portable.py
python tools/test_portable.py
```

封装版输出Debug/a50_portable/a50_portable_2000.hex。车上重新烧录的仍是原Debug/a50_sync_v4/a50_sync_v4_2000.hex，封装版未烧录。不要用旧build_a50_sync_v4.py构建新的兼容源码；该脚本属于旧版构建记录，不编译新增核心。

## 验证范围

默认参数下，将原V4和封装版分别接收同一组输入，经过50,000次1ms调用，逐字段序列化的控制轨迹完全一致（含PWM浮点值、目标、状态、计数、制动和故障时序）。测试覆盖非对称负载、扰动、停止重启、计数器回绕、驱动故障和编码器突跳；另测独立实例、自定义目标、状态读取、无效配置拒绝及原V4回归。

这是软件行为等价性验证，不替代移植到新板后的A相边沿、中断延时、电机真值表和轨迹实测。

## 原TEST新增前后往返任务 A50_ROUNDTRIP_V1

新库接口：`dual_motor_start_direction(&car, +1)`前进，`dual_motor_start_direction(&car, -1)`后退。只有非运行/非故障状态才能启动，成功返回true。后退时原始速度为负，本段累计数按后退方向从0增加，同步速度差也归一化到行进方向。前后都数A相双沿，不切换计数倍率。

配置新增`soft_stop_ms`，默认0保持V4立即制动；TEST配置为300。达到每轮本段2000时才开始约300ms的smoothstep占空比递减，再保持短路制动。没有在1900附近降低速度目标，所以不会恢复原来的低速停滞策略。制动过程不运行速度追赶，采集和计数继续，最终计数保留真实余量。

TEST的`Core/Src/motion_pd.c`负责：FORWARD -> 双轮STOPPED -> WAIT_2S -> 清零本段并调用后退接口 -> REVERSE -> DONE。2秒从两轮实际停稳后开始算。WAIT阶段仍可按PD0中止，故障或中止后不会自动倒退。基础库不包含“等2秒后后退”的应用策略。

新构建：`python tools/build_a50_roundtrip.py`，输出`Debug/a50_roundtrip/a50_roundtrip_2000.hex`。新测试`tests/test_roundtrip.c`验证方向、柔和制动、等待时序、计数保留与清零、取消和故障。`tools/test_portable.py`通过测试兼容层继续验证库默认参数和原V4逐步等价，不把新的往返应用当作原V4前进程序比较。

本次烧录以ST-LINK实际结果为准。逐渐刹停会在2000触发点以后继续产生边沿；前后各2000只保证触发阈值相同，不能据此承诺物理上恰好回到原点。

## TEST低速平缓同步配置 A40_ROUNDTRIP_V2

应用初始化改为cruise_target=40（A相双沿/50ms），同步行程增益0.45、速度差阻尼0.65、修正限幅±8、修正每次最多变化0.5、速度差滤波系数0.25、PWM每次最大变化0.04。参数位于Core/Src/motion_pd.c的motion_init；库默认值保留原V4，未修改。前后各2000、达到阈值后300ms渐停、停稳等2秒再清零后退、结束后不循环均保留。

构建python tools/build_a40_roundtrip.py，输出Debug/a40_roundtrip/a40_roundtrip_2000.hex；采集tools/capture_a40_roundtrip.py。tests/test_a40_profile.c检查低速负载模型、同步修正变化率和完整往返流程。实车稳定性以日志和轨迹观察为准，不能把目标40等同于实际40。

## 单轮转动接口与TEST的90度初测 A34_PIVOT_R90_V1

新增`dual_motor_start_wheels(&car, +1, 0, right_count_goal)`：左目标为0即全程制动，右轮独立位置式PD。左右目标都为0、负目标、无效方向、忙或故障时拒绝启动。原双轮入口继续给两轮相同目标，默认控制行为保留。

TEST当前单轮配置：右轮目标34个A相双沿/50ms，左轮0，保留500ms起步；sync_enabled=false、soft_stop_ms=0。右轮达到目标立即制动，最终真实余量照常保留；只执行一次，不自动往返。

这里90度指车体绕左轮转90度；右轮前进对应车体左转。几何初值N=angle_deg*132*420/(180*43)，90度约645个A相双沿。420/轮来自旧A/B四倍频840的一半，尚未单独实测确认。轮胎打滑、轮距与制动余量会影响转角，所以645仅是首轮标定初值。实际车体角度theta测出后，候选新值约Nnew=Nold*90/theta，需多次复测，不能将几何角度当作实测角度。

修改目标角度用Core/Inc/motion_pd.h的MOTION_PIVOT_ANGLE_DEG，换算函数motion_pivot_counts位于应用层；原有直行往返版本已备份。构建tools/build_a34_pivot.py，固件Debug/a34_pivot/a34_pivot_r90.hex；采集tools/capture_a34_pivot.py。已运行单轮输出、阈值急停、真实余量、负载模型、保护和V4默认等价性检查，物理角度待测。

## 已校准单轮的累计角度序列 A34_PIVOT_SEQ_V1

应用层采用初始右轮累计计数作为绝对基准，目标角度30/120/60/0，对应标定净计数204/814/407/0。每次用“下一绝对目标减去当前真实净计数”调用dual_motor_start_wheels(m, sign, 0, abs(difference))，因此分段阈值会计入前一段刹车后的余量。左轮全程制动，右轮34双沿/50ms、500ms起步、到点急停。前三段实际停稳后各等5000ms，第四段停止后DONE；不循环、不追加终点回调。

30/120/60/0是相对按PD0启动朝向的累计角度，不是独立再转30/120/60/0。只确认过前向90度标定精确，其余角度和反向机械效果待实测。归零是编码器基准归零，不是外部航向传感器闭环。

屏幕显示速度、PWM和相对启动基准的左右净计数，底部显示目标角度及等待秒数。遥测netR对应净计数，countR对应本段行程，absGoal为绝对目标，segmentGoal为本段阈值；两者含义不同。等待期间PD0仍可中止后续动作。

构建tools/build_a34_pivot_sequence.py，固件Debug/a34_pivot_sequence/a34_pivot_sequence.hex，测试tests/test_pivot_sequence.c，采集tools/capture_a34_pivot_sequence.py。

## 左轮第二测试 A34_LEFT_SEQ_V1

左轮90度611计数初值经用户确认精度高后，保持标定和全部PD参数，切换为累计30/120/60/0度；当前MOTION_PIVOT_ACTIVE_WHEEL=0，右轮全程制动。前三段停稳后各等5秒，最后回0停止，不额外修正或循环。速度目标幅值34双沿/50ms，急停，按实际左轮净计数决定下一段剩余行程。

已认可的右轮序列和左轮90度程序均在backups保留。构建tools/build_a34_left_sequence.py，固件Debug/a34_left_sequence/a34_left_sequence.hex，验证tests/test_left_pivot_sequence.c；已烧录校验成功，按用户偏好未开启监听。完整左轮角度序列实测待用户反馈。
