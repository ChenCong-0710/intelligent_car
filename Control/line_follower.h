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
 *   7. 3 分钟限时四面障碍物瞄准与 10 s 激光发射状态机 (LF_PROBING，方向由 probe_side_left 选)
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
 * @brief M1 面拟合有效性阈值（M2 起由 probe_fit_rms_mm 参数覆盖）
 * @note  纸盒靶面本身有 1~3 mm 的鼓包，200 mm 处 ±30° 扫掠下 rms 常到 6~8 mm；
 *        取 10 mm 既能放过真实纸盒，又能挡住「盒子+墙混在一起」这类 rms 20 mm 以上的垃圾拟合。
 */
#define LF_PROBE_FIT_RMS_MM 15.0f

/**
 * @brief 两段式 AIM 的第二段（细扫）参数
 * @note  第一段 ±sweep_us 粗扫定位平面，第二段以上一段求得的法线为中心、±120us(±12°) 细扫，
 *        步长 20us(2°)、约 10 个点。50 mm 窄面在第一段只有 3 个点，第二段能拿到 8~10 个点。
 */
#define LF_PROBE_AIM2_SPAN_US 120U
#define LF_PROBE_AIM2_STEP_US 20U

/**
 * @brief 相邻采样点距离跳变阈值 (mm)：超过它认为「换了一个物体」，结束当前连续段
 * @note  只靠「回波为 0」切段是不够的：旁边有桌子/墙时每个角度都有回波，
 *        整条扫掠会被当成一个面拟合，rms 直接飙到几十毫米。
 */
#define LF_PROBE_RUN_JUMP_MM 40U

/** @brief 一次扫掠里最多记录多少个「同一物体」的连续段（用于挑选最近的那个面） */
#define LF_PROBE_MAX_RUNS 8U

/** @brief M1 内点带半宽 (mm)：|d_meas - rho/cos(theta-phi)| 小于它才算内点 */
#define LF_PROBE_INLIER_BAND_MM 15.0f

/**
 * @brief M1 判定拟合有效所需的最少内点数
 * @note  50 mm 窄面在 150 mm 处只占 ±9.5°，加上波束外扩也只到 ±17°，
 *        在 ±60° 扫掠里通常只有 3~5 个采样点。取 3 是「能拟合」的下限；
 *        窄面的法线精度会降到 3° 左右，而侧视横扫只需用它决定转多少度，3° 足够。
 */
#define LF_PROBE_MIN_INLIERS 3U

/* ---------------------------------------------------------------------------
 * M2 侧视横扫（SIDE_LOCK / SIDE_SCAN / SIDE_MOVE + 复测）常数
 * 全部是宏，不占 Flash 参数区（LfProbeConfig 只放需要现场用蓝牙微调的量）
 * ------------------------------------------------------------------------- */

/** @brief 面内样本滑动窗口长度（8 点最小二乘 d = h0 + k·s，k 吸收车不平行造成的线性漂移） */
#define LF_SIDE_RING 8U

/** @brief 重建面内基线所需最少样本数（换向后重新建立 d(s) 模型；不足该数不做外点判定） */
#define LF_SIDE_RING_MIN 6U

/**
 * @brief 判定一个新边界前，自上次边界以来至少要有这么多帧「面内」样本
 * @note  台架实测：盒子边缘处回波断续（擦边），换向后只重建了 4 帧基线就连续丢帧，
 *        曾被误判成第二个边界，把宽度算成 -7 mm。要求 ~10 帧（≈36 mm @50 mm/s）面内证据
 *        后，擦边区的丢帧不会立即触发边界确认；真实边界的面内段远长于 10 帧。
 */
#define LF_SIDE_EDGE_MIN_INFACE 10U

/** @brief 面内直线拟合斜率钳位（|k| 上限，≈14°）：4~6 个噪声样本可能拟合出荒唐斜率，钳住防止预测发散 */
#define LF_SIDE_K_LIMIT 0.25f

/**
 * @brief 边界确认帧数（距离域“跳变”型）
 * @note  有回波但偏离模型（比如面外 60 mm 的墙/邻面）是强证据，连续 3 帧就确认。
 *        单个丢帧也是“偏离”，所以不能只用 2 帧——15% 丢帧率下 2 帧会有 2% 的假边界概率。
 */
#define LF_SIDE_EDGE_JUMP_FRAMES 3U

/**
 * @brief 边界确认帧数（整段无回波型）
 * @note  丢帧与“真的出界了”在单帧上不可区分，只能靠持续时间。按 15% 独立丢帧率估算：
 *        单窗口 6 连丢 = 0.15^6 ≈ 1.1e-5，一面扫掠约 90 个窗口 → 整面出一次假边界的概率约 0.1%；
 *        取 5 帧时整面累计约 0.6%，台架偶发“宽度不对”会被误当成算法 bug。
 *        边界位置仍取「最后一个面内样本 + 第一个外点样本」的中点，确认帧数不影响定位精度，
 *        只影响“确认前多走几帧”，所以复测/REV 的行程上限必须把这段确认距离算进去。
 */
#define LF_SIDE_EDGE_ZERO_FRAMES 10U

/** @brief 侧视锁定：舵机 90° 到位等待时间 (ms)。实测 1500→2400 的 90° 行程 ≤0.4 s（含 60 ms 测距节拍） */
#define LF_SIDE_LOCK_WAIT_MS 400U

/** @brief 侧视锁定：连测帧数（每 60 ms 一帧，取中位数） */
#define LF_SIDE_LOCK_FRAMES 5U

/** @brief 侧视锁定：|h - (rho + pan_x)| 自检容差 (mm)。rho 比卷尺大 15~20 mm，故放到 40 mm */
/* 2026-11-23 实车：TURN_PAR 后 h 比 ρ+pan_x 系统偏大 28~58 mm（11:09 实测 218 vs 160.1）。
 * 转向是绕轮轴中点做的，但实车会往离开箱面的方向平移 30~60 mm（轮子打滑/万向轮拖拽），
 * 40 mm 的刚性自检把正确的锁面判成失败。放到 70 mm：仍能挡住 11:09 那次 h=93 的挡块（Δ85）。
 * 2026-11-23 复盘：这一条不再放宽。任务书工况下 pred 与实锁的系统差仍是 28~58 mm（<70），
 * 而它同时是「转错角/认错面」的唯一报警（单测 probeside ⑩b 用 Δ=80 mm 守住）——放宽会削弱安全，
 * 真正的近距问题用 LF_LEG_H_MIN_MM 与测斜有效性门限解决。 */
#define LF_SIDE_LOCK_TOL_MM 70.0f

/** @brief 侧视站立距离 h 的物理有效区间 (mm)（<80 近盲区/自回波，>450 回波太弱）
 * @note  2026-11-23 台架：稳定平面在 366 mm（回波恒定 ±0，三帧完全一致），350 的旧上限把
 *        一个回波很强的面直接判成越界。放到 450：仍能挡住几百 mm 的背景墙。
 *        若现场发现 350~450 段误锁背景，把这行改回 350 即可。 */
#define LF_SIDE_H_MIN_MM 80.0f
#define LF_SIDE_H_MAX_MM 450.0f

/** @brief 侧视锁定超时 (ms)：400 ms 舵机 + 5 帧 × 60 ms = 700 ms，留 2.5 s 余量 */
#define LF_SIDE_LOCK_TIMEOUT_MS 9000U  /* 含预扫测斜：400 等待 + 60 mm 平移（80 mm/s≈1 s）+ 停稳 + h2/h3 各 5 帧 + 最多一次修正转 */

/** @brief 侧扫/开位末段降速：|err| 小于它时降到低速，避免过冲 */
#define LF_SIDE_SLOW_DIST_MM 20.0f
#define LF_SIDE_SLOW_SPEED_MM_S 25.0f

/** @brief 侧扫到位判据 (mm) 与到位后的主动制动稳定时间 (ms) */
#define LF_SIDE_ARRIVE_MM 2.0f
#define LF_SIDE_SETTLE_MS 150U

/**
 * @brief SIDE_MOVE 平移测斜门限：沿面走 ≥40 mm 时，侧距单调（非突变）漂移 ≥12 mm 才认作残余偏航
 * @note  2026-11-23 现场：TURN_PAR 后车头若仍差 10°，沿面走 100 mm 侧距会单调变化 ~17 mm，
 *        这种「慢慢增大/减小」的漂移就是转向角不对的直接证据；突变（>40 mm/帧）判为换面/边缘，不计入。
 */
/* 2026-11-23 M4e：以下 4 个门限**已停用**——SIDE_MOVE 到位后不再按漂移转身。
 * 那个转身发生在 SIDE_SCAN 之后、LASER 之前，probe_side_verify=0 时转完直接发射，光斑偏 26 mm；
 * 定义保留只为不动日志字段与历史，控制路径不再引用（见 line_follower.c 的 LF_SIDE_MOVE_DRIVE）。 */
#define LF_SIDE_MOVE_TILT_MIN_DS_MM 100.0f
#define LF_SIDE_MOVE_TILT_MIN_MM 12.0f
#define LF_SIDE_MOVE_TILT_DEADBAND_DEG 2.0f
#define LF_SIDE_MOVE_TILT_MAX_DEG 12.0f

/** @brief SIDE_MOVE 单次驱动超时 (ms)：最长的“扫失败后退回原点”约 280 mm，50 mm/s 需要 5.6 s */
#define LF_SIDE_MOVE_TIMEOUT_MS 10000U

/**
 * @brief 复测扫描的额外安全余量 (mm)：行程 = w/2 + h·tanγ + 确认距离 + 该余量
 * @note  h·tanγ（波束足印）与「连续 6 帧确认距离」都随 h/速度变化，**不能写死一个固定值**：
 *        早期版本写死 w/2+50，在 h>265 mm 时会在边界确认完成前就撞上行程上限，复测必失败。
 */
#define LF_SIDE_CONFIRM_EXTRA_MM 10.0f

/** @brief 复测一致性判据 (mm)：|s_m2 - s_m| 小于它才取平均放行 */
#define LF_SIDE_VERIFY_TOL_MM 8.0f

/** @brief 台架手推模式超时 (ms)：人工推 300 mm 大约 5~10 s */
#define LF_SIDE_PUSH_TIMEOUT_MS 25000U

/** @brief 超声触发周期 (ms)：侧扫按这个节拍取一帧（与 board.c TIM17 单脉冲节拍一致） */
#define LF_SIDE_PING_MS 60U

/* ---------------------------------------------------------------------------
 * M3 转平行（TURN_PAR）与绕行（LEG）常数
 * 按《M3计划_评审与修订.md》§4.5 采用宏而不是新 Flash 参数：M3 不升 Flash 版本，
 * M4 需要在线调 box_ 与 leg_margin 时再一次性加参数并升 8。
 * ------------------------------------------------------------------------- */

/** @brief TURN_PAR 超时 (ms)：90° @90 mm/s 实测 1.1~1.4 s，3 s 足够；超时=失败+停机 */
#define LF_TURN_PAR_TIMEOUT_MS 6000U  /* 转角降到 35 mm/s 后 90~105° 要 2.5~4 s，3 s 会误判超时 */

/** @brief 原地转到位后的主动制动稳定时间 (ms)：停稳后再锁存 turn_ach_deg，避免把滑行角当成到位角 */
#define LF_TURN_SETTLE_MS 150U

/** @brief 原地转的到位容差 (deg)：超过它判 turn_ok=0（M3 失败即停，不回 AIM） */
#define LF_TURN_TOL_DEG 8.0f

/** @brief AIM 的 |φ| 超过它认为结果不可信，拒绝 TURN_PAR（避免为了对齐法线把车转过 125°） */
#define LF_TURN_PAR_MAX_PHI_DEG 35.0f

/** @brief 箱体名义尺寸 (mm)：任务书 150×50；LEG 用标称值而不是实测 w（实测 w 有 ±8 mm 误差且 80~120 会误分类） */
#define LF_BOX_LONG_MM 150.0f
#define LF_BOX_SHORT_MM 50.0f

/** @brief LEG 直行速度 (mm/s) 与第一段越角余量 (mm) */
#define LF_LEG_SPEED_MM_S 100.0f
#define LF_LEG_MARGIN_MM 20.0f

/** @brief LEG 前站立距离门禁 (mm)：低于它转 90° 时车体扫掠可能擦箱（h=80 时轮轴到转角仅 106 mm）
 * @note  2026-11-23 按任务书真实工况放宽 120→100：任务书要求「<15cm 自动停车」，实际停车
 *        距离 10~15cm，TURN_PAR 后 h 常常只有 100~150；120 会把正常的 4 面循环全部挡死。
 *        方案 §5.2 的扫掠余量：h=120→+49mm、h=100→+21mm、h=80→−7mm，所以 100 仍留 21mm，
 *        低于 100 继续拒绝（此时宁可少打一面也不擦箱）。 */
#define LF_LEG_H_MIN_MM 100.0f

/** @brief LEG 总超时与单段直行超时 (ms) */
#define LF_LEG_TIMEOUT_MS 15000U
#define LF_LEG_DRIVE_TIMEOUT_MS 8000U

/**
 * @brief LEG 期间侧距连续采样：节拍 (ms) 与 DRIVE2 认面窗口半宽 (mm)
 * @note  宽面转完后舵机一直压在 side_us，超声本来就在测；这里只是把读数按 50 ms 留轨迹。
 *        DRIVE2 里落在 h_expected±75 mm 的帧即「窄面从旁边过」的直接证据。
 */
#define LF_LEG_PROX_PERIOD_MS 50U
#define LF_LEG_PROX_FACE_BAND_MM 75.0f

/**
 * @brief DRIVE2 收尾补救：面已滑出侧视时倒回的距离/速度
 * @note  2026-11-23 三次 faces=2 都在同一处失败：DRIVE2 停机时侧距=368（背景），
 *        但行进中 face_n≥5 明显看到过面——死算的 L2 越过窄面后沿约 30~50 mm。
 */
#define LF_LEG_BACKUP_MM 40.0f
#define LF_LEG_BACKUP_SPEED_MM_S 50.0f

/**
 * @brief LEG 前航向修正（ALIGN）已于 2026-11-22 21:36 **永久删除**
 * @note  台架结论：side_k=-0.25（≈14°）是侧扫弧段/电机死区造成的**测量偏置**，不是静态歪斜；
 *        车实际与面平行时也会稳定测出 -0.25。按它修 14° 反而把车头顶向箱面
 *        （h 143→约 100），转角时前角顶住箱角，外观就是「该左转却右拐」。
 *        去掉 ALIGN 后 DRIVE1 保持 h≈143，转向不再被箱角卡住；
 *        PROBE_SIDE_MID 的 k 只留作诊断，不再进入控制。
 */

/** @brief M3 转角降速 (mm/s)：LEG 转角 60、TURN_PAR 60；低 ρ 时减小前角擦箱与过冲 */
#define LF_LEG_TURN_SPEED_MM_S 50.0f
#define LF_TURN_PAR_SPEED_MM_S 50.0f

/**
 * @brief 面宽分类门限 (mm)：w < 它判窄面(50)，否则宽面(150)
 * @note  2026-11-22 台架：窄面真值 50 在 h≈205 时测得 ~109.5（波束足印偏置），
 *        宽面真值 150 在 h≈153 时测得 146~172。按实测偏置取 130；
 *        直接用真值 100 会把窄面 109.5 误判成宽面，第二次 LEG 用错 d_B 后失败。
 */
/* 2026-11-23：130 太宽——150 宽面在 h≈200 处只测到 118.5（11:12 实车），被判成窄面，
 * LEG 于是按 w_nom=50/d_b=150 算 L2=h+75-30=251，比正确值（h+25-30=197）多走 54 mm，
 * 云台越过 50 mm 窄面，面 2 的 SIDE_LOCK 读到 368（背景）直接失败。
 * 11-22 全部侧扫记录：宽面 109.5~187.4，窄面 43.6~79.7，分界取 100 两边都留有余量。 */
#define LF_SIDE_KIND_SPLIT_MM 100.0f

/**
 * @brief SIDE_LOCK 预扫测斜：沿面平移两点法（h1 → 前进基线 → h2 → 退回 → 转正 → h3）
 * @note  2026-11-22 的「原地 +5° 两点法」已废弃：5° 只有 h·tan5°≈14 mm 的横向基线，
 *        静态 h 噪声 3 mm 就被解成 14.7°，把本来平行的车转歪（11-22 15:56 日志）。
 *        side push（电机断电）跳过。2026-11-23 M4e 起**每面都做**，基线按实测面宽选：
 *        宽面(150) 60 mm、窄面(50) 30 mm（50 mm 的面放不下 60 mm 平移，单侧波束足印就有 23.7 mm）。
 */
#define LF_SIDE_TILT_PROBE_DEG 5.0f   /* 旧旋转法遗留常数，现行平移法不用 */
#define LF_SIDE_TILT_PROBE_SPEED_MM_S 80.0f
/* 2026-11-23 改为**沿面平移两点法**：宽面 60 mm×tan(10°)=10.6 mm 信号，是 h 中位数噪声（±1 mm）的 10 倍；
 * 解出的残余偏航 ε̂=atan2(h2−h1, 基线)，|ε̂|>死区 7°(≈5σ) 才原地转 corr=clamp(ε̂,±15°)，**每面最多转 1 次**。
 * 关键前提：h2 必须是「平移之后重新采的 5 帧」。2026-11-23 12:13 之前 side_ring_n 没清零，
 * h2 直接复用了 h1 的样本环 → h1==h2 恒成立 → corr 永远是 0（回归用例 probesidetilt）。
 * 背景：TURN_PAR 目标角=90°−φ_AIM，φ 抖 ±8° 会留 4~8° 残余偏航，直接让侧扫 w 在 118~159 之间跳。 */
#define LF_SIDE_TILT_PROBE_ENABLE 1
/** @brief 测斜平移距离 (mm)：沿面直行，比较前后两点 h（宽面；窄面用 LF_SIDE_TILT_SHIFT_NARROW_MM） */
#define LF_SIDE_TILT_SHIFT_MM 60.0f
/** @brief 窄面测斜基线 (mm)：50 mm 的面放不下 60 mm 平移（单侧足印 23.7 mm），压到 30 mm；
 *         能测就测——平移把波束带出面时 h2 重采不足 → warn=8 → 本面跳过测斜，不影响正确性 */
#define LF_SIDE_TILT_SHIFT_NARROW_MM 30.0f
/** @brief 测斜死区 (°)：60 mm 基线 σ_ε≈1.3°，7°≈5σ；平行时一次都不该转（台架 A 组判据） */
#define LF_SIDE_TILT_MIN_DEG 7.0f
/** @brief 单次修正钳位 (°)：每面最多转 1 次，残余 >15° 说明 AIM 的 φ 已经错了，是异常 */
#define LF_SIDE_TILT_MAX_DEG 15.0f
/** @brief 测斜有效性硬门限 (°)：|ε̂| 超过它判「本次测斜无效」（corr=0、warn=9、不转）
 * @note  任务书真实工况下（<15cm 停车、箱子斜 70~110°、50 mm 窄面），SIDE_LOCK 原点可能贴着
 *        面的边缘，60/30 mm 平移会把波束带到相邻面/背景上——此时 h2 仍是 [80,450] 内的合法值，
 *        却给出 30~50° 的假 ε̂。2026-11-23 实测 45.9°/31.7° 两例都被钳到 15° 硬转，把 SIDE_SCAN
 *        和 LEG 落点带歪（下一面 h=257 vs 预期 210，锁面自检失败）。>25° 已超出"残余偏航"模型
 *        （AIM 的 φ 错也不会有 25° 残余），按无效处理；15~25° 仍按钳位转一次（单测 probesidetiltclamp）。
 *        不转是安全的：性质①保证歪着打光斑仍然落在面中心。 */
#define LF_SIDE_TILT_ABORT_DEG 25.0f
/** @brief h2 重采超时 (ms)：停稳后最多等这么久凑 5 帧；凑不满 3 帧说明平移把光束带出了面
 *         （50 mm 窄面或强丢回波）→ 放弃测斜（corr=0, warn=8）继续，不许把整轮卡死在 SIDE_LOCK */
#define LF_SIDE_TILT_H2_TIMEOUT_MS 800U

/** @brief TURN_PAR 低 ρ 倒车：AIM ρ 小于它先倒 20 mm，给原地转的前角留间隙（现场可能停在 13~14 cm） */
/* 2026-11-23 用户现场判定「先退 20 mm」没多大用处：置 0 关闭（rho<0 永不成立 → turn_backup 恒为 1）。
 * 保留代码与 mm 参数，需要时把阈值改回 150/200 即可恢复。 */
#define LF_TURN_PAR_BACKUP_RHO_MM 0.0f
#define LF_TURN_PAR_BACKUP_MM 20.0f

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
    LF_PROBING,       /**< 9: 提高部分（2）四面障碍物探测与 10s 激光打靶状态（方向可选） */
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
    LF_PROBE_COMPLETE_BEEPS,/**< 9: 全部平面完成后，保持停止并连续发出 3 组“两短一长”蜂鸣器鸣叫 */
    /* ---- 以下为 v2 侧视法新增，数值 10~18 由《接口契约_M1.md》§3 冻结 ---- */
    LF_PROBE_SQUARE,        /**< 10: 模式 A（角度扫掠）占位，暂不实现 */
    LF_PROBE_EDGE,          /**< 11: 模式 A 占位，暂不实现 */
    LF_PROBE_CENTER,        /**< 12: 模式 A 占位，暂不实现 */
    LF_PROBE_ALIGN,         /**< 13: 模式 A 占位，暂不实现 */
    LF_PROBE_TURN_PAR,      /**< 14: 原地转到与面平行（M3 实现；M2 由人工/台架摆位替代） */
    LF_PROBE_SIDE_LOCK,     /**< 15: 舵机掰到侧视 ±90°，连测 5 帧取中位数得到站立距离 h 并自检 */
    LF_PROBE_SIDE_SCAN,     /**< 16: 沿面平移扫掠，靠回波跳变找左右两个边界 */
    LF_PROBE_SIDE_MOVE,     /**< 17: 沿面开到中点并停车（内部含 SETTLE 与 SIDE_VERIFY 复测子状态） */
    LF_PROBE_LEG            /**< 18: 沿切线绕到下一面的三段式绕行（M3 实现） */
} LfProbePhase;

/** @brief 侧视横扫子状态（LfController::side_stage） */
typedef enum {
    LF_SIDE_FWD = 0,   /**< 0: 沿 +u∥ 正扫（先找 +s 侧边界） */
    LF_SIDE_REV = 1,   /**< 1: 折返沿 -u∥ 反扫（找 -s 侧边界） */
    LF_SIDE_DONE = 2   /**< 2: 两边界齐 */
} LfSideStage;

/** @brief SIDE_MOVE 子状态（LfController::side_move_stage） */
typedef enum {
    LF_SIDE_MOVE_DRIVE = 0,   /**< 0: 沿面直行到 side_target */
    LF_SIDE_MOVE_SETTLE = 1,  /**< 1: 主动制动 150 ms 等停稳 */
    LF_SIDE_MOVE_VERIFY = 2,  /**< 2: 复测扫描（可选，probe_side_verify） */
    LF_SIDE_MOVE_DONE = 3     /**< 3: 就位，进入 LASER */
} LfSideMoveStage;

/** @brief 原地转（TURN_PAR / LEG 转角 / LEG 前航向修正）子状态（LfController::turn_stage） */
typedef enum {
    LF_TURN_IDLE = 0,    /**< 0: 未开始/需要重新锁存基准航向 */
    LF_TURN_DRIVE = 1,   /**< 1: 单向逼近目标角 */
    LF_TURN_SETTLE = 2,  /**< 2: 到位后主动制动，等停稳再测 ach */
    LF_TURN_DONE = 3     /**< 3: 已完成（turn_ok 表示是否在容差内） */
} LfTurnStage;

/** @brief LEG 三段（LfController::leg_stage） */
typedef enum {
    LF_LEG_DRIVE1 = 0,  /**< 0: 沿面切线越过拐角，L1 = w_nom/2 + pan_x + h + margin */
    LF_LEG_TURN = 1,    /**< 1: 原地转 ±90°（左视左转，右视右转；BOARD_SWAP_LR=0 时 + = 实车左转） */
    LF_LEG_DRIVE2 = 2,  /**< 2: 沿新切线开到下一面中心，L2 = h + d_B/2 - pan_x */
    LF_LEG_DONE = 3,    /**< 3: 完成 */
    LF_LEG_BACKUP = 4   /**< 4: DRIVE2 末尾面已滑出侧视（h>H_MAX/丢回波）时先倒回 LF_LEG_BACKUP_MM */
} LfLegStage;

/**
 * @brief LEG 遥测事件位掩码（LfController::leg_event，供 main.c 打印 PROBE_LEG 后清零）
 * @note  与 side_event 同一套机制：main.c 只在相位切换时打 PROBE_TURN，LEG 内部三段不换相位，
 *        所以必须用事件位让控制器把「哪一段刚结束」传出去。
 */
typedef enum {
    LF_LEG_EVENT_NONE = 0x00,    /**< 无事件 */
    LF_LEG_EVENT_DRIVE1 = 0x01,  /**< DRIVE1 结束 */
    LF_LEG_EVENT_TURN = 0x02,    /**< 转角结束 */
    LF_LEG_EVENT_DRIVE2 = 0x04,  /**< DRIVE2 结束（LEG 完成） */
    LF_LEG_EVENT_FAIL = 0x08,    /**< LEG 失败（超时/转角超差/h 门禁） */
    LF_LEG_EVENT_PROX = 0x10,    /**< 侧距连续采样一帧（50 ms 取、100 ms 打一行） */
    LF_LEG_EVENT_PROX2 = 0x20,   /**< DRIVE2 结束：窄面侧距统计（face_n/face_h） */
    LF_LEG_EVENT_BACKUP = 0x40   /**< 收尾倒回结束（面滑出侧视后的补救段） */
} LfLegEvent;

/**
 * @brief 侧扫遥测事件位掩码（LfController::side_event，供 main.c 打印 PROBE_SIDE_* 日志后清零）
 * @note  用位掩码而不是单值：REV 找到第二个边界的那一拍既产生「边界」事件、又立即由调用方
 *        判定「中点确定」，单值会把边界事件覆盖掉（PROBE_SIDE_EDGE,i=1 永远打不出来）。
 */
typedef enum {
    LF_SIDE_EVENT_NONE = 0x00,  /**< 无事件 */
    LF_SIDE_EVENT_EDGE = 0x01,  /**< 找到一个边界 */
    LF_SIDE_EVENT_MID = 0x02,   /**< 中点/宽度确定（含复测后更新） */
    LF_SIDE_EVENT_FAIL = 0x04,  /**< 本次扫描 / SIDE_LOCK 失败 */
    LF_SIDE_EVENT_YAW = 0x08    /**< 停用（M4e）：SIDE_MOVE 不再按平移漂移转身，本事件不再置位 */
} LfSideEvent;

/**
 * @brief 四面顺时针障碍物探测与激光打靶 (LF_PROBING) 可调参数结构体
 * @details 所有参数均可通过蓝牙串口 `set probe_*` 实时微调，无需反复重烧固件。
 */
typedef struct {
    bool clockwise;           /**< true = 顺时针；false = 逆时针（任务书未限定，按接近侧选） */
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

    /* ---- M2 侧视横扫参数（Flash v7 起保存；均可用蓝牙 set probe_* 在线微调） ---- */
    uint8_t scan_mode;        /**< 0 = 角度法过渡流程（保留 M1 行为）；1 = 侧视横扫（默认，主方案） */
    bool side_left;           /**< true = 箱体在左、逆时针巡边（本项目左侧接近；M3 的 TURN_PAR/LEG 用） */
    uint16_t side_us;         /**< 侧视舵机脉宽 (us，默认 2400 = 右掰 90°，即箱体在车左侧) */
    float fit_rms_mm;         /**< 面拟合残差上限 (mm，默认 15；纸盒起伏实测 rms 6~13) */
    float side_speed_mm_s;    /**< 侧扫/开位速度 (mm/s，默认 50；60 ms 走 3 mm 即一个样本) */
    float side_reach_mm;      /**< 单方向最大扫描行程 (mm，默认 260；从扫描起点算，FWD/REV 各一份) */
    float side_band_mm;       /**< 面内判据带宽 (mm，默认 8；实测静止噪声 ±2 mm) */
    float side_jump_mm;       /**< 跳变判据阈值 (mm，默认 25；必须大于 side_band_mm) */
    float beam_half_deg;      /**< 波束半角 γ (deg，默认 7.5，不标定：只影响宽度，不影响中点) */
    float pan_x_mm;           /**< 云台轴相对轮轴的前移量 (mm，默认 30；只用于 SIDE_LOCK 自检) */
    bool side_verify;         /**< true = 发射前以 s_m 为中心复测一次（默认 1；计时紧张时可关） */
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
    uint16_t servo_us;  /**< 云台舵机目标脉宽 (控制 PA15 TIM2_CH1，单位 us，600 ~ 2400 = 0° ~ 180°) */
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
 * @brief 一次 AIM 扫掠得到的障碍物平面模型
 * @note  M1 只写前 5 个字段（valid/phi/rho/rms/hits）；侧视横扫（M2）再写边界与站立距离。
 */
typedef struct {
    bool    valid;    /**< 本次拟合是否可信 */
    float   phi;      /**< 面法线方向在车体系的角度 (rad，0 = 车头正前) */
    float   rho;      /**< 云台轴到面的垂直距离 (mm) */
    float   rms;      /**< 拟合残差 (mm 量纲) */
    uint8_t hits;     /**< 落在内点带内的采样点数 */
    float   s_l;      /**< 左边界在扫描轴上的位置 (mm，M2) */
    float   s_r;      /**< 右边界在扫描轴上的位置 (mm，M2) */
    float   s_m;      /**< 中点位置 (mm，M2) */
    float   w;        /**< 面宽 (mm，γ 补偿后，M2) */
    float   h;        /**< 侧视站立距离 (mm，M2) */
    uint8_t kind;     /**< 0 = 窄面(≈50) 1 = 宽面(≈150)，M2 */
    uint8_t id;       /**< 第几面 0~3，M2 */
} LfFace;

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
    float phase_start_x_mm;  /**< 当前打靶子阶段起始时的世界系 x (mm)，侧扫有符号位移用 */
    float phase_start_y_mm;  /**< 当前打靶子阶段起始时的世界系 y (mm) */
    LfFace face;             /**< 当前障碍物平面模型（M1 起由 probe_fit_normal 填充） */
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
    uint16_t aim_span_us;                         /**< 本轮 AIM 扫掠半幅 (us)：第一段 = sweep_us，第二段 = ±120 */
    uint16_t aim_center_us;                       /**< 本轮 AIM 扫掠中心脉宽 (us)：第一段 = servo_center，第二段对准粗测法线 */
    uint8_t aim_pass;                             /**< 0 = 粗扫，1 = 细扫 */
    LfFace aim_first;                             /**< 粗扫结果备份（细扫失败时回退） */

    /* --- M2 侧视横扫：SIDE_LOCK / SIDE_SCAN / SIDE_MOVE + 复测 --- */
    uint8_t side_stage;        /**< 侧扫子状态 LF_SIDE_FWD / REV / DONE */
    uint8_t side_edges;        /**< 本次扫描已找到的边界数 0~2（0 号在 FWD 先发现，1 号在 REV 发现） */
    uint8_t side_bad;          /**< 当前连续外点帧数 */
    uint8_t side_out_zero;     /**< 当前外点段里"真无回波(d==0)"的帧数（跳变型与无回波型边界分开确认） */
    uint8_t side_ring_n;       /**< 面内样本滑动环已填充帧数 0~8（SIDE_LOCK 期间当作 5 帧缓冲用） */
    uint8_t side_ring_i;       /**< 面内样本滑动环写入下标 */
    uint8_t side_retry;        /**< 侧扫失败后已经“退回原点重扫”的次数 0~1 */
    uint8_t side_warn;         /**< 侧扫告警：0=初测正常 / 1=宽度超范围 / 2=复测一致取平均 / 3=复测不一致 / 4=开位超时（仅供日志） */
    uint8_t side_lock_warn;    /**< SIDE_LOCK 阶段告警：0=正常 / 5=h3 用兜底 / 6=平移卡住跳过测斜 / 7=h3 出界用 h2 /
                                    8=h2 重采不足（窄面/丢回波）跳过测斜 / 9=ε̂ 超模型（|ε̂|>25°）判无效跳过。
                                   与 side_warn 分开存：进 SIDE_SCAN 时 probe_side_begin() 会清 side_warn，
                                   以前 warn=6 就是在成功路径上被那次清零擦掉的（日志永远 warn=0）。 */
    uint8_t side_move_stage;   /**< SIDE_MOVE 子状态 LF_SIDE_MOVE_DRIVE / SETTLE / VERIFY / DONE */
    /* 以下 side_move_* 测斜字段自 M4e 起**不再赋值**（SIDE_MOVE 到位后不许再转身）；
     * 定义保留只为不动 main.c 的 PROBE_SIDE_YAW 日志格式，读出来恒 0。 */
    uint8_t side_move_mono;    /**< （停用）平移段侧距「同向增量」连续帧数 */
    uint32_t side_move_since;  /**< （停用）平移段侧距采样节拍时间戳 (ms，50 ms) */
    float side_move_h0;        /**< （停用）平移段起始侧距 (mm)，0=还没取到 */
    float side_move_h_last;    /**< （停用）平移段最近有效侧距 (mm) */
    float side_move_s0;        /**< （停用）平移段起始里程 (mm) */
    float side_move_ds;        /**< （停用）平移段已走距离 (mm) */
    float side_move_dh;        /**< （停用）平移段侧距总漂移 (mm，末−首) */
    float side_move_dh_prev;   /**< （停用）上一次采样增量 (mm)，判单调用 */
    float side_move_corr_rad;  /**< （停用）平移测出的残余偏航 (rad)；M4e 起恒 0，不再触发转身 */
    uint8_t side_verify_mode;  /**< 1 = 当前这一遍扫描是发射前复测（reach 收窄、原点已重锁存） */
    uint8_t side_verify_done;  /**< 1 = 本面已经做过一次复测（避免重复进 VERIFY） */
    uint8_t side_rescan;       /**< 1 = SIDE_MOVE 到位后回到 SIDE_SCAN 重扫一次 */
    uint8_t side_locked;       /**< 1 = 侧视锁定成功：LASER 必须保持 side_us 发射（性质①） */
    uint8_t side_bench;        /**< 1 = 台架侧扫命令启动（不走 AIM/LEG，单面结束） */
    uint8_t side_push;         /**< 1 = 台架手推模式：电机断电，人工沿面推车扫描 */
    uint8_t side_event;        /**< 侧扫遥测事件 LF_SIDE_EVENT_*，main.c 打印日志后清零 */
    float side_ring_s[LF_SIDE_RING]; /**< 最近 8 个面内样本的扫描坐标 s (mm) */
    float side_ring_d[LF_SIDE_RING]; /**< 最近 8 个面内样本的侧视回波距离 d (mm) */
    uint16_t side_inface;      /**< 当前这一遍扫描累计面内帧数（PROBE_SIDE_EDGE 的 n 用） */
    uint8_t side_edge_frames;  /**< 找到边界时累计的面内帧数（诊断用：正常 15~30） */
    float side_s_last_good;    /**< 最后一个面内样本的扫描坐标 (mm) */
    float side_s_out_first;    /**< 第一个外点样本的扫描坐标 (mm) */
    float side_s_edge[2];      /**< 两个边界的扫描坐标 (mm，未排序；边界 = 最后一个面内点与第一个外点的中点) */
    float side_s_turn;         /**< FWD 折返点的扫描坐标 (mm) */
    float side_h0;             /**< 面内直线模型截距 d = h0 + k·s (mm) */
    float side_k;              /**< 面内直线模型斜率 d = h0 + k·s (无量纲，吸收车不平行) */
    float side_h;              /**< SIDE_LOCK 实测的侧视站立距离 (mm) */
    float side_target;         /**< SIDE_MOVE 的目标扫描坐标 (mm，绝对量，正=车头方向) */
    float side_s_m_ref;        /**< 复测前的中点（扫描坐标，mm），用于一致性比较 */
    float side_verify_offset;  /**< 复测扫描原点在原扫描坐标系里的位置 (mm) */
    float side_w_prev;         /**< 复测前测得的面宽（复测失败时回退用，mm） */
    float side_origin_x_mm;    /**< 侧扫轴原点世界坐标 x (mm)：从 SIDE_SCAN 起点锁存，跨 SIDE_MOVE/复测共用 */
    float side_origin_y_mm;    /**< 侧扫轴原点世界坐标 y (mm) */
    float side_origin_rad;     /**< 侧扫轴方向（原点处的车头航向，rad） */
    uint32_t side_ping_since;  /**< 上一次取侧扫样本的时间戳 (ms)，按 60 ms 超声节拍取帧 */

    /* --- M3 转平行 TURN_PAR + 绕行 LEG --- */
    uint8_t turn_stage;        /**< 原地转子状态 LF_TURN_IDLE / DRIVE / SETTLE / DONE */
    uint8_t turn_ok;           /**< 1 = 停稳后 |ach - cmd| <= LF_TURN_TOL_DEG */
    uint8_t turn_backup;       /**< 1 = TURN_PAR 的低 ρ 倒车已完成/不需要；0 = 还没倒 */
    uint8_t side_lock_stage;   /**< SIDE_LOCK 预扫测斜：0=等舵机 1=h1 2=沿面平移 60 mm 3=h2 4=退回+转正 5=h3 6=完成 */
    float side_h1;             /**< 预扫第一点 h (mm) */
    float side_h2;             /**< 沿面平移 60 mm 后重采的 h (mm)（必须重新采样，不能复用 h1 的样本环） */
    float side_h3;             /**< 退回并转正后 h (mm) */
    float side_delta_rad;      /**< 旧旋转法转过的 +δ (rad)，现方法恒 0 */
    float side_tilt_corr_rad;  /**< 由 atan2(h2−h1, 本面基线) 求出的车头残余偏航修正角 (rad)，正=左转/CCW */
    float side_tilt_eps_deg;   /**< 本次两点测斜的 ε̂ (°，正 = 需左转)，供日志与诊断（M4e） */
    float side_tilt_shift_mm;  /**< 本面平移基线 (mm)：宽 60 / 窄 30（按面宽交替选择，M4e） */
    uint8_t side_tilt_iter;    /**< 本面已执行的修正转次数 0/1（每面最多一次，M4e） */
    float side_shift_start_mm; /**< 测斜平移起点 (mm)：SIDE_LOCK 沿面平移两点法的基准 */
    float side_shift_peak_mm;  /**< 测斜段实际走到的最大里程 (mm)：>0 才说明平移真的发生（诊断 h1==h2 是平行还是没动） */
    uint32_t side_shift_done_ms; /**< 平移结束（制动停稳）时间戳 (ms)，停稳后再测 h2 */
    uint8_t leg_stage;         /**< LEG 子状态 DRIVE1 / TURN / DRIVE2 / DONE */
    uint8_t leg_started;       /**< 1 = LEG 入口的原点/航向/L1/L2 已经锁存 */
    uint8_t leg_event;         /**< LEG 遥测事件 LF_LEG_EVENT_*，main.c 打印后清零 */
    uint8_t leg_fail;          /**< LEG 失败原因：0=正常，1=h 门禁，2=DRIVE1 超时，3=转角超差，4=DRIVE2 超时 */
    uint8_t leg_kind;          /**< 本段 LEG 当前面宽窄：0=窄(50)/1=宽(150)，首面实测+面序交替决定 */
    uint8_t box_first_kind;    /**< 首面宽窄基准（1=宽 150） */
    uint8_t box_latch_index;   /**< 首面基准对应的面号（probe_face-1） */
    uint8_t box_kind_ok;       /**< 1 = 首面基准已锁存 */
    float turn_base_rad;       /**< 本次原地转的基准航向 (rad)，TURN_PAR=入口航向，LEG TURN=DRIVE1 结束航向 */
    float turn_cmd_deg;        /**< 本次原地转的命令角 (deg，左正右负) */
    float turn_ach_deg;        /**< 停稳后实测角 (deg) */
    uint32_t turn_start_ms;    /**< 本次原地转开始时间戳 (ms) */
    uint32_t turn_settle_ms;   /**< 转到位、进入 SETTLE 的时间戳 (ms) */
    uint32_t turn_ms;          /**< 本次原地转耗时 (ms，到停稳为止) */
    float leg_origin_mm;       /**< LEG 入口累计里程 (mm) */
    float leg_turn_mm;         /**< 进 TURN 前的累计里程 (mm)，DRIVE2 的距离基准（原地转不改变里程） */
    float leg_hdg0;            /**< LEG 入口航向（DRIVE1 的纠偏基准，rad） */
    float leg_hdg1;            /**< DRIVE1 结束/进 TURN 前的航向 (rad) */
    float leg_hdg2;            /**< TURN 结束/DRIVE2 起始的航向 (rad) */
    float leg_l1_mm;           /**< DRIVE1 目标距离 (mm) */
    float leg_l2_mm;           /**< DRIVE2 目标距离 (mm) */
    float leg_cmd_mm;          /**< 最近一段直行的命令距离 (mm) */
    float leg_ach_mm;          /**< 最近一段直行的实测距离 (mm) */
    float leg_hdg_deg;         /**< 最近一段直行的航向偏差 (deg，相对该段基准) */
    float leg_turn_ach_deg;    /**< LEG 转角实测值 (deg) */
    uint32_t leg_turn_ms;      /**< LEG 转角耗时 (ms) */
    uint32_t leg_since;        /**< LEG 入口时间戳 (ms，总超时用) */
    uint32_t leg_drive_since;  /**< 当前直行段起始时间戳 (ms，单段超时用) */

    /* --- M4：LEG 期间侧距连续采样（宽面转完舵机不回正，超声一直看左侧） --- */
    uint32_t leg_prox_since;   /**< 上一次侧距采样时间戳 (ms)，50 ms 节拍 */
    uint32_t leg_face2_sum;    /**< DRIVE2 认面帧的距离和 (mm，求均值用) */
    uint16_t leg_prox_n;       /**< LEG 期间落在 [H_MIN,H_MAX] 的有效侧距帧数 */
    uint16_t leg_prox_last;    /**< 最近一帧有效侧距 (mm)，0=还没测到 */
    uint16_t leg_prox_min;     /**< LEG 期间最小有效侧距 (mm)，0=还没测到 */
    uint16_t leg_prox_max;     /**< LEG 期间最大有效侧距 (mm)，0=还没测到 */
    uint16_t leg_face2_n;      /**< DRIVE2 期间落在 h_expected±band 的帧数 */
    uint16_t leg_face2_h;      /**< 上述帧的均值 (mm)，0=DRIVE2 没测到面 */
    uint16_t leg_prox_now;     /**< 最近一帧侧距（不筛范围），收尾位置自检用 */
    uint8_t leg_prox_now_ok;   /**< 1 = 最近一帧侧距有效 */
    float leg_h_expected;      /**< 本段 LEG 预期的侧距 h_leg (mm)，DRIVE2 认面窗口中心 */
    float leg_backup_start_mm; /**< 收尾倒回段的起点里程 (mm) */

    /* --- 左右轮电机死区与全量程转速自动标定 (LF_MCAL：地面带载为主、空载作参考) --- */
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

/**
 * @brief 台架侧扫专用入口：跳过 AIM（车由人工摆成与面平行），直接 SIDE_LOCK -> SIDE_SCAN -> SIDE_MOVE -> LASER
 * @param c      控制器指针
 * @param now_ms 当前毫秒时间戳
 * @param push   true = 手推模式（电机断电，人工沿面推车，找齐两边界后打印 PROBE_SIDE_MID 并结束）；
 *               false = 自走模式（车辆以 probe_side_speed_mm_s 自己沿面平移扫描）
 * @return true 启动成功；false 当前非停止状态、有故障、或 probe_scan_mode=0
 * @note   M2 台架验收用（《问题排查手册》§17.12 第 4、7 步）；M3 起正常流程由 AIM->TURN_PAR 自动进入。
 */
bool lf_start_side_probe(LfController *c, uint32_t now_ms, bool push);

/**
 * @brief 台架绕行专用入口：用上一次 side 扫描得到的 face(w/kind/h) 直接跑一段 LEG
 * @param c      控制器指针
 * @param now_ms 当前毫秒时间戳
 * @return true 启动成功；false 当前非停止状态、有故障、probe_scan_mode=0，或没有可用的 face.w / side_h
 * @note  M3 台架验收用：先 side 扫到中点 → leg 绕到下一面 → 自动 SIDE_LOCK/SCAN/MOVE → COMPLETE_BEEPS。
 *        LEG 段（DRIVE1/TURN/DRIVE2）、h 门禁与总超时；失败即停并置 leg_event FAIL。
 */
bool lf_start_leg_probe(LfController *c, uint32_t now_ms);

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
