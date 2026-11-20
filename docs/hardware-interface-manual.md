# STM32G474 核心板与主扩展板硬件接口逐脚对应手册

本文档基于主扩展板嘉立创 EDA 实时导出网表 [live-main-netlist.json](file:///c:/Users/Seraphina/Documents/ESD/firmware/line-following-car/docs/live-main-netlist.json)、[hardware-relevant-parts.json](file:///c:/Users/Seraphina/Documents/ESD/firmware/line-following-car/docs/hardware-relevant-parts.json) 与《STM32G474VET6V1.1 核心板原理图》编制，详细列出 **每一个 MCU 引脚 → 核心板排针 → 主扩展板排母 → 板上调理电路/多路选择器 → 外部传感器/执行器** 的完整物理与电气对接关系。

---

## 1. 硬件总体架构与自动切源机制

整车电路由 **STM32G474VET6 核心板**、**主扩展板（Main Expansion Board）** 和 **下挂式 4 路红外传感器子板（IR-DC / IR-MOD）** 组成：

1. **核心板在位自动识别（无需拨动任何开关或跳线）**
   - 主扩展板排母 `J_C2` 的第 6 脚定义为专用检测网络 `MCU_PRESENT_n`，在扩展板上通过 `10 kΩ` 电阻上拉至 `SYS_5V` 并经 RC 滤波得 `PRESENT_FILTER`，送入施密特反相器 `U3`（`SN74AHCT14PWR`）第 1 脚（`1A`）。
   - **当拔除 G474 核心板时**：`J_C2-6` 悬空，`PRESENT_FILTER = 5V`，反相器第 2 脚（`1Y`）输出 **`MODE_MCU = 0`**。此时两颗四路二选一数据选择器 `U4`（`U_DRV_MUX`）和 `U18`（`U_US_SEL`）全部切向 **A 端（纯硬件模拟 PID 与硬件超声停车通道）**。
   - **当插入 G474 核心板时**：核心板排针 `J2-6` 直接连通核心板公共地 `GND`，将 `MCU_PRESENT_n` 拉低为 `0V`，反相器输出 **`MODE_MCU = 1`**。此时 `U4` 与 `U18` 自动切向 **B 端（STM32G474 数字控制通道）**。
2. **上电复位就绪控制（`POR_OK`）**
   - 扩展板上电后经 RC 延时与 `U3`（`5A->5Y->6A->6Y`）两级施密特整形生成高电平有效的 `POR_OK` 信号。
   - **电机驱动芯片 `DRV8833` 的睡眠脚 `nSLEEP`（`U5 Pin 1`）和红外子板的使能脚 `IR_ENABLE`（`J_IR Pin 6`）均直接由硬件 `POR_OK` 控制唤醒**，不占用任何 MCU GPIO 引脚（切勿将旧工程的 `PC1` 误作睡眠控制脚）。

---

## 2. MCU 引脚与外部板卡信号逐脚对接详表

### 2.1 供电、公共地、在位检测与调试下载接口（`J_C1` / `J_C2` / `J_SWD`）

| MCU 引脚 / 电源网 | 核心板排针 | 扩展板排母 | 扩展板网络名 | 对接电路与外部模块说明 | 软件配置要求 |
| --- | :---: | :---: | --- | --- | --- |
| `5V`（板载 AMS1117 输入） | `J1-1, J1-2` | `J_C1-1, 2` | `SYS_5V` | 接扩展板主电源自动切换芯片 `TPS2117` 输出（电池 `BASE_5V` 优先，调试口 `DBG_5V` 备用），向核心板供电 | 硬件供电 |
| `3.3V`（AMS1117 输出） | `J1-3, J1-4` | `J_C1-3, 4` | `MCU_3V3` | 核心板稳压输出 `3.3V`，反供扩展板上的双轮编码器、1.8寸 LCD、电平转换器 `SN74LVC1G17` 及 `3.3V` 上拉电阻 | 硬件供电 |
| `GND` | `J2-1..4` | `J_C2-1..4` | `GND` | 核心板与主扩展板完整公共地平面共地 | 硬件共地 |
| `GND`（在位检测专用脚） | `J2-6` | `J_C2-6` | `MCU_PRESENT_n` | 扩展板侧 `10kΩ` 上拉至 `SYS_5V` → RC 滤波 `PRESENT_FILTER` → `U3 (74AHCT14) Pin1 (1A)` → `Pin2 (1Y)` 输出 `MODE_MCU = 1` | 硬件自动检测，软件无需配置 GPIO |
| `PA13 / PA14 / NRST` | `J4-2 / 1 / 7` | 扩展板 2×4 母座 → `J_SWD` | `SWDIO / SWCLK / MCU_NRST` | 对接主扩展板专用受保护 5Pin 调试座 **`J_SWD`**（`Pin1=DBG_5V_IN` 经 `TPS2553` 限流保护，`Pin2=SWCLK`，`Pin3=GND`，`Pin4=SWDIO`，`Pin5=NRST`；按下扩展板 `RESET` 键同步拉低 `NRST`） | 保持默认 SWD 调试复用功能 |

> [!CAUTION]
> **ST-LINK 接线安全铁律**：下载或调试时，ST-LINK 的 `5V` 电源线**必须插在主扩展板边缘的 5Pin 专用座 `J_SWD` 上**（或拔掉 5V 线仅用电池供电），**严禁把 ST-LINK 的 5V 直接插在核心板顶部的 `J4-4` 排针上**，否则会绕过扩展板的限流与防倒灌保护芯片！

---

### 2.2 四路反射式红外循迹传感器接口（`PA0 ~ PA3` ↔ `J_IR`）

| MCU 引脚 | 核心板排针 | 扩展板排母 | 扩展板网络名 | 物理探头位置 | 硬件调理与外部板卡信号路径 | STM32 外设与软件配置 |
| --- | :---: | :---: | --- | --- | --- | --- |
| **`PA0`** | `J2-42` | `J_C2-42` | `IR_ADC_0` | **左外 `L2`** (`x = -19.5 mm`) | `J_IR Pin 7 (IR_TRACK_0)` → 扩展板 `4.7 kΩ + 100 pF` RC 隔离低通滤波 → `PA0` | `ADC1_IN1`，模拟输入，`247.5` 周期采样，TIM6 1kHz 触发 + DMA 5× 均值 |
| **`PA1`** | `J2-39` | `J_C2-39` | `IR_ADC_1` | **左内 `L1`** (`x = -6.5 mm`) | `J_IR Pin 9 (IR_TRACK_1)` → 扩展板 `4.7 kΩ + 100 pF` RC 隔离低通滤波 → `PA1` | `ADC1_IN2`，同上 |
| **`PA2`** | `J2-40` | `J_C2-40` | `IR_ADC_2` | **右内 `R1`** (`x = +6.5 mm`) | `J_IR Pin 11 (IR_TRACK_2)` → 扩展板 `4.7 kΩ + 100 pF` RC 隔离低通滤波 → `PA2` | `ADC1_IN3`，同上 |
| **`PA3`** | `J2-37` | `J_C2-37` | `IR_ADC_3` | **右外 `R2`** (`x = +19.5 mm`) | `J_IR Pin 13 (IR_TRACK_3)` → 扩展板 `4.7 kΩ + 100 pF` RC 隔离低通滤波 → `PA3` | `ADC1_IN4`，同上 |

- **电气特性**：红外子板输出 `0 ~ 3.3V` 模拟电压。照射白色底面时反射光强，电压高（接近 `2.5 ~ 3.0V`，ADC 值约 `3000+`）；照射 20 mm 黑色胶带时反射光弱，电压低（ADC 值约 `300 ~ 800`）。

---

### 2.3 双 H 桥电机驱动与故障保护接口（`PE9/11/13/14, PD2, PC4` ↔ `U4 / U5 DRV8833`）

| MCU 引脚 | 核心板排针 | 扩展板排母 | 扩展板网络名 | 硬件信号路径（芯片引脚级） | STM32 外设与软件配置 |
| --- | :---: | :---: | --- | --- | --- |
| **`PE9`** | `J2-25` | `J_C2-25` | `MCU_AIN1` | `PE9` → `U4 (SN74AHCT157 #2) Pin 3 (1B)` → `Pin 4 (1Y: DRV_AIN1)` → `U5 (DRV8833) Pin 16 (AIN1)` → 控制左电机正极 `MOT_L_P` | **`TIM1_CH1` (`GPIO_AF2_TIM1`)**，20 kHz 推挽复用输出 |
| **`PE11`** | `J2-23` | `J_C2-23` | `MCU_AIN2` | `PE11` → `U4 Pin 6 (2B)` → `Pin 7 (2Y: DRV_AIN2)` → `U5 Pin 15 (AIN2)` → 控制左电机负极 `MOT_L_N` | **`TIM1_CH2` (`GPIO_AF2_TIM1`)**，20 kHz 推挽复用输出 |
| **`PE13`** | `J2-21` | `J_C2-21` | `MCU_BIN1` | `PE13` → `U4 Pin 10 (3B)` → `Pin 9 (3Y: DRV_BIN1)` → `U5 Pin 9 (BIN1)` → 控制右电机（`J_MOT_R` 焊盘已硬件镜像反接，正占空比同样对应车轮向前转） | **`TIM1_CH3` (`GPIO_AF2_TIM1`)**，20 kHz 推挽复用输出 |
| **`PE14`** | `J2-22` | `J_C2-22` | `MCU_BIN2` | `PE14` → `U4 Pin 13 (4B)` → `Pin 12 (4Y: DRV_BIN2)` → `U5 Pin 10 (BIN2)` → 控制右电机另一端 | **`TIM1_CH4` (`GPIO_AF2_TIM1`)**，20 kHz 推挽复用输出（切勿配成 AF6！） |
| **`PD2`** | `J1-21` | `J_C1-21` | `MCU_MOTOR_EN` | `PD2`（板上配 `10 kΩ` 下拉电阻防复位误动）→ `U18 (74AHCT157 #1) Pin 6 (2B)` → `Pin 7 (2Y: MOTOR_ENABLE)` → `U3 (74AHCT14) Pin 5 (3A)` 反相 → `Pin 6 (3Y: MOTOR_nENABLE)` → 控制 `U4 Pin 15 (G#)` 总选通门 | `GPIO_MODE_OUTPUT_PP`，**高电平使能电机**；拉低时 `U4` 硬件强制输出 `0000` 关断双桥 |
| **`PC4`** | `J2-34` | `J_C2-34` | `DRV_nFAULT` | `U5 (DRV8833) Pin 8 (nFault)` 开漏输出（板上 `10 kΩ` 上拉至 `MCU_3V3`）→ `PC4` | `GPIO_MODE_INPUT` (`GPIO_PULLUP`)，**低电平表示过流/过温/欠压故障** |

---

### 2.4 左右轮霍尔正交编码器接口（`PB6/PB7, PA6/PA7` ↔ 左右 N20 电机编码器）

| MCU 引脚 | 核心板排针 | 扩展板排母 | 扩展板网络名 | 硬件调理与外部信号路径 | STM32 外设与软件配置 |
| --- | :---: | :---: | --- | --- | --- |
| **`PB6`** | `J1-30` | `J_C1-30` | `ENC_L_A` | 左电机霍尔 A 相（`MCU_3V3` 供电）→ `4.7 kΩ` 上拉 + `1 kΩ / 100 pF` RC 滤波 → `PB6` | **`TIM4_CH1` (`GPIO_AF2_TIM4`)**，正交四倍频 `TI12` 模式，`IC1F=6`；**初始化前必须调用 `HAL_PWREx_DisableUCPDDeadBattery()`** |
| **`PB7`** | `J1-31` | `J_C1-31` | `ENC_L_B` | 左电机霍尔 B 相 → `4.7 kΩ` 上拉 + `1 kΩ / 100 pF` RC 滤波 → `PB7` | **`TIM4_CH2` (`GPIO_AF2_TIM4`)**，`IC2F=6` |
| **`PA6`** | `J2-36` | `J_C2-36` | `ENC_R_A` | 右电机霍尔 A 相（`J_MOT_R` 焊盘已镜像交换 A/B，前进时计数同样递增）→ `4.7 kΩ` 上拉 + `1 kΩ / 100 pF` → `PA6` | **`TIM3_CH1` (`GPIO_AF2_TIM3`)**，正交四倍频 `TI12` 模式，`IC1F=6` |
| **`PA7`** | `J2-33` | `J_C2-33` | `ENC_R_B` | 右电机霍尔 B 相 → `4.7 kΩ` 上拉 + `1 kΩ / 100 pF` → `PA7` | **`TIM3_CH2` (`GPIO_AF2_TIM3`)**，`IC2F=6` |

---

### 2.5 超声波测距与水平云台舵机接口（`PB9, PC6, PA15, PC7`）

| MCU 引脚 | 核心板排针 | 扩展板排母 | 扩展板网络名 | 硬件调理与外部信号路径 | STM32 外设与软件配置 |
| --- | :---: | :---: | --- | --- | --- |
| **`PB9`** | `J1-33` | `J_C1-33` | `MCU_US_TRIG` | `PB9` → `U18 (74AHCT157 #1) Pin 3 (1B)` → `Pin 4 (1Y: HC_TRIG)`（升至 `5V` 逻辑）→ 超声波模块 `TRIG` 引脚 | **`TIM17_CH1` (`GPIO_AF1_TIM17`)**，硬件单脉冲模式（OPM），每 60 ms 自动发出精确 `45 µs` 触发脉冲 |
| **`PC6`** | `J1-6` | `J_C1-6` | `MCU_US_ECHO` | 超声波模块 `5V ECHO` 输出 → 经施密特缓冲器 `SN74LVC1G17DBVR`（`3.3V` 供电，输入耐受 `5V`）降压整形为 `3.3V` → `PC6` | **`TIM8_CH1` (`GPIO_AF4_TIM8`)**，1 MHz 硬件 PWM 输入捕获模式（`TI1FP1` 上升沿硬件清零计数器，`TI1FP2` 下降沿将脉宽 µs 锁存至 `CCR2`） |
| **`PA15`** | `J1-15` | `J_C1-15` | `MCU_SERVO_PWM` | `PA15` → 与门 `U_SERVO_GATE (74AHCT1G08GV) Pin 1 (A)` → 输出 `5V` 舵机 PWM 至水平云台舵机信号脚 | **`TIM2_CH1` (`GPIO_AF1_TIM2`)**，50 Hz（周期 `20000 µs`），脉宽 `900 ~ 2100 µs`（中位 `1500 µs`） |
| **`PC7`** | `J1-7` | `J_C1-7` | `MCU_SERVO_PWR_EN` | `PC7`（经 `1 kΩ` 串阻 + `47 kΩ` 下拉）→ 同时控制舵机独立升降压电源芯片 `U_PWR_SERVO (TPS63060 #2) EN` 脚与 `U_SERVO_GATE Pin 2 (B)` | `GPIO_MODE_OUTPUT_PP`，**高电平开启舵机 5V 电源**；软件在拉高 `PC7` 后延时 `5 ms` 再开启 `TIM2_CH1` 输出，彻底杜绝舵机断电期间信号倒灌 |

---

### 2.6 声光指示、激光发射与三路按键接口（`PD8, PC9, PC8, PC0, PD0, PD1, PD15`）

| MCU 引脚 | 核心板排针 | 扩展板排母 | 扩展板网络名 | 硬件调理与外部信号路径 | STM32 外设与软件配置 |
| --- | :---: | :---: | --- | --- | --- |
| **`PD8`** | `J2-14` | `J_C2-14` | `MCU_PARK_LAMP` | `PD8` → `U18 (74AHCT157 #1) Pin 10 (3B)` → `Pin 9 (3Y: PARK_LAMP_EN)` → 低侧 NMOS 驱动双红色停车指示灯 | `GPIO_MODE_OUTPUT_PP`，**高电平点亮停车指示灯** |
| **`PC9`** | `J1-9` | `J_C1-9` | `MCU_BUZZER` | `PC9` → `U18 Pin 13 (4B)` → `Pin 12 (4Y: BUZZER_EN)` → 低侧 NMOS 驱动有源蜂鸣器 | `GPIO_MODE_OUTPUT_PP`，**高电平鸣叫**（障碍停车长鸣 1s、激光打靶同步鸣叫 10s、完成时 3 组两短一长） |
| **`PC8`** | `J1-8` | `J_C1-8` | `MCU_LASER_EN` | `PC8` → `1 kΩ` 串阻 + `47 kΩ` 栅极下拉 → `BSS138BK` NMOS → 控制 `≤5mW` 激光模组电源 | `GPIO_MODE_OUTPUT_PP`，**高电平开启激光发射**（复位及纯电路期间由 `47 kΩ` 硬件强下拉确保绝对关断） |
| **`PD0`** | `J1-19` | `J_C1-19` | `KEY_TRACK_MCU` | 主板“循迹键” `SW2` 按下接 `GND`（`KEY_TRACK_N` 由 `R118=10kΩ` 上拉至 `SYS_5V`，`C87=100nF` 消抖）；**`PD0` 经 `R124=47kΩ / R114=100kΩ` 分压在 `U3` 之前直采 `KEY_TRACK_N`**（未经过 `U3.9->8` 反相器，后者仅喂给 `U16.3`） | `GPIO_MODE_INPUT` (`GPIO_NOPULL`，严禁开启内部上拉！)，**松开实测约 `3.185 V` (`HIGH`)，按下为 `0 V` 低电平 (`GPIO_PIN_RESET`) 有效** |
| **`PD1`** | `J1-20` | `J_C1-20` | `KEY_PROBE_MCU` | 主板“探测键” `SW3` 按下接 `GND`，板上配 `R119=10kΩ` 上拉至 `MCU_3V3` 与 `C88=100nF` 消抖电容 → `PD1` | `GPIO_MODE_INPUT` (`GPIO_PULLUP`)，**按下为低电平 (`GPIO_PIN_RESET`)** |
| **`PD15`** | `J1-5`（核心板内部接 `SW2`） | `J_C1-5`（悬空） | `CORE_KEY` | G474 核心板自带用户按键 `SW2`，按下接 `GND`；**核心板原理图无外部上拉电阻，完全依赖 MCU 内部弱上拉 (~40kΩ)** | `GPIO_MODE_INPUT` (`GPIO_PULLUP`)，**按下为低电平 (`GPIO_PIN_RESET`)** |
| **`PC0`** | 核心板内部 | — | `CORE_LED` | G474 核心板自带蓝色状态指示灯，阳极接 `3.3V` | `GPIO_MODE_OUTPUT_PP`，**低电平点亮 (`GPIO_PIN_RESET`)** |

---

### 2.7 蓝牙无线通信模块与 1.8 寸 TFT LCD 屏幕接口（`J_BT` / `J_LCD`）

| MCU 引脚 | 核心板排针 | 扩展板排母 | 扩展板网络名 | 对接模块引脚与电路说明 | STM32 外设与软件配置 |
| --- | :---: | :---: | --- | --- | --- |
| **`PA9`** | `J1-11` | `J_C1-11` | `BT_RXD_MCU` | `PA9`（MCU 发送）→ `100 Ω` 串阻 → **`J_BT Pin 2 (HC-04 RXD)`** | **`USART1_TX` (`GPIO_AF7_USART1`)**，上电自动适配 `9600->115200` 8N1 |
| **`PA10`** | `J1-12` | `J_C1-12` | `BT_TXD_MCU` | **`J_BT Pin 3 (HC-04 TXD)`**（蓝牙发送）→ `100 Ω` 串阻 → `PA10`（MCU 接收） | **`USART1_RX` (`GPIO_AF7_USART1`)**，8N1 |
| **`PA8`** | `J1-10` | `J_C1-10` | `BT_KEY_MCU` | `PA8` → `1 kΩ` 串阻 → **`J_BT Pin 6 (KEY)`**（上电探波特率时按需短暂拉高，平时保持低电平透明传输） | `GPIO_MODE_OUTPUT_PP`，默认输出低电平 |
| **`PC10`** | `J1-16` | `J_C1-16` | `BT_STATUS_MCU` | **`J_BT Pin 1 (STATUS)`** → `1 kΩ` 串阻 → `PC10`（HC-04 V2.6 原生未连接高/已连接低；`J_BT` Pin 4=`GND`、Pin 5=`SYS_5V`） | `GPIO_MODE_INPUT` (`GPIO_PULLUP`) |
| **`PB3 / PB5`** | `J1-27 / 29` | `J_C1-27 / 29` | `LCD_SCL / SDA` | 直连 `J_LCD Pin 3 (SCK) / Pin 4 (MOSI)`（主板网络直连无串阻；`J_LCD Pin 1=GND, Pin 2=MCU_3V3`） | **已启用 `SPI1` (`GPIO_AF5_SPI1`)**，10 MHz 单线只发驱动 `ST7735S` |
| **`PD10..13`** | `J2-12/9/10/7` | `J_C2-12/9/10/7` | `LCD_RES/CS/DC/BL` | 直连 `J_LCD Pin 5 (RES) / Pin 7 (CS) / Pin 6 (DC) / Pin 8 (BL)` | **已启用 `GPIO_MODE_OUTPUT_PP`**，`128x160` 竖屏（默认 `0xC0` 正向 + 电压正比进度条，支持 `lcdflip 0/1`） |

---

### 2.8 纯硬件模拟 PID 主电路实网参数与软件仿真 (`HW-EMU`) 1:1 映射表

以下数据全部核对自当前工作区 [live-main-netlist.json](file:///c:/Users/Seraphina/Documents/ESD/firmware/line-following-car/docs/live-main-netlist.json)：

| 功能模块 | 关键芯片与网络 | 实网器件参数 | 与固件 `hwemu` 参数对应关系 |
| --- | --- | --- | --- |
| **四路加权差分级** | `U6A` (`OPA4376`, 供电 `PWM_5V=5.0V`), `PWM_MID=2.50V` | `R39=R40=150kΩ` (`L2/R2`)、`R41=R42=300kΩ` (`L1/R1`)、`R43\|\|R45=R44\|\|R46=50kΩ` | `PID_E = 2.500 + hw_w_out*(V_L2-V_R2) + hw_w_in*(V_L1-V_R1)`（原板 `hw_w_out=1/3`, `hw_w_in=1/6`；换焊 `R39/R40` 即改 `hw_w_out=50k/R39`） |
| **比例支路 P** | `U6B` -> `V_P` | `R48=10kΩ`, `R49=10kΩ`, `RV_P=100kΩ`, 求和 `R60=20kΩ` | `hw_kp = (10k + RV_P)/10k` (`1.0 ~ 11.0`)；`hw_r60 = R60` (若 `Kp` 最小仍过强，可改焊 `R60=47kΩ`) |
| **微分支路 D** | `U6C` -> `V_D` | `C50=100nF`, `R54=22kΩ`, `R55=4.7kΩ`, `RV_D=100kΩ`, `C60=10nF`, 求和 `R61=20kΩ` | `hw_kd_ms = (4.7k + RV_D)*C50`（**原板 `C50=100nF` 最大仅 `10.47 ms`**）；`hw_tau1_ms = R54*C50 = 2.2 ms`；`hw_r61 = R61` |
| **积分支路 I** | `U6D` -> `V_I` -> `JP_I` | `R56=100kΩ`, `RV_I=1MΩ`, `C61=1uF`, `R57=1MΩ`, `R47=100kΩ` | `hw_i = 0/1`（对应拔掉/插上 `JP_I` 跳线帽），`hw_ti_ms = (100k + RV_I)*1uF` |
| **基速与三角波 PWM** | `U7A/B/C/D`, `U9`, `U10` (`TLV9034`) | `V_BASE`: `R67=6.2k`, `RV_V=10k`, `JP_V` (`1-2: PWM_MID` / `2-3: R68=6.2k->GND`)；`R74/R76=1.2k/2.0k` (`0.625`)；三角波实网阈值 **`0.351 ~ 2.851 V`** | `hw_v_base = V_BASE`（`2.500V` 时 `D0=48.5%`；**必须插 `JP_V 2-3` 才可下调 `V_BASE < 2.50V`**）；`hw_coast = 1` 对应 `U4` 将 `AIN2/BIN2` 接 `GND` 滑行模式 |
