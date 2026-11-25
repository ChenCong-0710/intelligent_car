# TEST 电机架空标定 PD65_CAL_V1

此固件只用于架空固定PWM辨识。不是65rpm闭环固件，不执行八动作，不包含超声、激光、舵机动作。原PD65_V2固件仍保留。修改前源码备份在 backups/before_cal_20261125_112813。

## 烧录与操作

固件：Debug/calibration/pd65_calibration.hex（同目录elf/bin）。本次只编译，未烧录。
复位后待机，两轮可靠悬空并固定车架。先启动 tools/capture_calibration.py，收到 CONFIRMED PD65_CAL_V1 且待机 error=OK 后再按下、释放PD0。PD0运行中按下即停止；复位后不会自动启动。故障需处理原因并复位。

自动顺序 LEFT_FWD、LEFT_REV、RIGHT_FWD、RIGHT_REV；每次只驱动一轮，另一轮制动。每方向固定PWM依次12/16/20/24/28/32/36/40%，100ms起步；每档5秒，舍弃前2秒，后3秒车端累计编码器计数算平均rpm。停稳100ms后再等待2秒。完成四方向后自动停止，不循环。最长约4分钟。

完整有效档位平均转速达到85rpm后跳过该方向更高档位；瞬时测速连续100ms超过110rpm则提前刹停，本档标记capped=1、valid=0并跳过更高档。160rpm超速故障保护保留。12~24%无转动可作为静摩擦数据；28%以上起步1.5秒后持续1秒无转动会报堵转。驱动故障、编码器突跳、方向错误、停车未稳定保护保留。

若40%仍未覆盖65rpm，不凭空外推；分析后另行决定档位。架空数据不作为落地验收。不要用手阻挡轮胎。

## 采集（只读，持续到明确停止）

python tools/capture_calibration.py --port COM4

自动生成带时间戳的 work/cal_*.log/.jsonl/.bin，不覆盖旧数据。脚本不发送命令。端口打开后仍需等固件标识确认，若收到旧M/PROBE帧会提示UNEXPECTED_FIRMWARE。3秒无有效数据告警；序号跳变报告SEQ_GAP。Ctrl+C停止并关闭端口。

M帧200ms周期，rpmL10/R10除以10为rpm，pwmL1000/R1000除以1000为占空比。step为0~7，results是已完成档位数，txdrop是MCU发送队列丢弃数。mode仍只是人工标签。
C帧为完整档位统计：group=0/1/2/3分别左前/左后/右前/右后，pwm1000有方向符号，samples为1ms统计样本数，counts为按命令方向归一化的计数，mean10/min10/max10除以10为rpm。有效完整统计samples=3000、valid=1。min/max来自20ms窗口测速。车端保存全部最多32组结果并循环重发；漏掉单帧仍可补收。复位清除结果，采集完成确认日志后再复位。

串口改为TX中断发送，避免逐字屏幕刷新拖慢发送。控制仍1ms编码器读取。原PD公式未调整，无停车后反转补齐。

## 构建与检查

python tools/build_calibration.py

使用E:/stm32cubeIDE内编译器，单独编译main/motion_pd，链接现有Debug内HAL/启动对象到Debug/calibration，排除initial_align，不覆盖原PD65_V2固件。不自动下载。

主机 tests/test_calibration.c 验证四方向完整流程、统计窗口、单轮输出、停止、限速跳档及保护；tests/test_motion_pd.c用于原动作控制回归。这些仅验证软件逻辑，实际转速和实车表现需要采集验证。

## V2 实测修正（已烧录并校验）

V1在用户未按停止时进入STOPPING，源码唯一显式路径为TIM6检测PD0低电平。电气原因尚未确认。V2将单次1ms低电平停止改为连续20ms，新增keylow（当前连续低电平ms）、keymax（历史最长低电平ms）、keyedges（低电平段数）、keystops（按键停止次数）。

当前标识PD65_CAL_V2，固件Debug/calibration/pd65_calibration_v2.hex。采集脚本已同步要求V2。V1日志work/cal_20261125_113135.*保留；V2开始采集日志work/cal_20261125_113437.*。V2已获ST-LINK写入及校验成功，且真实收到IDLE,error=OK。标定是否成功仍待实测。
