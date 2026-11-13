# MC02 大小 yaw 云台

参考旧工程 `application/gimbal/gimbal_big_yaw.c`、
`gimbal_yaw_pitch_direct.c` 的控制方法，接入当前工程的
`DjiMotor`、`DmMotor`、`Sliding`、`system_state`、板间通信和 FreeRTOS。

## 文件与任务

- `gimbal_config.h`：硬件、机械中值、限位、SMC、扫描和跟随参数宏。
- `app_gimbal_big_yaw.cpp`：主控的大 Yaw 目标状态机和云台任务入口。
- `app_gimbal_small_yaw.cpp`：主控的小 Yaw、Roll、Pitch 状态机及本地电机闭环。
- `gimbal_axis.hpp`：DJI、DM 反馈和共用的单轴 SMC。
- `app_gimbal.h`：编译后唯一的 `Gimbal_Init/Control/Task` 接口。
- `main.c`：在 FDCAN 和底盘初始化后调用 `Gimbal_Init()`，注册失败进入
  `Error_Handler()`。
- `freertos.c`：静态创建独立云台任务，以 `vTaskDelayUntil` 按 1 ms 周期运行。

在 LF 工程目录执行 `mingw32-make -j4` 编译主控，固件输出到 `build/board_master`。
主控直接连接遥控、裁判系统和 USB 上位机，CAN1 直驱小云台，CAN2 直驱发射，
CAN3 与枪口 DM-IMU 和独立的副板共线。副板负责底盘、大 Yaw 电机和超级电容。

## 填写宏

正式固件启动时初始化对应云台。烧录前填写 `gimbal_config.h` 中的实车配置。

大 Yaw 为副板驱动的 DM4310。小 Yaw 为直驱 GM6020，Roll 和 Pitch
为 DM4310，三者共用主控 CAN1。CAN 口、ID、方向、中位、限位及 SMC 参数均待实测。

`GIMBAL_YAW_MID_ECD` 填小 yaw 居中时的原始编码器计数 `[0,8191]`，
关节角为当前计数减中值后折算的单圈有符号偏角。`*_MID_RAD` 填 DM
在相应机械中位时报告的原始输出轴角度，取安装方向修正前的数值。
过洞要求大 Yaw 与底盘机械对齐：手动模式驱动大 Yaw 回中；导航模式保持洞口航向、由导航转动底盘对齐。
用于关节闭环的实际行程须完整落在
DM 配置的反馈位置范围内。pitch 和 roll 的机械限位须在中位两侧半圈以内。

本地两台 DM 用调试助手配置 **MIT**，CAN ID、Master ID 和 P/V/T 编码范围要与宏一致。
MC02 的 SMC 输出力矩，MIT 帧的 `kp/kd` 为零。`*_OUTPUT_MAX_NM` 为软件力矩限幅，
`GIMBAL_DM_TORQUE_MAX_NM` 是协议编码范围。DM4310-2EC 有输出轴单圈绝对编码器，
驱动器已经将位置反馈折算到输出轴。10∶1 是电机内部减速比，MC02 按外部传动比 1 处理。
GM6020 的 `GIMBAL_YAW_OUTPUT_MAX` 为原始电压指令限幅。所有控制角为 rad，
角速度为 rad/s。大/小 yaw 的机构正方向必须一致。大 yaw 的扫描要求机构允许连续旋转。

## 控制行为

主控的 `Gimbal_Control()` 先完成小云台闭环，再运行大 Yaw 目标状态机并发布参考。
底盘的遥控、USB 命令选择和速度限幅也在主控完成。副板以收到的目标计算电机输出及功率限制。
主控 BoardLink 将直接遥控的枪口系平移速度按大 Yaw + 小 Yaw 总偏角转换到底盘系。
自动模式的 USB `vx/vy/wz` 使用导航工程 `chassis_link` 底盘坐标系，直接限幅下发。
`0x310` 始终携带底盘系目标，副板按这一坐标系执行速度闭环。
`vx_b = cos(a)*vx - sin(a)*vy`，`vy_b = sin(a)*vx + cos(a)*vy`；`wz` 保持主控目标。
旋转后若超过现有底盘轴向速度上限，等比例缩放平移速度以保持方向。
直接遥控所需偏角失效或超时时发送无效底盘目标。副板收到的速度已在底盘系，不再进行坐标旋转。
大 yaw 先由 `selectMode` 选出目标模式，再由 `handleException` 处理失能、反馈故障及 DM 使能确认。
小 yaw/pitch 根据本板 USB 目标进入 AUTO_AIM。SCAN 时小 yaw 相对大 yaw 居中、pitch 正弦运动。
异常处理会跳过目标与闭环计算。GM6020 发送零指令，DM 发送失能命令并读取驱动状态。

遥控在线时，右拨杆下档锁定、中档手动调试、上档自动。遥控离线时，收到
第一帧 RobotCmd 或导航会话帧后进入自主云台控制，有有效瞄准目标时跟踪，其余普通运行时间扫描。
大 yaw 中档手动由本地拨杆选择。离线和上档共用自动分支，依据小云台实际运行状态选择
GAP、SCAN 或自瞄及阈值跟随。进入扫描前先经 GAP 捕获当前 IMU 朝向。

| 模式 | 行为 |
| --- | --- |
| 零力 | 遥控下档、板间断联或反馈故障时停止输出 |
| INIT | 主控按机械角将小 yaw 和 pitch 回中；成功后 GAP，超时进入 FAULT，需下档清故障。大 yaw 解锁后直接捕获当前 IMU 朝向 |
| GAP | 以进入时的姿态为参考，等待协同端就绪 |
| 中档 MANUAL | 右水平摇杆驱动大 yaw，小 yaw 机械居中；右竖直摇杆控制 pitch |
| SCAN | 大 yaw 连续扫描；小 yaw 相对大 yaw 居中，pitch 正弦扫描 |
| AUTO_AIM | 小 yaw/pitch 跟踪 USB 下发的 IMU 目标角；大 yaw 保持进入时姿态 |
| FOLLOW（大 yaw） | 小 yaw 偏角达到进入阈值后，大 yaw 向该方向跟随，退出阈值内停止跟随 |
| NAVIGATION（大 yaw） | 收到有效导航航向时保持洞口目标航向；进入过洞后继续保持该惯性航向 |

跟随使用进入/退出双阈值，避免临界点反复切换；与旧代码一样，小 yaw
保持 IMU 瞄准目标，大 yaw 的运动令小 yaw 的机械偏角回到中心。
自瞄跟随阈值为进入 `1.2 rad`、退出 `0.05 rad`。两个 yaw 的 IMU 闭环
使用单圈角，位置误差和相邻参考的差值都取最短圆周差。大 yaw 的普通模式从切换时的
IMU 朝向起算，移动参考领先当前朝向最多 `GIMBAL_BIG_YAW_MAX_LEAD_RAD`。
小 yaw 的回中、手动、扫描和过洞都使用机械角及电机相对速度。
小云台模式切换会重置本地 SMC 历史，机械限位同时约束关节目标和向外输出。
小云台电机 CAN 发送失败时锁定故障。驱动器自身的通信超时保护通过调试助手设置。

主控 MC02 固定在大 Yaw 转动件上，板载 BMI088 发布大 Yaw 姿态到 `system_state.imu`。
枪口 DM-IMU 只周期发送加速度和角速度，主控在 INS_Task 内独立解算并发布 `system_state.muzzle_imu`。
DM 欧拉角和四元数接口保留，正常控制不请求。六轴解算 yaw 从零积分，安装轴向、零偏和长时间漂移须实测。

## 板间通信与 USB

完整帧布局、1000 Hz 调度、设备输出设置和超时规则见 [CAN3 v3 协议](../../../CAN3_PROTOCOL.md)。
新导航命令/反馈、topic 映射及过洞握手见 [导航 USB 协议](../../../USB_NAVIGATION_PROTOCOL.md)。
主控下发底盘系速度、大 Yaw 两种模式的目标/参考、裁判功率快照，各一帧；
副板以一帧反馈实测底盘速度和大 Yaw 机械偏角。加上 DM 两类原始采样，共六帧每毫秒，最坏预算 81%。
主控对直接遥控使用总偏角完成坐标转换，不再下发独立偏角帧。
USB RobotMotion 保持 ID 0x08、10 ms 周期，上报底盘系实测速度和 CAN 接收时间，无效时停发。
JointState 继续上报有效的小 Yaw、Pitch 关节角。默认 IOC/FDCAN 配置保持不变。

USB RobotCmd 的 yaw/pitch 与 SENTRY_MAIN 一致，为**云台 IMU 坐标系下的绝对目标角（rad）**。
板端直接写入 `RobotCmdState.gimbal_yaw/gimbal_pitch`。
闭环用当前 IMU 姿态计算误差，yaw 取最短圆周差，pitch 按机械行程限幅。
仅新的、长度/CRC 正确且发送时间戳前进的 RobotCmd 帧刷新本地指令时间；
重复轮询旧缓存、其他 USB 包不会让旧瞄准目标保持有效。
RobotCmd（ID `0x01`）固定 58 字节，复用原底盘速度、小云台目标和发射字段，
通过 `data.navigation` 携带导航模式、有效位、会话和大 Yaw 航向。
上位机重启后以新会话发送中性帧，收到回显后开始发送有效命令。
使能的 mode 3/4 分别锁存折叠和回正请求。mode 0/1/2、失能与断流保留已请求形态。
完整布局与握手见 [导航 USB 协议](../../../USB_NAVIGATION_PROTOCOL.md)。

普通控制沿用 SENTRY 的目标变化判据。收到有效的 yaw/pitch 目标变化后进入自瞄。
目标连续 `GIMBAL_AIM_LOST_MS` 未变化，或 RobotCmd 超过 `GIMBAL_AIM_TIMEOUT_MS`
未更新时，经 GAP 转入扫描。

云台任务直接发送小云台的 GM6020 组帧和两台 DM 的独立 MIT 帧。
初始化检查发送帧归属及同一 CAN 总线上 DM 的 ID 冲突。

## 验证范围

实机调试先确认反馈轴向、机械零位和全行程间隙，再整定 SMC 和力矩限幅。

## 过洞模式（小 yaw → roll → pitch）

云台进入 `Mode::kPassage` 后，由主控的 `PassageStage` 状态机控制小 yaw、roll、pitch。
手动过洞时，大 yaw 与小 yaw、pitch 同时对齐。大 yaw 限速回到
`GIMBAL_BIG_YAW_MID_RAD` 对应的机械中位，实测到位后发确认。导航过洞则保持 BMI088 洞口航向，
在底盘已由上位机对齐并停稳后折叠；需同时确认大 Yaw 机械角与惯性航向到位。
退出后以当前 IMU 朝向建立普通模式的参考。
右拨杆在中档时，ET08A 第 5 通道（`rc.s[2]`）的辅助开关控制过洞。
上档等效于过洞请求 `1`，中档或下档等效于 `0`。辅助开关由
`GIMBAL_PASS_SWITCH_INDEX/POSITION` 配置。右拨杆上档或遥控离线时，
从 USB 获取过洞请求及自瞄目标。过洞过程中也按当前控制档选择有效请求。
底盘和云台共用 SB（第 7 通道、`rc.s[0]`）。下档锁定，中档手动，上档自动。

| 阶段 | 小云台 | 大 Yaw |
| --- | --- | --- |
| Align | 小 yaw、pitch 沿轨迹到停放角，roll 保持直立。等待本地关节到位和大 yaw 确认 | 手动回中；导航保持航向，均需机械对齐确认 |
| Fold | roll 限速侧翻，小 yaw、pitch 闭环守住固定停放角 | 手动保持中位；导航保持洞口航向 |
| Hold | 三个关节保持，等待退出请求 | 保持 |
| Unfold | roll 从实际位置回正，小 yaw/pitch 保持原目标 | 保持 |
| Wait | 回正后等请求撤销且 IMU 可用，再按当前控制档恢复手动、扫描或自瞄 | 保持 |
| Fault | 锁定故障，停止输出 | 停止输出 |

进入普通模式时按当前角度重建参考。
过洞期间小云台各轴使用机械角及电机相对速度；导航大 Yaw 使用主板 BMI088 姿态闭环。
调试对象 `gimbal::g_debug.passage_stage` 显示过洞阶段，
`joint_roll_rad/target_roll_rad/roll_torque_nm` 用于观察 roll。

手动进入 `kPassage` 后，主控下发大 Yaw 机械回中轨迹，并在小云台退出过洞前保持中位。
导航进入该模式则保持洞口航向；USB 断流期间保留该参考，底盘停止接收运动目标。
小云台使用进入该流程后收到的大 Yaw 新机械反馈确认回中，确认稳定后才开始折叠。
对齐、放倒或回正超过 `GIMBAL_PASS_PHASE_TIMEOUT_MS` 后进入 Fault。
断联、锁定、电机/CAN 故障会打断流程，不能自动回普通初始化。
过洞故障后，等反馈和控制条件恢复，切到手动云台控制，再将辅助过洞请求先撤销、后重新发出以尝试回正。
导航 mode 3/4 不自动触发故障恢复。
恢复时保留小 yaw/pitch 的实际机械位置，等待大 yaw 回中确认后再回正 roll，随后进入 Wait。
开机检测到 roll 已明显侧翻时也不会直接自动立起，需按上述显式恢复流程处理。

过洞默认 pitch 为机械 0、roll 为单侧 +π/2；方向、限位、速度/加速度、位置/速度容差及稳定时间
均需按机构实测填写。
