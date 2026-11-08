# MC02 大小 yaw 云台

参考旧工程 `application/gimbal/gimbal_big_yaw.c`、
`gimbal_yaw_pitch_direct.c` 的控制方法，接入当前工程的
`DjiMotor`、`DmMotor`、`Sliding`、`system_state`、板间通信和 FreeRTOS。

## 文件与任务

- `gimbal_config.h`：硬件、机械中值、限位、SMC、扫描和跟随参数宏。
- `app_gimbal_big_yaw.cpp`：底盘板大 yaw 控制状态机和任务入口。
- `app_gimbal_small_yaw.cpp`：云台板小 yaw、roll、pitch 控制状态机和任务入口。
- `gimbal_axis.hpp`：DJI、DM 反馈和共用的单轴 SMC。
- `app_gimbal.h`：编译后唯一的 `Gimbal_Init/Control/Task` 接口。
- `main.c`：在 FDCAN 和底盘初始化后调用 `Gimbal_Init()`，注册失败进入
  `Error_Handler()`。
- `freertos.c`：静态创建独立云台任务，以 `vTaskDelayUntil` 按 1 ms 周期运行。

| `BOARD_ROLE` | 本地执行器 | 遥控输入 | 协同输入 |
| --- | --- | --- | --- |
| `1`（底盘板） | 大 yaw | 本板接收机 | 云台板的小 yaw 偏角、模式 |
| `2`（云台板） | 小 yaw + roll + pitch | 底盘板经板间转发 | 本板 IMU、USB 目标 |

大小 yaw 与板间通信使用同一个 `BOARD_ROLE`，不再手动切换云台开关：

```powershell
mingw32-make board-chassis -j4  # BOARD_ROLE=1，大 yaw，输出 build/board_chassis
mingw32-make board-gimbal -j4   # BOARD_ROLE=2，小 yaw/pitch，输出 build/board_gimbal
```

两个独立 cpp 内部仍通过 `#if` 排除另一套控制实现。
VS Code 的 `STM32H723-Chassis/Gimbal` 配置也通过 `BOARD_ROLE` 自动解析对应云台。
本地调参构建使用自己的启动代码和任务，并从源文件列表排除生产云台。

## 填写宏

正式固件启动时初始化对应云台。烧录前填写 `gimbal_config.h` 中的实车配置。

大 yaw 为 DM4310，默认底盘板 CAN2。小 yaw 为直驱 GM6020，roll 和 pitch
为 DM4310，三者共用云台板 CAN1。CAN 口、ID、方向、中位、限位及 SMC 参数均待实测。

`GIMBAL_YAW_MID_ECD` 填小 yaw 居中时的原始编码器计数 `[0,8191]`，
关节角为当前计数减中值后折算的单圈有符号偏角。`*_MID_RAD` 填 DM
在相应机械中位时报告的原始输出轴角度，取安装方向修正前的数值。
过洞时大 yaw 保持进入时的机械角。用于关节闭环的实际行程须完整落在
DM 配置的反馈位置范围内。pitch 和 roll 的机械限位须在中位两侧半圈以内。

三台 DM 用调试助手配置 **MIT**，CAN ID、Master ID 和 P/V/T 编码范围要与宏一致。
MC02 的 SMC 输出力矩，MIT 帧的 `kp/kd` 为零。`*_OUTPUT_MAX_NM` 为软件力矩限幅，
`GIMBAL_DM_TORQUE_MAX_NM` 是协议编码范围。DM4310-2EC 有输出轴单圈绝对编码器，
驱动器已经将位置反馈折算到输出轴。10∶1 是电机内部减速比，MC02 按外部传动比 1 处理。
GM6020 的 `GIMBAL_YAW_OUTPUT_MAX` 为原始电压指令限幅。所有控制角为 rad，
角速度为 rad/s。大/小 yaw 的机构正方向必须一致。大 yaw 的扫描要求机构允许连续旋转。

## 控制行为

两块板各自的 `Gimbal_Control()` 采集反馈、处理模式和异常、生成目标、计算输出、发送并发布状态。
大 yaw 先由 `selectMode` 选出目标模式，再由 `handleException` 处理失能、反馈故障及 DM 使能确认。
小 yaw/pitch 根据本板 USB 目标进入 AUTO_AIM。SCAN 时小 yaw 相对大 yaw 居中、pitch 正弦运动。
异常处理会跳过目标与闭环计算。GM6020 发送零指令，DM 发送失能命令并读取驱动状态。

遥控在线时，右拨杆下档锁定、中档手动调试、上档自动。遥控离线时，收到
第一帧 RobotCmd 后进入自主控制，有有效目标时跟踪，其余普通运行时间扫描。
大 yaw 中档手动由本地拨杆选择。离线和上档共用自动分支，依据小云台实际运行状态选择
GAP、SCAN 或自瞄及阈值跟随。进入扫描前先经 GAP 捕获当前 IMU 朝向。

| 模式 | 行为 |
| --- | --- |
| 零力 | 遥控下档、板间断联或反馈故障时停止输出 |
| INIT | 云台板按机械角将小 yaw 和 pitch 回中；成功后 GAP，超时进入 FAULT，需下档清故障。大 yaw 解锁后直接捕获当前 IMU 朝向 |
| GAP | 以进入时的姿态为参考，等待协同端就绪 |
| 中档 MANUAL | 右水平摇杆驱动大 yaw，小 yaw 机械居中；右竖直摇杆控制 pitch |
| SCAN | 大 yaw 连续扫描；小 yaw 相对大 yaw 居中，pitch 正弦扫描 |
| AUTO_AIM | 小 yaw/pitch 跟踪 USB 下发的 IMU 目标角；大 yaw 保持进入时姿态 |
| FOLLOW（大 yaw） | 小 yaw 偏角达到进入阈值后，大 yaw 向该方向跟随，退出阈值内停止跟随 |

跟随使用进入/退出双阈值，避免临界点反复切换；与旧代码一样，小 yaw
保持 IMU 瞄准目标，大 yaw 的运动令小 yaw 的机械偏角回到中心。
自瞄跟随阈值为进入 `1.2 rad`、退出 `0.05 rad`。两个 yaw 的 IMU 闭环
使用单圈角，位置误差和相邻参考的差值都取最短圆周差。大 yaw 的普通模式从切换时的
IMU 朝向起算，移动参考领先当前朝向最多 `GIMBAL_BIG_YAW_MAX_LEAD_RAD`。
小 yaw 的回中、手动、扫描和过洞都使用机械角及电机相对速度。
模式切换重置 SMC 历史，机械限位同时约束关节目标和向外输出。
CAN 发送失败锁定故障。需在调试助手配置驱动器自身的通信超时保护。

每板的本地 IMU 必须能反映该板所控 yaw 轴的姿态，云台板 IMU 还须反映
pitch。若底盘板 IMU 固定在底盘、不能测量大 yaw 的旋转，不能直接用该
姿态闭环：需要先通过机械运动学合成大 yaw 姿态再接入。
当前使用工程 INS 的轴定义，并由 body q/r 换算 Euler yaw/pitch 角速度；
这不构成任意倾斜下的完整机械解耦，接近 pitch ±90° 时停止控制。
IMU 新鲜度表示 INS 发布周期正常，不代表已验证传感器的硬件健康。

## 板间通信与 USB

底盘板直接接收遥控器，
在 `ChassisToGimbalPayload` 中转发遥控通道、拨杆和键鼠。
云台板在 `GimbalToChassisPayload` 中发送
`small_yaw_offset_rad/gimbal_mode`。偏角用于阈值跟随，模式用于区分扫描和自瞄；
同帧的 `gimbal_cmd/gimbal_cmd_tick_ms` 转发 RobotCmd 的过洞请求和云台板接收时刻。
两个板的本地时钟独立，底盘板使用板间接收时间判断链路是否在线。
不额外发送 ready、pitch joint、online 或数据年龄。
小云台 IMU yaw/pitch 留在云台板用于瞄准，不通过板间发送。
底盘接收后更新 FXCore `GimbalState` 的 yaw 关节角、模式和接收状态，
其中 `joint_yaw_rad` 来自小 yaw 偏角；底盘镜像的 `yaw_deg/pitch_deg`
没有远端姿态含义，不可用于瞄准。底盘板直接接收裁判系统，
在 `ChassisToGimbalPayload` 中转发机器人 ID、剩余血量、底盘功率上限和
缓冲能量。云台板从板间通信读取遥控状态，不创建裁判接收任务。
底盘控制和超电控制直接使用本板裁判数据。两板需要同时更新固件，
布局不匹配的固件不会通过长度校验，版本不同的固件不会通过版本校验。
自动分支收到小云台的 INIT/GAP/FAULT 状态时，大 yaw 保持当前 IMU 朝向，
收到零力状态时停止输出。板间中断时大 yaw 停止输出。

云台使用当前公共接口的 `ImuState.last_update_ms` 和
`RobotCmdState.last_update_ms` 检查数据新鲜度；时间戳为 0 表示尚未发布有效数据。
本地状态发布接入代码在收到新 IMU/指令时记录本板时间，不依赖旧的 `valid/updated_at_ms` 字段。
`system_state` 保存关节角和模式状态，
`gimbal::g_debug` 可在调试器观察反馈、参考、原始命令、模式及发送失败计数。
大 yaw 的电机反馈、控制状态和 debug 留在本地，不写入 FXCore `GimbalState`。
`GimbalState` 表示小 yaw/pitch 瞄准组件：IMU yaw 为姿态角，
`joint_yaw_rad` 为小 yaw 相对大 yaw 的机械偏角。底盘只保存小 yaw 偏角与模式，
不镜像 pitch 或瞄准姿态，也不根据接收报文推定 ready。
接收镜像保留最后一帧，不额外修改其 online/ready；判断通信是否在线使用已有
`isPeerOnline()`。大小 yaw 控制器仍按该通信状态处理断联。
云台收到的大 yaw IMU 和裁判功率字段保存在 `getRemoteChassisData()` 中：
前者不能覆盖小云台的本地 IMU，后者目前没有对应的 FXCore 功率状态字段。
云台板通过现有 USB `JointState` 包发送真实小 yaw/pitch 关节角，单位 rad；
无有效反馈时不发送伪造的零关节角。

USB RobotCmd 的 yaw/pitch 与 SENTRY_MAIN 一致，为**云台 IMU 坐标系下的绝对目标角（rad）**。
板端直接写入 `RobotCmdState.gimbal_yaw/gimbal_pitch`。
闭环用当前 IMU 姿态计算误差，yaw 取最短圆周差，pitch 按机械行程限幅。
仅新的、长度/CRC 正确且发送时间戳前进的 RobotCmd 帧刷新本地指令时间；
重复轮询旧缓存、其他 USB 包不会让旧瞄准目标保持有效。
发送时间戳按 uint32 毫秒递增并支持回绕；上位机重启后应保持该时间基准，
或同步重启板端接收状态，不能重新从较小时间戳开始而直接继续控制。

RobotCmd 继续使用 ID `0x01` 和现有的 `uint32_t time_stamp`。新版在发射字段
之后、CRC16 之前追加一个字节 `gimbal_cmd`，总长 49 字节，头部长度字段填 43。
原 48 字节包按 `gimbal_cmd=0` 处理。CRC8 和 CRC16 按实际包长度重新计算。

| `gimbal_cmd` | 云台行为 |
| --- | --- |
| 0 | 普通控制。过洞中收到时先回正，再根据目标状态进入扫描或自瞄 |
| 1 | 对齐后进入过洞，保持侧翻直到撤销请求 |

普通控制沿用 SENTRY 的目标变化判据。收到有效的 yaw/pitch 目标变化后进入自瞄。
目标连续 `GIMBAL_AIM_LOST_MS` 未变化，或 RobotCmd 超过 `GIMBAL_AIM_TIMEOUT_MS`
未更新时，经 GAP 转入扫描。

云台任务统一发送 GM6020 组帧和各台 DM 的独立 MIT 帧。
初始化检查发送帧归属及同一 CAN 总线上 DM 的 ID 冲突。

## 验证范围

实机调试先确认反馈轴向、机械零位和全行程间隙，再整定 SMC 和力矩限幅。

## 过洞模式（小 yaw → roll → pitch）

云台进入 `Mode::kPassage` 后，由云台板的 `PassageStage` 状态机控制小 yaw、roll、pitch。
大 yaw 进入时记录 DM4310 输出轴的机械角，过洞期间使用该角度和电机相对角速度闭环，
使大 yaw 与底盘的相对角度保持固定。退出后以当前 IMU 朝向建立普通模式的参考。
右拨杆在中档时，ET08A 第 5 通道（`rc.s[2]`）的辅助开关控制过洞。
上档等效于过洞请求 `1`，中档或下档等效于 `0`。辅助开关由
`GIMBAL_PASS_SWITCH_INDEX/POSITION` 配置。右拨杆上档或遥控离线时，
从 USB 获取过洞请求及自瞄目标。过洞过程中也按当前控制档选择有效请求。
SB（第 7 通道、`rc.s[0]`）下档仍为总锁定；SC（第 8 通道、`rc.s[1]`）继续选择底盘模式。
板间协议已经转发四个开关槽位，此调整不改变报文布局。

| 阶段 | 云台板 | 大 yaw |
| --- | --- | --- |
| Align | 小 yaw 回中，pitch 到停放角，roll 保持直立。等待关节到位和大 yaw 确认 | 捕获进入时机械角并保持 |
| Fold | roll 限速侧翻 | 保持 |
| Hold | 三个关节保持，等待退出请求 | 保持 |
| Unfold | roll 从实际位置回正，小 yaw/pitch 保持原目标 | 保持 |
| Wait | 回正后等请求撤销且 IMU 可用，再按当前控制档恢复手动、扫描或自瞄 | 保持 |
| Fault | 锁定故障，停止输出 | 大 yaw 条件正常时保持原机械角 |

进入普通模式时按当前角度重建参考。
过洞期间各轴使用机械角及电机相对速度。
调试对象 `gimbal::g_debug.passage_stage` 显示过洞阶段，
`joint_roll_rad/target_roll_rad/roll_torque_nm` 用于观察 roll。

云台通过原 `gimbal_mode` 报告 `kPassage`，大 yaw 使用原 `big_yaw_hold_mode` 确认保持。
确认必须来自进入过洞后新收到的报文。两板需同时支持新的过洞模式值。
对齐、放倒或回正超过 `GIMBAL_PASS_PHASE_TIMEOUT_MS` 后进入 Fault。
断联、锁定、电机/CAN 故障会打断流程，不能自动回普通初始化。
过洞故障后，等反馈和控制条件恢复，再将当前过洞请求先撤销、后重新发出以尝试回正。
恢复时保留小 yaw/pitch 的实际机械位置，回正后进入 Wait。
开机检测到 roll 已明显侧翻时也不会直接自动立起，需按上述显式恢复流程处理。

过洞默认 pitch 为机械 0、roll 为单侧 +π/2；方向、限位、速度/加速度、位置/速度容差及稳定时间
均需按机构实测填写。

## 与合并后底盘的安装约束

2026-11-05：本次适配已通过 `BOARD_ROLE=1/2` 的 ARM 完整编译和 ELF 链接，
产物及日志位于 `build/gimbal_compat_chassis*`、`build/gimbal_compat_small*`。
主机模拟通过手动/扫描/自瞄超时、IMU 未发布/过期/时间戳回绕、独立过洞开关、
过洞完成/取消/无确认/未对齐/断联/CAN 发送失败/电机掉线和大 yaw 保持检查。
模拟使用真实云台状态机和 SMC，硬件及板间输入由桩替代；未烧录、未做实机验证。

本次兼容适配保留远程底盘、INS、运动补偿和公共状态结构定义。
当前大 yaw 控制要求本板 IMU 反映大 yaw 姿态，而新底盘运动补偿直接把本板
IMU 角速度和线性加速度作为底盘数据。IMU 随大 yaw 转动时，这两个坐标系并不相同；
仅修改云台时间戳和开关不能解决该问题。大 yaw 单独转动会污染打滑判断，
进而可能使底盘停车。需要另行确认 IMU 安装和底盘运动数据来源，在底盘侧接入
独立 IMU 或经过关节运动学转换的数据；本次没有修改该底盘算法。
