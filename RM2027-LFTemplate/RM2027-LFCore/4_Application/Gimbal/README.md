# MC02 大小 yaw 云台

参考旧工程 `application/gimbal/gimbal_big_yaw.c`、
`gimbal_yaw_pitch_direct.c` 的控制方法，接入当前工程的
`DjiMotor`、`Sliding`、`system_state`、板间通信和 FreeRTOS。

## 文件与任务

- `gimbal_config.h`：硬件、机械中值、限位、SMC、扫描和跟随参数宏。
- `app_gimbal_big_yaw.cpp`：底盘板大 yaw 控制状态机和任务入口。
- `app_gimbal_small_yaw.cpp`：云台板小 yaw/pitch 控制状态机和任务入口。
- `gimbal_axis.hpp`：两者共用的 DJI 反馈、机械零位和 SMC 单轴实现。
- `app_gimbal.h`：编译后唯一的 `Gimbal_Init/Control/Task` 接口。
- `main.c`：云台启用时，在 FDCAN 和底盘初始化后调用 `Gimbal_Init()`，注册失败进入
  `Error_Handler()`。禁用时初始化调用和控制实现均被预处理排除。
- `freertos.c`：云台启用时静态创建独立云台任务，以 `vTaskDelayUntil` 按 1 ms 周期运行。
  禁用时不编译任务创建代码、任务栈和控制块。
  底盘 `Control_Task` 保留；云台板不注册/控制四个底盘轮电机。

| `BIG_YAW_ENABLE` / `SMALL_ENABLE`（均带 `GIMBAL_` 前缀） | 本地执行器 | 遥控输入 | 协同输入 |
| --- | --- | --- | --- |
| `1` / `0`（底盘板自动选择） | 大 yaw | 本板接收机 | 云台板的小 yaw 偏角、模式 |
| `0` / `1`（云台板自动选择） | 小 yaw + pitch | 底盘板经板间转发 | 本板 IMU、USB 目标 |
| `0` / `0` | 不编译云台控制与任务 | — | — |

大小 yaw 与板间通信使用同一个 `BOARD_ROLE`，不再手动切换云台开关：

```powershell
mingw32-make board-chassis -j4  # BOARD_ROLE=1，大 yaw，输出 build/board_chassis
mingw32-make board-gimbal -j4   # BOARD_ROLE=2，小 yaw/pitch，输出 build/board_gimbal
```

两个独立 cpp 内部仍通过 `#if` 排除另一套控制实现。
VS Code 的 `STM32H723-Chassis/Gimbal` 配置也通过 `BOARD_ROLE` 自动解析对应云台。
特殊构建仍可显式覆盖两个 `GIMBAL_*_ENABLE` 宏；正常两板构建无需覆盖。
本地调参的 `TUNING_BENCH=1` 仍排除生产云台。仅编译不会烧录。

## 填写宏

硬件信息尚未提供。`gimbal_config.h` 根据 `BOARD_ROLE` 自动开启对应云台；
每轴硬件和控制参数仍需填写。
`GIMBAL_AUTO_AIM_ENABLE` 也默认 `0`，须确认下述上位机目标约定后再开启。

每轴填写 `TYPE/CAN/ID/DIRECTION/RATIO/MID_ECD`。当前适配器使用 DJI
电机接口，默认型号、CAN 口、ID 和参数只是占位示例，并非实车参数；
`TYPE` 可以使用现有 DJI 类型，不能直接替换为 DM/LK 类型。
SMC 参数参考旧工程，未在 MC02 实机整定。

`MID_ECD` 为机构居中时读取的原始编码器计数 `[0,8191]`，
不要直接填旧工程以 rad 表示的 `GIMBAL_*_MID`。
当前驱动的 `angle_rad/total_angle_rad` 相对首次反馈，并非绝对编码器角。
应用首次有效反馈建立机械零位，再用累计角追踪关节，避免在编码器跨零时跳变。
单圈编码器无法确定带减速机构的输出轴绝对圈数；此实现的开机回中定位
适用于直驱云台或已明确输出轴初始圈数的机构。

所有控制角为 rad，角速度为 rad/s。方向由驱动统一应用，应用层不再乘一次。
大/小 yaw 的机构正方向必须一致。`*_OUTPUT_MAX` 和 debug 中的
`*_command_raw` 为 DJI 原始控制量：GM6020 是电压指令，不是 A。
限位是相对各轴机械中值的关节角，须实测填写；大 yaw 默认允许连续旋转，
不适用于存在缠线限制的机构。

## 控制行为

两块板各自的 `Gimbal_Control()` 都按 `observe`（采集 IMU、电机、遥控和协同状态）
→ `handleException`（零输出/故障）→ `selectMode`（选择本板模式）
→ `updateReference`（生成目标）→ `calculateOutput`（闭环与限位）
→ `send`（发送电机帧）→ `publish`（发布状态）的顺序执行。
大小 yaw 的 `selectMode` 和 `updateReference` 分别留在各自文件中：
大 yaw 根据小 yaw 偏角进入 FOLLOW、连续 SCAN，AUTO_AIM 时保持参考；
小 yaw/pitch 根据本板 USB 目标进入 AUTO_AIM，SCAN 时小 yaw 相对大 yaw 居中、pitch 正弦运动。
异常处理会跳过后续目标与闭环计算，但仍发送零输出并发布状态。

使用现有工程的右拨杆 `RC_SW_RIGHT`，右水平/竖直摇杆控制 yaw/pitch。

| 模式 | 行为 |
| --- | --- |
| 下档 / 无效拨杆 / 遥控离线 / 板间离线 / IMU 过期 / 电机离线或故障 | 零输出，取消校准状态 |
| INIT | 机械中值回中；成功后 GAP，超时进入 FAULT，需下档清故障 |
| GAP | 以进入时的姿态为参考，等待协同端就绪 |
| 中档 MANUAL | 积分摇杆速度，IMU 角度/角速度闭环 |
| 上档 SCAN | 大 yaw 连续扫描；小 yaw 相对大 yaw 居中，pitch 正弦扫描 |
| 上档 AUTO_AIM | 小 yaw/pitch 跟踪上位机绝对 IMU 目标；大 yaw 保持进入时姿态 |
| FOLLOW（大 yaw） | 小 yaw 偏角达到进入阈值后，大 yaw 向该方向跟随，退出阈值内停止跟随 |

跟随使用进入/退出双阈值，避免临界点反复切换；与旧代码一样，小 yaw
保持 IMU 瞄准目标，大 yaw 的运动令小 yaw 的机械偏角回到中心。
自瞄跟随阈值仍为进入 `1.2 rad`、退出 `0.05 rad`。扫描时小 yaw 使用机械角
和电机相对角速度闭环，使偏角趋向零；不使用 IMU 绝对角速度阻碍大 yaw 扫描。
模式切换会重置对应坐标系的控制历史；debug yaw 目标统一换算到 IMU 坐标系。
所有模式进入时先重置 SMC 历史并对齐参考，机械限位同时约束目标和向外输出。
校准超时不视为校准成功；CAN 发送失败锁定故障，后续周期尝试发送零指令。
发送失败时无法保证驱动器已收到零帧，应保留电调本身的通信超时保护。

每板的本地 IMU 必须能反映该板所控 yaw 轴的姿态，云台板 IMU 还须反映
pitch。若底盘板 IMU 固定在底盘、不能测量大 yaw 的旋转，不能直接用该
姿态闭环：需要先通过机械运动学合成大 yaw 姿态再接入。
当前使用工程 INS 的轴定义，并由 body q/r 换算 Euler yaw/pitch 角速度；
这不构成任意倾斜下的完整机械解耦，接近 pitch ±90° 时停止控制。
IMU 新鲜度表示 INS 发布周期正常，不代表已验证传感器的硬件健康。

## 板间通信与 USB

板间 `kProtocolVersion` 当前为 **3**：底盘板直接接收遥控器，
在 `ChassisToGimbalPayload` 中转发遥控通道、拨杆和键鼠。
云台板在 `GimbalToChassisPayload` 中发送
`small_yaw_offset_rad/gimbal_mode`。偏角用于阈值跟随，模式用于区分扫描和自瞄；
不额外发送 ready、pitch joint、online 或数据年龄。
小云台 IMU yaw/pitch 留在云台板用于瞄准，不通过板间发送。
底盘接收后更新 FXCore `GimbalState` 的 yaw 关节角、模式和接收状态，
其中 `joint_yaw_rad` 来自小 yaw 偏角；底盘镜像的 `yaw_deg/pitch_deg`
没有远端姿态含义，不可用于瞄准。底盘板直接接收裁判系统，
在 `ChassisToGimbalPayload` 中转发机器人 ID、剩余血量、底盘功率上限和
缓冲能量。云台板从板间通信读取遥控状态，不创建裁判接收任务。
底盘控制和超电控制直接使用本板裁判数据。两板需要同时更新固件，
布局不匹配的固件不会通过长度校验，版本不同的固件不会通过版本校验。
小云台报告 INIT/GAP/FAULT/零力模式时，大 yaw 校准后保持 GAP；
板间中断时两侧均零输出。

`system_state` 增加 IMU/指令更新时间、关节角和模式状态，
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

自瞄沿用 `RobotCmdState.gimbal_yaw/gimbal_pitch`，约定为**本板 IMU
坐标系的绝对角度（默认 rad）**，不采用机械关节角。若上位机使用 deg，
修改 `GIMBAL_AIM_ANGLE_TO_RAD`。yaw 选择距离当前角度最近的等效连续角。
仅新的、长度/CRC 正确且发送时间戳前进的 RobotCmd 帧刷新本地指令时间；
重复轮询旧缓存、其他 USB 包不会让旧瞄准目标保持有效。
发送时间戳按 uint32 毫秒递增并支持回绕；上位机重启后应保持该时间基准，
或同步重启板端接收状态，不能重新从较小时间戳开始而直接继续控制。

现有 RobotCmd 没有单独的 `target_valid` 标志。启用自瞄前需确认上位机
只在有有效目标时持续发送 RobotCmd，且目标坐标系符合上述约定。
若无目标时仍为底盘速度而持续发送该包，应先扩展目标有效标志；不能用
“目标角数值未变化”判断丢失目标。超过 `GIMBAL_AIM_TIMEOUT_MS` 没有新
有效目标指令则进入扫描；遥控离线不会自动切换为自主运行。

每个 `(CAN, TX ID)` 只能有一个周期发送者。初始化会拒绝云台与其他已注册
模块共用发送帧；小 yaw/pitch 自己共用一帧时每周期只发送一次。
本地调参构建 `TUNING_BENCH=1` 会禁用生产云台控制器，避免抢占台架电机。

## 验证范围

已执行两种板角色的 ARM 固件编译/链接，包括默认禁用配置和宏开启的
大 yaw / 小 yaw + pitch + 自瞄配置。另在 `build/gimbal_host_check/`
用真实云台状态机和 SMC、模拟硬件接口验证：模式切换、±π 连续角、
大小 yaw 跟随回差、机械限位、指令过期、板间/IMU 失联、校准超时与
CAN 发送失败。未改动现有 `tests/`，未烧录或驱动实物；反馈轴向、
机械零位、限位及 SMC 参数仍需要实机确认。

## 过洞模式（小 yaw → roll → pitch）

云台板控制小 yaw、roll、pitch，底盘板只协调大 yaw；四轮底盘和发射逻辑不参与此模式。
左拨杆下档请求进入，离开该档请求退出。输入由
`GIMBAL_PASS_SWITCH_INDEX/POSITION` 配置，右下档仍为原来的总锁定。

| 阶段 | 云台板 | 大 yaw |
| --- | --- | --- |
| PassAlign | 小 yaw 到机械中位、pitch 到过洞停放角、roll 保持直立；位置和相对速度稳定后确认 | 捕获进入时 IMU 朝向并保持，不回机械中位 |
| PassFold | 收到本阶段确认后，roll 按速度/加速度约束侧翻 | 保持同一个进入朝向 |
| PassHold | 三个关节机械闭环保持，等待退出请求 | 保持 |
| PassUnfold | 先使 roll 从实际位置回正，小 yaw/pitch 保持原目标 | 保持 |
| PassWait | 继续保持；等待右拨杆在进入本阶段后切到新的中/上档 | 保持 |
| PassFault | 锁定故障、停止输出，不自动立起或切回扫描 | 正常时仍保持原朝向 |

复位期间提前切换右拨杆不会解除 PassWait；退出过洞请求也必须已撤销。
进入普通模式还要求本地 IMU 可用，随后按当前角度重建普通控制参考。
过洞期间使用编码器机械角及电机相对速度，不依赖枪管板的 Euler yaw/pitch。
调试对象 `gimbal::g_debug.joint_control` 表明 yaw/pitch 目标是否为机械关节角；
`joint_roll_rad/target_roll_rad/roll_command_raw` 用于观察 roll。

云台通过原 `gimbal_mode` 报告阶段；底盘到云台仅新增 `big_yaw_hold_mode` 一字节确认。
确认必须来自阶段开始后新收到的报文。两板需同步更新 v3 固件；不发送 roll 姿态或额外 ready/age 字段。
对齐、放倒或回正超过 `GIMBAL_PASS_PHASE_TIMEOUT_MS` 后进入 PassFault。
断联、锁定、电机/CAN 故障会打断流程，不能自动回普通初始化。
故障后先右拨杆下档清故障，再解锁，将过洞请求先撤销、再重新触发，才显式执行 roll 回正；
这次恢复保留当时小 yaw/pitch 的实际机械位置，回正后仍进入 PassWait。
开机检测到 roll 已明显侧翻时也不会直接自动立起，需按上述显式恢复流程处理。

`GIMBAL_ROLL_*` 的默认 DJI GM6020、CAN2、ID4、机械中值及 SMC 均为占位配置。
过洞默认 pitch 为机械 0、roll 为单侧 +π/2；方向、限位、速度/加速度、位置/速度容差及稳定时间
均需按机构实测填写。当前适配器只支持已有 DJI 驱动，其他协议电机需另接驱动。
三个轴共享 DJI 发送帧时由同一个任务每周期只发送一次。

本机检查包含完整流程、中途取消、未对齐/无确认、超时、断联/电机/CAN 故障、明确恢复、
侧翻后 IMU 不适用于瞄准时的机械闭环、大 yaw 跨阶段固定朝向、普通扫描/自瞄及轨迹速度/加速度约束。
这些检查不替代轴向、机械间隙、roll 承载能力和实车运动验证。
