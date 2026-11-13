# MC02 导航 USB 接口

主控以一条 `RobotCmd` 接收底盘、自瞄和过洞命令。数据包定义在
[usb_typdef.h](RM2027-LFCore/2_Device/USB/usb_typdef.h)。上位机桥接把导航和自瞄输入合成最新的一条命令。
每个消息 ID 对应固定长度。实车桥接需要按此接口实现。

## 公共编码

所有整数小端，float 为 IEEE754 binary32，结构体无对齐填充。平移速度 m/s，角度 rad，角速度 rad/s，
已有枪口 IMU 帧 `0x02` 的单位沿用其定义。

| 偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | uint8 | SOF = `0x5A` |
| 1 | uint8 | length = 总字节数 − 6 |
| 2 | uint8 | 消息 ID |
| 3 | uint8 | 前三字节 CRC8 |
| 4…末尾−2 | bytes | 载荷 |
| 末尾 2 字节 | uint16 | 前面所有字节的 CRC16 |

收发复用 CDC 和现有 CRC8/CRC16 实现。USB 单次接收可以包含半帧或多帧。

## 上位机 → MC02

`RobotCmd` 使用 ID `0x01`，总长 **58 字节**，头为 `5A 34 01 CRC8`。

| 偏移 | 类型 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | 4 bytes | frame_header | 通用帧头 |
| 4 | uint32 | time_stamp | 当前桥接会话内递增的毫秒时间戳 |
| 8/12/16 | float ×3 | data.speed_vector.vx/vy/wz | 唯一一组底盘目标速度，`chassis_link` 坐标系 |
| 20/24/28/32 | float ×4 | data.chassis.roll/pitch/yaw/leg_lenth | RobotCmd 原有字段，全向导航填 0 |
| 36/40 | float ×2 | data.gimbal.pitch/yaw | 枪口 IMU 参考系内的小云台目标角 |
| 44/45 | uint8 ×2 | data.shoot.fire/fric_on | RobotCmd 原有请求字段 |
| 46 | uint8 | data.navigation.mode | 0 普通，1 慢旋转，2 快旋转，3 折叠，4 回正 |
| 47 | uint8 | data.navigation.flags | bit0 底盘/导航使能，bit1 洞口航向有效，bit2 小云台目标有效 |
| 48 | uint32 | data.navigation.session | 非零桥接会话标识，进程重启换新值 |
| 52 | float | data.navigation.heading | 主板 BMI088 惯性参考系中的大 Yaw 绝对航向，±π |
| 56 | uint16 | checksum | CRC16 |

桥接启动先发中性帧建立会话，`mode=0`，`flags=0`，底盘速度和 `fire/fric_on` 为零。
从 `0x21` 看到对应的 session 和命令时间戳回显后，再发送有效命令。
同会话使用时间戳有符号差判断前进，支持 uint32 回绕。重复、倒序、错误长度或 CRC 均不刷新命令时效。

建议每 20 ms 发送最新整帧。导航速度来自 `/nav_executor_omni/cmd_vel` 的完整三轴输出，
形态模式来自 `/nav_executor_omni/chassis_cmd` 的 `mode`。mode 1/2 使用同帧的 `wz`。
导航速度源过期时清 bit0 并填零速度。无有效自瞄数据时清 bit2，发射请求置零。
桥接应分别检查各 ROS 输入的时效，再组合到同一帧。MC02 底盘命令超时为 100 ms。
ET08A 的 SB 中档选择手动，CH5 控制手动过洞。SB 上档选择自动，AUTO 的速度已经在底盘系。
遥控直控的平移量按枪口相对底盘的总偏角转换。

## MC02 → 上位机

导航侧复用现有帧，并读取补充反馈 `NavigationStatus`。

| ID | 帧 | 上位机使用 |
| --- | --- | --- |
| `0x08` | RobotMotion | 22 字节，实测底盘系 vx/vy/wz。使用主控收到该 CAN 反馈的时间戳，有效时发送 |
| `0x0C` | JointState | 18 字节，小 Pitch/Yaw 机械角，使用关节状态更新时间，有效时发送 |
| `0x07` | GameStatus | 13 字节，`game_progress` 和阶段剩余时间，映射至 CompStage |
| `0x0B` | RobotStatus | 读取 `robot_id` 和 `chassis_power_limit`，用于阵营与 NavStatus 功率 |
| `0x21` | NavigationStatus | 39 字节，20 Hz，过洞状态、命令回显、主控惯性航向、大 Yaw 机械角和缓冲能量 |

`NavigationStatus` 头为 `5A 21 21 CRC8`。

| 偏移 | 类型 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | 4 bytes | frame_header | 通用帧头 |
| 4 | uint32 | time_stamp | MCU 打包时间，ms |
| 8 | uint8 | data.transform_state | 执行状态 |
| 9 | uint8 | data.transformed | 最近完成的形态，0 未知、3 折叠、4 直立 |
| 10 | uint8 | data.flags | 遥测有效位 |
| 11/15 | uint32 ×2 | data.session/command_time_stamp | 最近接受的命令回显，未建立会话时为零 |
| 19 | uint32 | data.main_imu.time_stamp | 主控 BMI088 样本时间，ms |
| 23 | float | data.main_imu.yaw | 大 Yaw 上的主板惯性航向，rad |
| 27/31 | uint32/float | data.big_yaw.time_stamp/angle | 大 Yaw 机械角的 CAN 接收时间及角度，ms/rad |
| 35 | uint16 | data.buffer_energy_j | 裁判缓冲能量，J |
| 37 | uint16 | crc | CRC16 |

`flags` 的 bit0/1 分别表示主板航向和大 Yaw 机械角有效，bit2 表示 RobotStatus
的裁判功率上限可用，bit3 表示缓冲能量有效，bit4 表示裁判在线。无效航向、机械角和缓冲字段为零。
裁判在线时使用新鲜的 `0x07` 生成比赛阶段。bit2 有效且 `0x0B` 新鲜时，
可按 RobotStatus 的 ID 范围确定阵营。1…11 为红，101…111 为蓝。
超级电容能量当前由桥接填 0，按导航保守策略处理。

桥接收到 `0x21` 时从其过洞状态与缓冲能量、最新有效的 `0x0B` 功率组合并发布
`/serial_bridge/chassis_status`。按收到的状态顺序保留 BLOCKED → READY。
`0x08` 提供 `/chassis/motion`，可按导航的 20 Hz 节奏发布，每次先处理已收到的状态，
再发布更新的运动样本，避免状态过期或重复运动样本推进控制。源时间戳须映射到 ROS 时钟；
桥接对各帧独立检查时效，状态断流时停止使用缓存 READY。

## 过洞握手

| transform_state | 状态 | 导航处理 |
| --- | --- | --- |
| 0 | Disabled，非 AUTO 或命令失能/超时 | STOPPED |
| 1 | Recovering，初始化或反馈未就绪 | STOPPED |
| 3 | Preparing，停车等待、对齐或折叠中 | BLOCKED |
| 4 | Ready，请求形态已到位 | NORMAL |
| 5 | Restoring，停车等待或回正中 | BLOCKED |
| 6 | Fault，机构故障 | STOPPED |

1. 上位机先发洞口航向。MC02 保持大 Yaw 航向，上位机利用 `map → chassis_link` 和
   `map → big_yaw_link` 转动底盘对齐。
2. 使能且带有效航向的 mode 3 锁存折叠请求。主控下发底盘零速，实测平移 ≤ 0.05 m/s、
   自转 ≤ 0.10 rad/s 才开始机构对齐。
3. 大 Yaw 航向及机械角到位、低速稳定，并收到进入流程后的新反馈，小 Yaw/Pitch 到位后才放倒 Roll。
   折叠完成后上报 `transformed=3, transform_state=4`，放行底盘。
4. 使能的 mode 4 请求回正。先停稳，再从实际位置抬起 Roll，完成后上报
   `transformed=4, transform_state=4`。mode 0/1/2、失能、断流和新会话握手保持已锁存的形态请求。
5. 机构故障时需要检查反馈和机械状态，再按手动流程恢复。

MID360 固定在大 Yaw 上。上位机用其定位姿态与安装外参求 `map → big_yaw_link`，
用大 Yaw 机械角与机构外参求 `big_yaw_link → chassis_link`，组合出底盘定位。
桥接用对应时刻的主板 BMI088 航向把 `/hole_heading` 从 map 系换成主板惯性系。
平地且安装轴已校正时，可用 `wrap(洞口 map 航向 − 当前大 Yaw map 航向 + 当前主板 imu_yaw)`。
`0x02` 是枪口 IMU。主板航向从启动零位积分，上位机需要处理零偏与漂移。

主机回归可运行 `python tests/test_navigation_usb.py` 和 `python tests/test_navigation_passage.py`，
分别检查固定帧解析、命令锁存、反馈和过洞状态机。实车验证还需要按本协议实现桥接并校验 TF。

