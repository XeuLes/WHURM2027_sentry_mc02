# MC02 与 NUC 的 USB 导航接口

`mainControl` 使用 `LFSentryCommon27/PROTOCOL.md` 的 USB CDC 帧格式。帧结构定义在 [usb_typdef.h](RM2027-LFCore/2_Device/USB/usb_typdef.h)。多字节字段按小端编码，长度包含帧头、长度字节、载荷和 CRC8。CRC8 初值为 0，多项式为 `0x07`，覆盖帧尾校验字节之前的全部内容。

| 方向 | 帧 | 总长 | 调度 |
| --- | --- | --- | --- |
| NUC → MC02 | `0xA0` SyncRequest | 7 B | 上位机请求 |
| NUC → MC02 | `0xA1` RoboCmd | 22 B | 上位机 20 Hz |
| NUC → MC02 | `0xA2` ShootCmd | 12 B | 上位机 100 Hz。校验通过后整体写入 LFCore 的 `shoot_cmd`；云台按 200 ms 时效进入自瞄，发射机构按 `shoot_mode` 发弹 |
| MC02 → NUC | `0xB0` SyncResponse | 15 B | 回显 A0 的序号 |
| MC02 → NUC | `0xB1` RawGimbalIMU | 31 B | 250 Hz，新鲜样本 |
| MC02 → NUC | `0xB2` GimbalStatus | 52 B | 100 Hz，有效姿态与关节反馈 |
| MC02 → NUC | `0xB3` NavStatus | 24 B | 20 Hz |
| MC02 → NUC | `0xB4` BigYawHeading | 11 B | 20 Hz，有效 BMI088 样本 |
| MC02 → NUC | `0xC0` CompStage | 6 B | 1 Hz |
| MC02 → NUC | `0xC1` CompEvent | 7 B | 1 Hz，来源裁判 0x0101 |
| MC02 → NUC | `0xC2` AlliedHealth | 19 B | 3 Hz，来源裁判 0x0003 |
| MC02 → NUC | `0xC3` AlliedPos | 35 B | 1 Hz，来源裁判 0x020B |
| MC02 → NUC | `0xC4` RobotStatus | 14 B | 10 Hz，来源裁判 0x0201 和 0x0208 |
| MC02 → NUC | `0xC5` RobotPos | 11 B | 1 Hz，来源裁判 0x0203 |
| MC02 → NUC | `0xC6` RFIDStatus | 8 B | 3 Hz，来源裁判 0x0209 |
| MC02 → NUC | `0xC7` HealthReduction | 4 B | 逐包转发裁判 0x0206 |
| MC02 → NUC | `0xC8` SentryInfo | 17 B | 1 Hz，来源裁判 0x020D |
| MC02 → NUC | `0xC9` EnemyPos | 43 B | 雷达站，暂缓接入 |
| MC02 → NUC | `0xCA` OperatorCmd | 13 B | 逐包转发裁判 0x0303 的纯坐标地图点 |
| MC02 → NUC | `0xCB` EnemyHealth | 13 B | 雷达站，暂缓接入 |
| MC02 → NUC | `0xCC` EnemyBuff | 18 B | 雷达站，暂缓接入 |

MC02 可分多次读取同一帧，也可一次读取多帧。`B0` 使用 TIM2 和 HAL 时基计算 100 µs 计数。`B1/B2` 保留实际样本的本地毫秒时间，再换算成协议刻度。`B1` 使用现有 DM-IMU 的三轴加速度和角速度。`B2` 上报主控融合出的姿态及已到位的关节形态，非导航的参考字段填 0。

`A1.velocity`、`A1.lateral_velocity` 和 `A1.omega` 分别对应底盘系 `vx/vy/wz`。串口桥从一条 `RoboCmd.msg` 取得三轴速度、模式和可选的过洞航向。有效 A1 刷新 100 ms 指令时效。断开 USB 时立即失能底盘命令。

`A2.yaw/pitch` 是电控世界系下期望的云台欧拉角，`A2.shoot_mode` 的 0/1/2 分别表示不发弹、单发、连发。USB 校验档位范围与浮点有效性后整体写入 `ShootCmdState`，并置 `enabled`；断开 USB 时立即清零。小云台在遥控器上档或断遥控时，用 200 ms 内的有效 A2 进入自瞄（`GIMBAL_AIM_TIMEOUT_MS`），超时回到 GAP/扫描；大 Yaw 随之进入 `kAutoAim`/`kFollow`。发射任务在中档或断遥控时按 `shoot_mode` 拉起摩擦轮并选择单发或连发，超过 200 ms 未刷新按不发弹处理；遥控器下档仍可随时强制停机。

B4 上报主板 BMI088 航向及采样时间，NUC 用同时间的 TF 把通行航向换算到主板惯性系，在 A1 模式 0 阶段就开始给大 Yaw 下发目标。NUC 收到新鲜的 `B3=4`，并确认通行航向、底盘与大 Yaw 对齐、实测停车和变形净空后，才发 A1 模式 3。

MC02 锁存模式 3 并上报 `B3=3`，收纳到位后上报 `B3=4`。自动过洞大 Yaw 的机械偏角和惯性航向误差各使用 `0.08 rad` 容差，沿用实测停车和关节到位联锁。NUC 收到本次的阻塞到就绪反馈后，保持模式 3 开始通行。整车出洞且停车后，NUC 发模式 4 清除折叠请求。MC02 上报 `B3=5`，Roll 回正、姿态恢复且退出过洞模式后上报 `B3=4`。CDC 忙时，阻塞反馈保留到提交成功。`B2.transformed` 单独表示最近到位的形态。

`B3` 的三轴实测速度来自副板 CAN3 反馈。缓冲能量和底盘功率上限使用有效的裁判快照，当前无来源的超电剩余能量和实时底盘功率填 0。裁判链路离线时 `C0` 发阶段 0，其余定频裁判状态帧暂停。链路在线时，USB 读取裁判模块已保存的最近值。`C4` 收齐机器人性能和允许发弹量后上报，`C8` 收到过有效哨兵状态后上报。

`C7` 保留裁判扣血事件的原始字节。`CA` 转发有限的原始地图坐标（m），发送的按键和目标机器人 ID 字段均为 0。两帧共用 16 条事件的队列，按接收顺序提交，CDC 忙时重试。队列满时保留已入队的事件，USB 断开时清空。`C9/CB/CC` 当前保留帧号、帧长和入口注释。

USB 的 `A3` 是 NUC 下行的 `RfrCmd`。USART1 的 `0x0301/0x0120` 由原有裁判任务独立发送，命令值仍沿用原分支。当前 USB 对 `A3/A4/A5` 做长度和 CRC8 校验后跳过。`A4` 的 USB 布局为 64 字节，裁判 `0x0307` 的转换仍待按目标端格式核对。

离线回归可运行 `python tests/test_navigation_usb.py` 和 `python tests/test_navigation_passage.py`。实车需核对传感器和机械关节轴向、USB 调度及过洞反馈。
