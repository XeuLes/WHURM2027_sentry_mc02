# subControl 大 Yaw 执行器

本分支的模式仲裁、目标与过洞轨迹由 mainControl 完成。副板通过 CAN3 接收目标，
使用现有 DmAxis/SMC 输出力矩；小云台与发射不在本固件运行。

完整接线、协议、偏角语义、构建与验证边界见 [SUBCONTROL.md](../../../SUBCONTROL.md)。

- `app_gimbal_big_yaw.cpp`：读取远端参考，握手、闭环、失效处理及反馈发布。
- `gimbal_axis.hpp`：沿用 main 的电机反馈、DM 使能和单轴 SMC。
- `gimbal_config.h`：沿用 main 的 CAN 口、ID、方向、机械中位、SMC 与力矩限幅。
- `app_gimbal_small_yaw.cpp`：保留参考，不编入副板。

普通模式使用主控 BMI088 航向/角速度；机械模式使用本地 DM 位置/速度和主控轨迹。
机械零位由 `GIMBAL_BIG_YAW_MID_RAD` 定义，不等同于 IMU 航向零位。
大 Yaw 的反馈量是相对底盘的机械偏角，ready 表示电机使能握手完成。
即使目标无效或失能，副板也继续发送 0x319 心跳；电机无有效反馈时清除有效位。

软件默认参数不能代替实车机械零位、正方向和 SMC 整定。当前只完成编译与主机测试，未烧录联调。
