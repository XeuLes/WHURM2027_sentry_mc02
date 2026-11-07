# 底盘坡道重力前馈

该模块把底盘姿态投影得到的重力分量转换为车体坐标系 `Fx/Fy` 前馈力：

```text
gravity_x = g * sin(pitch)
gravity_y = -g * cos(pitch) * sin(roll)
force_x = -mass * gravity_x * scale_x
force_y = -mass * gravity_y * scale_y
```

坐标约定为 `x` 向前、`y` 向左、`z` 向上。实车必须核对 Roll/Pitch
正负号与轮速旋转方向是否一致。

## 安全处理

- IMU 姿态无效时立即输出零并清空滤波状态。
- 超过最大倾角时输出零。
- 前馈力限制到 `max_force_n`。
- 前馈力按 `slew_rate_nps` 缓慢建立，避免上下坡切换时产生电流冲击。
- `mass_kg == 0` 时仍计算重力投影供标定，但实际补偿力保持为零。

## 当前底盘配置

`chassis_config.hpp` 中当前质量为零，因此：

- `gravity_accel_x/y` 可用于调试观察；
- `gravity_force_x/y` 保持为零；
- `Estimate` 或底盘反馈中的 `gravity_applied` 为 `false`。

标定质量、方向和比例后，再启用非零补偿。
