# 全向底盘运动补偿算法

本模块实现《全向底盘运动补偿》文档中的三段处理，并增加了基于预测速度的打滑检测：

1. 四轮轮速逆解，得到车体 `vx/vy`。
2. 使用 `[vx, ax, vy, ay]` 恒加速度卡尔曼滤波融合轮速和 IMU 加速度。
3. 可选的 `Kp = K * |yaw_rate|` 目标速度补偿。
4. 在轮速校正前，用预测 `vx/vy`、IMU 角速度和轮半径计算逐轮打滑残差。

## 输入要求

- 四个轮速为逻辑轮序 1~4 的轮缘线速度，单位 `m/s`。
- 轮位图至少有三轮有效才会进行轮速逆解。
- Yaw 角必须是期望/云台坐标系相对底盘坐标系的相对角，单位 `rad`。
- 加速度必须已经转到**底盘平面坐标系**并去除重力，单位 `m/s²`。
- `valid_wheel_mask` 只表示轮速是否可用于逆解，不表示轮地是否打滑。
- 打滑检测还需要有效的 IMU 角速度、正数轮半径和至少三个有效轮速。
- 底盘使用融合后的 `vx/vy` 作为速度环反馈，并应用受限的旋转目标补偿。
- IMU 加速度无效或融合尚未建立时，底盘自动退回轮速一阶低通。

## 使用示例

```cpp
#include "alg_motion_compensation.hpp"

static motion_compensation::OmniMotionCompensator compensator;

void Init()
{
    compensator.init();  // 默认参数仅用于初步调试
}

void Update1ms()
{
    motion_compensation::Measurement measurement {};
    measurement.wheel_linear_velocity_mps[0] = wheel_linear[0];
    measurement.wheel_linear_velocity_mps[1] = wheel_linear[1];
    measurement.wheel_linear_velocity_mps[2] = wheel_linear[2];
    measurement.wheel_linear_velocity_mps[3] = wheel_linear[3];
    measurement.valid_wheel_mask = motion_compensation::kAllWheelsValidMask;

    measurement.yaw_rad = relative_yaw_rad;
    measurement.yaw_rate_rad_s = relative_yaw_rate_rad_s;
    measurement.yaw_rate_valid = imu_yaw_rate_valid;
    measurement.wheel_radius_m = wheel_radius;
    measurement.acceleration_x_mps2 = chassis_ax_mps2;
    measurement.acceleration_y_mps2 = chassis_ay_mps2;
    measurement.acceleration_valid = imu_acceleration_valid;
    measurement.dt_s = 0.001f;

    (void)compensator.update(measurement);

    const auto estimate = compensator.estimate();
    const bool slipping = estimate.slip_detected;
    const auto wheel_slip_mask = estimate.slip_wheel_mask;
    const auto corrected = compensator.compensate(target_vx_mps, target_vy_mps);
    target_vx_mps = corrected.vx_mps;
    target_vy_mps = corrected.vy_mps;
}
```

## 接入底盘前的验证

- 确认四个轮速顺序与底盘逆解矩阵一致。
- 确认 Yaw 角的正负和零位。
- 确认 BMI088 三轴方向；`app_ins` 会先在 g 域去除重力，再换算为 `m/s²`
  发布给底盘，换板或改变 IMU 安装方向后必须重新核对正负号。
- 用实车分别测试仅轮速、轮速加加速度、再加入目标补偿三组效果。
- 打滑检测与传感器有效性解耦：不要仅因为疑似打滑就把轮子从 `valid_wheel_mask` 中移除。

## 打滑检测

每个控制周期先根据 IMU 加速度和上一状态得到不带轮速校正的预测速度，
再使用下式计算逐轮残差：

```text
expected_i = row_x_i * predicted_vx
           + row_y_i * predicted_vy
           - wheel_radius * yaw_rate
residual_i = wheel_linear_velocity_i - expected_i
score_i = abs(residual_i) / max(abs(actual_i), abs(expected_i), min_speed)
```

打滑分数经过一阶低通、进入/退出双阈值和持续时间确认后，写入
`Estimate::slip_score`、`slip_wheel_mask` 和 `slip_detected`。

- `valid_wheel_mask` 仍只表示轮速传感器是否可用。
- `slip_wheel_mask` 独立表示通过预测残差确认的打滑轮。
- 打滑轮会降低 `slip_confidence`，并使用置信度加权的最小二乘重新解算 `vx/vy`。
- 至少保留三个高置信轮时才更新轮速速度；否则短时使用 IMU 预测并标记轮速测量无效。
- 当前底盘安全层会在轮速速度测量无效时停车，因此两轮及以上持续打滑会触发保守停车。
- 当前不修改四轮力矩分配，也不把打滑轮强制判为离线。
- 检测要求 IMU 加速度和角速度均为本周期有效数据，避免使用陈旧预测。
- 实车启用前必须核对 IMU 角速度正负与轮速旋转项符号一致。

