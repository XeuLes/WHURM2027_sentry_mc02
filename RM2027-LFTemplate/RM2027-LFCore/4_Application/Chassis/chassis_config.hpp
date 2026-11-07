/**
 * @file chassis_config.hpp
 * @brief 四轮全向底盘车型参数。
 */

#ifndef CHASSIS_CONFIG_HPP
#define CHASSIS_CONFIG_HPP

#include <stdint.h>

namespace chassis_app {

constexpr uint8_t kWheelCount = 4U;
constexpr uint8_t kForceDof = 3U;

// 逻辑轮序与 ForceSolver/buildOmniDistribution 的矩阵行顺序一致。
constexpr uint8_t kWheelMotorId[kWheelCount] = {2U, 3U, 4U, 1U};
constexpr int8_t kWheelDirection[kWheelCount] = {1, 1, -1, -1};

constexpr uint32_t kChassisTxId = 0x200U;
constexpr uint32_t kOfflineTimeoutMs = 10U;

constexpr float kWheelRadius = 0.050f;
constexpr float kWheelCenterDistance = 0.250f;
constexpr float kWheelTorquePerAmp = 0.150f;
constexpr float kWheelTorqueMax = 2.500f;
constexpr float kC620MaxCurrent = 20.0f;

constexpr float kFallbackChassisPowerLimitW = 60.0f;
constexpr float kPowerSafetyMargin = 0.90f;

constexpr float kMaxVx = 1.0f;
constexpr float kMaxVy = 1.0f;
constexpr float kMaxWz = 2.0f;
constexpr float kMaxFx = 70.0f;
constexpr float kMaxFy = 70.0f;
constexpr float kMaxMz = 12.0f;

constexpr float kRcDeadzone = 0.05f;
constexpr float kRcXSign = 1.0f;
constexpr float kRcYSign = -1.0f;
constexpr float kRcWzSign = -1.0f;

constexpr float kVelocityFilterAlpha = 0.75f;
constexpr float kForceVelocityFilterAlpha = 0.8f;
constexpr float kControlPeriodS = 0.001f;
constexpr bool kEnableMotionTargetCompensation = true;
constexpr uint32_t kImuDataTimeoutMs = 10U;
constexpr uint32_t kAutoCommandTimeoutMs = 100U;

// 基于 IMU 预测速度的逐轮打滑检测参数。
constexpr bool kEnableSlipDetection = true;
constexpr float kSlipMinSpeedMps = 0.05f;
constexpr float kSlipScoreEnter = 0.35f;
constexpr float kSlipScoreExit = 0.15f;
constexpr float kSlipScoreFilterAlpha = 0.60f;
constexpr float kSlipEnterHoldS = 0.005f;
constexpr float kSlipExitHoldS = 0.020f;

// 坡道重力前馈。整车质量标定前保持为 0，算法输出补偿力为零。
constexpr bool kEnableGravityCompensation = true;
constexpr float kGravityCompensationMassKg = 0.0f;
constexpr float kGravityMps2 = 9.718f;
constexpr float kGravityScaleX = 1.0f;
constexpr float kGravityScaleY = 1.0f;
constexpr float kGravityFilterAlpha = 0.95f;
constexpr float kGravitySlewRateNps = 200.0f;
constexpr float kGravityMaxForceN = kMaxFx;
constexpr float kGravityMaxTiltDeg = 35.0f;

constexpr float kPidVxKp = 25.0f;
constexpr float kPidVxKi = 5.0f;
constexpr float kPidVyKp = 25.0f;
constexpr float kPidVyKi = 5.0f;
constexpr float kPidWzKp = 4.0f;
constexpr float kPidWzKi = 0.5f;

// ForceSolver 使用轮输出轴坐标，ratio=1 时 getOut() 直接得到目标电流 A。
constexpr float kForceReductionRatio = 1.0f;
constexpr int8_t kForceDirection = 1;
constexpr float kForceInertia = 0.0f;
constexpr float kForceViscousFriction = 0.0f;
constexpr float kForceCoulombFriction = 0.0f;
constexpr float kForceGravityAmplitude = 0.0f;
constexpr float kForceGravityOffset = 0.0f;

} // namespace chassis_app

#endif // CHASSIS_CONFIG_HPP
