#pragma once

// 两块板均设为 1 后重新编译。默认关闭，待确认接线、拨杆和机械参数。
#ifndef SHOOT_ENABLE
#define SHOOT_ENABLE 1
#endif

#include "fdcan.h"
#include "remote_control.h"

namespace shoot_config {

// FDCAN2 / 0x200，槽位依次为拨弹盘、右轮、左轮、第三轮。
inline FDCAN_HandleTypeDef* motorBus() { return &hfdcan2; }
constexpr uint8_t kTriggerId = 1U;
constexpr uint8_t kFricIds[] = {2U, 3U, 4U};
// 前两轮沿用参考转向，第三轮暂取正向，按实物调整。
constexpr int8_t kFricDirections[] = {1, 1, -1};
constexpr int8_t kTriggerDirection = 1; // 沿用 SENTRY 的负向进弹
constexpr float kFricReduction = 1.0f;
// 拨弹盘直连 P36 输出轴，输出轴角度和转速就是拨弹盘的角度和转速。
constexpr float kTriggerReduction = 36.0f;
constexpr uint8_t kBulletNum = 8U; // 沿用 SENTRY 的拨弹盘槽数

// ET08A CH6 独立拨杆。CH7/CH8 已供整车使能和底盘模式使用。
#ifndef SHOOT_MODE_CHANNEL
#if RC_TYPE == RC_ET08A
#define SHOOT_MODE_CHANNEL 3U
#else
#define SHOOT_MODE_CHANNEL RC_SW_LEFT
#endif
#endif
constexpr uint8_t kModeChannel = SHOOT_MODE_CHANNEL;

constexpr uint32_t kInitMs = 201U;
constexpr uint32_t kControlMs = 1U;
constexpr float kControlSeconds = kControlMs * 0.001f;
constexpr uint32_t kAimLostMs = 3000U;
constexpr uint32_t kRobotCmdTimeoutMs = 200U;
constexpr uint32_t kRefereeTimeoutMs = 300U;
constexpr uint32_t kBlockTimeMs = 1000U;
constexpr uint32_t kReverseTimeMs = 1250U;

// 速度均为机构正方向的 rad/s。摩擦轮物理转向由上面的电机方向配置。
constexpr float kFricSpeedRadS[] = {1000.0f, 1000.0f, 1000.0f};
constexpr float kFricSpeedLimitRadS = 800.0f;
constexpr float kFricSpeedErrorRadS = 20.0f;
// 参考代码的拨弹盘速度环使用转子速度。现用 P36 输出轴反馈。
constexpr float kBlockTriggerSpeedRadS = 5.0f / kTriggerReduction;
constexpr float kReverseSpeedRadS = 20.0f / kTriggerReduction;
constexpr float kTriggerSpeedRadS = -350.0f / kTriggerReduction;
constexpr float kSingleAngleToleranceRad = 0.01f;

// 实测参数按转子 rpm 标定，按各自减速比换算到输出轴 rad/s。
constexpr float kRotorRpmPerRadS = 9.549296586f;
constexpr float kFricRpmPerRadS = kFricReduction * kRotorRpmPerRadS;
constexpr float kTriggerRpmPerRadS = kTriggerReduction * kRotorRpmPerRadS;
constexpr float kFricSlewRadS2 = 80000.0f / kFricRpmPerRadS;
constexpr float kTriggerSlewRadS2 = 80000.0f / kTriggerRpmPerRadS;
constexpr float kSmcSatLimit = 1.0f;

} // namespace shoot_config

// 参数顺序：J, K, c, c1, c2, p, q, beta, epsilon。
#define SHOOT_FRIC_SMC_PARAMS \
    {0.05f * shoot_config::kFricRpmPerRadS, 100.0f, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
     0.1f / shoot_config::kFricRpmPerRadS}
#define SHOOT_TRIGGER_SMC_PARAMS \
    {0.05f * shoot_config::kTriggerRpmPerRadS, 100.0f, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
     0.1f / shoot_config::kTriggerRpmPerRadS}
