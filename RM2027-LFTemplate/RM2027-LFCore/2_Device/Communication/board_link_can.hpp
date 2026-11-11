#pragma once

#include <stdint.h>

namespace board_link {
namespace can3 {

// 1 Mbit/s, 标准经典 CAN 数据帧。与枪口 DM-IMU (0x01/0x11) 共线。
constexpr uint32_t kBitrate = 1000000U;
// 标准数据帧，含帧间隔及从 SOF 到 CRC 序列的最坏位填充。
constexpr uint32_t maxFrameBits(uint32_t bytes) {
    return 47U + 8U * bytes + (33U + 8U * bytes) / 4U;
}
constexpr uint32_t kMaxFrameBits = maxFrameBits(8U);
constexpr uint16_t kChassisTargetId = 0x310U;
constexpr uint16_t kBigYawTargetId = 0x311U;
constexpr uint16_t kGimbalOffsetId = 0x312U;
constexpr uint16_t kRefereePowerId = 0x313U;
constexpr uint16_t kChassisFeedbackId = 0x318U;
constexpr uint16_t kBigYawFeedbackId = 0x319U;

// 每个状态字的高半字节为协议版本，低半字节含有效位或模式。
constexpr uint8_t kVersion = 0x20U;
constexpr uint8_t kVersionMask = 0xF0U;
constexpr float kAngleScale = 10000.0f; // rad
constexpr float kSpeedScale = 1000.0f; // m/s 或 rad/s
constexpr float kYawSpeedScale = 500.0f; // 大 Yaw 角速度，rad/s
constexpr float kAccelerationScale = 100.0f; // rad/s²
constexpr uint32_t kAgeUnitMs = 10U; // 向上取整，255 表示饱和
constexpr uint32_t kCommandPeriodMs = 1U; // 底盘、大 Yaw 和裁判数据
constexpr uint32_t kBigYawFeedbackPeriodMs = 2U;
constexpr uint32_t kChassisFeedbackPeriodMs = 10U; // 与 USB RobotMotion 一致
constexpr uint32_t kChassisFeedbackTimeoutMs = 50U;
constexpr uint32_t kChassisFeedbackSourceTimeoutMs = 10U;
constexpr uint32_t kGimbalOffsetPeriodMs = 10U;
constexpr uint32_t kBigYawTimeoutMs = 20U;
constexpr uint32_t kChassisTimeoutMs = 50U;
constexpr float kBigYawSlowSpeedRadS = 0.05f;

constexpr uint8_t kChassisModeMask = 0x03U;
constexpr uint8_t kChassisTargetValid = 0x04U;
constexpr uint8_t kChassisFeedbackValid = 0x01U;
constexpr uint8_t kBigYawEnable = 0x01U;
constexpr uint8_t kBigYawTargetValid = 0x02U;
constexpr uint8_t kBigYawJointFrame = 0x04U;
constexpr uint8_t kBigYawFeedbackValid = 0x01U;
constexpr uint8_t kBigYawFeedbackReady = 0x02U;
constexpr uint8_t kBigYawFeedbackSlow = 0x04U;
constexpr uint8_t kBigYawOffsetValid = 0x01U;
constexpr uint8_t kGimbalOffsetValid = 0x02U;
constexpr uint8_t kRefereeOnline = 0x01U;
constexpr uint8_t kPowerReceived = 0x02U;
constexpr uint8_t kBufferReceived = 0x04U;

// 全部多字节字段使用小端。副板以本地收帧时间判断目标超时并失能。
struct __attribute__((packed)) ChassisTargetFrame {
    int16_t vx;
    int16_t vy;
    int16_t wz;
    uint8_t flags;
    uint8_t sequence;
};

struct __attribute__((packed)) BigYawTargetFrame {
    int16_t target;
    int16_t position_or_acceleration; // BMI088 航向；机械模式下为轨迹目标加速度
    int16_t velocity; // BMI088 航向速度；机械模式下为轨迹目标角速度
    uint8_t flags; // 绝对航向或机械角；使能请求与目标有效分开
    uint8_t sequence;
};

struct __attribute__((packed)) GimbalOffsetFrame {
    int16_t big_yaw_to_chassis;
    int16_t muzzle_to_chassis;
    uint8_t flags;
    uint8_t sequence;
};

struct __attribute__((packed)) RefereePowerFrame {
    uint16_t power_limit_w;
    uint16_t buffer_energy_j;
    uint8_t power_age;
    uint8_t buffer_age;
    uint8_t flags;
    uint8_t sequence;
};

struct __attribute__((packed)) BigYawFeedbackFrame {
    int16_t joint;
    uint8_t flags; // 有电机反馈、已使能就绪、本地测得低速
    uint8_t sequence;
};

struct __attribute__((packed)) ChassisFeedbackFrame {
    int16_t vx; // 实测底盘系速度，不能用目标值代替
    int16_t vy;
    int16_t wz;
    uint8_t flags;
    uint8_t sequence;
};

static_assert(sizeof(ChassisTargetFrame) == 8U, "CAN chassis target layout");
static_assert(sizeof(BigYawTargetFrame) == 8U, "CAN big yaw target layout");
static_assert(sizeof(GimbalOffsetFrame) == 6U, "CAN offset layout");
static_assert(sizeof(RefereePowerFrame) == 8U, "CAN referee power layout");
static_assert(sizeof(BigYawFeedbackFrame) == 4U, "CAN big yaw feedback layout");
static_assert(sizeof(ChassisFeedbackFrame) == 8U, "CAN chassis feedback layout");

} // namespace can3

} // namespace board_link
