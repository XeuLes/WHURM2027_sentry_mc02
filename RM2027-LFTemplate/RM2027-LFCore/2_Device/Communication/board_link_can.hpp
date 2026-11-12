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
constexpr uint16_t kRefereePowerId = 0x313U;
// New ID prevents v2 feedback from being interpreted as the packed v3 format.
constexpr uint16_t kFeedbackId = 0x31AU;
constexpr uint32_t kFeedbackPeriodMs = 1U;
// Signed 14-bit telemetry: preserve the previous +/-32.768 speed range.
constexpr float kFeedbackSpeedScale = 250.0f; // 0.004 m/s or rad/s per LSB
constexpr float kFeedbackAngleScale = 2500.0f; // 0.0004 rad per LSB
constexpr int16_t kFeedbackMin = -8192;
constexpr int16_t kFeedbackMax = 8191;

// 下行状态字的高半字节为协议版本，低半字节含有效位或模式。
constexpr uint8_t kVersion = 0x30U;
constexpr uint8_t kVersionMask = 0xF0U;
constexpr float kAngleScale = 10000.0f; // rad
constexpr float kSpeedScale = 1000.0f; // m/s 或 rad/s
constexpr float kYawSpeedScale = 500.0f; // 大 Yaw 角速度，rad/s
constexpr float kAccelerationScale = 100.0f; // rad/s²
constexpr uint32_t kAgeUnitMs = 10U; // 向上取整，255 表示饱和
constexpr uint32_t kCommandPeriodMs = 1U; // 底盘、大 Yaw 和裁判数据
constexpr uint32_t kChassisFeedbackTimeoutMs = 50U;
constexpr uint32_t kChassisFeedbackSourceTimeoutMs = 10U;
constexpr uint32_t kBigYawTimeoutMs = 20U;
constexpr uint32_t kChassisTimeoutMs = 50U;
constexpr float kBigYawSlowSpeedRadS = 0.05f;

constexpr uint8_t kChassisModeMask = 0x03U;
constexpr uint8_t kChassisTargetValid = 0x04U;
constexpr uint8_t kChassisFeedbackValid = 0x08U;
constexpr uint8_t kBigYawEnable = 0x01U;
constexpr uint8_t kBigYawTargetValid = 0x02U;
constexpr uint8_t kBigYawJointFrame = 0x04U;
constexpr uint8_t kBigYawFeedbackValid = 0x01U;
constexpr uint8_t kBigYawFeedbackReady = 0x02U;
constexpr uint8_t kBigYawFeedbackSlow = 0x04U;
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

struct __attribute__((packed)) RefereePowerFrame {
    uint16_t power_limit_w;
    uint16_t buffer_energy_j;
    uint8_t power_age;
    uint8_t buffer_age;
    uint8_t flags;
    uint8_t sequence;
};

// Wire bits 0..55: vx, vy, wz, joint, each signed 14-bit two's complement.
// Byte 7: low nibble validity/ready/slow flags, high nibble modulo-16 sequence.
// Version is identified by kFeedbackId; there is no version nibble in this frame.
struct __attribute__((packed)) FeedbackFrame { uint8_t bytes[8]; };
struct FeedbackValues {
    int16_t vx = 0, vy = 0, wz = 0, joint = 0;
    uint8_t flags = 0, sequence = 0;
};
inline FeedbackFrame encodeFeedback(const FeedbackValues& value) {
    const uint64_t bits = (static_cast<uint64_t>(value.vx) & 0x3FFFU) |
        ((static_cast<uint64_t>(value.vy) & 0x3FFFU) << 14) |
        ((static_cast<uint64_t>(value.wz) & 0x3FFFU) << 28) |
        ((static_cast<uint64_t>(value.joint) & 0x3FFFU) << 42);
    FeedbackFrame frame{};
    for (unsigned i = 0; i < 7; ++i) { frame.bytes[i] = static_cast<uint8_t>(bits >> (8*i)); }
    frame.bytes[7] = (value.flags & 0x0FU) | ((value.sequence & 0x0FU) << 4);
    return frame;
}
inline FeedbackValues decodeFeedback(const FeedbackFrame& frame) {
    uint64_t bits = 0;
    for (unsigned i = 0; i < 7; ++i) { bits |= static_cast<uint64_t>(frame.bytes[i]) << (8*i); }
    const auto signed14 = [](uint64_t v) -> int16_t {
        const int32_t raw = static_cast<int32_t>(v & 0x3FFFU);
        return static_cast<int16_t>(raw >= 8192 ? raw - 16384 : raw);
    };
    return {signed14(bits), signed14(bits >> 14), signed14(bits >> 28), signed14(bits >> 42),
            static_cast<uint8_t>(frame.bytes[7] & 0x0FU),
            static_cast<uint8_t>(frame.bytes[7] >> 4)};
}

// Two DM raw reports, three downlink commands and one combined feedback per ms.
constexpr uint32_t kWorstCaseBitsPerSecond =
    2U * 1000U * kMaxFrameBits + 3U * 1000U / kCommandPeriodMs * kMaxFrameBits +
    1000U / kFeedbackPeriodMs * maxFrameBits(sizeof(FeedbackFrame));
static_assert(kWorstCaseBitsPerSecond <= kBitrate * 81U / 100U, "CAN3 exceeds agreed 81% budget");
static_assert(sizeof(ChassisTargetFrame) == 8U, "CAN chassis target layout");
static_assert(sizeof(BigYawTargetFrame) == 8U, "CAN big yaw target layout");
static_assert(sizeof(RefereePowerFrame) == 8U, "CAN referee power layout");
static_assert(sizeof(FeedbackFrame) == 8U, "CAN combined feedback layout");

} // namespace can3
} // namespace board_link
