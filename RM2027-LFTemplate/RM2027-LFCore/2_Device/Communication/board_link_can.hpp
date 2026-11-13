#pragma once

#include <stdint.h>

namespace board_link {
namespace can3 {

// CAN3 标称速率，单位 bit/s。板间通信使用 11 位标准经典 CAN 帧。
// 同一总线上的枪口 DM-IMU 设备 ID 为 0x01，主机 ID 为 0x11。
constexpr uint32_t kBitrate = 1000000U;

// 47 位固定开销含帧间隔。从 SOF 到 CRC 序列共有 34 + 8 × bytes 位参与位填充，
// 预算按该位数减 1 后除以 4 计算最坏填充位数。
constexpr uint32_t maxFrameBits(uint32_t bytes) {
    return 47U + 8U * bytes + (33U + 8U * bytes) / 4U;
}
constexpr uint32_t kMaxFrameBits = maxFrameBits(8U); // 8 字节数据帧按 135 bit 计入预算

// 11 位 CAN 标准帧 ID。前三帧由主控下发，合并反馈由副板上报。
constexpr uint16_t kChassisTargetId = 0x310U; // 底盘系 vx/vy/wz 目标、控制模式
constexpr uint16_t kBigYawTargetId = 0x311U; // 大 Yaw 控制模式、目标及闭环所需数据
constexpr uint16_t kRefereePowerId = 0x313U; // 裁判功率上限、缓冲能量及数据年龄
constexpr uint16_t kFeedbackId = 0x31AU; // v3 合并反馈，含底盘实测速度和大 Yaw 机械偏角

constexpr uint32_t kFeedbackPeriodMs = 1U; // 副板发送合并反馈的周期，单位 ms

// 合并反馈用有符号 14 位整数。发送端将物理量乘以比例并取整，接收端除以比例。
constexpr float kFeedbackSpeedScale = 250.0f; // 每计数 0.004 m/s 或 0.004 rad/s
constexpr float kFeedbackAngleScale = 2500.0f; // 大 Yaw 机械偏角，每计数 0.0004 rad
// 14 位补码整数的上下限，用于检查底盘速度能否编码。
constexpr int16_t kFeedbackMin = -8192;
constexpr int16_t kFeedbackMax = 8191;

// 下行 flags 的高半字节用于校验协议版本，低半字节的含义由帧 ID 决定。
constexpr uint8_t kVersion = 0x30U; // v3 编码到 bit4~7
constexpr uint8_t kVersionMask = 0xF0U; // 取出版本位，接收端与 kVersion 比较

// 下行 int16_t 字段的编码比例，物理量乘以比例后取整。
constexpr float kAngleScale = 10000.0f; // 0x311 目标角和 BMI088 航向，每计数 0.0001 rad
constexpr float kSpeedScale = 1000.0f; // 0x310 底盘速度，每计数 0.001 m/s 或 0.001 rad/s
constexpr float kYawSpeedScale = 500.0f; // 0x311 大 Yaw 角速度，每计数 0.002 rad/s
constexpr float kAccelerationScale = 100.0f; // 0x311 机械模式轨迹角加速度，每计数 0.01 rad/s²

constexpr uint32_t kAgeUnitMs = 10U; // 裁判源数据年龄按 10 ms 向上取整，编码 255 表示饱和
constexpr uint32_t kCommandPeriodMs = 1U; // 主控发送底盘、大 Yaw、裁判三帧的周期，单位 ms
constexpr uint32_t kChassisFeedbackTimeoutMs = 50U; // 主控收到反馈后使用底盘实测速度的时效上限
constexpr uint32_t kChassisFeedbackSourceTimeoutMs = 10U; // 副板打包前检查底盘速度快照的时效上限
constexpr uint32_t kBigYawTimeoutMs = 20U; // 大 Yaw 目标及反馈的时效上限，也用于主控判断副板在线
constexpr uint32_t kChassisTimeoutMs = 50U; // 副板接收底盘目标和裁判快照的超时阈值
constexpr float kBigYawSlowSpeedRadS = 0.05f; // 实测机械角速度绝对值小于此值时标记低速，单位 rad/s

// 0x310 底盘目标的 flags。
constexpr uint8_t kChassisModeMask = 0x03U; // bit0~1，0 禁用、1 遥控、2 上位机
constexpr uint8_t kChassisTargetValid = 0x04U; // bit2，底盘速度目标有效
constexpr uint8_t kChassisMotionBlocked = 0x08U; // bit3，变形联锁，目标有效时副板闭环控制零速

constexpr uint8_t kChassisFeedbackValid = 0x08U; // 0x31A 状态 bit3，实测底盘速度有效

// 0x311 大 Yaw 目标的 flags。
constexpr uint8_t kBigYawEnable = 0x01U; // bit0，请求副板使能大 Yaw 电机
constexpr uint8_t kBigYawTargetValid = 0x02U; // bit1，允许副板使用目标进行闭环
constexpr uint8_t kBigYawJointFrame = 0x04U; // bit2，1 为机械偏角模式，0 为 BMI088 惯性航向模式

// 0x31A 合并反馈中的大 Yaw 状态。
constexpr uint8_t kBigYawFeedbackValid = 0x01U; // bit0，大 Yaw 电机反馈及机械偏角有效
constexpr uint8_t kBigYawFeedbackReady = 0x02U; // bit1，电机已使能并就绪
constexpr uint8_t kBigYawFeedbackSlow = 0x04U; // bit2，电机实测机械角速度低于阈值

// 0x313 裁判快照的 flags。副板结合源数据年龄判断样本是否仍可使用。
constexpr uint8_t kRefereeOnline = 0x01U; // bit0，主控判定裁判系统在线
constexpr uint8_t kPowerReceived = 0x02U; // bit1，主控已收到功率上限
constexpr uint8_t kBufferReceived = 0x04U; // bit2，主控已收到缓冲能量

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

// bit0~55 按小端位流依次放入 vx、vy、wz 和 joint，每个量用 14 位补码表示。
// 最后一个字节的低 4 位为状态，高 4 位为模 16 序号。反馈格式由 kFeedbackId 标识。
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

// 预算按每毫秒 2 帧 DM-IMU 原始数据、3 帧主控下行和 1 帧副板反馈计算。
// 当前周期下为 810000 bit/s，占 1 Mbit/s 总线速率的 81%。
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
