#include "app_board_link.hpp"

#include "cmsis_os.h"
#include "main.h"
#include "referee.hpp"
#include "chassis_config.hpp"
#include "app_chassis.h"
#include "lf_core.hpp"

#include <cmath>

namespace board_link {
namespace {
using namespace can3;

constexpr uint32_t kChassisSourceTimeoutMs = 10U;

// 枪口仅主动上报欧拉角与角速度，各 1 kHz，板载 BMI088 不占用 CAN3。
constexpr uint32_t kWorstCaseBitsPerSecond =
    2U * 1000U * kMaxFrameBits +
    3U * 1000U / kCommandPeriodMs * kMaxFrameBits +
    1000U / kBigYawFeedbackPeriodMs * maxFrameBits(sizeof(BigYawFeedbackFrame)) +
    1000U / kChassisFeedbackPeriodMs * maxFrameBits(sizeof(ChassisFeedbackFrame)) +
    1000U / kGimbalOffsetPeriodMs * maxFrameBits(sizeof(GimbalOffsetFrame));
static_assert(kGimbalOffsetPeriodMs % kCommandPeriodMs == 0U, "CAN3 offset period must align");
static_assert(kWorstCaseBitsPerSecond <= kBitrate * 75U / 100U, "CAN3 frame budget exceeds 75 percent");

BigYawTarget g_yaw_target{};
ChassisTarget g_chassis_target{};

int16_t encodeSigned(float value, float scale) {
    if (!std::isfinite(value)) { return 0; }
    const float scaled = value * scale;
    return static_cast<int16_t>(std::round(std::fmax(-32768.0f, std::fmin(32767.0f, scaled))));
}

uint8_t encodeAge(uint32_t age_ms) {
    if (age_ms >= UINT8_MAX * kAgeUnitMs) { return UINT8_MAX; }
    return static_cast<uint8_t>((age_ms + kAgeUnitMs - 1U) / kAgeUnitMs);
}

void sendYaw(uint32_t now, const BigYawTarget& target, bool peer_online, uint8_t sequence) {
    const bool fresh = now - target.updated_at_ms < kBigYawTimeoutMs;
    const bool enable = fresh && peer_online && target.enable;
    const float position_or_accel = target.joint_frame ? target.passage_acceleration_rad_s2
                                                       : target.position_rad;
    const bool valid = enable && target.valid && std::isfinite(target.target_rad) &&
                       std::isfinite(position_or_accel) && std::isfinite(target.velocity_rad_s);
    BigYawTargetFrame yaw{};
    yaw.flags = kVersion;
    yaw.sequence = sequence;
    if (enable) { yaw.flags |= kBigYawEnable; }
    if (valid) {
        yaw.flags |= kBigYawTargetValid;
        if (target.joint_frame) { yaw.flags |= kBigYawJointFrame; }
        yaw.target = encodeSigned(target.target_rad, kAngleScale);
        yaw.position_or_acceleration = encodeSigned(position_or_accel,
            target.joint_frame ? kAccelerationScale : kAngleScale);
        yaw.velocity = encodeSigned(target.velocity_rad_s, kYawSpeedScale);
    }
    can3::send(yaw);
}

void sendOffset(uint32_t now, const BigYawTarget& target, bool peer_online, uint8_t sequence) {
    const bool fresh = now - target.updated_at_ms < kBigYawTimeoutMs;
    GimbalOffsetFrame offset{};
    offset.flags = kVersion;
    offset.sequence = sequence;
    if (fresh && peer_online && target.big_yaw_offset_valid &&
        std::isfinite(target.big_yaw_to_chassis_rad)) {
        offset.flags |= kBigYawOffsetValid;
        offset.big_yaw_to_chassis = encodeSigned(target.big_yaw_to_chassis_rad, kAngleScale);
        if (target.muzzle_offset_valid && std::isfinite(target.muzzle_to_chassis_rad)) {
            offset.flags |= kGimbalOffsetValid;
            offset.muzzle_to_chassis = encodeSigned(target.muzzle_to_chassis_rad, kAngleScale);
        }
    }
    can3::send(offset);
}

void sendChassis(uint32_t now, const ChassisTarget& target, bool peer_online, uint8_t sequence) {
    static uint8_t last_sent_mode = UINT8_MAX;
    ChassisTargetFrame frame{};
    frame.flags = kVersion | (target.mode & kChassisModeMask);
    frame.sequence = sequence;
    if (peer_online && target.valid && target.mode == last_sent_mode &&
        now - target.updated_at_ms < kChassisSourceTimeoutMs && std::isfinite(target.vx) &&
        std::isfinite(target.vy) && std::isfinite(target.wz)) {
        frame.flags |= kChassisTargetValid;
        frame.vx = encodeSigned(target.vx, kSpeedScale);
        frame.vy = encodeSigned(target.vy, kSpeedScale);
        frame.wz = encodeSigned(target.wz, kSpeedScale);
    }
    if (can3::send(frame)) {
        last_sent_mode = target.mode;
    }
}

void sendReferee(uint8_t sequence) {
    const RefereePowerSnapshot power = referee.GetPowerSupplySnapshot();
    const uint32_t now = HAL_GetTick();
    RefereePowerFrame frame{};
    frame.flags = kVersion;
    frame.sequence = sequence;
    if (power.online) { frame.flags |= kRefereeOnline; }
    if (power.power_received) {
        frame.flags |= kPowerReceived;
        frame.power_limit_w = power.power_limit_w;
        frame.power_age = encodeAge(now - power.power_tick);
    }
    if (power.buffer_received) {
        frame.flags |= kBufferReceived;
        frame.buffer_energy_j = power.buffer_energy_j;
        frame.buffer_age = encodeAge(now - power.buffer_tick);
    }
    can3::send(frame);
}

void exchangeTargets(bool send_offset) {
    static uint8_t command_sequence = 0U;
    static uint8_t offset_sequence = 0U;

    taskENTER_CRITICAL();
    auto chassis_target = g_chassis_target;
    const auto yaw_target = g_yaw_target;
    taskEXIT_CRITICAL();

    const auto feedback = can3::readFeedback();
    const uint32_t now = HAL_GetTick();
    const bool peer_online = feedback.fresh(now, kBigYawTimeoutMs);

    // RC and host translation both use the muzzle frame, including small yaw.
    // Planning stays here; subControl must not rotate the target again.
    if (chassis_target.valid) {
        const float offset_rad = yaw_target.muzzle_to_chassis_rad;
        const bool offset_ok = now - yaw_target.updated_at_ms < kBigYawTimeoutMs &&
            yaw_target.big_yaw_offset_valid && yaw_target.muzzle_offset_valid &&
            std::isfinite(offset_rad);
        if (!offset_ok) {
            chassis_target.valid = false;
        } else {
            const float c = std::cos(offset_rad);
            const float s = std::sin(offset_rad);
            const float vx = c * chassis_target.vx - s * chassis_target.vy;
            const float vy = s * chassis_target.vx + c * chassis_target.vy;
            // Preserve direction while keeping existing chassis-axis limits.
            const float scale = std::fmax(1.0f, std::fmax(
                std::fabs(vx) / chassis_app::kMaxVx,
                std::fabs(vy) / chassis_app::kMaxVy));
            chassis_target.vx = vx / scale;
            chassis_target.vy = vy / scale;
        }
    }

    const uint8_t sequence = command_sequence++;
    sendYaw(now, yaw_target, peer_online, sequence);
    sendChassis(now, chassis_target, peer_online, sequence);
    sendReferee(sequence);
    if (send_offset) { sendOffset(now, yaw_target, peer_online, offset_sequence++); }
    const auto chassis_rx = can3::readChassisFeedback();
    LuojiaFox::ChassisState chassis_state{};
    chassis_state.last_update_ms = chassis_rx.rx_ms;
    chassis_state.valid = chassis_rx.fresh(now, kChassisFeedbackTimeoutMs) &&
        (chassis_rx.data.flags & kChassisFeedbackValid);
    if (chassis_state.valid) {
        chassis_state.vx = chassis_rx.data.vx / kSpeedScale;
        chassis_state.vy = chassis_rx.data.vy / kSpeedScale;
        chassis_state.wz = chassis_rx.data.wz / kSpeedScale;
    }
    LuojiaFox::system_state.updateChassis(chassis_state);
    can3::updateBusLoad(now);
}
} // namespace

void publishChassisTarget(const ChassisTarget& target) {
    taskENTER_CRITICAL();
    g_chassis_target = target;
    taskEXIT_CRITICAL();
}

void publishBigYawTarget(const BigYawTarget& target) {
    taskENTER_CRITICAL();
    g_yaw_target = target;
    taskEXIT_CRITICAL();
}

bool getPeerBigYawFeedback(BigYawFeedback& feedback) {
    const auto received = can3::readFeedback();
    const bool fresh = received.fresh(HAL_GetTick(), kBigYawTimeoutMs);
    const auto& frame = received.data;
    const bool valid = fresh && (frame.flags & kBigYawFeedbackValid) &&
                       frame.joint >= -31416 && frame.joint <= 31416;
    feedback = {frame.joint / kAngleScale, valid, valid && (frame.flags & kBigYawFeedbackReady),
                valid && (frame.flags & kBigYawFeedbackSlow), received.revision, received.rx_ms};
    return fresh;
}

bool isPeerOnline() {
    const auto received = can3::readFeedback();
    return received.fresh(HAL_GetTick(), kBigYawTimeoutMs);
}

} // namespace board_link

extern "C" void BoardLink_Task(void const* argument) {
    (void)argument;
    if (!board_link::can3::init()) {
        Error_Handler();
        return;
    }

    using namespace board_link::can3;
    const TickType_t delay = pdMS_TO_TICKS(kCommandPeriodMs);
    configASSERT(delay > 0U);
    uint32_t phase_ms = 0U;
    for (;;) {
        board_link::exchangeTargets(phase_ms == 0U);
        phase_ms = (phase_ms + kCommandPeriodMs) % kGimbalOffsetPeriodMs;
        // 延误后从当前时刻继续，避免补发积压的周期帧。
        vTaskDelay(delay);
    }
}
