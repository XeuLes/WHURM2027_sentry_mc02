#include "app_board_link.hpp"
#include "cmsis_os.h"
#include "main.h"
#include "supercap_config.h"
#include <cmath>

namespace board_link {
namespace {
ChassisModeTypedef mode = CHASSIS_MODE_DISABLED;
BigYawFeedback feedback{};
ChassisFeedback chassis_feedback{};
}
chassis_app::ChassisCommandSelection getChassisSelection(uint32_t now) {
    const auto rx = can3::readTargets().chassis;
    chassis_app::ChassisCommandSelection result{};
    result.mode = rx.fresh(now, can3::kChassisTimeoutMs)
        ? static_cast<ChassisModeTypedef>(rx.data.flags & can3::kChassisModeMask)
        : CHASSIS_MODE_DISABLED;
    taskENTER_CRITICAL();
    result.mode_changed = mode != result.mode;
    mode = result.mode;
    taskEXIT_CRITICAL();
    result.command.source_update_ms = rx.rx_ms;
    result.command.ready = rx.fresh(now, can3::kChassisTimeoutMs) &&
        (rx.data.flags & can3::kChassisTargetValid) &&
        result.mode != CHASSIS_MODE_DISABLED && !result.mode_changed;
    if (result.command.ready) {
        // Already rotated into chassis coordinates by mainControl.
        result.command.vx = rx.data.vx / can3::kSpeedScale;
        result.command.vy = rx.data.vy / can3::kSpeedScale;
        result.command.wz = rx.data.wz / can3::kSpeedScale;
    }
    return result;
}
ChassisModeTypedef getChassisMode() {
    taskENTER_CRITICAL();
    const auto result = mode;
    taskEXIT_CRITICAL();
    return result;
}
BigYawTarget getBigYawTarget(uint32_t now) {
    const auto rx = can3::readTargets().yaw;
    BigYawTarget result{};
    result.revision = rx.revision;
    result.rx_ms = rx.rx_ms;
    result.enable = rx.fresh(now, can3::kBigYawTimeoutMs) && (rx.data.flags & can3::kBigYawEnable);
    result.valid = result.enable && (rx.data.flags & can3::kBigYawTargetValid);
    result.joint_frame = (rx.data.flags & can3::kBigYawJointFrame) != 0U;
    result.target_rad = rx.data.target / can3::kAngleScale;
    result.velocity_rad_s = rx.data.velocity / can3::kYawSpeedScale;
    if (result.joint_frame) {
        result.acceleration_rad_s2 = rx.data.position_or_acceleration / can3::kAccelerationScale;
    } else {
        result.position_rad = rx.data.position_or_acceleration / can3::kAngleScale;
    }
    return result;
}
RefereePowerSnapshot getRefereePowerSnapshot() {
    const auto rx = can3::readTargets().referee;
    const uint32_t now = HAL_GetTick();
    RefereePowerSnapshot result{};
    // Preserve source age across the bridge. Repeated snapshots do not renew it.
    result.power_tick = rx.rx_ms - rx.data.power_age * can3::kAgeUnitMs;
    result.buffer_tick = rx.rx_ms - rx.data.buffer_age * can3::kAgeUnitMs;
    result.power_limit_w = rx.data.power_limit_w;
    result.buffer_energy_j = rx.data.buffer_energy_j;
    result.power_received = rx.seen && (rx.data.flags & can3::kPowerReceived);
    result.buffer_received = rx.seen && (rx.data.flags & can3::kBufferReceived);
    result.online = rx.fresh(now, can3::kChassisTimeoutMs) &&
        (rx.data.flags & can3::kRefereeOnline) && result.power_received &&
        rx.data.power_age != UINT8_MAX &&
        now - result.power_tick <= SUPERCAP_REFEREE_TIMEOUT_MS;
    return result;
}
void publishBigYawFeedback(const BigYawFeedback& value) {
    taskENTER_CRITICAL();
    feedback = value;
    taskEXIT_CRITICAL();
}
void publishChassisFeedback(const ChassisFeedback& value) {
    taskENTER_CRITICAL();
    chassis_feedback = value;
    taskEXIT_CRITICAL();
}
void sendFeedback(uint32_t now) {
    using namespace can3;
    static uint8_t sequence = 0;
    taskENTER_CRITICAL();
    const auto yaw = feedback;
    const auto chassis = chassis_feedback;
    taskEXIT_CRITICAL();
    FeedbackValues value{};
    value.sequence = sequence++;
    if (now - yaw.updated_at_ms < kBigYawTimeoutMs && yaw.motor_online &&
        std::isfinite(yaw.joint_rad) && std::fabs(yaw.joint_rad) <= 3.141593f) {
        value.joint = static_cast<int16_t>(std::round(yaw.joint_rad * kFeedbackAngleScale));
        value.flags |= kBigYawFeedbackValid;
        if (yaw.ready) { value.flags |= kBigYawFeedbackReady; }
        if (yaw.slow) { value.flags |= kBigYawFeedbackSlow; }
    }
    const auto encodable = [](float v) {
        return std::isfinite(v) && v >= kFeedbackMin / kFeedbackSpeedScale &&
            v <= kFeedbackMax / kFeedbackSpeedScale;
    };
    if (chassis.valid && now - chassis.updated_at_ms < kChassisFeedbackSourceTimeoutMs &&
        encodable(chassis.vx) && encodable(chassis.vy) && encodable(chassis.wz)) {
        value.vx = static_cast<int16_t>(std::round(chassis.vx * kFeedbackSpeedScale));
        value.vy = static_cast<int16_t>(std::round(chassis.vy * kFeedbackSpeedScale));
        value.wz = static_cast<int16_t>(std::round(chassis.wz * kFeedbackSpeedScale));
        value.flags |= kChassisFeedbackValid;
    }
    // Keep heartbeat and each source's validity independent, also while disabled.
    can3::send(encodeFeedback(value));
}
}
extern "C" void BoardLink_Task(void const* argument) {
    (void)argument;
    if (!board_link::can3::init()) { Error_Handler(); return; }
    const TickType_t delay = pdMS_TO_TICKS(board_link::can3::kFeedbackPeriodMs);
    configASSERT(delay > 0U);
    for (;;) {
        board_link::sendFeedback(HAL_GetTick());
        // No catch-up burst after a delayed task wakeup.
        vTaskDelay(delay);
    }
}
