/**
 * @file chassis_command.cpp
 * @brief 底盘模式与遥控/上位机命令选择实现。
 */

#include "chassis_command.hpp"

#include "alg_basic.hpp"
#include "app_board_link.hpp"
#include "app_gimbal.h"
#include "chassis_config.hpp"
#include "gimbal_config.h"
#include "remote_control.h"

#include <cmath>

namespace chassis_app {

ChassisCommandSelector g_chassisCommandSelector;

namespace {

/**
 * @brief 对归一化遥控输入应用死区。
 * @param value 死区前后的归一化通道值，范围约为 -1~1。
 * @return 死区内返回 0，死区外原样返回输入值。
 */
float applyDeadzone(float value)
{
    return (basic_math::abs(value) < kRcDeadzone) ? 0.0f : value;
}

/** @brief 读取枪口系遥控平移目标，坐标转换由 BoardLink 统一完成。 */
void readRcTranslation(ChassisCommand& command)
{
    command.vx = kRcXSign * applyDeadzone(getDt7RcCh(RC_CH_LEFT_VERTICAL)) * kMaxVx;
    command.vy = kRcYSign * applyDeadzone(getDt7RcCh(RC_CH_LEFT_HORIZONTAL)) * kMaxVy;
}

/** @brief 手动过洞分为停车、底盘对齐和折叠完成后平移三个阶段。 */
void updateManualPassage(
    const LuojiaFox::SystemStateSnapshot& state,
    uint32_t now_ms,
    ChassisCommandSelection& selection
)
{
    using LuojiaFox::GimbalShape;
    using LuojiaFox::GimbalTransformState;
    const auto& gimbal_state = state.gimbal;
    selection.motion_blocked = true;

    board_link::BigYawFeedback yaw {};
    board_link::getPeerBigYawFeedback(yaw);
    const bool feedback_ready =
        now_ms - gimbal_state.updated_at_ms < GIMBAL_IMU_TIMEOUT_MS &&
        yaw.ready && std::isfinite(yaw.joint_rad) && gimbal::manualPassageHeadingReady() &&
        state.chassis.valid &&
        now_ms - state.chassis.last_update_ms < board_link::can3::kChassisFeedbackTimeoutMs;
    if (!feedback_ready) {
        return;
    }

    const bool folded = gimbal_state.manual_passage &&
        gimbal_state.transformed == GimbalShape::Folded &&
        gimbal_state.transform_state == GimbalTransformState::Ready;
    if (folded) {
        selection.motion_blocked = false;
        readRcTranslation(selection.command);
        return;
    }

    const bool aligning = gimbal_state.manual_passage_aligning &&
        gimbal_state.transform_state == GimbalTransformState::Preparing &&
        std::hypot(state.chassis.vx, state.chassis.vy) <= kPassageStoppedSpeed &&
        std::fabs(yaw.joint_rad) > GIMBAL_PASS_CHASSIS_ALIGN_TOL_RAD;
    if (aligning) {
        // 大 Yaw 机械偏角 = 大 Yaw 航向 - 底盘航向。
        selection.motion_blocked = false;
        selection.command.wz = basic_math::constrain(kPassageAlignGain * yaw.joint_rad,
            -kPassageAlignMaxWz, kPassageAlignMaxWz);
    }
}

/** @brief 遥控正常行驶；过洞请求与退出过程由各自联锁处理。 */
void updateDirectRcCommand(
    const LuojiaFox::SystemStateSnapshot& state,
    uint32_t now_ms,
    ChassisCommandSelection& selection
)
{
    auto& command = selection.command;
    command.source_update_ms = now_ms;
    command.ready = true;

    const bool passage_requested =
        getDt7RcSw(GIMBAL_PASS_SWITCH_INDEX) == GIMBAL_PASS_SWITCH_POSITION;
    if (passage_requested) {
        updateManualPassage(state, now_ms, selection);
        return;
    }

    // 撤销请求后保持停车，直到云台完成回正并退出过洞。
    selection.motion_blocked = state.gimbal.manual_passage ||
        state.gimbal.mode == static_cast<uint8_t>(gimbal::Mode::kPassage);
    if (selection.motion_blocked) {
        return;
    }
    readRcTranslation(command);
    command.wz = kRcWzSign * applyDeadzone(getDt7RcCh(RC_CH_RIGHT_HORIZONTAL)) * kMaxWz;
}

} // namespace

/**
 * @brief 根据拨杆状态解析目标底盘模式。
 * @note 遥控离线时保留最近模式，允许自动哨兵不依赖遥控器运行。
 */
ChassisModeTypedef ChassisCommandSelector::selectMode(bool rc_online) const
{
    // 遥控离线时保留最近模式，使自动哨兵可以脱离遥控器运行。
    if (!rc_online) {
        return mode_;
    }

    // The chassis and gimbal use the same mode switch (ET08A SB / CH7).
    const char mode_switch = getDt7RcSw(RC_SW_RIGHT);
    if (SWITCH_IS_UP(mode_switch)) {
        return CHASSIS_MODE_AUTO_VISION;
    }
    if (SWITCH_IS_MID(mode_switch)) {
        return CHASSIS_MODE_DIRECT_RC;
    }
    return CHASSIS_MODE_DISABLED;
}

/**
 * @brief 更新模式并生成当前控制源的底盘速度命令。
 * @note 自动模式要求上位机命令时间戳新鲜且 vx/vy/wz 均为有限值。
 */
ChassisCommandSelection ChassisCommandSelector::update(
    const LuojiaFox::SystemStateSnapshot& state,
    bool rc_online,
    uint32_t now_ms
)
{
    ChassisCommandSelection selection {};
    const ChassisModeTypedef requested_mode = selectMode(rc_online);
    selection.mode = requested_mode;

    if (requested_mode != mode_) {
        mode_ = requested_mode;
        source_ready_ = false;
        selection.mode_changed = true;
        return selection;
    }

    using LuojiaFox::GimbalShape;
    using LuojiaFox::GimbalTransformState;
    const auto& gimbal_state = state.gimbal;
    const bool gimbal_fresh = now_ms - gimbal_state.updated_at_ms < GIMBAL_IMU_TIMEOUT_MS;
    selection.motion_blocked = gimbal_state.manual_passage;

    ChassisCommand& command = selection.command;
    if (mode_ == CHASSIS_MODE_DIRECT_RC && rc_online) {
        updateDirectRcCommand(state, now_ms, selection);
    } else if (mode_ == CHASSIS_MODE_AUTO_VISION) {
        const auto& robot_cmd = state.robot_cmd;
        const auto requested_shape = robot_cmd.passage_requested ? GimbalShape::Folded : GimbalShape::Upright;
        selection.motion_blocked = selection.motion_blocked || !gimbal_fresh ||
            gimbal_state.transform_state != GimbalTransformState::Ready ||
            gimbal_state.transformed != requested_shape ||
            (robot_cmd.passage_requested && !robot_cmd.heading_valid);
        command.source_update_ms = robot_cmd.last_update_ms;

        const bool fresh =
            robot_cmd.last_update_ms != 0U
            && now_ms - robot_cmd.last_update_ms < kAutoCommandTimeoutMs;
        const bool finite =
            std::isfinite(robot_cmd.vx)
            && std::isfinite(robot_cmd.vy)
            && std::isfinite(robot_cmd.wz);

        if (robot_cmd.chassis_enabled && fresh && finite) {
            command.vx = basic_math::constrain(robot_cmd.vx, -kMaxVx, kMaxVx);
            command.vy = basic_math::constrain(robot_cmd.vy, -kMaxVy, kMaxVy);
            command.wz = basic_math::constrain(robot_cmd.wz, -kMaxWz, kMaxWz);
            command.ready = true;
        }
    }

    if (selection.motion_blocked) {
        command.vx = command.vy = command.wz = 0.0f;
    }
    source_update_ms_ = command.source_update_ms;
    source_ready_ = command.ready;
    return selection;
}

} // namespace chassis_app
