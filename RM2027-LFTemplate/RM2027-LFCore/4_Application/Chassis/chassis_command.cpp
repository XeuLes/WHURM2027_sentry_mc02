/**
 * @file chassis_command.cpp
 * @brief 底盘模式与遥控/上位机命令选择实现。
 */

#include "chassis_command.hpp"

#include "alg_basic.hpp"
#include "chassis_config.hpp"
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

    if (SWITCH_IS_DOWN(getDt7RcSw(RC_SW_RIGHT))) {
        return CHASSIS_MODE_DISABLED;
    }

    const char left_switch = getDt7RcSw(RC_SW_LEFT);
    if (SWITCH_IS_UP(left_switch)) {
        return CHASSIS_MODE_AUTO_VISION;
    }
    if (SWITCH_IS_MID(left_switch)) {
        return CHASSIS_MODE_DIRECT_RC;
    }
    return CHASSIS_MODE_DISABLED;
}

/**
 * @brief 更新模式并生成当前控制源的底盘速度命令。
 * @note 自动模式要求上位机命令时间戳新鲜且 vx/vy/wz 均为有限值。
 */
ChassisCommandSelection ChassisCommandSelector::update(
    const LuojiaFox::RobotCmdState& robot_cmd,
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

    ChassisCommand& command = selection.command;  //左值引用，给selection.command取别名command
    if (mode_ == CHASSIS_MODE_DIRECT_RC && rc_online) {
        command.vx = kRcXSign * applyDeadzone(
            getDt7RcCh(RC_CH_LEFT_VERTICAL)
        ) * kMaxVx;
        command.vy = kRcYSign * applyDeadzone(
            getDt7RcCh(RC_CH_LEFT_HORIZONTAL)
        ) * kMaxVy;
        command.wz = kRcWzSign * applyDeadzone(
            getDt7RcCh(RC_CH_RIGHT_HORIZONTAL)
        ) * kMaxWz;
        command.source_update_ms = now_ms;
        command.ready = true;
    } else if (mode_ == CHASSIS_MODE_AUTO_VISION) {
        command.source_update_ms = robot_cmd.last_update_ms;

        const bool fresh =
            robot_cmd.last_update_ms != 0U
            && now_ms - robot_cmd.last_update_ms <= kAutoCommandTimeoutMs;
        const bool finite =
            std::isfinite(robot_cmd.vx)
            && std::isfinite(robot_cmd.vy)
            && std::isfinite(robot_cmd.wz);

        if (fresh && finite) {
            command.vx = basic_math::constrain(robot_cmd.vx, -kMaxVx, kMaxVx);
            command.vy = basic_math::constrain(robot_cmd.vy, -kMaxVy, kMaxVy);
            command.wz = basic_math::constrain(robot_cmd.wz, -kMaxWz, kMaxWz);
            command.ready = true;
        }
    }

    source_update_ms_ = command.source_update_ms;
    source_ready_ = command.ready;
    return selection;
}

} // namespace chassis_app
