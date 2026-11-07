/**
 * @file chassis_command.hpp
 * @brief 底盘模式与控制源选择接口。
 */

#ifndef CHASSIS_COMMAND_HPP
#define CHASSIS_COMMAND_HPP

#include "app_chassis.h"
#include "lf_core.hpp"

#include <stdint.h>

namespace chassis_app {

struct ChassisCommand {
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    uint32_t source_update_ms = 0U;
    bool ready = false;
};

struct ChassisCommandSelection {
    ChassisCommand command {};
    ChassisModeTypedef mode = CHASSIS_MODE_AUTO_VISION;
    bool mode_changed = false;
};

class ChassisCommandSelector {
public:
    /**
     * @brief 更新底盘模式并读取当前控制源命令。
     * @param[in] robot_cmd 上位机最近一次发布的底盘命令。
     * @param[in] rc_online 遥控接收机当前是否在线。
     * @param[in] now_ms 当前系统时间，单位 ms。
     * @return 本次模式、命令及模式切换标志。
     * @note 模式切换时返回的 command.ready 为 false，调用方应停车一个周期。
     */
    ChassisCommandSelection update(
        const LuojiaFox::RobotCmdState& robot_cmd,
        bool rc_online,
        uint32_t now_ms
    );

    /** @brief 获取最近一次选择后的底盘模式。 */
    ChassisModeTypedef mode() const
    {
        return mode_;
    }

    /** @brief 查询当前控制源是否通过在线、时效和数值有效性检查。 */
    bool sourceReady() const
    {
        return source_ready_;
    }

    /** @brief 获取当前控制源最近更新时间，单位 ms。 */
    uint32_t sourceUpdateMs() const
    {
        return source_update_ms_;
    }

private:
    /**
     * @brief 根据遥控拨杆选择底盘模式。
     * @param[in] rc_online 遥控接收机当前是否在线。
     * @return 请求切换到的工作模式。
     */
    ChassisModeTypedef selectMode(bool rc_online) const;

    ChassisModeTypedef mode_ = CHASSIS_MODE_AUTO_VISION;
    bool source_ready_ = false;
    uint32_t source_update_ms_ = 0U;
};

extern ChassisCommandSelector g_chassisCommandSelector;

} // namespace chassis_app

#endif // CHASSIS_COMMAND_HPP
