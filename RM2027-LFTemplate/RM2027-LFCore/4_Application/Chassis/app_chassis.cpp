/**
 * @file app_chassis.cpp
 * @brief 主控底盘目标计算。
 *
 * 使用 chassis_command 仲裁模式并生成速度目标，由 BoardLink 下发。
 */

#include "app_chassis.h"

#include "chassis_command.hpp"
#include "cmsis_os.h"
#include "dvc_buzzer.hpp"
#include "lf_core.hpp"
#include "main.h"
#include "remote_control.h"
#include "app_board_link.hpp"

void Chassis_Init(void)
{
    chassis_app::g_chassisCommandSelector =
        chassis_app::ChassisCommandSelector();
}

void Chassis_Control(void)
{
    const LuojiaFox::SystemStateSnapshot state =
        LuojiaFox::system_state.read();
    const uint32_t now_ms = HAL_GetTick();
    const bool rc_online = !getRcOffline();

    const chassis_app::ChassisCommandSelection selection =
        chassis_app::g_chassisCommandSelector.update(
            state.robot_cmd,
            rc_online,
            now_ms
        );
    buzzer.update(now_ms);

    // BoardLink 在模式变化时先发送无效目标；本任务只做提示和目标更新。
    if (selection.mode_changed) {
        switch (selection.mode) {
        case CHASSIS_MODE_DISABLED:
            buzzer.playBeeps(1U);
            break;
        case CHASSIS_MODE_DIRECT_RC:
            buzzer.playBeeps(2U);
            break;
        case CHASSIS_MODE_AUTO_VISION:
            buzzer.playBeeps(3U);
            break;
        default:
            break;
        }
    }

    const auto& command = selection.command;
    board_link::publishChassisTarget({
        command.vx, command.vy, command.wz,
        static_cast<uint8_t>(selection.mode), command.ready, now_ms
    });
}

ChassisModeTypedef Chassis_GetMode(void)
{
    return chassis_app::g_chassisCommandSelector.mode();
}

extern "C" void Control_Task(void const *argument)
{
    (void)argument;
    for (;;) {
        Chassis_Control();
        osDelay(1);
    }
}
