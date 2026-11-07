/**
 * @file app_chassis.cpp
 * @brief 四轮全向底盘应用编排。
 *
 * 本文件只负责初始化和单周期调度。车型参数、命令选择、功率仲裁以及
 * 反馈/控制算法分别位于 chassis_config、chassis_command、
 * chassis_power_manager 和 chassis_controller。
 */

#include "app_chassis.h"

#include "chassis_command.hpp"
#include "chassis_config.hpp"
#include "chassis_controller.hpp"
#include "chassis_power_manager.hpp"
#include "cmsis_os.h"
#include "dvc_buzzer.hpp"
#include "lf_core.hpp"
#include "main.h"
#include "remote_control.h"

namespace {

//反馈回调函数
void publishChassisVelocity(const chassis_app::ChassisFeedback& feedback)
{
    LuojiaFox::system_state.updateChassis({
        feedback.vx,
        feedback.vy,
        feedback.wz
    });
}

} // namespace

void Chassis_Init(void)
{
    chassis_app::g_chassisCommandSelector =
        chassis_app::ChassisCommandSelector();
    chassis_app::g_chassisPowerManager.init();

    if (!chassis_app::g_chassisController.init()) {
        Error_Handler();
    }
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
    const chassis_app::ChassisFeedback feedback =
        chassis_app::g_chassisController.readFeedback(state.imu, now_ms);
    publishChassisVelocity(feedback);

    // 模式切换后的首周期先停车并清除积分，避免继承上一控制源的历史状态。
    // 模式切换时通过蜂鸣器提示。
    if (selection.mode_changed) {
        chassis_app::g_chassisController.stop();
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
        return;
    }

    float target_current[chassis_app::kWheelCount] = {};
    if (!chassis_app::g_chassisController.calculate(
            selection.command,
            feedback,
            target_current
        )) {
        chassis_app::g_chassisController.sendZero();
        return;
    }

    float limited_current[chassis_app::kWheelCount] = {};
    if (!chassis_app::g_chassisPowerManager.apply(
            feedback.wheel_omega,
            target_current,
            limited_current
        )) {
        chassis_app::g_chassisController.stop();
        return;
    }

    chassis_app::g_chassisController.sendCurrents(limited_current);
}

ChassisModeTypedef Chassis_GetMode(void)
{
    return chassis_app::g_chassisCommandSelector.mode();
}

#if BOARD_ROLE == 1
extern "C" void Control_Task(void const *argument)
{
    (void)argument;
    for (;;) {
        Chassis_Control();
        osDelay(1);
    }
}
#endif
