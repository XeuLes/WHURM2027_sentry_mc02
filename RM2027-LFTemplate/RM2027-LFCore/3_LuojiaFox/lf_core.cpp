/**
 * @file LF.cpp
 * @author BayekQ (206858817@qq.com)
 * @brief LuojiaFox 驱动
 * @version 0.1
 * @date TODO
 *
 * @copyright WHU-LuojiaFox (c) 2026
 *
 */


#include "lf_core.hpp"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

extern "C" {
LuojiaFox::SystemStateSnapshot g_system_state{};
}

namespace LuojiaFox {

SystemStateStore system_state;

void SystemStateStore::updateImu(const ImuState &state)
{
    taskENTER_CRITICAL();
    g_system_state.imu = state;
    g_system_state.imu.last_update_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateMuzzleImu(const ImuState &state)
{
    taskENTER_CRITICAL();
    g_system_state.muzzle_imu = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateChassis(const ChassisState &state)
{
    taskENTER_CRITICAL();
    g_system_state.chassis = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateGimbal(const GimbalState &state)
{
    taskENTER_CRITICAL();
    g_system_state.gimbal = state;
    g_system_state.gimbal.updated_at_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateShooter(const ShooterState &state)
{
    taskENTER_CRITICAL();
    g_system_state.shooter = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateReferee(const RefereeState &state)
{
    taskENTER_CRITICAL();
    g_system_state.referee = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateRobotCmd(const RobotCmdState &state)
{
    taskENTER_CRITICAL();
    g_system_state.robot_cmd = state;
    // USB publishes only newly accepted RobotCmd frames; board-link publishes
    // received commands. Keep the local receipt time used by the gimbal.
    g_system_state.robot_cmd.last_update_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateUsbLink(const UsbLinkState &state)
{
    taskENTER_CRITICAL();
    g_system_state.usb_link = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateVirtualRc(const VirtualRcState &state)
{
    taskENTER_CRITICAL();
    g_system_state.virtual_rc = state;
    taskEXIT_CRITICAL();
}

SystemStateSnapshot SystemStateStore::read() const
{
    taskENTER_CRITICAL();
    const SystemStateSnapshot result = g_system_state;
    taskEXIT_CRITICAL();
    return result;
}

} // namespace LuojiaFox
