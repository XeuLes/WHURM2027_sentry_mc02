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

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "Board_cfg/board_cfg.hpp"
#include "remote_control.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

#ifdef __cplusplus

extern "C++" {
namespace LuojiaFox {

struct ImuState
{
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float yaw_rate_dps;
    float pitch_rate_dps;
    float roll_rate_dps;
    float acceleration_x_mps2;
    float acceleration_y_mps2;
    bool acceleration_valid;
    uint32_t last_update_ms;
};

struct ChassisState
{
    float vx;
    float vy;
    float wz;
};

struct GimbalState
{
    float yaw_deg;
    float pitch_deg;
    bool online;
    float joint_yaw_rad = 0.0f;
    float joint_pitch_rad = 0.0f;
    uint8_t mode = 0U;
    bool ready = false;
    uint32_t updated_at_ms = 0U;
};

struct ShooterState
{
    uint16_t friction_wheel_rpm;
    uint16_t ammunition_remaining;
    bool ready;
};

struct RefereeState
{
    uint16_t robot_id;
    uint16_t remaining_hp;
    bool connected;
};

struct RobotCmdState
{
    float vx;
    float vy;
    float wz;
    float chassis_roll;
    float chassis_pitch;
    float chassis_yaw;
    float chassis_leg_length;
    float gimbal_pitch;
    float gimbal_yaw;
    uint8_t fire;
    uint8_t fric_on;
    uint32_t last_update_ms;
};

struct UsbLinkState
{
    bool offline;
};

struct VirtualRcState
{
    RcControl rc;
};

struct SystemStateSnapshot
{
    ImuState imu;
    ChassisState chassis;
    GimbalState gimbal;
    ShooterState shooter;
    RefereeState referee;
    RobotCmdState robot_cmd;
    UsbLinkState usb_link;
    VirtualRcState virtual_rc;
};

class SystemStateStore
{
public:
    // Call these methods from FreeRTOS task context, not from an ISR.
    void updateImu(const ImuState &state);
    void updateChassis(const ChassisState &state);
    void updateGimbal(const GimbalState &state);
    void updateShooter(const ShooterState &state);
    void updateReferee(const RefereeState &state);
    void updateRobotCmd(const RobotCmdState &state);
    void updateUsbLink(const UsbLinkState &state);
    void updateVirtualRc(const VirtualRcState &state);
    SystemStateSnapshot read() const;
};

extern SystemStateStore system_state;

} // namespace LuojiaFox
}

extern "C" {
extern LuojiaFox::SystemStateSnapshot g_system_state;
}

#endif
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
