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
    float quaternion_wxyz[4] = {};
};

struct ChassisState
{
    float vx = 0, vy = 0, wz = 0; // 底盘坐标系，m/s、m/s、rad/s
    bool valid = false;
    uint32_t last_update_ms = 0; // 副板反馈接收时间，重复读取不得刷新
};

enum class GimbalTransformState : uint8_t {
    Disabled = 0, Recovering = 1, Preparing = 3, Ready = 4, Restoring = 5, Fault = 6
};
enum class GimbalShape : uint8_t { Unknown = 0, Folded = 3, Upright = 4 };

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
    bool yaw_joint_online = false;
    float joint_roll_rad = 0;
    GimbalTransformState transform_state = GimbalTransformState::Disabled;
    GimbalShape transformed = GimbalShape::Unknown;
    bool manual_passage = false;          // CH5 启动的过洞流程，回正结束前保持
    bool manual_passage_aligning = false; // 底盘正在对齐锁定的大 Yaw 航向
    bool manual_passage_recovering = false; // 故障后的机械回中与回正
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

// A1 导航指令及其在主控上的状态。
struct RobotCmdState
{
    float vx;
    float vy;
    float wz;
    uint32_t last_update_ms;
    bool passage_requested = false; // 由折叠/回正命令锁存
    float big_yaw_heading_rad = 0.0f;
    bool chassis_enabled = false;
    bool heading_valid = false;
};

// A2 自瞄射击指令及其在主控上的状态。
struct ShootCmdState
{
    float yaw = 0.0f;         // 电控世界系下期望的云台 yaw，rad
    float pitch = 0.0f;       // 电控世界系下期望的云台 pitch，rad
    uint8_t shoot_mode = 0U;  // 0 不发弹，1 单发，2 连发
    uint32_t last_update_ms = 0U;
    bool enabled = false;     // 由有效 A2 置位，断链立即清零
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
    GimbalState gimbal;
    ShooterState shooter;
    RefereeState referee;
    RobotCmdState robot_cmd;
    ShootCmdState shoot_cmd;
    UsbLinkState usb_link;
    VirtualRcState virtual_rc;
    ImuState muzzle_imu; // imu 仍是固定在大 Yaw 上的板载 INS
    ChassisState chassis;
};

class SystemStateStore
{
public:
    // Call these methods from FreeRTOS task context, not from an ISR.
    void updateImu(const ImuState &state);
    void updateMuzzleImu(const ImuState &state);
    void updateChassis(const ChassisState &state);
    void updateGimbal(const GimbalState &state);
    void updateShooter(const ShooterState &state);
    void updateReferee(const RefereeState &state);
    void updateRobotCmd(const RobotCmdState &state);
    void updateShootCmd(const ShootCmdState &state);
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
