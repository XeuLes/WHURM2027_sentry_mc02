#pragma once
#include "dvc_board_link_can.hpp"
#include "chassis_command.hpp"
#include "referee.hpp"

namespace board_link {
struct BigYawTarget {
    float target_rad = 0, position_rad = 0, velocity_rad_s = 0;
    float acceleration_rad_s2 = 0;
    bool enable = false, valid = false, joint_frame = false;
    uint32_t revision = 0, rx_ms = 0;
};
struct BigYawFeedback {
    float joint_rad = 0;
    bool motor_online = false, ready = false, slow = false;
    uint32_t updated_at_ms = 0;
};
struct ChassisFeedback {
    float vx = 0, vy = 0, wz = 0;
    bool valid = false;
    uint32_t updated_at_ms = 0;
};
// Called only by the chassis task; modes are selected by mainControl.
chassis_app::ChassisCommandSelection getChassisSelection(uint32_t now);
ChassisModeTypedef getChassisMode();
BigYawTarget getBigYawTarget(uint32_t now);
RefereePowerSnapshot getRefereePowerSnapshot();
void publishBigYawFeedback(const BigYawFeedback& feedback);
void sendFeedback(uint32_t now);
void publishChassisFeedback(const ChassisFeedback& feedback);
}
extern "C" void BoardLink_Task(void const* argument);
