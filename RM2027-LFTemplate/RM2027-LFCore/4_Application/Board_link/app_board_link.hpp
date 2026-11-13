#pragma once

#include "dvc_board_link_can.hpp"

namespace board_link {

struct ChassisTarget {
    float vx;
    float vy;
    float wz;
    uint8_t mode;
    bool valid;
    uint32_t updated_at_ms;
    bool motion_blocked = false;
};

struct BigYawTarget {
    float target_rad;
    float position_rad;
    float velocity_rad_s;
    float big_yaw_to_chassis_rad;
    float muzzle_to_chassis_rad;
    float passage_acceleration_rad_s2;
    bool enable;
    bool valid;
    bool joint_frame;
    bool big_yaw_offset_valid;
    bool muzzle_offset_valid;
    uint32_t updated_at_ms;
};

struct BigYawFeedback {
    float joint_rad;
    bool motor_online;
    bool ready;
    bool slow;
    uint32_t revision;
    uint32_t rx_ms;
};

void publishChassisTarget(const ChassisTarget& target);
void publishBigYawTarget(const BigYawTarget& target);
bool getPeerBigYawFeedback(BigYawFeedback& feedback);

bool isPeerOnline();

} // namespace board_link

extern "C" void BoardLink_Task(void const* argument);

