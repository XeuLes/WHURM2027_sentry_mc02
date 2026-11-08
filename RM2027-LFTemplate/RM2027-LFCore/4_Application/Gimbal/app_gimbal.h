#pragma once
#include "core_linkage.h"
#include <stdbool.h>

LF_EXTERN_C bool Gimbal_Init(void);
LF_EXTERN_C void Gimbal_Control(void);
LF_EXTERN_C void Gimbal_Task(void const* argument);

#ifdef __cplusplus
#include <stdint.h>
namespace gimbal {
enum class Mode : uint8_t {
    kZeroForce = 0, kInit, kGap, kManual, kScan, kAutoAim, kFollow, kFault,
    kPassage = 14 // Wire IDs 8-13 were used by the old passage phases.
};
enum class PassageStage : uint8_t {
    kIdle = 0, kAlign, kFold, kHold, kUnfold, kWait, kFault
};
struct DebugState {
    Mode mode;
    float yaw_rad, pitch_rad;
    float joint_yaw_rad, joint_pitch_rad;
    float target_yaw_rad, target_pitch_rad;
    int16_t yaw_command_raw; // GM6020 only
    float yaw_torque_nm, pitch_torque_nm, roll_torque_nm;
    bool ready, following, init_failed;
    uint32_t tx_failures;
    float joint_roll_rad, target_roll_rad;
    uint8_t passage_hold_mode;
    PassageStage passage_stage;
    bool joint_control;
};
extern DebugState g_debug;
} // namespace gimbal
#endif
