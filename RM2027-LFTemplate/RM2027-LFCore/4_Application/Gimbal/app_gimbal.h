#pragma once
#include "core_linkage.h"
#include "gimbal_config.h"
#include <stdbool.h>

#if GIMBAL_CONTROL_ENABLED
LF_EXTERN_C bool Gimbal_Init(void);
LF_EXTERN_C void Gimbal_Control(void);
LF_EXTERN_C void Gimbal_Task(void const* argument);
#endif

#ifdef __cplusplus
#include <stdint.h>
namespace gimbal {
enum class Mode : uint8_t {
    kZeroForce = 0, kInit, kGap, kManual, kScan, kAutoAim, kFollow, kFault
};
struct DebugState {
    Mode mode;
    float yaw_rad, pitch_rad;
    float joint_yaw_rad, joint_pitch_rad;
    float target_yaw_rad, target_pitch_rad;
    int16_t yaw_command_raw, pitch_command_raw;
    bool ready, following, init_failed;
    uint32_t tx_failures;
};
extern DebugState g_debug;
} // namespace gimbal
#endif
