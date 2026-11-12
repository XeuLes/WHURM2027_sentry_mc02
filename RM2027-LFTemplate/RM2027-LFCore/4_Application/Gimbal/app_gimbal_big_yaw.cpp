// subControl executes mainControl references; no local mode or trajectory planner.
#include "app_gimbal.h"
#include "app_board_link.hpp"
#include "fdcan.h"
#include "cmsis_os.h"
#include "gimbal_axis.hpp"

namespace gimbal {
using namespace detail;
DebugState g_debug{};
namespace {
DmAxis yaw;
bool initialized = false, fault = false, controlling = false;
bool previous_joint_frame = false;
uint32_t previous_revision = 0, previous_rx_ms = 0;
float previous_target = 0, reference_velocity = 0;
}
}
extern "C" bool Gimbal_Init(void) {
    using namespace gimbal;
    if (initialized) { return true; }
    const float params[9] = GIMBAL_BIG_YAW_SMC_PARAMS;
    initialized = yaw.init(GIMBAL_BIG_YAW_CAN, GIMBAL_BIG_YAW_ID, GIMBAL_BIG_YAW_MASTER_ID,
        GIMBAL_BIG_YAW_DIRECTION, GIMBAL_BIG_YAW_MID_RAD, params,
        EXPONENT, GIMBAL_BIG_YAW_OUTPUT_MAX_NM);
    return initialized;
}
extern "C" void Gimbal_Control(void) {
    using namespace gimbal;
    if (!initialized) { return; }
    const uint32_t now = HAL_GetTick();
    const auto target = board_link::getBigYawTarget(now);
    const bool motor_ok = yaw.observe(now);
    const bool fresh = target.revision != 0U && now - target.rx_ms < board_link::can3::kBigYawTimeoutMs;
    // Explicit master disable clears a fault. Link loss alone cannot clear it.
    if (fresh && !target.enable) { fault = false; }
    if (controlling && target.enable && (!motor_ok || !yaw.ready)) { fault = true; }
    bool enable = target.enable && !fault;
    const bool run = enable && target.valid && motor_ok && yaw.ready;
    const float position = target.joint_frame ? yaw.joint : target.position_rad;
    const float rate = target.joint_frame ? yaw.velocity : target.velocity_rad_s;
    if (!run) {
        yaw.reset(position);
        reference_velocity = 0;
    } else {
        const bool reset = !controlling || previous_joint_frame != target.joint_frame;
        if (reset) { yaw.reset(position); }
        if (target.joint_frame) {
            reference_velocity = target.velocity_rad_s;
        } else if (reset) {
            reference_velocity = 0;
        } else if (target.revision != previous_revision) {
            const uint32_t elapsed = target.rx_ms - previous_rx_ms;
            reference_velocity = elapsed != 0U
                ? wrap(target.target_rad - previous_target) / (elapsed * 0.001f) : 0.0f;
        }
        yaw.target = target.target_rad;
        // Use received trajectory speed, or target differences over actual RX
        // intervals. Reusing a packet must not generate 1 ms derivative spikes.
        yaw.smc.error.tar_last = yaw.target - reference_velocity * kDt;
        const float ff = target.joint_frame ? yaw.smc.param.J * target.acceleration_rad_s2 : 0.0f;
        if (!yaw.control(position, rate, false, 0, 0, true, ff)) {
            fault = true;
            enable = false;
        }
    }
    if (!yaw.send(now, enable, run && !fault ? yaw.output : 0.0f)) {
        ++g_debug.tx_failures;
        fault = true;
        yaw.output = 0;
    }
    controlling = run && !fault;
    previous_joint_frame = target.joint_frame;
    previous_revision = target.revision;
    previous_rx_ms = target.rx_ms;
    previous_target = target.target_rad;
    board_link::publishBigYawFeedback({yaw.joint, motor_ok,
        motor_ok && yaw.ready && enable && !fault,
        motor_ok && fabsf(yaw.velocity) < board_link::can3::kBigYawSlowSpeedRadS, now});
    g_debug.mode = fault ? Mode::kFault : !enable ? Mode::kZeroForce :
        target.joint_frame ? Mode::kPassage : Mode::kGap;
    g_debug.yaw_rad = position;
    g_debug.joint_yaw_rad = yaw.joint;
    g_debug.target_yaw_rad = yaw.target;
    g_debug.yaw_torque_nm = controlling ? yaw.output : 0;
    g_debug.joint_control = target.joint_frame;
    g_debug.ready = controlling;
    g_debug.init_failed = fault;
}
extern "C" void Gimbal_Task(void const* argument) {
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        Gimbal_Control();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(GIMBAL_PERIOD_MS));
    }
}
