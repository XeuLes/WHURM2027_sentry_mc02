// Chassis board: large yaw only. The small-yaw controller lives in its own file.
#include "app_gimbal.h"
#if BOARD_ROLE == 1
#include "app_board_link.hpp"
#include "lf_core.hpp"
#include "fdcan.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#include "gimbal_axis.hpp"
#include "gimbal_config.h"

namespace gimbal {
using namespace detail;
DebugState g_debug{};
namespace {
DmAxis yaw;
JointTrajectory passage_path;
uint32_t passage_centered_ms = 0;
bool initialized = false, init_failed = false;
Mode mode = Mode::kZeroForce;

struct ControlCycle {
    uint32_t now = 0;
    float yaw_position = 0;
    float pitch = 0;
    float yaw_rate = 0;
    float yaw_feedforward_nm = 0;
    bool imu_ok = false;
    bool motor_ok = false;
    bool peer_online = false;
    bool rc_online = false;
    bool stop = false;
    bool enable = false;
    uint8_t sw = 0;
    int16_t rc_yaw = 0;
    Mode small_mode = Mode::kZeroForce;
    board_link::GimbalToChassisPayload remote{};
};

void setMode(Mode next, float yaw_position) {
    if (mode == next) { return; }
    mode = next;
    yaw.reset(mode == Mode::kPassage ? yaw.joint : yaw_position);
    passage_centered_ms = 0;
    if (mode == Mode::kPassage) {
        passage_path.reset(yaw.joint, yaw.velocity);
    }
}

void send(const ControlCycle& cycle) {
    const bool ok = yaw.send(cycle.now, cycle.enable && !init_failed,
                             cycle.stop ? 0.0f : yaw.output);
    if (!ok) {
        ++g_debug.tx_failures;
        yaw.output = 0;
        init_failed = true;
    }
}

void publishLocalState(const ControlCycle& cycle) {
    // This is the heavy big-yaw actuator's local state. GimbalState describes
    // the aiming small-yaw/pitch assembly, so do not publish big yaw there.
    g_debug.mode = mode;
    g_debug.yaw_rad = cycle.yaw_position;
    g_debug.pitch_rad = cycle.pitch;
    g_debug.joint_yaw_rad = yaw.joint;
    g_debug.joint_pitch_rad = 0;
    g_debug.target_pitch_rad = 0;
    g_debug.target_yaw_rad = yaw.target;
    g_debug.yaw_torque_nm = yaw.output;
    g_debug.joint_control = mode == Mode::kPassage;
    g_debug.ready = !cycle.stop && !init_failed &&
                    mode != Mode::kFault && mode != Mode::kZeroForce;
    g_debug.following = mode == Mode::kFollow;
    g_debug.init_failed = init_failed;
    bool centered = false;
    if (g_debug.ready && mode == Mode::kPassage) {
        if (passage_centered_ms < GIMBAL_PASS_BIG_YAW_SETTLE_MS) {
            const bool settled = jointSettled(yaw, 0.0f) &&
                fabsf(passage_path.position) < GIMBAL_PASS_POSITION_TOL_RAD &&
                fabsf(passage_path.velocity) < GIMBAL_PASS_SPEED_TOL_RAD_S;
            passage_centered_ms = settled ? passage_centered_ms + GIMBAL_PERIOD_MS : 0;
        }
        // Allow small holding corrections, but withdraw the acknowledgement on a large drift.
        centered = passage_centered_ms >= GIMBAL_PASS_BIG_YAW_SETTLE_MS &&
            fabsf(yaw.joint) < 2.0f * GIMBAL_PASS_POSITION_TOL_RAD;
    }
    taskENTER_CRITICAL();
    g_debug.passage_yaw_centered = centered ? 1U : 0U;
    taskEXIT_CRITICAL();
}

void observe(ControlCycle& cycle) {
    cycle.now = HAL_GetTick();
    const auto state = LuojiaFox::system_state.read();
    const auto& imu = state.imu;
    cycle.yaw_position = imu.yaw_deg * kDegToRad;
    cycle.pitch = imu.pitch_deg * kDegToRad;

    const float r = imu.roll_deg * kDegToRad;
    cycle.imu_ok = imu.last_update_ms != 0U &&
        (cycle.now - imu.last_update_ms < GIMBAL_IMU_TIMEOUT_MS) &&
        isfinite(cycle.yaw_position) && isfinite(cycle.pitch) && isfinite(r) &&
        isfinite(imu.yaw_rate_dps) && isfinite(imu.roll_rate_dps) &&
        fabsf(cosf(cycle.pitch)) > 0.1f;
    // The INS publishes body Y (q) in roll_rate_dps.
    const float q = imu.roll_rate_dps * kDegToRad;
    const float z = imu.yaw_rate_dps * kDegToRad;
    cycle.yaw_rate = cycle.imu_ok ?
        (q * sinf(r) + z * cosf(r)) / cosf(cycle.pitch) : 0;
    cycle.motor_ok = yaw.observe(cycle.now);
    cycle.peer_online = board_link::isPeerOnline(cycle.now);
    cycle.rc_online = !getRcOffline();
    const RcControl* rc = getRemoteControlPoint();
    if (rc != nullptr) {
        taskENTER_CRITICAL();
        cycle.sw = rc->rc.s[RC_SW_RIGHT];
        cycle.rc_yaw = rc->rc.ch[RC_CH_RIGHT_HORIZONTAL];
        taskEXIT_CRITICAL();
    }
    board_link::getRemoteGimbalData(&cycle.remote);
    cycle.small_mode = static_cast<Mode>(cycle.remote.gimbal_mode);
}

Mode selectAutomaticMode(const ControlCycle& cycle) {
    switch (cycle.small_mode) {
    case Mode::kZeroForce:
    case Mode::kPassage:
        return cycle.small_mode;
    case Mode::kScan:
        // Capture the current heading in GAP before starting or restarting scan.
        return mode == Mode::kGap || mode == Mode::kScan ? Mode::kScan : Mode::kGap;
    case Mode::kAutoAim: {
        const float offset = fabsf(cycle.remote.small_yaw_offset_rad);
        if (!isfinite(offset)) { return Mode::kGap; }
        const bool follow = mode == Mode::kFollow ? offset > GIMBAL_FOLLOW_EXIT_RAD :
                                                   offset >= GIMBAL_FOLLOW_ENTER_RAD;
        return follow ? Mode::kFollow : Mode::kAutoAim;
    }
    default:
        return Mode::kGap;
    }
}

Mode selectMode(const ControlCycle& cycle) {
    if (!cycle.rc_online) { return selectAutomaticMode(cycle); }

    switch (cycle.sw) {
    case RC_SW_MID:
        // Keep the heading locked until the small gimbal completes passage.
        return cycle.small_mode == Mode::kPassage ? Mode::kPassage : Mode::kManual;
    case RC_SW_UP:
        return selectAutomaticMode(cycle);
    default:
        return Mode::kZeroForce;
    }
}

void handleException(ControlCycle& cycle, Mode next) {
    const bool feedback_ok = cycle.motor_ok && (next == Mode::kPassage || cycle.imu_ok);
    if (next == Mode::kZeroForce || cycle.small_mode == Mode::kFault ||
        !feedback_ok || !cycle.peer_online) {
        if (cycle.rc_online && cycle.sw == RC_SW_DOWN) {
            init_failed = false;
        }
        setMode(Mode::kZeroForce, cycle.yaw_position);
        yaw.output = 0;
        cycle.stop = true;
        return;
    }
    if (!yaw.ready && mode != Mode::kZeroForce) { init_failed = true; }
    if (init_failed) {
        setMode(Mode::kFault, cycle.yaw_position);
        yaw.output = 0;
        cycle.stop = true;
        return;
    }
    cycle.enable = true;
    cycle.stop = !yaw.ready;
}

void updateReference(ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (mode == Mode::kPassage) {
        if (passage_centered_ms >= GIMBAL_PASS_BIG_YAW_SETTLE_MS) {
            yaw.target = 0.0f;
            return;
        }
        const float previous_velocity = passage_path.velocity;
        yaw.target = passage_path.advance(0.0f, GIMBAL_PASS_BIG_YAW_SPEED_RAD_S,
                                            GIMBAL_PASS_BIG_YAW_ACCEL_RAD_S2);
        cycle.yaw_feedforward_nm = yaw.smc.param.J * (passage_path.velocity - previous_velocity) / kDt;
        return;
    }
    float rate = 0.0f;
    if (mode == Mode::kManual) {
        rate = -stick(cycle.rc_yaw) * GIMBAL_MANUAL_YAW_RAD_S;
    } else if (mode == Mode::kScan) {
        rate = GIMBAL_SCAN_YAW_RAD_S;
    } else if (mode == Mode::kFollow) {
        rate = clamp(GIMBAL_FOLLOW_GAIN * cycle.remote.small_yaw_offset_rad,
            -GIMBAL_FOLLOW_MAX_RAD_S, GIMBAL_FOLLOW_MAX_RAD_S);
    }
    if (rate == 0.0f) { return; }
    const float lead = wrap(yaw.target - cycle.yaw_position) + rate * kDt;
    yaw.target = wrap(cycle.yaw_position +
        clamp(lead, -GIMBAL_BIG_YAW_MAX_LEAD_RAD, GIMBAL_BIG_YAW_MAX_LEAD_RAD));
}

void calculateOutput(const ControlCycle& cycle) {
    if (cycle.stop) { return; }
    const bool mechanical_hold = mode == Mode::kPassage;
    const float position = mechanical_hold ? yaw.joint : cycle.yaw_position;
    const float rate = mechanical_hold ? yaw.velocity : cycle.yaw_rate;
    if (!yaw.control(position, rate, false, 0, 0, true, cycle.yaw_feedforward_nm)) {
        init_failed = true;
        setMode(Mode::kFault, cycle.yaw_position);
    }
}
} // namespace
} // namespace gimbal

extern "C" bool Gimbal_Init(void) {
    using namespace gimbal;
    if (initialized) { return true; }
    const float big_params[9] = GIMBAL_BIG_YAW_SMC_PARAMS;
    initialized = yaw.init(GIMBAL_BIG_YAW_CAN, GIMBAL_BIG_YAW_ID, GIMBAL_BIG_YAW_MASTER_ID,
                           GIMBAL_BIG_YAW_DIRECTION, GIMBAL_BIG_YAW_MID_RAD, big_params,
                           EXPONENT, GIMBAL_BIG_YAW_OUTPUT_MAX_NM);
    return initialized;
}

extern "C" void Gimbal_Control(void) {
    using namespace gimbal;
    if (!initialized) { return; }
    ControlCycle cycle{};
    observe(cycle);
    const Mode next = selectMode(cycle);
    handleException(cycle, next);
    if (!cycle.stop) { setMode(next, cycle.yaw_position); }
    updateReference(cycle);
    calculateOutput(cycle);
    send(cycle);
    publishLocalState(cycle);
}

extern "C" void Gimbal_Task(void const* argument) {
    (void)argument;
    // Registration is completed in main(), before the scheduler starts.
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(GIMBAL_PERIOD_MS);
    configASSERT(period > 0U);
    for (;;) {
        Gimbal_Control();
        vTaskDelayUntil(&wake, period);
    }
}
#endif // BOARD_ROLE == 1
