// Chassis board: large yaw only. The small-yaw controller lives in its own file.
#include "app_gimbal.h"
#include "gimbal_config.h"
#if GIMBAL_CONTROL_ENABLED && GIMBAL_BIG_YAW_ENABLE
#include "app_board_link.hpp"
#include "dvc_dji_motor.hpp"
#include "alg_smc.hpp"
#include "lf_core.hpp"
#include "fdcan.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#include "gimbal_axis.hpp"

namespace gimbal {
using namespace detail;
DebugState g_debug{};
namespace {
Axis yaw;
bool initialized = false, imu_seen = false;
bool calibrated = false, init_failed = false, following = false;
float yaw_total = 0, last_yaw = 0;
uint32_t init_start = 0;
Mode mode = Mode::kZeroForce;

struct ControlCycle {
    uint32_t now = 0;
    float pitch = 0;
    float yaw_rate = 0;
    bool online = false;
    bool peer_online = false;
    bool rc_online = false;
    bool stop = false;
    uint8_t sw = 0;
    int16_t rc_yaw = 0;
    board_link::GimbalToChassisPayload remote{};
};

void setMode(Mode next, float yaw_position, float pitch_position) {
    if (mode == next) { return; }
    mode = next;
    yaw.reset(yaw_position);
    (void)pitch_position;
    if (next != Mode::kFollow) { following = false; }
}

void send() {
    bool ok = DjiMotor::sendGroup(yaw.motor.bus(), yaw.motor.txIdentifier());
    if (!ok) {
        ++g_debug.tx_failures;
        yaw.motor.setCommand(0);
        // A FIFO failure cannot retract an old frame. Invalidate calibration
        // so subsequent successful frames cannot resume the old reference.
        calibrated = false;
        init_failed = true;
    }
}

void publishLocalState(float yaw_pos, float pitch_pos, bool online) {
    // This is the heavy big-yaw actuator's local state. GimbalState describes
    // the aiming small-yaw/pitch assembly, so do not publish big yaw there.
    g_debug.mode = mode;
    g_debug.yaw_rad = yaw_pos;
    g_debug.pitch_rad = pitch_pos;
    g_debug.joint_yaw_rad = yaw.joint;
    g_debug.joint_pitch_rad = 0;
    g_debug.target_pitch_rad = 0;
    g_debug.target_yaw_rad = yaw.target;
    g_debug.yaw_command_raw = yaw.motor.command();
    g_debug.pitch_command_raw = 0;
    g_debug.ready = online && calibrated && mode != Mode::kFault &&
                    mode != Mode::kZeroForce;
    g_debug.following = following;
    g_debug.init_failed = init_failed;
    // One-byte acknowledgement is published only after a successful control/send cycle.
    taskENTER_CRITICAL();
    g_debug.passage_hold_mode = (g_debug.ready && !init_failed && isPassageMode(mode)) ?
        static_cast<uint8_t>(mode) : 0U;
    taskEXIT_CRITICAL();
}

void observe(ControlCycle& cycle) {
    cycle.now = HAL_GetTick();
    const auto state = LuojiaFox::system_state.read();
    const auto& imu = state.imu;
    const float y = imu.yaw_deg * kDegToRad;
    cycle.pitch = imu.pitch_deg * kDegToRad;
    
    const float r = imu.roll_deg * kDegToRad;
    const bool imu_ok = imu.valid &&
        (cycle.now - imu.updated_at_ms < GIMBAL_IMU_TIMEOUT_MS) &&
        isfinite(y) && isfinite(cycle.pitch) && isfinite(r) &&
        isfinite(imu.yaw_rate_dps) && isfinite(imu.pitch_rate_dps) &&
        fabsf(cosf(cycle.pitch)) > 0.1f;
    if (imu_ok) {
        if (!imu_seen) { yaw_total = y; imu_seen = true; }
        else { yaw_total += wrap(y - last_yaw); }
        last_yaw = y;
    }
    // INS provides body q/r; the angle loop needs Euler yaw rate.
    const float q = imu.pitch_rate_dps * kDegToRad;
    const float z = imu.yaw_rate_dps * kDegToRad;
    cycle.yaw_rate = imu_ok ?
        (q * sinf(r) + z * cosf(r)) / cosf(cycle.pitch) : 0;
    const bool yaw_ok = yaw.observe(cycle.now);
    cycle.online = imu_ok && yaw_ok;
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
}

void handleException(ControlCycle& cycle) {
    const bool locked = cycle.sw != RC_SW_MID && cycle.sw != RC_SW_UP;
    if (locked || !cycle.rc_online || !cycle.online || !cycle.peer_online) {
        calibrated = false;
        if (cycle.rc_online && cycle.sw == RC_SW_DOWN) { init_failed = false; }
        setMode(Mode::kZeroForce, yaw_total, cycle.pitch);
        yaw.reset(yaw_total);
        cycle.stop = true;
    } else if (init_failed) {
        setMode(Mode::kFault, yaw_total, cycle.pitch);
        yaw.motor.setCommand(0);
        cycle.stop = true;
    }
}

void selectMode(ControlCycle& cycle) {
    if (cycle.stop) { return; }
    const Mode requested = static_cast<Mode>(cycle.remote.gimbal_mode);
    if (isPassageMode(requested)) {
        // Capture once on entry; phase changes must not reset the held heading.
        if (!isPassageMode(mode)) { setMode(requested, yaw_total, cycle.pitch); }
        else { mode = requested; }
        calibrated = true;
        following = false;
        return;
    }
    if (!calibrated) {
        if (mode != Mode::kInit) {
            setMode(Mode::kInit, yaw_total, cycle.pitch);
            init_start = cycle.now;
        }
        return;
    }
    const bool aim = isfinite(cycle.remote.small_yaw_offset_rad) &&
        cycle.remote.gimbal_mode == static_cast<uint8_t>(Mode::kAutoAim);
    Mode next = cycle.sw == RC_SW_MID ? Mode::kManual :
                (aim ? Mode::kAutoAim : Mode::kScan);
    if (!isfinite(cycle.remote.small_yaw_offset_rad) ||
        (cycle.remote.gimbal_mode != static_cast<uint8_t>(Mode::kManual) &&
         cycle.remote.gimbal_mode != static_cast<uint8_t>(Mode::kScan) &&
         cycle.remote.gimbal_mode != static_cast<uint8_t>(Mode::kAutoAim))) {
        next = Mode::kGap;
    } else if (cycle.sw == RC_SW_UP && aim) {
        const float offset = fabsf(cycle.remote.small_yaw_offset_rad);
        following = following ? offset > GIMBAL_FOLLOW_EXIT_RAD :
                                offset >= GIMBAL_FOLLOW_ENTER_RAD;
        if (following) { next = Mode::kFollow; }
    }
    setMode(next, yaw_total, cycle.pitch);
}

void updateReference(const ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (!calibrated) {
        // Center the encoder in the current IMU frame.
        yaw.target = yaw_total - wrap(yaw.joint);
        const bool centered = fabsf(wrap(yaw.joint)) < GIMBAL_INIT_TOLERANCE_RAD;
        yaw.smc.error.tar_last = yaw.target;
        if (centered) {
            calibrated = true;
            setMode(Mode::kGap, yaw_total, cycle.pitch);
        } else if (cycle.now - init_start >= GIMBAL_INIT_TIMEOUT_MS) {
            init_failed = true;
            setMode(Mode::kFault, yaw_total, cycle.pitch);
        }
    } else if (mode == Mode::kManual) {
        yaw.target -= stick(cycle.rc_yaw) * GIMBAL_MANUAL_YAW_RAD_S * kDt;
    } else if (mode == Mode::kScan) {
        yaw.target += GIMBAL_SCAN_YAW_RAD_S * kDt;
    } else if (mode == Mode::kFollow) {
        yaw.target += clamp(GIMBAL_FOLLOW_GAIN * cycle.remote.small_yaw_offset_rad,
            -GIMBAL_FOLLOW_MAX_RAD_S, GIMBAL_FOLLOW_MAX_RAD_S) * kDt;
    }
    // AUTO_AIM and GAP hold the target captured on mode entry.
}

void calculateOutput(const ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (mode == Mode::kFault) {
        yaw.motor.setCommand(0);
    } else if (!yaw.control(yaw_total, cycle.yaw_rate, false, 0, 0)) {
        init_failed = true;
        setMode(Mode::kFault, yaw_total, cycle.pitch);
    }
}
} // namespace
} // namespace gimbal

extern "C" bool Gimbal_Init(void) {
    using namespace gimbal;
    if (initialized) { return true; }
    const float big_params[9] = GIMBAL_BIG_YAW_SMC_PARAMS;
    bool ok = yaw.init(GIMBAL_BIG_YAW_TYPE, GIMBAL_BIG_YAW_CAN,
            GIMBAL_BIG_YAW_ID, GIMBAL_BIG_YAW_DIRECTION, GIMBAL_BIG_YAW_RATIO,
            GIMBAL_BIG_YAW_MID_ECD, big_params, EXPONENT, GIMBAL_BIG_YAW_OUTPUT_MAX);
    // One task must own each shared DJI TX frame. Reject groups already
    // driven by the chassis or another application, even with different slots.
    for (uint8_t i = 0; ok && i < MotorBase::motorCount(); ++i) {
        const MotorBase* other = MotorBase::motorAt(i);
        if (other == &yaw.motor) { continue; }
        if (other->bus() == yaw.motor.bus() &&
            other->txIdentifier() == yaw.motor.txIdentifier()) {
            ok = false;
        }
    }
    initialized = ok;
    return ok;
}

extern "C" void Gimbal_Control(void) {
    using namespace gimbal;
    if (!initialized) { return; }
    ControlCycle cycle{};
    observe(cycle);
    handleException(cycle);
    selectMode(cycle);
    updateReference(cycle);
    calculateOutput(cycle);
    send();
    publishLocalState(yaw_total, cycle.pitch, cycle.online);
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
#endif // GIMBAL_CONTROL_ENABLED && GIMBAL_BIG_YAW_ENABLE
