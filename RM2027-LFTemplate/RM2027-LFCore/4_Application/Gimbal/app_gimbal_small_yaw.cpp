// Gimbal board: small yaw and pitch only. The large-yaw controller is separate.
#include "app_gimbal.h"
#include "gimbal_config.h"
#if GIMBAL_CONTROL_ENABLED && GIMBAL_SMALL_ENABLE
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
Axis pitch;
bool initialized = false, imu_seen = false;
bool calibrated = false, init_failed = false;
float yaw_total = 0, last_yaw = 0, scan_phase = 0;
uint32_t init_start = 0;
Mode mode = Mode::kZeroForce;

struct ControlCycle {
    uint32_t now = 0;
    float yaw = 0;
    float pitch = 0;
    float yaw_rate = 0;
    float pitch_rate = 0;
    float aim_yaw = 0;
    float aim_pitch = 0;
    bool aim_valid = false;
    bool online = false;
    bool peer_online = false;
    bool rc_online = false;
    bool stop = false;
    uint8_t sw = 0;
    int16_t rc_yaw = 0;
    int16_t rc_pitch = 0;
};

void setMode(Mode next, float yaw_position, float pitch_position) {
    if (mode == next) { return; }
    mode = next;
    // SCAN centers the joint relative to big yaw; other modes aim in IMU space.
    // Reset target history in the new frame so frame changes create no target-rate spike.
    yaw.reset(next == Mode::kScan ? 0.0f : yaw_position);
    pitch.reset(pitch_position);
    scan_phase = 0;
}

void send() {
    bool ok = DjiMotor::sendGroup(yaw.motor.bus(), yaw.motor.txIdentifier());
    if (pitch.motor.bus() != yaw.motor.bus() ||
         pitch.motor.txIdentifier() != yaw.motor.txIdentifier()) {
        ok = DjiMotor::sendGroup(pitch.motor.bus(), pitch.motor.txIdentifier()) && ok;
    }
    if (!ok) {
        ++g_debug.tx_failures;
        yaw.motor.setCommand(0);
        pitch.motor.setCommand(0);
        // A FIFO failure cannot retract an old frame. Invalidate calibration
        // so subsequent successful frames cannot resume the old reference.
        calibrated = false;
        init_failed = true;
    }
}

void publish(float yaw_pos, float pitch_pos, bool online) {
    g_debug.mode = mode;
    g_debug.yaw_rad = yaw_pos;
    g_debug.pitch_rad = pitch_pos;
    g_debug.joint_yaw_rad = yaw.joint;
    g_debug.joint_pitch_rad = pitch.joint;
    g_debug.target_pitch_rad = pitch.target;
    // Debug targets always use the IMU frame, including joint-centering SCAN.
    g_debug.target_yaw_rad = mode == Mode::kScan ?
        yaw_pos + yaw.target - yaw.joint : yaw.target;
    g_debug.yaw_command_raw = yaw.motor.command();
    g_debug.pitch_command_raw = pitch.motor.command();
    g_debug.ready = online && calibrated && mode != Mode::kFault &&
                    mode != Mode::kZeroForce;
    g_debug.following = false;
    g_debug.init_failed = init_failed;
    LuojiaFox::GimbalState state{};
    state.yaw_deg = yaw_pos / kDegToRad;
    state.pitch_deg = pitch_pos / kDegToRad;
    state.online = online;
    state.joint_yaw_rad = yaw.joint;
    state.joint_pitch_rad = g_debug.joint_pitch_rad;
    state.mode = static_cast<uint8_t>(mode);
    state.ready = g_debug.ready;
    LuojiaFox::system_state.updateGimbal(state);
}

void observe(ControlCycle& cycle) {
    cycle.now = HAL_GetTick();
    const auto state = LuojiaFox::system_state.read();
    const auto& imu = state.imu;
    cycle.yaw = imu.yaw_deg * kDegToRad;
    cycle.pitch = imu.pitch_deg * kDegToRad;
    const float r = imu.roll_deg * kDegToRad;
    const bool imu_ok = imu.valid &&
        (cycle.now - imu.updated_at_ms < GIMBAL_IMU_TIMEOUT_MS) &&
        isfinite(cycle.yaw) && isfinite(cycle.pitch) && isfinite(r) &&
        isfinite(imu.yaw_rate_dps) && isfinite(imu.pitch_rate_dps) &&
        fabsf(cosf(cycle.pitch)) > 0.1f;
    if (imu_ok) {
        if (!imu_seen) { yaw_total = cycle.yaw; imu_seen = true; }
        else { yaw_total += wrap(cycle.yaw - last_yaw); }
        last_yaw = cycle.yaw;
    }
    // INS provides body q/r; the angle loops need Euler rates.
    const float q = imu.pitch_rate_dps * kDegToRad;
    const float z = imu.yaw_rate_dps * kDegToRad;
    cycle.yaw_rate = imu_ok ?
        (q * sinf(r) + z * cosf(r)) / cosf(cycle.pitch) : 0;
    cycle.pitch_rate = q * cosf(r) - z * sinf(r);
    const bool yaw_ok = yaw.observe(cycle.now);
    const bool pitch_ok = pitch.observe(cycle.now);
    cycle.online = imu_ok && yaw_ok && pitch_ok;
    cycle.peer_online = board_link::isPeerOnline(cycle.now);
    cycle.rc_online = !getRcOffline();
    const RcControl* rc = getRemoteControlPoint();
    if (rc != nullptr) {
        taskENTER_CRITICAL();
        cycle.sw = rc->rc.s[RC_SW_RIGHT];
        cycle.rc_yaw = rc->rc.ch[RC_CH_RIGHT_HORIZONTAL];
        cycle.rc_pitch = rc->rc.ch[RC_CH_RIGHT_VERTICAL];
        taskEXIT_CRITICAL();
    }
    cycle.aim_yaw = state.robot_cmd.gimbal_yaw;
    cycle.aim_pitch = state.robot_cmd.gimbal_pitch;
    cycle.aim_valid = GIMBAL_AUTO_AIM_ENABLE && state.robot_cmd.valid &&
        cycle.now - state.robot_cmd.updated_at_ms < GIMBAL_AIM_TIMEOUT_MS &&
        isfinite(cycle.aim_yaw) && isfinite(cycle.aim_pitch);
}

void handleException(ControlCycle& cycle) {
    const bool locked = cycle.sw != RC_SW_MID && cycle.sw != RC_SW_UP;
    if (locked || !cycle.rc_online || !cycle.online || !cycle.peer_online) {
        calibrated = false;
        if (cycle.rc_online && cycle.sw == RC_SW_DOWN) { init_failed = false; }
        setMode(Mode::kZeroForce, yaw_total, cycle.pitch);
        yaw.reset(yaw_total);
        pitch.reset(cycle.pitch);
        cycle.stop = true;
    } else if (init_failed) {
        setMode(Mode::kFault, yaw_total, cycle.pitch);
        yaw.motor.setCommand(0);
        pitch.motor.setCommand(0);
        cycle.stop = true;
    }
}

void selectMode(ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (!calibrated) {
        if (mode != Mode::kInit) {
            setMode(Mode::kInit, yaw_total, cycle.pitch);
            init_start = cycle.now;
        }
        return;
    }
    const Mode next = cycle.sw == RC_SW_MID ? Mode::kManual :
                      (cycle.aim_valid ? Mode::kAutoAim : Mode::kScan);
    setMode(next, yaw_total, cycle.pitch);
}

void updateReference(const ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (!calibrated) {
        // Center both encoders in the current IMU frame.
        yaw.target = yaw_total - yaw.joint;
        pitch.target = cycle.pitch - pitch.joint;
        pitch.smc.error.tar_last = pitch.target;
        const bool centered = fabsf(yaw.joint) < GIMBAL_INIT_TOLERANCE_RAD &&
                              fabsf(pitch.joint) < GIMBAL_INIT_TOLERANCE_RAD;
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
        pitch.target -= stick(cycle.rc_pitch) * GIMBAL_MANUAL_PITCH_RAD_S * kDt;
    } else if (mode == Mode::kScan) {
        yaw.target = 0.0f;
        scan_phase += 2.0f * kPi * kDt / GIMBAL_SCAN_PITCH_PERIOD_S;
        scan_phase = wrap(scan_phase);
        pitch.target = cycle.pitch - pitch.joint +
            GIMBAL_SCAN_PITCH_AMPLITUDE_RAD * sinf(scan_phase);
    } else if (mode == Mode::kAutoAim) {
        yaw.target = yaw_total + wrap(cycle.aim_yaw *
            GIMBAL_AIM_ANGLE_TO_RAD - cycle.yaw);
        pitch.target = cycle.aim_pitch * GIMBAL_AIM_ANGLE_TO_RAD;
    }
}

void calculateOutput(const ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (mode == Mode::kFault) {
        yaw.motor.setCommand(0);
        pitch.motor.setCommand(0);
        return;
    }
    // Relative motor velocity excludes big-yaw rotation. Using IMU yaw rate
    // here would brake the parent rotation even when the small joint is centered.
    const bool center_joint = mode == Mode::kScan;
    const float yaw_position = center_joint ? yaw.joint : yaw_total;
    const float yaw_rate = center_joint ? yaw.motor.data().velocity_rad_s : cycle.yaw_rate;
    yaw.target = clamp(yaw.target, yaw_position + GIMBAL_SMALL_YAW_MIN_RAD - yaw.joint,
                                      yaw_position + GIMBAL_SMALL_YAW_MAX_RAD - yaw.joint);
    pitch.target = clamp(pitch.target,
        cycle.pitch + GIMBAL_PITCH_MIN_RAD - pitch.joint,
        cycle.pitch + GIMBAL_PITCH_MAX_RAD - pitch.joint);
    const bool y_valid = yaw.control(yaw_position, yaw_rate, true,
        GIMBAL_SMALL_YAW_MIN_RAD, GIMBAL_SMALL_YAW_MAX_RAD);
    const bool p_valid = pitch.control(cycle.pitch, cycle.pitch_rate, true,
        GIMBAL_PITCH_MIN_RAD, GIMBAL_PITCH_MAX_RAD);
    if (!y_valid || !p_valid) {
        init_failed = true;
        setMode(Mode::kFault, yaw_total, cycle.pitch);
    }
}
} // namespace
} // namespace gimbal

extern "C" bool Gimbal_Init(void) {
    using namespace gimbal;
    if (initialized) { return true; }
    const float yaw_params[9] = GIMBAL_YAW_SMC_PARAMS;
    const float pitch_params[9] = GIMBAL_PITCH_SMC_PARAMS;
    bool ok = yaw.init(GIMBAL_YAW_TYPE, GIMBAL_YAW_CAN, GIMBAL_YAW_ID,
            GIMBAL_YAW_DIRECTION, GIMBAL_YAW_RATIO, GIMBAL_YAW_MID_ECD,
            yaw_params, EXPONENT, GIMBAL_YAW_OUTPUT_MAX) &&
             pitch.init(GIMBAL_PITCH_TYPE, GIMBAL_PITCH_CAN, GIMBAL_PITCH_ID,
            GIMBAL_PITCH_DIRECTION, GIMBAL_PITCH_RATIO, GIMBAL_PITCH_MID_ECD,
            pitch_params, EISMC, GIMBAL_PITCH_OUTPUT_MAX);
    // One task must own each shared DJI TX frame. Reject groups already
    // driven by the chassis or another application, even with different slots.
    for (uint8_t i = 0; ok && i < MotorBase::motorCount(); ++i) {
        const MotorBase* other = MotorBase::motorAt(i);
        if (other == &yaw.motor) { continue; }
        if (other == &pitch.motor) { continue; }
        bool shared = other->bus() == yaw.motor.bus() &&
                      other->txIdentifier() == yaw.motor.txIdentifier();
        shared = shared || (other->bus() == pitch.motor.bus() &&
                            other->txIdentifier() == pitch.motor.txIdentifier());
        if (shared) {
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
    publish(yaw_total, cycle.pitch, cycle.online);
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
#endif // GIMBAL_CONTROL_ENABLED && GIMBAL_SMALL_ENABLE
