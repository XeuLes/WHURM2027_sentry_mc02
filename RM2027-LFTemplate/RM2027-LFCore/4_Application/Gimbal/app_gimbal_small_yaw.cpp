// 主控直驱小 Yaw、Roll 和 Pitch。
#include "app_gimbal.h"
#include "app_board_link.hpp"
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
DjiAxis yaw;
DmAxis pitch, roll;
struct PassageControl {
    PassageStage stage = PassageStage::kIdle;
    JointTrajectory yaw_path, pitch_path, roll_path;
    float yaw_goal = 0, pitch_goal = GIMBAL_PASS_PITCH_RAD;
    uint32_t stage_start = 0, settled_ms = 0, entry_feedback_revision = 0;
    bool center_confirmed = false, rearmed = false;
} passage;
bool initialized = false;
bool calibrated = false, init_failed = false;
float scan_phase = 0;
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
    float roll_feedforward_nm = 0;
    bool online = false;
    bool yaw_joint_online = false;
    bool peer_online = false;
    bool rc_online = false;
    bool stop = false;
    bool enable = false;
    bool imu_ok = false;
    bool passage_requested = false;
    Mode normal_mode = Mode::kZeroForce;
    bool big_yaw_centered = false;
    uint32_t big_yaw_feedback_revision = 0;
    uint8_t sw = 0;
    int16_t rc_pitch = 0;
};

bool useJointPitch(Mode selected) {
    return selected == Mode::kInit || selected == Mode::kScan || selected == Mode::kPassage;
}

void setMode(Mode next, float yaw_position, float pitch_position) {
    if (mode == next) { return; }
    mode = next;
    if (next == Mode::kPassage) { return; }
    // Reset history in the new frame. Manual and scan both center small yaw.
    yaw.reset(next == Mode::kAutoAim ? yaw_position : 0.0f);
    pitch.reset(useJointPitch(next) ? 0.0f : pitch_position);
    roll.reset(roll.joint);
    scan_phase = 0;
}

void setPassageStage(PassageStage next, uint32_t now) {
    passage.stage = next;
    passage.stage_start = now;
    passage.settled_ms = 0;
    if (next == PassageStage::kFold) {
        // The aligned joints now hold fixed references with zero target velocity.
        yaw.smc.error.tar_last = passage.yaw_goal;
        pitch.smc.error.tar_last = passage.pitch_goal;
    }
    roll.reset(roll.joint);
    passage.roll_path.reset(roll.joint, roll.velocity);
}

void startPassage(const ControlCycle& cycle, PassageStage first, float yaw_goal, float pitch_goal) {
    passage.yaw_goal = yaw_goal;
    passage.pitch_goal = pitch_goal;
    passage.entry_feedback_revision = cycle.big_yaw_feedback_revision;
    passage.center_confirmed = false;
    passage.rearmed = false;
    setMode(Mode::kPassage, cycle.yaw, cycle.pitch);
    yaw.reset(yaw.joint);
    pitch.reset(pitch.joint);
    if (first == PassageStage::kAlign) {
        passage.yaw_path.reset(yaw.joint, yaw.velocity);
        passage.pitch_path.reset(pitch.joint, pitch.velocity);
    }
    setPassageStage(first, cycle.now);
}

void latchPassageFault() {
    passage.stage = PassageStage::kFault;
    passage.rearmed = false;
}

void send(const ControlCycle& cycle) {
    const bool enable = cycle.enable && !init_failed;
    const bool active = enable && !cycle.stop;
    yaw.motor.setCommand(active ? static_cast<int16_t>(yaw.output) : 0);
    bool ok = DjiMotor::sendGroup(yaw.motor.bus(), yaw.motor.txIdentifier());
    ok = pitch.send(cycle.now, enable, active ? pitch.output : 0.0f) && ok;
    ok = roll.send(cycle.now, enable, active ? roll.output : 0.0f) && ok;
    if (!ok) {
        ++g_debug.tx_failures;
        yaw.output = pitch.output = roll.output = 0;
        if (mode == Mode::kPassage) {
            latchPassageFault();
            setMode(Mode::kFault, cycle.yaw, cycle.pitch);
        }
        // A FIFO failure cannot retract an old frame. Invalidate calibration
        // so subsequent successful frames cannot resume the old reference.
        calibrated = false;
        init_failed = true;
    }
}

void publish(const ControlCycle& cycle) {
    const float yaw_pos = cycle.yaw;
    const float pitch_pos = cycle.pitch;
    g_debug.mode = mode;
    g_debug.passage_stage = passage.stage;
    g_debug.yaw_rad = yaw_pos;
    g_debug.pitch_rad = pitch_pos;
    g_debug.joint_yaw_rad = yaw.joint;
    g_debug.joint_pitch_rad = pitch.joint;
    g_debug.target_pitch_rad = pitch.target;
    g_debug.target_yaw_rad = mode == Mode::kManual ?
        wrap(yaw_pos + yaw.target - yaw.joint) : yaw.target;
    g_debug.yaw_command_raw = yaw.motor.command();
    g_debug.pitch_torque_nm = pitch.output;
    g_debug.joint_roll_rad = roll.joint;
    g_debug.target_roll_rad = roll.target;
    g_debug.roll_torque_nm = roll.output;
    g_debug.joint_control = useJointPitch(mode);
    g_debug.ready = cycle.online && calibrated && !init_failed && mode != Mode::kFault &&
                    mode != Mode::kZeroForce && passage.stage != PassageStage::kFault;
    g_debug.following = false;
    g_debug.init_failed = init_failed;
    LuojiaFox::GimbalState state{};
    state.yaw_deg = yaw_pos / kDegToRad;
    state.pitch_deg = pitch_pos / kDegToRad;
    state.online = cycle.online;
    state.joint_yaw_rad = yaw.joint;
    state.yaw_joint_online = cycle.yaw_joint_online;
    state.joint_pitch_rad = g_debug.joint_pitch_rad;
    state.mode = static_cast<uint8_t>(mode);
    state.ready = g_debug.ready;
    LuojiaFox::system_state.updateGimbal(state);
}

void selectControlRequest(ControlCycle& cycle, const LuojiaFox::RobotCmdState& aim,
                          bool aux_passage) {
    const bool robot_cmd_seen = aim.last_update_ms != 0U;
    if (cycle.rc_online) {
        switch (cycle.sw) {
        case RC_SW_MID:
            cycle.normal_mode = Mode::kManual;
            cycle.passage_requested = aux_passage;
            return;
        case RC_SW_UP:
            break;
        default:
            return;
        }
    } else if (!robot_cmd_seen) {
        return;
    }

    cycle.passage_requested = aim.passage_requested;
    cycle.aim_yaw = aim.gimbal_yaw;
    cycle.aim_pitch = aim.gimbal_pitch;
    const bool fresh_aim = robot_cmd_seen &&
        cycle.now - aim.last_update_ms < GIMBAL_AIM_TIMEOUT_MS &&
        isfinite(cycle.aim_yaw) && isfinite(cycle.aim_pitch);
    const bool tracking = aim.gimbal_aim_changed_ms != 0U &&
        cycle.now - aim.gimbal_aim_changed_ms < GIMBAL_AIM_LOST_MS;
    const Mode scan_mode = mode == Mode::kGap || mode == Mode::kScan ? Mode::kScan : Mode::kGap;
    cycle.normal_mode = fresh_aim && tracking ? Mode::kAutoAim : scan_mode;
}

void observe(ControlCycle& cycle) {
    const auto state = LuojiaFox::system_state.read();
    cycle.now = HAL_GetTick();
    const auto& imu = state.muzzle_imu;
    cycle.yaw = imu.yaw_deg * kDegToRad;
    cycle.pitch = imu.pitch_deg * kDegToRad;
    const float r = imu.roll_deg * kDegToRad;
    const bool imu_ok = imu.last_update_ms != 0U &&
        (cycle.now - imu.last_update_ms < GIMBAL_IMU_TIMEOUT_MS) &&
        isfinite(cycle.yaw) && isfinite(cycle.pitch) && isfinite(r) &&
        isfinite(imu.yaw_rate_dps) && isfinite(imu.roll_rate_dps) &&
        fabsf(cosf(cycle.pitch)) > 0.1f;
    cycle.imu_ok = imu_ok;
    // The INS publishes body Y (q) in roll_rate_dps.
    const float q = imu.roll_rate_dps * kDegToRad;
    const float z = imu.yaw_rate_dps * kDegToRad;
    cycle.yaw_rate = imu_ok ?
        (q * sinf(r) + z * cosf(r)) / cosf(cycle.pitch) : 0;
    cycle.pitch_rate = q * cosf(r) - z * sinf(r);
    cycle.yaw_joint_online = yaw.observe(cycle.now);
    const bool pitch_ok = pitch.observe(cycle.now);
    const bool roll_ok = roll.observe(cycle.now);
    board_link::BigYawFeedback big_yaw{};
    cycle.peer_online = board_link::getPeerBigYawFeedback(big_yaw);
    cycle.big_yaw_centered = big_yaw.ready && gimbal::bigYawCentered();
    cycle.big_yaw_feedback_revision = big_yaw.revision;
    cycle.rc_online = !getRcOffline();
    bool aux_passage = false;
    const RcControl* rc = getRemoteControlPoint();
    if (cycle.rc_online && rc != nullptr) {
        taskENTER_CRITICAL();
        cycle.sw = rc->rc.s[RC_SW_RIGHT];
        cycle.rc_pitch = rc->rc.ch[RC_CH_RIGHT_VERTICAL];
        aux_passage = rc->rc.s[GIMBAL_PASS_SWITCH_INDEX] == GIMBAL_PASS_SWITCH_POSITION;
        taskEXIT_CRITICAL();
    }
    selectControlRequest(cycle, state.robot_cmd, aux_passage);

    const bool mechanical_mode = mode == Mode::kPassage || passage.stage == PassageStage::kFault ||
                                 cycle.passage_requested;
    cycle.online = cycle.yaw_joint_online && pitch_ok && roll_ok && (mechanical_mode || imu_ok);
}

void handleException(ControlCycle& cycle) {
    if (cycle.normal_mode == Mode::kZeroForce || !cycle.online || !cycle.peer_online) {
        if (mode == Mode::kPassage ||
            fabsf(roll.joint - GIMBAL_ROLL_UPRIGHT_RAD) > GIMBAL_PASS_POSITION_TOL_RAD) {
            latchPassageFault();
        }
        calibrated = false;
        if (cycle.rc_online && cycle.sw == RC_SW_DOWN) {
            init_failed = false;
        }
        setMode(passage.stage == PassageStage::kFault ? Mode::kFault : Mode::kZeroForce,
                cycle.yaw, cycle.pitch);
        yaw.output = pitch.output = roll.output = 0;
        cycle.stop = true;
        return;
    }
    const bool motors_ready = pitch.ready && roll.ready;
    if (!motors_ready && mode != Mode::kZeroForce && passage.stage != PassageStage::kFault) {
        if (mode == Mode::kPassage) { latchPassageFault(); }
        init_failed = true;
    }
    const bool passage_fault = passage.stage == PassageStage::kFault;
    // Release and reassert the passage request to recover upright.
    if (passage_fault && !cycle.passage_requested) {
        passage.rearmed = true;
        init_failed = false;
    }
    if (init_failed) {
        setMode(Mode::kFault, cycle.yaw, cycle.pitch);
        yaw.output = pitch.output = roll.output = 0;
        cycle.stop = true;
        return;
    }
    if (passage_fault) {
        setMode(Mode::kFault, cycle.yaw, cycle.pitch);
    }
    cycle.enable = !passage_fault || (passage.rearmed && cycle.passage_requested);
    cycle.stop = !cycle.enable || !motors_ready;
}

void failPassage(const ControlCycle& cycle) {
    latchPassageFault();
    init_failed = true;
    setMode(Mode::kFault, cycle.yaw, cycle.pitch);
}

void updatePassageStage(const ControlCycle& cycle) {
    if (!cycle.passage_requested && passage.stage == PassageStage::kAlign) {
        // Cancel alignment and hold the current yaw and pitch while returning upright.
        startPassage(cycle, PassageStage::kUnfold, yaw.joint, pitch.joint);
    } else if (!cycle.passage_requested &&
        (passage.stage == PassageStage::kFold || passage.stage == PassageStage::kHold)) {
        // Rebuild the trajectory from actual feedback, not the old fold target.
        setPassageStage(PassageStage::kUnfold, cycle.now);
    }
    if (passage.stage == PassageStage::kWait) {
        if (!cycle.passage_requested && cycle.imu_ok) {
            calibrated = true;
            passage.stage = PassageStage::kIdle;
            setMode(cycle.normal_mode, cycle.yaw, cycle.pitch);
        }
        return;
    }

    if (passage.center_confirmed && !cycle.big_yaw_centered) { failPassage(cycle); return; }
    if (cycle.big_yaw_centered && cycle.big_yaw_feedback_revision != passage.entry_feedback_revision) {
        passage.center_confirmed = true;
    }
    if (passage.stage == PassageStage::kHold) { return; }

    const float roll_goal = passage.stage == PassageStage::kFold ?
        GIMBAL_PASS_ROLL_RAD : GIMBAL_ROLL_UPRIGHT_RAD;
    const bool settled = jointSettled(yaw, passage.yaw_goal) &&
        jointSettled(pitch, passage.pitch_goal) && jointSettled(roll, roll_goal);
    passage.settled_ms = settled ? passage.settled_ms + GIMBAL_PERIOD_MS : 0;
    const uint32_t settle_ms = passage.stage == PassageStage::kAlign ?
        GIMBAL_PASS_SETTLE_MS : GIMBAL_PASS_ROLL_SETTLE_MS;
    if (passage.center_confirmed && passage.settled_ms >= settle_ms) {
        switch (passage.stage) {
        case PassageStage::kAlign:
            setPassageStage(PassageStage::kFold, cycle.now);
            break;
        case PassageStage::kFold:
            setPassageStage(PassageStage::kHold, cycle.now);
            break;
        case PassageStage::kUnfold:
            setPassageStage(PassageStage::kWait, cycle.now);
            break;
        default:
            break;
        }
    } else if (cycle.now - passage.stage_start >= GIMBAL_PASS_PHASE_TIMEOUT_MS) {
        failPassage(cycle);
    }
}

void selectMode(ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (passage.stage == PassageStage::kFault) {
        // Reaching this point means the explicit recovery request was armed.
        startPassage(cycle, PassageStage::kUnfold, yaw.joint, pitch.joint);
        return;
    }
    if (mode == Mode::kPassage) { updatePassageStage(cycle); return; }
    // A folded assembly must not be straightened by ordinary initialization.
    if (fabsf(roll.joint - GIMBAL_ROLL_UPRIGHT_RAD) > GIMBAL_PASS_POSITION_TOL_RAD) {
        failPassage(cycle); return;
    }
    if (cycle.passage_requested) {
        startPassage(cycle, PassageStage::kAlign, 0.0f, GIMBAL_PASS_PITCH_RAD);
        return;
    }
    if (!calibrated) {
        if (mode != Mode::kInit) {
            setMode(Mode::kInit, cycle.yaw, cycle.pitch);
            init_start = cycle.now;
        }
        return;
    }
    setMode(cycle.normal_mode, cycle.yaw, cycle.pitch);
}

void updatePassageReference(ControlCycle& cycle) {
    if (passage.stage == PassageStage::kFault) { return; }
    yaw.target = passage.yaw_goal;
    pitch.target = passage.pitch_goal;
    if (passage.stage == PassageStage::kAlign) {
        yaw.target = passage.yaw_path.advance(passage.yaw_goal,
            GIMBAL_PASS_JOINT_SPEED_RAD_S, GIMBAL_PASS_JOINT_ACCEL_RAD_S2);
        pitch.target = passage.pitch_path.advance(passage.pitch_goal,
            GIMBAL_PASS_JOINT_SPEED_RAD_S, GIMBAL_PASS_JOINT_ACCEL_RAD_S2);
    }
    float roll_goal = GIMBAL_ROLL_UPRIGHT_RAD;
    if (passage.stage == PassageStage::kFold || passage.stage == PassageStage::kHold) {
        roll_goal = GIMBAL_PASS_ROLL_RAD;
    } else if (passage.stage == PassageStage::kUnfold && !passage.center_confirmed) {
        roll_goal = passage.roll_path.position;
    }
    const bool moving_roll = passage.stage == PassageStage::kFold ||
        (passage.stage == PassageStage::kUnfold && passage.center_confirmed);
    const float previous_velocity = passage.roll_path.velocity;
    roll.target = passage.roll_path.advance(roll_goal,
        moving_roll ? GIMBAL_PASS_ROLL_SPEED_RAD_S : GIMBAL_ROLL_SPEED_RAD_S,
        moving_roll ? GIMBAL_PASS_ROLL_ACCEL_RAD_S2 : GIMBAL_ROLL_ACCEL_RAD_S2);
    if (moving_roll) {
        // Use the trajectory acceleration directly to avoid differentiating across a stage reset.
        cycle.roll_feedforward_nm = roll.smc.param.J *
            (passage.roll_path.velocity - previous_velocity) / kDt;
    }
}

void updateReference(ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (mode == Mode::kPassage) { updatePassageReference(cycle); return; }
    roll.target = GIMBAL_ROLL_UPRIGHT_RAD;
    if (!calibrated) {
        // INIT references are both zero in joint space.
        const bool centered = fabsf(yaw.joint) < GIMBAL_INIT_TOLERANCE_RAD &&
                              fabsf(pitch.joint) < GIMBAL_INIT_TOLERANCE_RAD;
        if (centered) {
            calibrated = true;
            setMode(Mode::kGap, cycle.yaw, cycle.pitch);
        } else if (cycle.now - init_start >= GIMBAL_INIT_TIMEOUT_MS) {
            init_failed = true;
            setMode(Mode::kFault, cycle.yaw, cycle.pitch);
        }
    } else if (mode == Mode::kManual) {
        pitch.target -= stick(cycle.rc_pitch) * GIMBAL_MANUAL_PITCH_RAD_S * kDt;
    } else if (mode == Mode::kScan) {
        yaw.target = 0.0f;
        scan_phase += 2.0f * kPi * kDt / GIMBAL_SCAN_PITCH_PERIOD_S;
        scan_phase = wrap(scan_phase);
        pitch.target = GIMBAL_SCAN_PITCH_AMPLITUDE_RAD * sinf(scan_phase);
    } else if (mode == Mode::kAutoAim) {
        yaw.target = cycle.aim_yaw;
        pitch.target = cycle.aim_pitch;
    }
}

void calculateOutput(const ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (mode == Mode::kFault || passage.stage == PassageStage::kFault) {
        yaw.output = pitch.output = roll.output = 0;
        return;
    }
    // Relative motor velocity excludes big-yaw rotation. Using IMU yaw rate
    // here would brake the parent rotation even when the small joint is centered.
    const bool aim = mode == Mode::kAutoAim;
    const bool pitch_joint = useJointPitch(mode);
    const float yaw_position = aim ? cycle.yaw : yaw.joint;
    const float yaw_rate = aim ? cycle.yaw_rate : yaw.velocity;
    const float yaw_delta = aim ? wrap(yaw.target - yaw_position) : yaw.target - yaw_position;
    yaw.target = yaw_position + clamp(yaw_delta,
        GIMBAL_SMALL_YAW_MIN_RAD - yaw.joint, GIMBAL_SMALL_YAW_MAX_RAD - yaw.joint);
    if (aim) { yaw.target = wrap(yaw.target); }
    const float pitch_position = pitch_joint ? pitch.joint : cycle.pitch;
    const float pitch_rate = pitch_joint ? pitch.velocity : cycle.pitch_rate;
    pitch.target = clamp(pitch.target,
        pitch_position + GIMBAL_PITCH_MIN_RAD - pitch.joint,
        pitch_position + GIMBAL_PITCH_MAX_RAD - pitch.joint);
    if (aim) {
        // USB supplies position steps without a reference angular velocity.
        yaw.smc.error.tar_last = yaw.target;
        pitch.smc.error.tar_last = pitch.target;
    }
    const bool y_valid = yaw.control(yaw_position, yaw_rate, true,
        GIMBAL_SMALL_YAW_MIN_RAD, GIMBAL_SMALL_YAW_MAX_RAD, aim);
    const bool p_valid = pitch.control(pitch_position, pitch_rate, true,
        GIMBAL_PITCH_MIN_RAD, GIMBAL_PITCH_MAX_RAD);
    const bool r_valid = roll.control(roll.joint, roll.velocity, true,
        GIMBAL_ROLL_MIN_RAD, GIMBAL_ROLL_MAX_RAD, false, cycle.roll_feedforward_nm);
    if (!y_valid || !p_valid || !r_valid) {
        if (mode == Mode::kPassage) { failPassage(cycle); return; }
        init_failed = true;
        setMode(Mode::kFault, cycle.yaw, cycle.pitch);
    }
}
} // namespace
} // namespace gimbal

bool gimbal::initSmallGimbal() {
    using namespace gimbal;
    if (initialized) { return true; }
    const float yaw_params[9] = GIMBAL_YAW_SMC_PARAMS;
    const float pitch_params[9] = GIMBAL_PITCH_SMC_PARAMS;
    const float roll_params[9] = GIMBAL_ROLL_SMC_PARAMS;
    initialized = yaw.init(GIMBAL_SMALL_CAN, GIMBAL_YAW_ID, GIMBAL_YAW_DIRECTION,
                           GIMBAL_YAW_MID_ECD, yaw_params, EXPONENT, GIMBAL_YAW_OUTPUT_MAX) &&
                  pitch.init(GIMBAL_SMALL_CAN, GIMBAL_PITCH_ID, GIMBAL_PITCH_MASTER_ID,
                             GIMBAL_PITCH_DIRECTION, GIMBAL_PITCH_MID_RAD, pitch_params,
                             EISMC, GIMBAL_PITCH_OUTPUT_MAX_NM) &&
                  roll.init(GIMBAL_SMALL_CAN, GIMBAL_ROLL_ID, GIMBAL_ROLL_MASTER_ID,
                            GIMBAL_ROLL_DIRECTION, GIMBAL_ROLL_MID_RAD, roll_params,
                            EXPONENT, GIMBAL_ROLL_OUTPUT_MAX_NM);
    return initialized;
}

void gimbal::controlSmallGimbal() {
    using namespace gimbal;
    if (!initialized) { return; }
    ControlCycle cycle{};
    observe(cycle);
    handleException(cycle);
    selectMode(cycle);
    updateReference(cycle);
    calculateOutput(cycle);
    send(cycle);
    publish(cycle);
}
