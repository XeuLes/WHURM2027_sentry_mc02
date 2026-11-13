// 主控规划大 Yaw 的运动参考，副板执行电机闭环。
#include "app_gimbal.h"
#include "app_board_link.hpp"
#include "lf_core.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#include "gimbal_axis.hpp"
#include "gimbal_config.h"
#include "chassis_config.hpp"

namespace gimbal {
using namespace detail;
DebugState g_big_yaw_debug{};
namespace {
board_link::BigYawFeedback yaw{};
board_link::BigYawFeedback previous_yaw{};
float estimated_joint_velocity = 0.0f;
float target_rad = 0.0f;
float navigation_heading_rad = 0.0f;
bool navigation_heading_latched = false;
JointTrajectory passage_path;
uint32_t passage_centered_ms = 0;
bool passage_yaw_centered = false;
bool init_failed = false;
Mode mode = Mode::kZeroForce;

struct ControlCycle {
    uint32_t now = 0;
    float yaw_position = 0;
    float pitch = 0;
    float yaw_rate = 0;
    float passage_acceleration = 0;
    float small_yaw_offset = 0;
    bool imu_ok = false;
    bool navigation_heading = false;
    bool small_yaw_online = false;
    bool rc_online = false;
    bool stop = false;
    bool enable = false;
    uint8_t sw = 0;
    int16_t rc_yaw = 0;
    Mode small_mode = Mode::kZeroForce;
};

void setMode(Mode next, float yaw_position) {
    if (mode == next) { return; }
    mode = next;
    target_rad = mode == Mode::kPassage ? yaw.joint_rad : yaw_position;
    passage_centered_ms = 0;
    if (mode == Mode::kPassage) {
        passage_path.reset(yaw.joint_rad, estimated_joint_velocity);
    }
}

void publishTarget(const ControlCycle& cycle) {
    const bool joint_frame = mode == Mode::kPassage && !cycle.navigation_heading;
    const bool holding_center = passage_centered_ms >= GIMBAL_PASS_BIG_YAW_SETTLE_MS;
    board_link::publishBigYawTarget({
        wrap(target_rad),
        wrap(cycle.yaw_position),
        joint_frame ? (holding_center ? 0.0f : passage_path.velocity) : cycle.yaw_rate,
        yaw.joint_rad,
        wrap(yaw.joint_rad + cycle.small_yaw_offset),
        cycle.passage_acceleration,
        cycle.enable && !init_failed, !cycle.stop && !init_failed, joint_frame,
        yaw.motor_online, yaw.motor_online && cycle.small_yaw_online,
        cycle.now
    });
}

void publishDebug(const ControlCycle& cycle) {
    g_big_yaw_debug.mode = mode;
    g_big_yaw_debug.yaw_rad = cycle.yaw_position;
    g_big_yaw_debug.pitch_rad = cycle.pitch;
    g_big_yaw_debug.joint_yaw_rad = yaw.joint_rad;
    g_big_yaw_debug.target_yaw_rad = target_rad;
    g_big_yaw_debug.joint_control = mode == Mode::kPassage && !cycle.navigation_heading;
    g_big_yaw_debug.ready = !cycle.stop && !init_failed &&
                    mode != Mode::kFault && mode != Mode::kZeroForce;
    g_big_yaw_debug.following = mode == Mode::kFollow;
    g_big_yaw_debug.init_failed = init_failed;
    g_big_yaw_debug.passage_yaw_centered = passage_yaw_centered ? 1U : 0U;
}

void updatePassageCentered(const ControlCycle& cycle) {
    bool centered = false;
    if (!cycle.stop && !init_failed && mode == Mode::kPassage) {
        if (passage_centered_ms < GIMBAL_PASS_BIG_YAW_SETTLE_MS) {
            const bool reference_settled = cycle.navigation_heading ?
                fabsf(wrap(navigation_heading_rad - cycle.yaw_position)) < GIMBAL_PASS_POSITION_TOL_RAD :
                (fabsf(passage_path.position) < GIMBAL_PASS_POSITION_TOL_RAD &&
                 fabsf(passage_path.velocity) < GIMBAL_PASS_SPEED_TOL_RAD_S);
            const bool settled = fabsf(yaw.joint_rad) < GIMBAL_PASS_POSITION_TOL_RAD &&
                yaw.slow && reference_settled;
            passage_centered_ms = settled ? passage_centered_ms + GIMBAL_PERIOD_MS : 0;
        }
        // Allow small holding corrections, but withdraw the acknowledgement on a large drift.
        centered = passage_centered_ms >= GIMBAL_PASS_BIG_YAW_SETTLE_MS &&
            fabsf(yaw.joint_rad) < 2.0f * GIMBAL_PASS_POSITION_TOL_RAD;
    }
    taskENTER_CRITICAL();
    passage_yaw_centered = centered;
    taskEXIT_CRITICAL();
}

void updateJointVelocity() {
    if (!yaw.motor_online) {
        previous_yaw = {};
        estimated_joint_velocity = 0.0f;
        return;
    }
    if (previous_yaw.motor_online && previous_yaw.revision == yaw.revision) { return; }
    const uint32_t elapsed_ms = yaw.rx_ms - previous_yaw.rx_ms;
    estimated_joint_velocity = 0.0f;
    if (previous_yaw.motor_online && !yaw.slow && elapsed_ms > 0U &&
        elapsed_ms < board_link::can3::kBigYawTimeoutMs) {
        const float rate = wrap(yaw.joint_rad - previous_yaw.joint_rad) * 1000.0f / elapsed_ms;
        estimated_joint_velocity = clamp(rate, -GIMBAL_DM_VEL_MAX_RAD_S, GIMBAL_DM_VEL_MAX_RAD_S);
    }
    previous_yaw = yaw;
}

void observe(ControlCycle& cycle) {
    const auto state = LuojiaFox::system_state.read();
    cycle.now = HAL_GetTick();
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
    board_link::getPeerBigYawFeedback(yaw);
    updateJointVelocity();
    cycle.rc_online = !getRcOffline();
    const RcControl* rc = getRemoteControlPoint();
    if (rc != nullptr) {
        taskENTER_CRITICAL();
        cycle.sw = rc->rc.s[RC_SW_RIGHT];
        cycle.rc_yaw = rc->rc.ch[RC_CH_RIGHT_HORIZONTAL];
        taskEXIT_CRITICAL();
    }
    cycle.small_mode = static_cast<Mode>(state.gimbal.mode);
    const bool automatic = !cycle.rc_online || cycle.sw == RC_SW_UP;
    const auto& cmd = state.robot_cmd;
    const bool heading_fresh = automatic && cmd.chassis_enabled && cmd.heading_valid &&
        cmd.last_update_ms != 0U && cycle.now - cmd.last_update_ms < chassis_app::kAutoCommandTimeoutMs;
    if (heading_fresh) {
        navigation_heading_rad = cmd.big_yaw_heading_rad;
        navigation_heading_latched = true;
    } else if (cycle.small_mode != Mode::kPassage) {
        navigation_heading_latched = false;
    }
    cycle.navigation_heading = automatic && navigation_heading_latched;
    cycle.small_yaw_offset = state.gimbal.joint_yaw_rad;
    cycle.small_yaw_online = state.gimbal.yaw_joint_online && isfinite(cycle.small_yaw_offset);
}

Mode selectAutomaticMode(const ControlCycle& cycle) {
    if (cycle.navigation_heading && cycle.small_mode != Mode::kZeroForce && cycle.small_mode != Mode::kFault) {
        return cycle.small_mode == Mode::kPassage ? Mode::kPassage : Mode::kNavigation;
    }
    switch (cycle.small_mode) {
    case Mode::kZeroForce:
    case Mode::kPassage:
        return cycle.small_mode;
    case Mode::kScan:
        // Capture the current heading in GAP before starting or restarting scan.
        return mode == Mode::kGap || mode == Mode::kScan ? Mode::kScan : Mode::kGap;
    case Mode::kAutoAim: {
        if (!cycle.small_yaw_online) { return Mode::kGap; }
        const float offset = fabsf(cycle.small_yaw_offset);
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
    const bool feedback_ok = yaw.motor_online &&
        ((next == Mode::kPassage && !cycle.navigation_heading) || cycle.imu_ok);
    if (next == Mode::kZeroForce || cycle.small_mode == Mode::kFault ||
        !feedback_ok) {
        if (cycle.rc_online && cycle.sw == RC_SW_DOWN) {
            init_failed = false;
        }
        setMode(Mode::kZeroForce, cycle.yaw_position);
        cycle.stop = true;
        return;
    }
    if (!yaw.ready && mode != Mode::kZeroForce) { init_failed = true; }
    if (init_failed) {
        setMode(Mode::kFault, cycle.yaw_position);
        cycle.stop = true;
        return;
    }
    cycle.enable = true;
    cycle.stop = !yaw.ready;
}

void updateReference(ControlCycle& cycle) {
    if (cycle.stop) { return; }
    if (cycle.navigation_heading) {
        target_rad = wrap(target_rad + clamp(wrap(navigation_heading_rad - target_rad),
            -GIMBAL_PASS_BIG_YAW_SPEED_RAD_S * kDt, GIMBAL_PASS_BIG_YAW_SPEED_RAD_S * kDt));
        return;
    }
    if (mode == Mode::kPassage) {
        if (passage_centered_ms >= GIMBAL_PASS_BIG_YAW_SETTLE_MS) {
            target_rad = 0.0f;
            return;
        }
        const float previous_velocity = passage_path.velocity;
        target_rad = passage_path.advance(0.0f, GIMBAL_PASS_BIG_YAW_SPEED_RAD_S,
                                            GIMBAL_PASS_BIG_YAW_ACCEL_RAD_S2);
        cycle.passage_acceleration = (passage_path.velocity - previous_velocity) / kDt;
        return;
    }
    float rate = 0.0f;
    if (mode == Mode::kManual) {
        rate = -stick(cycle.rc_yaw) * GIMBAL_MANUAL_YAW_RAD_S;
    } else if (mode == Mode::kScan) {
        rate = GIMBAL_SCAN_YAW_RAD_S;
    } else if (mode == Mode::kFollow) {
        rate = clamp(GIMBAL_FOLLOW_GAIN * cycle.small_yaw_offset,
            -GIMBAL_FOLLOW_MAX_RAD_S, GIMBAL_FOLLOW_MAX_RAD_S);
    }
    if (rate == 0.0f) { return; }
    const float lead = wrap(target_rad - cycle.yaw_position) + rate * kDt;
    target_rad = wrap(cycle.yaw_position +
        clamp(lead, -GIMBAL_BIG_YAW_MAX_LEAD_RAD, GIMBAL_BIG_YAW_MAX_LEAD_RAD));
}

} // namespace

bool bigYawCentered() {
    taskENTER_CRITICAL();
    const bool centered = passage_yaw_centered;
    taskEXIT_CRITICAL();
    return centered;
}
} // namespace gimbal

extern "C" bool Gimbal_Init(void) {
    return gimbal::initSmallGimbal();
}

extern "C" void Gimbal_Control(void) {
    using namespace gimbal;
    controlSmallGimbal();
    ControlCycle cycle{};
    observe(cycle);
    const Mode next = selectMode(cycle);
    handleException(cycle, next);
    if (!cycle.stop) {
        const bool enter_navigation_passage = next == Mode::kPassage && mode != next && cycle.navigation_heading;
        setMode(next, cycle.yaw_position);
        if (enter_navigation_passage) { target_rad = cycle.yaw_position; }
    }
    updateReference(cycle);
    publishTarget(cycle);
    updatePassageCentered(cycle);
    publishDebug(cycle);
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
