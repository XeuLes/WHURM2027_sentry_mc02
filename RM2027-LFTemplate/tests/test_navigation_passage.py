"""Host regression of production passage state transitions; no motor simulation.

Requires Python and g++. Hardware observations are supplied explicitly. Function
bodies and constants are read from the firmware, rather than copied into tests.
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build/navigation_host_tests/passage_selector"
OUT.mkdir(parents=True, exist_ok=True)
GIMBAL = ROOT / "RM2027-LFCore/4_Application/Gimbal"
CHASSIS = ROOT / "RM2027-LFCore/4_Application/Chassis"
BOARD_LINK = ROOT / "RM2027-LFCore/4_Application/Board_link"
CAN = ROOT / "RM2027-LFCore/2_Device/Communication"
RC = ROOT / "RM2027-LFCore/2_Device/RC"
BASIC = ROOT / "RM2027-LFCore/1_Middleware/Algorithm/Basic"


def body(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def run(name, source, *other_sources):
    cpp, exe = OUT / (name + ".cpp"), OUT / (name + ".exe")
    cpp.write_text(source, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I" + str(OUT), "-I" + str(GIMBAL), "-I" + str(CHASSIS),
                    "-I" + str(ROOT / "RM2027-LFCore/3_LuojiaFox"),
                    "-I" + str(BOARD_LINK), "-I" + str(CAN), "-I" + str(RC), "-I" + str(BASIC),
                    str(cpp), *map(str, other_sources), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


(OUT / "stm32h7xx_hal.h").write_text("#pragma once\n")
(OUT / "arm_math.h").write_text("#pragma once\n#include <stdint.h>\n")
axis = (GIMBAL / "gimbal_axis.hpp").read_text(encoding="utf-8")
small = (GIMBAL / "app_gimbal_small_yaw.cpp").read_text(encoding="utf-8")
big = (GIMBAL / "app_gimbal_big_yaw.cpp").read_text(encoding="utf-8")
common = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include "app_gimbal.h"
#include "gimbal_config.h"
#include "lf_core.hpp"
using namespace gimbal;
using LuojiaFox::GimbalShape;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDt = GIMBAL_PERIOD_MS * 0.001f;
float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(x, hi)); }
float wrap(float x) { return remainderf(x, 2*kPi); }
float stick(int16_t) { return 0; } // manual input is not exercised
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
'''
common += body(axis, "struct JointTrajectory") + ";\n"
source = common + r'''
struct Axis {
    float joint = 0, velocity = 0, target = 0;
    struct { struct { float tar_last = 0; } error; } smc;
    void reset(float p) { target = p; }
} yaw, pitch, roll;
bool calibrated = true, init_failed = false;
float scan_phase = 0;
uint32_t init_start = 0;
GimbalShape completed_shape = GimbalShape::Upright;
Mode mode = Mode::kScan;
bool manual_aligning = false;
uint32_t manual_align_started_ms = 0;
'''
source += body(axis, "inline bool jointSettled") + "\n"
source += body(small, "struct ControlCycle") + ";\n"
source += body(small, "struct PassageControl") + " passage;\n"
for signature in ["bool useJointPitch", "void setMode", "void setPassageStage",
                  "void startPassage", "void latchPassageFault", "void failPassage",
                  "void updatePassageStage", "void selectMode"]:
    source += body(small, signature) + "\n"
source += r'''
void navigationPassage() {
    ControlCycle c{};
    c.now = 1000;
    c.normal_mode = Mode::kScan;
    c.navigation_control = c.passage_requested = true;
    c.big_yaw_feedback_revision = 7;
    c.chassis_stopped = true;
    selectMode(c);
    assert(mode == Mode::kScan); // a latched request alone cannot start a new fold
    c.navigation_heading_ready = true;
    c.chassis_stopped = false;
    selectMode(c);
    assert(mode == Mode::kScan); // measured motion must stop before folding
    c.chassis_stopped = true;
    selectMode(c);
    assert(passage.stage == PassageStage::kAlign);
    assert(completed_shape == GimbalShape::Upright);
    c.big_yaw_centered = true;
    for (unsigned i = 0; i < GIMBAL_PASS_SETTLE_MS; ++i) {
        ++c.now;
        selectMode(c);
    }
    assert(passage.stage == PassageStage::kAlign); // old confirmation is insufficient
    ++c.big_yaw_feedback_revision;
    c.chassis_stopped = false;
    selectMode(c);
    assert(passage.stage == PassageStage::kAlign); // chassis must still be stopped when Roll starts
    c.chassis_stopped = true;
    selectMode(c);
    assert(passage.stage == PassageStage::kFold);
    assert(completed_shape == GimbalShape::Upright);
    for (unsigned i = 0; i < 200; ++i) { ++c.now; selectMode(c); }
    assert(passage.stage == PassageStage::kFold); // a command alone is not completion
    roll.joint = GIMBAL_PASS_ROLL_RAD;
    for (unsigned i = 0; i < GIMBAL_PASS_ROLL_SETTLE_MS; ++i) { ++c.now; selectMode(c); }
    assert(passage.stage == PassageStage::kHold);
    assert(completed_shape == GimbalShape::Folded);
    for (unsigned i = 0; i < 600; ++i) { ++c.now; selectMode(c); }
    assert(passage.stage == PassageStage::kHold); // retained request, no repeated fold
    c.passage_requested = false;
    c.chassis_stopped = false;
    selectMode(c);
    assert(passage.stage == PassageStage::kHold);
    c.chassis_stopped = true;
    selectMode(c);
    assert(passage.stage == PassageStage::kUnfold);
    assert(completed_shape == GimbalShape::Folded);
    roll.joint = GIMBAL_ROLL_UPRIGHT_RAD;
    for (unsigned i = 0; i < GIMBAL_PASS_ROLL_SETTLE_MS; ++i) { ++c.now; selectMode(c); }
    assert(passage.stage == PassageStage::kWait);
    assert(completed_shape == GimbalShape::Upright);
    selectMode(c);
    assert(passage.stage == PassageStage::kWait); // muzzle IMU needed to resume aiming
    c.imu_ok = true;
    selectMode(c);
    assert(passage.stage == PassageStage::kIdle && mode == Mode::kScan);
    // A lost big-yaw confirmation faults a folded mechanism.
    mode = Mode::kPassage;
    passage.stage = PassageStage::kHold;
    passage.center_confirmed = true;
    c.passage_requested = true;
    c.big_yaw_centered = false;
    selectMode(c);
    assert(passage.stage == PassageStage::kFault && mode == Mode::kFault);
    puts("Passage passed: stop, fresh alignment, physical fold, hold, stop-before-restore, upright, fault");
}

void manualPassage() {
    yaw = {}; pitch = {}; roll = {}; passage = {};
    mode = Mode::kManual;
    calibrated = true;
    init_failed = false;
    completed_shape = GimbalShape::Upright;

    ControlCycle c{};
    c.now = 1000;
    c.normal_mode = Mode::kManual;
    c.passage_requested = true;
    c.big_yaw_feedback_revision = 7;
    selectMode(c);
    assert(manual_aligning && passage.stage == PassageStage::kIdle);
    c.manual_heading_ready = c.chassis_aligned = true;
    selectMode(c);
    assert(manual_aligning && passage.stage == PassageStage::kIdle); // still rotating
    c.chassis_stopped = true;
    selectMode(c);
    assert(manual_aligning && passage.stage == PassageStage::kIdle); // heading not settled
    c.big_yaw_centered = true;
    selectMode(c);
    assert(!manual_aligning && passage.manual && passage.stage == PassageStage::kAlign);
    ++c.big_yaw_feedback_revision;
    for (unsigned i = 0; i < GIMBAL_PASS_SETTLE_MS; ++i) { ++c.now; selectMode(c); }
    assert(passage.stage == PassageStage::kFold);
    roll.joint = GIMBAL_PASS_ROLL_RAD;
    for (unsigned i = 0; i < GIMBAL_PASS_ROLL_SETTLE_MS; ++i) { ++c.now; selectMode(c); }
    assert(passage.stage == PassageStage::kHold);
    c.passage_requested = c.chassis_stopped = false;
    selectMode(c);
    assert(passage.stage == PassageStage::kHold);
    c.chassis_stopped = true;
    selectMode(c);
    assert(passage.stage == PassageStage::kUnfold);
    roll.joint = GIMBAL_ROLL_UPRIGHT_RAD;
    for (unsigned i = 0; i < GIMBAL_PASS_ROLL_SETTLE_MS; ++i) { ++c.now; selectMode(c); }
    assert(passage.stage == PassageStage::kWait);
    c.imu_ok = true;
    selectMode(c);
    assert(mode == Mode::kManual && passage.stage == PassageStage::kIdle && !passage.manual);
    puts("Manual passage passed: wait for heading and chassis stop, fold, hold, stop-before-restore");
}

int main() {
    navigationPassage();
    manualPassage();
}
'''
run("passage", source)

source = common + r'''
namespace board_link {
struct BigYawTarget {
    float target_rad, position_rad, velocity_rad_s, joint, total_joint, acceleration;
    bool enable, valid, joint_frame, motor_online, joints_online;
    uint32_t updated_at_ms;
} published{};
void publishBigYawTarget(const BigYawTarget& target) { published = target; }
}
struct { float joint_rad = 0; bool slow = true, motor_online = true; } yaw;
float target_rad = 0, navigation_heading_rad = 1.2f;
float manual_heading_rad = 0;
bool manual_heading_latched = false;
uint32_t passage_centered_ms = 0;
bool passage_yaw_centered = false, init_failed = false;
Mode mode = Mode::kPassage;
JointTrajectory passage_path;
'''
source += body(big, "struct ControlCycle") + ";\n"
for signature in ["bool useJointFrame", "bool hasPassageHeading", "float passageHeading",
                  "void updateManualHeading", "void publishTarget", "void updatePassageCentered",
                  "void updateReference"]:
    source += body(big, signature) + "\n"
source += r'''
int main() {
    ControlCycle c{};
    c.navigation_heading = c.enable = c.small_yaw_online = true;
    c.yaw_position = 1.2f;
    c.yaw_rate = 0.15f;
    for (unsigned i = 0; i < 500; ++i) updateReference(c);
    assert(fabsf(target_rad - navigation_heading_rad) < 1e-5f);
    publishTarget(c);
    assert(!board_link::published.joint_frame);
    assert(board_link::published.position_rad == c.yaw_position);
    assert(board_link::published.velocity_rad_s == c.yaw_rate);
    yaw.joint_rad = 0.2f;
    for (unsigned i = 0; i < 100; ++i) updatePassageCentered(c);
    assert(!passage_yaw_centered); // correct world heading but chassis not aligned
    yaw.joint_rad = 0;
    c.yaw_position = 0.8f;
    for (unsigned i = 0; i < 100; ++i) updatePassageCentered(c);
    assert(!passage_yaw_centered); // joint zero alone is insufficient
    c.yaw_position = navigation_heading_rad;
    for (unsigned i = 0; i < GIMBAL_PASS_BIG_YAW_SETTLE_MS; ++i) updatePassageCentered(c);
    assert(passage_yaw_centered);
    yaw.joint_rad = 0.2f;
    updatePassageCentered(c);
    assert(!passage_yaw_centered);

    c.navigation_heading = false;
    c.manual_passage = c.imu_ok = true;
    c.yaw_position = target_rad = 2.1f;
    updateManualHeading(c);
    c.yaw_position = 1.9f;
    updateManualHeading(c);
    updateReference(c);
    publishTarget(c);
    assert(!board_link::published.joint_frame && board_link::published.valid);
    assert(fabsf(board_link::published.target_rad - 2.1f) < 1e-5f);
    assert(board_link::published.position_rad == 1.9f);

    c.manual_passage = false;
    updateManualHeading(c);
    c.mechanical_recovery = true;
    publishTarget(c);
    assert(board_link::published.joint_frame); // explicit fault recovery can still use the joint
    passage_centered_ms = GIMBAL_PASS_BIG_YAW_SETTLE_MS;
    updateReference(c);
    assert(target_rad == 0);
    puts("Big yaw passed: navigation heading, manual heading latch, mechanical fault recovery");
}
'''
run("big_yaw", source)

master_can = CAN / "board_link_can.hpp"
source = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include "chassis_command.hpp"
#include "chassis_config.hpp"
#include "app_board_link.hpp"
#include "app_gimbal.h"
#include "gimbal_config.h"

uint32_t tick = 1000;
char mode_switch = RC_SW_MID;
char passage_switch = RC_SW_DOWN;
bool heading_ready = false;
float stick[4] = {};

char getDt7RcSw(uint8_t sw) {
    return sw == RC_SW_RIGHT ? mode_switch : passage_switch;
}
float getDt7RcCh(uint8_t ch) { return stick[ch]; }

namespace gimbal {
bool manualPassageHeadingReady() { return heading_ready; }
}
namespace board_link {
BigYawFeedback feedback{};
bool getPeerBigYawFeedback(BigYawFeedback& value) { value = feedback; return feedback.motor_online; }
}

int main() {
    using LuojiaFox::GimbalShape;
    using LuojiaFox::GimbalTransformState;
    LuojiaFox::SystemStateSnapshot state{};
    state.gimbal.updated_at_ms = tick;
    state.gimbal.transform_state = GimbalTransformState::Preparing;
    state.chassis = {0, 0, 0, true, tick};
    stick[RC_CH_LEFT_VERTICAL] = 0.7f;
    stick[RC_CH_LEFT_HORIZONTAL] = -0.3f;
    stick[RC_CH_RIGHT_HORIZONTAL] = -0.9f;
    board_link::feedback = {0.3f, true, true, true, 1, tick};

    chassis_app::ChassisCommandSelector selector;
    auto select = [&]() { return selector.update(state, true, tick); };
    assert(select().mode_changed);
    const auto manual = select();
    assert(manual.mode == CHASSIS_MODE_DIRECT_RC && manual.command.ready && !manual.motion_blocked);
    assert(manual.command.wz == chassis_app::kRcWzSign * stick[RC_CH_RIGHT_HORIZONTAL] * chassis_app::kMaxWz);

    passage_switch = GIMBAL_PASS_SWITCH_POSITION;
    assert(select().motion_blocked); // CH5 brakes even before the gimbal task sees it
    state.gimbal.manual_passage = state.gimbal.manual_passage_aligning = true;
    assert(select().motion_blocked); // wait for the big-Yaw heading lock
    heading_ready = true;
    state.chassis.vx = 0.2f;
    assert(select().motion_blocked); // brake translation first
    state.chassis.vx = 0.0f;
    auto alignment = select();
    assert(alignment.mode == CHASSIS_MODE_DIRECT_RC && alignment.command.ready && !alignment.motion_blocked);
    assert(alignment.command.vx == 0.0f && alignment.command.vy == 0.0f);
    assert(std::fabs(alignment.command.wz - 0.6f) < 1e-5f);
    board_link::feedback.joint_rad = -1.2f;
    assert(select().command.wz == -chassis_app::kPassageAlignMaxWz);
    board_link::feedback.joint_rad = 0.01f;
    assert(select().motion_blocked); // controlled zero at alignment
    state.gimbal.manual_passage_aligning = false;
    assert(select().motion_blocked); // lock throughout the fold

    state.gimbal.transformed = GimbalShape::Folded;
    state.gimbal.transform_state = GimbalTransformState::Ready;
    auto folded = select();
    assert(!folded.motion_blocked && folded.command.ready);
    assert(folded.command.vx == manual.command.vx && folded.command.vy == manual.command.vy);
    assert(folded.command.wz == 0.0f);
    board_link::feedback.ready = false;
    assert(select().motion_blocked);
    board_link::feedback.ready = true;
    heading_ready = false;
    assert(select().motion_blocked);
    heading_ready = true;
    tick += board_link::can3::kChassisFeedbackTimeoutMs;
    state.gimbal.updated_at_ms = tick;
    assert(select().motion_blocked); // stale chassis feedback
    state.chassis.last_update_ms = tick;
    passage_switch = RC_SW_DOWN;
    assert(select().motion_blocked); // brake as soon as CH5 is released
    assert(!selector.update(state, false, tick).command.ready);

    mode_switch = RC_SW_UP;
    state.robot_cmd.chassis_enabled = true;
    state.robot_cmd.last_update_ms = tick;
    state.robot_cmd.vx = 0.3f;
    assert(select().mode_changed);
    auto automatic = select();
    assert(automatic.command.ready && automatic.motion_blocked && automatic.command.vx == 0.0f);
    state.gimbal.manual_passage = false;
    state.gimbal.transformed = GimbalShape::Upright;
    automatic = select();
    assert(!automatic.motion_blocked && automatic.command.vx == 0.3f);
    puts("Chassis selector passed: regular RC, CH5 alignment and braking, folded driving, restore and AUTO");
}
'''
run("manual_chassis", source, CHASSIS / "chassis_command.cpp")

# Exercise the deputy's real decision function with deliberately nonzero payload
# while the interlock bit is set; a sender-side zero alone must not be relied on.
sub = ROOT.parents[1] / "subControl/RM2027-LFTemplate"
can_header = sub / "RM2027-LFCore/2_Device/Communication/board_link_can.hpp"


def protocol_code(path):
    return [code for line in path.read_text(encoding="utf-8").splitlines()
            if (code := line.split("//", 1)[0].strip())]


assert protocol_code(can_header) == protocol_code(master_can)
source = r'''
#include <cassert>
#include <cstdio>
#include <stdint.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
enum ChassisModeTypedef { CHASSIS_MODE_DISABLED=0, CHASSIS_MODE_DIRECT_RC=1, CHASSIS_MODE_AUTO_VISION=2 };
'''
source += can_header.read_text(encoding="utf-8").replace("#pragma once", "")
selection = (sub / "RM2027-LFCore/4_Application/Chassis/chassis_command.hpp").read_text(encoding="utf-8")
source += "namespace chassis_app {\n"
source += body(selection, "struct ChassisCommand {") + ";\n"
source += body(selection, "struct ChassisCommandSelection") + ";\n}\n"
source += r'''
namespace board_link { namespace can3 {
struct Received {
    ChassisTargetFrame data{};
    uint32_t rx_ms = 100;
    bool fresh(uint32_t now, uint32_t timeout) const { return now-rx_ms < timeout; }
};
struct { Received chassis; } targets;
auto readTargets() { return targets; }
}
ChassisModeTypedef mode = CHASSIS_MODE_DISABLED;
'''
sub_link = (sub / "RM2027-LFCore/4_Application/Board_link/app_board_link.cpp").read_text(encoding="utf-8")
source += body(sub_link, "chassis_app::ChassisCommandSelection getChassisSelection") + "\n}\n"
source += r'''
int main() {
    using namespace board_link;
    using namespace board_link::can3;
    targets.chassis.data = {500, -200, 700,
        uint8_t(kVersion | CHASSIS_MODE_AUTO_VISION | kChassisTargetValid | kChassisMotionBlocked), 1};
    auto r = getChassisSelection(100);
    assert(!r.command.ready); // existing mode-change pause
    r = getChassisSelection(101);
    assert(r.command.ready && r.command.vx == 0 && r.command.vy == 0 && r.command.wz == 0);
    targets.chassis.data.flags &= ~kChassisMotionBlocked;
    r = getChassisSelection(102);
    assert(r.command.ready && r.command.vx == 0.5f && r.command.vy == -0.2f && r.command.wz == 0.7f);
    r = getChassisSelection(200);
    assert(!r.command.ready && r.mode == CHASSIS_MODE_DISABLED);
    puts("Deputy interlock passed: blocked nonzero payload, controlled zero, release, CAN timeout");
}
'''
run("deputy_interlock", source)
