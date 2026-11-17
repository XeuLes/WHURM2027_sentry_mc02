"""Offline regression of the production USB CDC task and its navigation frames."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "RM2027-LFCore"
USB = CORE / "2_Device/USB"
REFEREE = CORE / "2_Device/Referee"
ALGORITHM = CORE / "1_Middleware/Algorithm"
CDC = ROOT / "USB_DEVICE/App/usbd_cdc_if.c"
BUILD = ROOT / "build"
HEADERS = {
    "stm32h7xx_hal.h": """
#pragma once
#include <stdint.h>
#define __packed __attribute__((packed))
#ifdef __cplusplus
struct FDCAN_HandleTypeDef {};
extern "C" {
#endif
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __set_PRIMASK(uint32_t);
void __DMB(void);
#ifdef __cplusplus
}
#endif
""",
    "remote_control.h": "#pragma once\nstruct RcControl {};",
    "app_board_link.hpp": """
#pragma once
#include <stdint.h>
namespace board_link {
namespace can3 { constexpr uint32_t kChassisFeedbackTimeoutMs = 50; }
struct BigYawFeedback { float joint_rad; bool motor_online; bool ready; };
bool getPeerBigYawFeedback(BigYawFeedback& feedback);
}
""",
    "usbd_cdc_if.h": """
#pragma once
#include "stm32h7xx_hal.h"
enum { USBD_OK, USBD_BUSY, USBD_FAIL, USBD_STATE_CONFIGURED };
int8_t USB_Receive(uint8_t*, uint32_t*);
uint8_t CDC_Transmit_HS(uint8_t*, uint16_t);
""",
}

TEST = r"""
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

#include "app_board_link.hpp"
#include "app_chassis.h"
#include "CRC8_CRC16.h"
#include "dm_imu.hpp"
#include "drv_tick.h"
#include "lf_core.hpp"
#include "referee.hpp"
#include "usb.hpp"
#include "usbd_cdc_if.h"

#undef NDEBUG
#include <cassert>

using Bytes = std::vector<uint8_t>;
using namespace LuojiaFox;

namespace {
uint32_t now = 0;
uint32_t sync_subtick = 0;
Bytes input;
std::vector<Bytes> output;
bool connected = true;
bool busy = false;
SystemStateSnapshot state{};
imu_t imu{};
unsigned command_updates = 0;
unsigned shoot_updates = 0;

uint8_t crc8(const Bytes& packet) {
    // 合并两个半字节步骤，与固件逐位法独立。
    constexpr uint8_t lut[] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D
    };
    uint8_t crc = 0;
    for (size_t i = 0; i + 1 < packet.size(); ++i) {
        crc ^= packet[i];
        crc = static_cast<uint8_t>((crc << 4) ^ lut[crc >> 4]);
        crc = static_cast<uint8_t>((crc << 4) ^ lut[crc >> 4]);
    }
    return crc;
}

template <typename Packet>
Bytes encode(uint8_t sof, Packet packet) {
    packet.frame_header = {sof, sizeof(Packet)};
    const auto* bytes = reinterpret_cast<const uint8_t*>(&packet);
    Bytes result(bytes, bytes + sizeof(Packet));
    result.back() = crc8(result);
    return result;
}

template <typename Packet>
Packet latest(uint8_t sof) {
    const auto found = std::find_if(output.rbegin(), output.rend(),
        [sof](const Bytes& bytes) { return bytes.front() == sof; });
    assert(found != output.rend() && found->size() == sizeof(Packet));
    assert((*found)[1] == sizeof(Packet) && found->back() == crc8(*found));
    Packet packet{};
    std::memcpy(&packet, found->data(), sizeof(Packet));
    return packet;
}

Bytes command(uint8_t mode, float velocity, float omega, float lateral_velocity = 0.0f,
              uint8_t heading_valid = 0, float heading = 0.0f) {
    ReceiveRoboCmd_s frame{};
    frame.mode = mode;
    frame.velocity = velocity;
    frame.omega = omega;
    frame.lateral_velocity = lateral_velocity;
    frame.heading_valid = heading_valid;
    frame.big_yaw_heading = heading;
    return encode(USB_ROBO_CMD_SOF, frame);
}

void keepFeedbackFresh() {
    state.chassis = {0.75f, -0.25f, -0.5f, true, now};
    state.gimbal.updated_at_ms = now;
}

void feedReferee(uint16_t command, const void* data, size_t size, bool valid_crc = true) {
    Bytes frame(REF_HEADER_CRC_CMDID_LEN + size);
    frame[0] = HEADER_SOF;
    frame[1] = static_cast<uint8_t>(size);
    frame[REF_PROTOCOL_HEADER_SIZE] = static_cast<uint8_t>(command);
    frame[REF_PROTOCOL_HEADER_SIZE + 1] = static_cast<uint8_t>(command >> 8);
    if (size != 0U) {
        std::memcpy(frame.data() + REF_HEADER_CMDID_LEN, data, size);
    }
    append_CRC8_check_sum(frame.data(), REF_PROTOCOL_HEADER_SIZE);
    append_CRC16_check_sum(frame.data(), frame.size());
    if (!valid_crc) {
        frame.back() ^= 1U;
    }
    referee.FeedBytes(reinterpret_cast<const char*>(frame.data()), frame.size());
    referee.UnpackFifoData();
}

template <typename Payload>
void feedReferee(uint16_t command, const Payload& data) {
    feedReferee(command, &data, sizeof(data));
}

bool hasFrame(uint8_t sof) {
    return std::any_of(output.begin(), output.end(),
                       [sof](const Bytes& bytes) { return bytes.front() == sof; });
}
}  // namespace

uint32_t HAL_GetTick() { return now; }
uint32_t getTick100us() { return now * 10U + sync_subtick++; }
ChassisModeTypedef Chassis_GetMode() { return CHASSIS_MODE_AUTO_VISION; }

SystemStateStore LuojiaFox::system_state;
SystemStateSnapshot SystemStateStore::read() const { return state; }
void SystemStateStore::updateRobotCmd(const RobotCmdState& cmd) {
    state.robot_cmd = cmd;
    state.robot_cmd.last_update_ms = now;
    ++command_updates;
}
void SystemStateStore::updateUsbLink(const UsbLinkState& link) { state.usb_link = link; }
void SystemStateStore::updateShootCmd(const ShootCmdState& cmd) {
    state.shoot_cmd = cmd;
    state.shoot_cmd.last_update_ms = now;
    ++shoot_updates;
}
bool board_link::getPeerBigYawFeedback(BigYawFeedback& feedback) {
    feedback = {0.25f, true, true};
    return true;
}
bool dm_imu_get_data(imu_t* result) {
    *result = imu;
    return imu.received_mask != 0U;
}
struct Device { int dev_state; void* pClassData; } hUsbDeviceHS{USBD_STATE_CONFIGURED, &state};
uint8_t UserRxBufferHS[512];
uint32_t rx_pending_length = 0, rx_pending_offset = 0;
uint32_t interrupt_mask = 0, endpoint_rearms = 0;

uint32_t __get_PRIMASK() { return interrupt_mask; }
void __disable_irq() { interrupt_mask = 1; }
void __enable_irq() { interrupt_mask = 0; }
void __set_PRIMASK(uint32_t mask) { interrupt_mask = mask; }
void __DMB() {}
int USBD_CDC_SetRxBuffer(Device*, uint8_t* buffer) {
    assert(buffer == UserRxBufferHS);
    return USBD_OK;
}
int USBD_CDC_ReceivePacket(Device*) {
    ++endpoint_rearms;
    return USBD_OK;
}

/* Production CDC receive functions */

void step(Usb& link) {
    hUsbDeviceHS.dev_state = connected ? USBD_STATE_CONFIGURED : 0;
    if (!input.empty()) {
        assert(rx_pending_length == 0 && input.size() <= sizeof(UserRxBufferHS));
        uint32_t size = static_cast<uint32_t>(input.size());
        std::memcpy(UserRxBufferHS, input.data(), size);
        input.clear();
        const auto before = endpoint_rearms;
        assert(CDC_Receive_HS(UserRxBufferHS, &size) == USBD_OK);
        assert(endpoint_rearms == before);
    }
    do {
        link.Update();
        assert(interrupt_mask == 0);
    } while (rx_pending_length != 0);
}

uint8_t CDC_Transmit_HS(uint8_t* data, uint16_t size) {
    if (busy || !connected) { return USBD_BUSY; }
    output.emplace_back(data, data + size);
    return USBD_OK;
}

int main() {
    referee.Init();
    assert(fifo_s_init(&referee.fifo, referee.fifo_buf, sizeof(referee.fifo_buf)) == 0);
    Usb link;
    link.Init();
    assert(state.usb_link.offline);

    now = 1000;
    state.gimbal.online = true;
    state.gimbal.joint_pitch_rad = -0.2f;
    state.gimbal.joint_yaw_rad = 0.3f;
    state.gimbal.joint_roll_rad = -0.25f;
    state.gimbal.transform_state = GimbalTransformState::Ready;
    state.gimbal.transformed = GimbalShape::Upright;
    state.muzzle_imu.last_update_ms = now - 1;
    state.muzzle_imu.quaternion_wxyz[0] = 1.0f;
    state.imu.last_update_ms = now - 2;
    state.imu.yaw_deg = 90.0f;
    imu.received_mask = 0x0C;
    imu.accel_rx_ms = now - 1;
    imu.gyro_rx_ms = now;
    imu.gyro_revision = 11;
    imu.accel[2] = 9.8f;
    imu.gyro[1] = 0.125f;
    robot_status_t robot{};
    robot.robot_id = 107;
    robot.current_HP = 513;
    robot.maximum_HP = 800;
    robot.chassis_power_limit = 80;
    power_heat_data_t power{};
    power.buffer_energy = 47;
    feedReferee(ROBOT_STATE_CMD_ID, robot);
    feedReferee(POWER_HEAT_DATA_CMD_ID, power);
    feedReferee(GAME_STATE_CMD_ID, game_status_t{0, 4, 133, 0});
    feedReferee(FIELD_EVENTS_CMD_ID, event_data_t{0x10203040U});
    feedReferee(GAME_ROBOT_HP_CMD_ID, game_robot_HP_t{11, 22, 33, 44, -5, 66, 77, 88, 99, 110});
    feedReferee(GROUND_ROBOT_POS_ID, ground_robot_position_t{1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
    feedReferee(ROBOT_POS_CMD_ID, robot_pos_t{1.25f, 6.5f, 90.0f});
    feedReferee(RFID_STATUS_ID, rfid_status_t{0x81234567U, 0x15});
    keepFeedbackFresh();

    // 分两次收同一条 RoboCmd。
    const auto drive = command(0, 1.25f, -0.625f);
    input.assign(drive.begin(), drive.begin() + 5);
    step(link);
    assert(command_updates == 0);
    assert(latest<SendNavStatus_s>(0xB3).transform_state == 0);
    const auto raw = latest<SendRawGimbalImu_s>(0xB1);
    assert(raw.sample_ticks == (now - 1) * 10U && raw.accel_z == 9.8f && raw.gyro_y == .125f);
    const auto pose = latest<SendGimbalStatus_s>(0xB2);
    assert(pose.w == 1.0f && pose.sample_ticks == (now - 1) * 10U);
    assert(pose.pitch == -0.2f && pose.yaw == 0.3f && pose.big_yaw == 0.25f);
    assert(pose.roll == -0.25f && pose.transformed == 4);
    const auto heading = latest<SendBigYawHeading_s>(0xB4);
    assert(heading.sample_ticks == (now - 2) * 10U && std::fabs(heading.yaw - 1.5707963f) < 1e-6f);
    const auto stage = latest<SendCompStage_s>(0xC0);
    assert(stage.game_progress == 4 && stage.stage_remain_time == 133);
    assert(latest<SendCompEvent_s>(0xC1).event_data == 0x10203040U);
    const auto allied_health = latest<SendAlliedHealth_s>(0xC2);
    assert(allied_health.ally_1_robot_hp == 11 && allied_health.ally_4_robot_hp == 44);
    assert(allied_health.ally_outpost_hp == 77 && allied_health.ally_base_hp == 88);
    assert(allied_health.enemy_outpost_hp == 99 && allied_health.enemy_base_hp == 110);
    const auto allied_pos = latest<SendAlliedPos_s>(0xC3);
    assert(allied_pos.hero_x == 1 && allied_pos.engineer_y == 4 && allied_pos.standard_4_y == 8);
    const auto self_pos = latest<SendRobotPos_s>(0xC5);
    assert(self_pos.self_x == 1.25f && self_pos.self_y == 6.5f);
    const auto rfid = latest<SendRfidStatus_s>(0xC6);
    assert(rfid.rfid_status == 0x81234567U && rfid.rfid_status_2 == 0x15);

    assert(!hasFrame(0xC4) && !hasFrame(0xC8));
    feedReferee(PROJECTILE_ALLOWANCE_CMD_ID, projectile_allowance_t{0x1234, 0x1122, 0x2345, 0x3456});
    feedReferee(SENTRY_DECISION_INFO_ID, sentry_info_t{0x12345678U, 0x9ABCU, 0x1020304050607080ULL});
    input.assign(drive.begin() + 5, drive.end());
    step(link);
    assert(command_updates == 1 && !state.usb_link.offline);
    assert(state.robot_cmd.vx == 1.25f && state.robot_cmd.vy == 0 && state.robot_cmd.wz == -0.625f);
    const auto robot_status = latest<SendRobotStatus_s>(0xC4);
    assert(robot_status.robot_id == 107 && robot_status.current_hp == 513 && robot_status.maximum_hp == 800);
    assert(robot_status.projectile_allowance_17mm == 0x1234 && robot_status.remaining_gold_coin == 0x2345);
    assert(robot_status.projectile_allowance_fortress == 0x3456);
    const auto sentry = latest<SendSentryInfo_s>(0xC8);
    assert(sentry.sentry_info == 0x12345678U && sentry.sentry_info_2 == 0x9ABCU);
    assert(sentry.sentry_info_3 == 0x1020304050607080ULL);

    // 同值事件按接收顺序各发一次，USB 忙时保留队头。
    const uint8_t hurt = 0x54;
    feedReferee(ROBOT_HURT_CMD_ID, hurt);
    feedReferee(ROBOT_HURT_CMD_ID, &hurt, sizeof(hurt), false);
    feedReferee(ROBOT_HURT_CMD_ID, nullptr, 0);
    MapCommandData_t waypoint{2.25f, 4.5f, 0, 0, 0x0101};
    feedReferee(MAP_COMMAND_CMD_ID, waypoint);
    feedReferee(ROBOT_HURT_CMD_ID, hurt);
    waypoint.target_position_x = 6.25f;
    Bytes full_map_payload(15U, 0xA5);
    std::memcpy(full_map_payload.data(), &waypoint, sizeof(waypoint));
    feedReferee(MAP_COMMAND_CMD_ID, full_map_payload.data(), full_map_payload.size());
    feedReferee(MAP_COMMAND_CMD_ID, &waypoint, sizeof(waypoint) - 1U);
    waypoint.cmd_keyboard = 0x41;
    feedReferee(MAP_COMMAND_CMD_ID, waypoint);
    waypoint.cmd_keyboard = 0U;
    waypoint.target_robot_id = 3U;
    feedReferee(MAP_COMMAND_CMD_ID, waypoint);
    waypoint.target_robot_id = 0U;
    waypoint.target_position_x = std::numeric_limits<float>::infinity();
    feedReferee(MAP_COMMAND_CMD_ID, waypoint);
    feedReferee(STUDENT_INTERACTIVE_DATA_CMD_ID, robot_interaction_data_t{});
    const size_t event_start = output.size();
    busy = true;
    step(link);
    assert(output.size() == event_start);
    busy = false;
    step(link);
    assert(latest<SendHealthReduction_s>(0xC7).armor_and_reason == hurt);
    step(link);
    assert(latest<SendOperatorCmd_s>(0xCA).target_position_x == 2.25f);
    step(link);
    step(link);
    step(link);
    assert(output.size() == event_start + 4);
    assert(output[event_start] == output[event_start + 2]);
    assert(output[event_start + 1].front() == 0xCA && output[event_start + 3].front() == 0xCA);
    const auto op = latest<SendOperatorCmd_s>(0xCA);
    assert(op.target_position_x == 6.25f && op.target_position_y == 4.5f);
    assert(op.cmd_keyboard == 0 && op.target_robot_id == 0);
    assert(!hasFrame(0xC9) && !hasFrame(0xCB) && !hasFrame(0xCC));

    // 坏 CRC、旧帧长度、非法模式和非有限数不能刷新命令。
    auto corrupt = drive;
    corrupt.back() ^= 1;
    auto old_layout = drive;
    old_layout.resize(13U);
    old_layout[1] = static_cast<uint8_t>(old_layout.size());
    old_layout.back() = crc8(old_layout);
    auto bad_mode = command(5, 1.0f, 0);
    auto bad_speed = command(0, std::numeric_limits<float>::infinity(), 0);
    input = corrupt;
    input.insert(input.end(), old_layout.begin(), old_layout.end());
    input.insert(input.end(), bad_mode.begin(), bad_mode.end());
    input.insert(input.end(), bad_speed.begin(), bad_speed.end());
    const auto rfr = encode(USB_RFR_CMD_SOF, ReceiveRfrCmd_s{});
    const auto path = encode(USB_GLOBAL_PATH_SOF, ReceiveGlobalPath_s{});
    input.insert(input.end(), rfr.begin(), rfr.end());
    input.insert(input.end(), path.begin(), path.end());
    ++now;
    step(link);
    assert(command_updates == 1);

    // 同步请求粘包，CDC 忙时保留响应，恢复后依序回显。
    ReceiveSyncRequest_s sync{};
    sync.sequence = 0x12345678U;
    input = encode(0xA0, sync);
    ++sync.sequence;
    const auto second = encode(0xA0, sync);
    input.insert(input.end(), second.begin(), second.end());
    busy = true;
    const size_t sent_count = output.size();
    step(link);
    assert(command_updates == 1 && output.size() == sent_count);
    busy = false;
    ++now;
    step(link);
    auto response = latest<SendSyncResponse_s>(0xB0);
    assert(response.sequence == 0x12345678U && response.t2_ticks < response.t3_ticks);
    ++now;
    step(link);
    response = latest<SendSyncResponse_s>(0xB0);
    assert(response.sequence == 0x12345679U && command_updates == 1);

    now = 1050;
    keepFeedbackFresh();
    step(link);
    auto nav = latest<SendNavStatus_s>(0xB3);
    assert(nav.transform_state == 4 && nav.vx == 0.75f && nav.vy == -.25f && nav.omega == -.5f);
    assert(nav.remaining_energy_buffercap == 47 && nav.rfr_pwr_limit == 80);
    assert(nav.remaining_energy_supercap == 0 && nav.curr_chassis_pwr == 0);

    now = 1100;
    keepFeedbackFresh();
    step(link);
    assert(state.usb_link.offline && latest<SendNavStatus_s>(0xB3).transform_state == 0);

    now = 1150;
    keepFeedbackFresh();
    input = command(3, 0, 0);
    step(link);
    assert(state.robot_cmd.passage_requested && !state.robot_cmd.heading_valid);
    assert(latest<SendNavStatus_s>(0xB3).transform_state == 3);
    input = command(0, 0, 0);
    step(link);
    assert(state.robot_cmd.passage_requested);
    input = command(4, 0, 0);
    step(link);
    assert(!state.robot_cmd.passage_requested);
    assert(endpoint_rearms == 7);

    now += 50;
    keepFeedbackFresh();
    input = command(3, 0.5f, -0.25f, 0.125f, 1, -0.75f);
    step(link);
    assert(state.robot_cmd.heading_valid && state.robot_cmd.big_yaw_heading_rad == -0.75f);
    assert(state.robot_cmd.passage_requested && state.robot_cmd.vx == 0.5f);
    assert(state.robot_cmd.vy == 0.125f && state.robot_cmd.wz == -0.25f);
    assert(latest<SendNavStatus_s>(0xB3).transform_state == 3);

    now += 50;
    keepFeedbackFresh();
    state.gimbal.transformed = GimbalShape::Folded;
    input = command(3, 0.5f, -0.25f, 0.125f, 1, -0.75f);
    step(link);
    assert(latest<SendNavStatus_s>(0xB3).transform_state == 4);

    now += 50;
    keepFeedbackFresh();
    const auto applied = command_updates;
    input = command(3, 0.5f, -0.25f, 0.125f, 2, -0.75f);
    const auto bad_lateral = command(3, 0.5f, -0.25f, std::numeric_limits<float>::quiet_NaN(), 1, -0.75f);
    input.insert(input.end(), bad_lateral.begin(), bad_lateral.end());
    step(link);
    assert(command_updates == applied);

    now += 50;
    keepFeedbackFresh();
    input = command(3, 0.0f, 0.0f);
    step(link);
    assert(state.robot_cmd.passage_requested && !state.robot_cmd.heading_valid);
    assert(latest<SendNavStatus_s>(0xB3).transform_state == 3);

    now += 50;
    keepFeedbackFresh();
    input = command(4, 0.0f, 0.0f, 0.0f, 1, -0.75f);
    step(link);
    assert(!state.robot_cmd.passage_requested);
    assert(latest<SendNavStatus_s>(0xB3).transform_state == 5);

    now += 50;
    keepFeedbackFresh();
    state.gimbal.transformed = GimbalShape::Upright;
    input = command(4, 0.0f, 0.0f, 0.0f, 1, -0.75f);
    step(link);
    assert(latest<SendNavStatus_s>(0xB3).transform_state == 4);

    // BLOCKED must reach USB even if CDC was busy throughout the physical motion.
    for (const auto requested : {USB_ROBO_MODE_FOLD, USB_ROBO_MODE_RESTORE}) {
        const bool fold = requested == USB_ROBO_MODE_FOLD;
        const auto shape_command = command(requested, 0, 0, 0, 1, -0.75f);
        now += 50;
        keepFeedbackFresh();
        input = shape_command;
        const size_t before_blocked = output.size();
        busy = true;
        step(link);
        assert(output.size() == before_blocked);

        now += 50;
        keepFeedbackFresh();
        state.gimbal.transformed = fold ? GimbalShape::Folded : GimbalShape::Upright;
        input = shape_command;
        busy = false;
        step(link);
        assert(latest<SendNavStatus_s>(0xB3).transform_state == (fold ? 3 : 5));

        now += 50;
        keepFeedbackFresh();
        input = shape_command;
        step(link);
        assert(latest<SendNavStatus_s>(0xB3).transform_state == 4);
    }

    now += 1000;
    const auto frames_before_offline = output.size();
    step(link);
    const auto offline_stage = latest<SendCompStage_s>(0xC0);
    assert(offline_stage.game_progress == 0 && offline_stage.stage_remain_time == 0);
    assert(std::none_of(output.begin() + frames_before_offline, output.end(),
        [](const Bytes& bytes) {
            const auto sof = bytes.front();
            return sof >= USB_COMP_EVENT_SOF && sof <= USB_ENEMY_BUFF_SOF;
        }));

    feedReferee(ROBOT_HURT_CMD_ID, hurt);
    connected = false;
    step(link);
    assert(state.usb_link.offline && !state.robot_cmd.chassis_enabled);
    assert(state.robot_cmd.vx == 0 && state.robot_cmd.wz == 0);
    feedReferee(ROBOT_HURT_CMD_ID, hurt);
    const size_t reconnect_start = output.size();
    connected = true;
    step(link);
    assert(std::none_of(output.begin() + reconnect_start, output.end(),
        [](const Bytes& bytes) { return bytes.front() == USB_HEALTH_REDUCTION_SOF; }));

    // A2 跨两次接收时，后续 A1 仍能正确分帧。
    // Independent wire fixture: yaw 1.25f, pitch -0.5f, mode 1.
    Bytes a2{0xA2, 12, 0, 0, 0xA0, 0x3F, 0, 0, 0, 0xBF, 1, 0};
    a2.back() = crc8(a2);
    const auto before_a2 = command_updates;
    const auto before_shoot = shoot_updates;
    input = a2;
    input.back() ^= 1U;
    step(link);
    assert(shoot_updates == before_shoot);

    input.assign(a2.begin(), a2.begin() + 5);
    step(link);
    assert(shoot_updates == before_shoot);
    assert(command_updates == before_a2 && state.usb_link.offline);

    now += 10;
    input.assign(a2.begin() + 5, a2.end());
    const auto stationary = command(USB_ROBO_MODE_NORMAL, 0.0f, 0.0f);
    input.insert(input.end(), stationary.begin(), stationary.end());
    step(link);
    assert(command_updates == before_a2 + 1 && !state.usb_link.offline);
    assert(shoot_updates == before_shoot + 1);
    assert(state.shoot_cmd.enabled && state.shoot_cmd.last_update_ms == now);
    assert(state.shoot_cmd.yaw == 1.25f && state.shoot_cmd.pitch == -0.5f);
    assert(state.shoot_cmd.shoot_mode == USB_SHOOT_MODE_SINGLE);

    // 非法档位和非有限角度不能刷新自瞄指令。
    Bytes bad_shoot_mode = a2;
    bad_shoot_mode[10] = 3U;
    bad_shoot_mode.back() = crc8(bad_shoot_mode);
    Bytes nan_yaw{0xA2, 12, 0, 0, 0xC0, 0x7F, 0, 0, 0, 0xBF, 2, 0};
    nan_yaw.back() = crc8(nan_yaw);
    input = bad_shoot_mode;
    input.insert(input.end(), nan_yaw.begin(), nan_yaw.end());
    ++now;
    step(link);
    assert(shoot_updates == before_shoot + 1);
    assert(state.shoot_cmd.shoot_mode == USB_SHOOT_MODE_SINGLE);
    assert(state.shoot_cmd.yaw == 1.25f);

    const auto last_a1_ms = state.robot_cmd.last_update_ms;
    now += 1000;
    input = a2;
    step(link);
    assert(shoot_updates == before_shoot + 2 && state.shoot_cmd.last_update_ms == now);
    assert(command_updates == before_a2 + 1 && state.robot_cmd.last_update_ms == last_a1_ms);
    assert(state.usb_link.offline);

    // 断链后立即撤掉 A2 的自瞄和发弹请求。
    connected = false;
    step(link);
    assert(!state.shoot_cmd.enabled && state.shoot_cmd.shoot_mode == USB_SHOOT_MODE_NONE);
    connected = true;
    step(link);
    std::puts("PASS: A2 framing, ShootCmd validation and A1 isolation");
    std::puts("PASS: USB stream and timing, referee UART parsing and USB telemetry, disconnect");
}
"""

def cdc_receive_functions():
    source = CDC.read_text(encoding="utf-8")
    bodies = []
    for signature in (
        "static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)",
        "int8_t USB_Receive(uint8_t* Buf, uint32_t *Len)",
    ):
        start = source.index(signature)
        end = source.index("\n}", start) + 2
        bodies.append(source[start:end])
    return "\n\n".join(bodies)


def main():
    BUILD.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="navigation_usb_", dir=BUILD) as directory:
        temp = Path(directory)
        for name, contents in HEADERS.items():
            (temp / name).write_text(contents, encoding="utf-8")
        source = temp / "test.cpp"
        source.write_text(
            TEST.replace("/* Production CDC receive functions */", cdc_receive_functions()),
            encoding="utf-8",
        )
        includes = [
            temp, USB, CORE / "3_LuojiaFox", CORE / "2_Device/DMIMU",
            CORE / "1_Middleware/Driver/Tick", CORE / "4_Application/Chassis",
            CORE / "4_Application/Gimbal", CORE / "4_Application/Referee",
            ROOT / "Core/Inc", REFEREE, ALGORITHM / "CRC", ALGORITHM / "FIFO",
        ]
        include_flags = ["-I" + str(path) for path in includes]
        c_objects = []
        for c_source in (ALGORITHM / "CRC/CRC8_CRC16.c", ALGORITHM / "FIFO/fifo.c"):
            obj = temp / (c_source.stem + ".o")
            subprocess.run(
                ["gcc", "-std=c11", *include_flags, "-c", str(c_source), "-o", str(obj)],
                check=True,
            )
            c_objects.append(str(obj))
        executable = temp / "test.exe"
        subprocess.run(
            ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
             *include_flags, str(source), str(USB / "usb.cpp"), str(REFEREE / "referee.cpp"),
             *c_objects, "-o", str(executable)],
            check=True,
        )
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
