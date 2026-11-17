/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb.hpp"

#include "app_board_link.hpp"
#include "app_chassis.h"
#include "chassis_config.hpp"
#include "dm_imu.hpp"
#include "drv_tick.h"
#include "gimbal_config.h"
#include "lf_core.hpp"
#include "referee.hpp"
#include "usbd_cdc_if.h"

#include <cmath>
#include <cstring>

namespace {

constexpr uint32_t kRawImuPeriodMs = 4U;
constexpr uint32_t kGimbalStatusPeriodMs = 10U;
constexpr uint32_t kNavStatusPeriodMs = 50U;
constexpr uint32_t kBigYawHeadingPeriodMs = 50U;
constexpr uint32_t kRefereeSlowPeriodMs = 1000U;
constexpr uint32_t kRefereeFastPeriodMs = 333U;
constexpr uint32_t kRobotStatusPeriodMs = 100U;
constexpr uint32_t kRefereePowerTimeoutMs = 1000U;
constexpr uint32_t kRawImuTimeoutMs = 10U;
constexpr float kDegToRad = 0.01745329252f;

uint8_t UsbCrc8(const uint8_t* bytes, uint32_t size)
{
    uint8_t crc = 0U;
    for (uint32_t i = 0U; i < size; ++i) {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            const uint8_t next = static_cast<uint8_t>(crc << 1U);
            crc = (crc & 0x80U) != 0U ? next ^ 0x07U : next;
        }
    }
    return crc;
}

uint8_t ReceiveLength(uint8_t sof)
{
    switch (sof) {
        case USB_SYNC_REQUEST_SOF: return sizeof(ReceiveSyncRequest_s);
        case USB_ROBO_CMD_SOF: return sizeof(ReceiveRoboCmd_s);
        case USB_SHOOT_CMD_SOF: return sizeof(ReceiveShootCmd_s);
        case USB_RFR_CMD_SOF: return sizeof(ReceiveRfrCmd_s);
        case USB_GLOBAL_PATH_SOF: return sizeof(ReceiveGlobalPath_s);
        case USB_DECISION_GIMBAL_CMD_SOF: return sizeof(ReceiveDecisionGimbalCmd_s);
        default: return 0U;
    }
}

template <typename Packet>
bool SendFrame(uint8_t sof, Packet& packet)
{
    packet.frame_header.sof = sof;
    packet.frame_header.len = sizeof(packet);
    auto* bytes = reinterpret_cast<uint8_t*>(&packet);
    packet.checksum = UsbCrc8(bytes, sizeof(packet) - 1U);
    return CDC_Transmit_HS(bytes, sizeof(packet)) == USBD_OK;
}

int16_t ToStatusInt16(uint16_t value)
{
    return static_cast<int16_t>(value > INT16_MAX ? INT16_MAX : value);
}

template <typename Status>
bool ReadRefereeStatus(const Status& source, Status& status)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool online = !referee.IsOffline();
    if (online) {
        status = source;
    }
    __set_PRIMASK(mask);
    return online;
}

}  // namespace

Usb usb;

void Usb::Init(void)
{
    LuojiaFox::system_state.updateUsbLink({true});
}

void Usb::Update(void)
{
    const bool connected = Receive();
    referee.SetUsbEventForwarding(connected);
    const uint32_t now = HAL_GetTick();
    const bool offline = !has_robot_cmd ||
        now - last_robot_cmd_ms >= chassis_app::kAutoCommandTimeoutMs;
    LuojiaFox::system_state.updateUsbLink({offline});
    if (connected) {
        SendAll();
    }
}

bool Usb::Receive(void)
{
    uint32_t received = sizeof(rx_buf) - rx_size;
    if (USB_Receive(rx_buf + rx_size, &received) != USBD_OK) {
        rx_size = 0U;
        sync_pending = false;
        if (has_robot_cmd) {
            LuojiaFox::RobotCmdState stopped{};
            stopped.passage_requested = LuojiaFox::system_state.read().robot_cmd.passage_requested;
            LuojiaFox::system_state.updateRobotCmd(stopped);
            has_robot_cmd = false;
        }
        if (has_shoot_cmd) {
            // 断链后立即撤掉自瞄与发弹请求，消费方无需再等超时。
            LuojiaFox::ShootCmdState stopped{};
            LuojiaFox::system_state.updateShootCmd(stopped);
            has_shoot_cmd = false;
        }
        return false;
    }
    rx_size += received;

    uint32_t consumed = 0U;
    while (rx_size - consumed >= sizeof(FrameHeader_t)) {
        const auto* frame = rx_buf + consumed;
        const uint8_t size = ReceiveLength(frame[0]);
        if (size == 0U || frame[1] != size) {
            ++consumed;
            continue;
        }
        if (rx_size - consumed < size) {
            break;
        }
        if (UsbCrc8(frame, size - 1U) != frame[size - 1U]) {
            ++consumed;
            continue;
        }
        if (frame[0] == USB_SYNC_REQUEST_SOF && sync_pending) {
            // 等前一条回包提交后，再处理下一条同步请求。
            break;
        }

        switch (frame[0]) {
            case USB_SYNC_REQUEST_SOF: {
                ReceiveSyncRequest_s request{};
                memcpy(&request, frame, sizeof(request));
                sync_response.sequence = request.sequence;
                sync_response.t2_ticks = getTick100us();
                sync_pending = true;
                break;
            }
            case USB_ROBO_CMD_SOF: {
                ReceiveRoboCmd_s command{};
                memcpy(&command, frame, sizeof(command));
                UpdateRoboCmd(command);
                break;
            }
            case USB_SHOOT_CMD_SOF: {
                ReceiveShootCmd_s command{};
                memcpy(&command, frame, sizeof(command));
                UpdateShootCmd(command);
                break;
            }
            default:
                break;
        }
        consumed += size;
    }
    rx_size -= consumed;
    memmove(rx_buf, rx_buf + consumed, rx_size);
    return true;
}

void Usb::UpdateRoboCmd(const ReceiveRoboCmd_s& frame)
{
    if (frame.mode > USB_ROBO_MODE_RESTORE || frame.heading_valid > 1U ||
        !std::isfinite(frame.velocity) || !std::isfinite(frame.lateral_velocity) ||
        !std::isfinite(frame.omega) || !std::isfinite(frame.big_yaw_heading)) {
        return;
    }

    LuojiaFox::RobotCmdState command{};
    command.vx = frame.velocity;
    command.vy = frame.lateral_velocity;
    command.wz = frame.omega;
    command.chassis_enabled = true;
    command.heading_valid = frame.heading_valid != 0U;
    if (command.heading_valid) {
        command.big_yaw_heading_rad = frame.big_yaw_heading;
    }
    const bool previous_passage_request = LuojiaFox::system_state.read().robot_cmd.passage_requested;
    command.passage_requested = previous_passage_request;
    if (frame.mode == USB_ROBO_MODE_FOLD) {
        command.passage_requested = true;
    } else if (frame.mode == USB_ROBO_MODE_RESTORE) {
        command.passage_requested = false;
    }
    if (command.passage_requested != previous_passage_request) {
        transform_blocked_pending = true;
    }
    last_robot_cmd_ms = HAL_GetTick();
    command.last_update_ms = last_robot_cmd_ms;
    LuojiaFox::system_state.updateRobotCmd(command);
    has_robot_cmd = true;
}

void Usb::UpdateShootCmd(const ReceiveShootCmd_s& frame)
{
    if (frame.shoot_mode > USB_SHOOT_MODE_BURST ||
        !std::isfinite(frame.yaw) || !std::isfinite(frame.pitch)) {
        return;
    }

    // 角度只做有效性检查，圆周与机械限幅由云台控制环处理。
    LuojiaFox::ShootCmdState command{};
    command.yaw = frame.yaw;
    command.pitch = frame.pitch;
    command.shoot_mode = frame.shoot_mode;
    command.enabled = true;
    LuojiaFox::system_state.updateShootCmd(command);
    has_shoot_cmd = true;
}

void Usb::SendAll(void)
{
    if (sync_pending) {
        sync_response.t3_ticks = getTick100us();
        if (!SendFrame(USB_SYNC_RESPONSE_SOF, sync_response)) {
            return;
        }
        sync_pending = false;
    }

    const uint32_t now = HAL_GetTick();
    if (now - last_nav_status_ms >= kNavStatusPeriodMs && SendNavStatus(now)) {
        last_nav_status_ms = now;
    }
    if (now - last_gimbal_status_ms >= kGimbalStatusPeriodMs && SendGimbalStatus(now)) {
        last_gimbal_status_ms = now;
    }
    if (now - last_big_yaw_heading_ms >= kBigYawHeadingPeriodMs && SendBigYawHeading(now)) {
        last_big_yaw_heading_ms = now;
    }
    if (now - last_raw_imu_ms >= kRawImuPeriodMs && SendRawGimbalImu(now)) {
        last_raw_imu_ms = now;
    }
    SendRefereeEvent();
    if (now - last_comp_stage_ms >= kRefereeSlowPeriodMs && SendCompStage()) {
        last_comp_stage_ms = now;
    }
    if (now - last_comp_event_ms >= kRefereeSlowPeriodMs && SendCompEvent()) {
        last_comp_event_ms = now;
    }
    if (now - last_allied_health_ms >= kRefereeFastPeriodMs && SendAlliedHealth()) {
        last_allied_health_ms = now;
    }
    if (now - last_allied_pos_ms >= kRefereeSlowPeriodMs && SendAlliedPos()) {
        last_allied_pos_ms = now;
    }
    if (now - last_robot_status_ms >= kRobotStatusPeriodMs && SendRobotStatus()) {
        last_robot_status_ms = now;
    }
    if (now - last_robot_pos_ms >= kRefereeSlowPeriodMs && SendRobotPos()) {
        last_robot_pos_ms = now;
    }
    if (now - last_rfid_status_ms >= kRefereeFastPeriodMs && SendRfidStatus()) {
        last_rfid_status_ms = now;
    }
    if (now - last_sentry_info_ms >= kRefereeSlowPeriodMs && SendSentryInfo()) {
        last_sentry_info_ms = now;
    }
    // 雷达站的 C9 / CB / CC 发送入口预留，暂不提交 CDC。
}

bool Usb::SendRawGimbalImu(uint32_t now)
{
    constexpr uint8_t required = (1U << (QUAT_DATA - ACCEL_DATA)) |
                                 (1U << (QUAT_DATA - GYRO_DATA));
    imu_t sample{};
    if (!dm_imu_get_data(&sample) || (sample.received_mask & required) != required ||
        now - sample.accel_rx_ms >= kRawImuTimeoutMs ||
        now - sample.gyro_rx_ms >= kRawImuTimeoutMs ||
        sample.gyro_revision == last_gyro_revision) {
        return false;
    }

    SendRawGimbalImu_s frame{};
    // CAN 加速度与角速度分帧到达，沿用融合侧对样本时效的保守定义。
    const uint32_t sample_ms = now - sample.accel_rx_ms >= now - sample.gyro_rx_ms
        ? sample.accel_rx_ms : sample.gyro_rx_ms;
    frame.sample_ticks = sample_ms * 10U;
    frame.accel_x = sample.accel[0];
    frame.accel_y = sample.accel[1];
    frame.accel_z = sample.accel[2];
    frame.gyro_x = sample.gyro[0];
    frame.gyro_y = sample.gyro[1];
    frame.gyro_z = sample.gyro[2];
    if (!SendFrame(USB_RAW_GIMBAL_IMU_SOF, frame)) {
        return false;
    }
    last_gyro_revision = sample.gyro_revision;
    return true;
}

bool Usb::SendGimbalStatus(uint32_t now)
{
    using LuojiaFox::GimbalShape;
    const auto state = LuojiaFox::system_state.read();
    const auto& imu = state.muzzle_imu;
    const auto& joint = state.gimbal;
    board_link::BigYawFeedback big_yaw{};
    board_link::getPeerBigYawFeedback(big_yaw);
    if (imu.last_update_ms == 0U || imu.last_update_ms == last_attitude_sample_ms ||
        now - imu.last_update_ms >= GIMBAL_IMU_TIMEOUT_MS ||
        !joint.online || now - joint.updated_at_ms >= GIMBAL_IMU_TIMEOUT_MS ||
        !big_yaw.motor_online || !std::isfinite(big_yaw.joint_rad) ||
        !std::isfinite(joint.joint_pitch_rad) || !std::isfinite(joint.joint_yaw_rad) ||
        !std::isfinite(joint.joint_roll_rad) ||
        (joint.transformed != GimbalShape::Folded && joint.transformed != GimbalShape::Upright)) {
        return false;
    }

    SendGimbalStatus_s frame{};
    frame.sample_ticks = imu.last_update_ms * 10U;
    frame.w = imu.quaternion_wxyz[0];
    frame.x = imu.quaternion_wxyz[1];
    frame.y = imu.quaternion_wxyz[2];
    frame.z = imu.quaternion_wxyz[3];
    const float norm_squared = frame.w * frame.w + frame.x * frame.x +
                               frame.y * frame.y + frame.z * frame.z;
    if (!std::isfinite(norm_squared) || norm_squared == 0.0f) {
        return false;
    }
    frame.pitch = joint.joint_pitch_rad;
    frame.yaw = joint.joint_yaw_rad;
    frame.big_yaw = big_yaw.joint_rad;
    frame.roll = joint.joint_roll_rad;
    frame.transformed = static_cast<uint8_t>(joint.transformed);
    if (!SendFrame(USB_GIMBAL_STATUS_SOF, frame)) {
        return false;
    }
    last_attitude_sample_ms = imu.last_update_ms;
    return true;
}

bool Usb::SendNavStatus(uint32_t now)
{
    using LuojiaFox::GimbalShape;
    using LuojiaFox::GimbalTransformState;
    const auto state = LuojiaFox::system_state.read();
    const auto& chassis = state.chassis;
    const auto& joint = state.gimbal;
    const auto& command = state.robot_cmd;
    const bool motion_valid = chassis.valid &&
        now - chassis.last_update_ms < board_link::can3::kChassisFeedbackTimeoutMs &&
        std::isfinite(chassis.vx) && std::isfinite(chassis.vy) && std::isfinite(chassis.wz);

    SendNavStatus_s frame{};
    if (motion_valid) {
        frame.vx = chassis.vx;
        frame.vy = chassis.vy;
        frame.omega = chassis.wz;
    }
    auto transform = joint.transform_state;
    if (Chassis_GetMode() != CHASSIS_MODE_AUTO_VISION || !command.chassis_enabled ||
        state.usb_link.offline) {
        transform = GimbalTransformState::Disabled;
    } else if (transform != GimbalTransformState::Fault) {
        board_link::BigYawFeedback big_yaw{};
        board_link::getPeerBigYawFeedback(big_yaw);
        if (!motion_valid || !joint.online || joint.manual_passage ||
            now - joint.updated_at_ms >= GIMBAL_IMU_TIMEOUT_MS || !big_yaw.ready) {
            transform = GimbalTransformState::Recovering;
        } else if (command.passage_requested &&
                   (!command.heading_valid || joint.transformed != GimbalShape::Folded)) {
            transform = GimbalTransformState::Preparing;
        } else if (transform == GimbalTransformState::Ready &&
                   !command.passage_requested && joint.transformed != GimbalShape::Upright) {
            transform = GimbalTransformState::Restoring;
        }
    }
    const auto blocked = command.passage_requested ? GimbalTransformState::Preparing
                                                   : GimbalTransformState::Restoring;
    if (transform_blocked_pending && transform == GimbalTransformState::Ready) {
        // 即使 CDC 忙过整个动作，也先提交本次请求的阻塞反馈。
        transform = blocked;
    }
    frame.transform_state = static_cast<uint8_t>(transform);

    const auto power = referee.GetPowerSupplySnapshot();
    if (power.online && power.buffer_received && now - power.buffer_tick < kRefereePowerTimeoutMs) {
        frame.remaining_energy_buffercap = ToStatusInt16(power.buffer_energy_j);
    }
    if (power.online && power.power_received && now - power.power_tick < kRefereePowerTimeoutMs) {
        frame.rfr_pwr_limit = ToStatusInt16(power.power_limit_w);
    }
    if (!SendFrame(USB_NAV_STATUS_SOF, frame)) {
        return false;
    }
    if (transform == blocked) {
        transform_blocked_pending = false;
    }
    return true;
}

bool Usb::SendBigYawHeading(uint32_t now)
{
    const auto imu = LuojiaFox::system_state.read().imu;
    if (imu.last_update_ms == 0U || now - imu.last_update_ms >= GIMBAL_IMU_TIMEOUT_MS ||
        !std::isfinite(imu.yaw_deg)) {
        return false;
    }

    SendBigYawHeading_s frame{};
    frame.sample_ticks = imu.last_update_ms * 10U;
    frame.yaw = imu.yaw_deg * kDegToRad;
    return SendFrame(USB_BIG_YAW_HEADING_SOF, frame);
}

bool Usb::SendCompStage(void)
{
    game_status_t status{};
    SendCompStage_s frame{};
    if (ReadRefereeStatus(referee.game_status, status) && status.game_progress <= PROGRESS_CALCULATING) {
        frame.game_progress = status.game_progress;
        frame.stage_remain_time = status.stage_remain_time;
    }
    return SendFrame(USB_COMP_STAGE_SOF, frame);
}

bool Usb::SendCompEvent(void)
{
    event_data_t status{};
    if (!ReadRefereeStatus(referee.field_event, status)) {
        return false;
    }
    SendCompEvent_s frame{};
    frame.event_data = status.event_data;
    return SendFrame(USB_COMP_EVENT_SOF, frame);
}

bool Usb::SendAlliedHealth(void)
{
    game_robot_HP_t status{};
    if (!ReadRefereeStatus(referee.game_robot_HP, status)) {
        return false;
    }
    SendAlliedHealth_s frame{};
    frame.ally_1_robot_hp = status.ally_1_robot_HP;
    frame.ally_2_robot_hp = status.ally_2_robot_HP;
    frame.ally_3_robot_hp = status.ally_3_robot_HP;
    frame.ally_4_robot_hp = status.ally_4_robot_HP;
    frame.ally_outpost_hp = status.ally_outpost_HP;
    frame.ally_base_hp = status.ally_base_HP;
    frame.enemy_outpost_hp = status.enemy_outpost_HP;
    frame.enemy_base_hp = status.enemy_base_HP;
    return SendFrame(USB_ALLIED_HEALTH_SOF, frame);
}

bool Usb::SendAlliedPos(void)
{
    ground_robot_position_t status{};
    if (!ReadRefereeStatus(referee.ground_robot_pos_t, status)) {
        return false;
    }
    SendAlliedPos_s frame{};
    frame.hero_x = status.hero_x;
    frame.hero_y = status.hero_y;
    frame.engineer_x = status.engineer_x;
    frame.engineer_y = status.engineer_y;
    frame.standard_3_x = status.standard_3_x;
    frame.standard_3_y = status.standard_3_y;
    frame.standard_4_x = status.standard_4_x;
    frame.standard_4_y = status.standard_4_y;
    return SendFrame(USB_ALLIED_POS_SOF, frame);
}

bool Usb::SendRobotStatus(void)
{
    robot_status_t status{};
    projectile_allowance_t allowance{};
    if (!referee.ReadRobotStatus(status, allowance)) {
        return false;
    }
    SendRobotStatus_s frame{};
    frame.robot_id = status.robot_id;
    frame.current_hp = status.current_HP;
    frame.maximum_hp = status.maximum_HP;
    frame.projectile_allowance_17mm = allowance.projectile_allowance_17mm;
    frame.remaining_gold_coin = allowance.remaining_gold_coin;
    frame.projectile_allowance_fortress = allowance.projectile_allowance_fortress;
    return SendFrame(USB_ROBOT_STATUS_SOF, frame);
}

bool Usb::SendRobotPos(void)
{
    robot_pos_t status{};
    if (!ReadRefereeStatus(referee.game_robot_pos_t, status)) {
        return false;
    }
    SendRobotPos_s frame{};
    frame.self_x = status.x;
    frame.self_y = status.y;
    return SendFrame(USB_ROBOT_POS_SOF, frame);
}

bool Usb::SendRfidStatus(void)
{
    rfid_status_t status{};
    if (!ReadRefereeStatus(referee.RFID_STATUS, status)) {
        return false;
    }
    SendRfidStatus_s frame{};
    frame.rfid_status = status.rfid_status;
    frame.rfid_status_2 = status.rfid_status_2;
    return SendFrame(USB_RFID_STATUS_SOF, frame);
}

bool Usb::SendSentryInfo(void)
{
    sentry_info_t status{};
    if (!referee.ReadSentryInfo(status)) {
        return false;
    }
    SendSentryInfo_s frame{};
    frame.sentry_info = status.sentry_info;
    frame.sentry_info_2 = status.sentry_info_2;
    frame.sentry_info_3 = status.sentry_info_3;
    return SendFrame(USB_SENTRY_INFO_SOF, frame);
}

void Usb::SendRefereeEvent(void)
{
    RefereeUsbEvent event{};
    if (!referee.PeekUsbEvent(event)) {
        return;
    }
    bool sent;
    if (event.type == RefereeUsbEvent::Type::HealthReduction) {
        SendHealthReduction_s frame{};
        frame.armor_and_reason = event.armor_and_reason;
        sent = SendFrame(USB_HEALTH_REDUCTION_SOF, frame);
    } else {
        SendOperatorCmd_s frame{};
        frame.target_position_x = event.waypoint_x;
        frame.target_position_y = event.waypoint_y;
        sent = SendFrame(USB_OPERATOR_CMD_SOF, frame);
    }
    if (sent) {
        referee.PopUsbEvent();
    }
}
