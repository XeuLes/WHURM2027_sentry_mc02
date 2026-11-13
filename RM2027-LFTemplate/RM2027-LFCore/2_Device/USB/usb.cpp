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

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "gimbal_config.h"
//#include "macro_typedef.h"  遥控器宏定义
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "referee.hpp"
#include "lf_core.hpp"
#include "board_link_can.hpp"
#include "app_board_link.hpp"
#include "app_chassis.h"
#include "chassis_config.hpp"


#define USB_LINK_TIMEOUT_MS 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Debug       5   // ms
#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   10  // ms
#define SEND_DURATION_Event       10  // ms
#define SEND_DURATION_Pid         10  // ms
#define SEND_DURATION_AllRobotHp  10  // ms
#define SEND_DURATION_GameStatus  10  // ms
#define SEND_DURATION_RobotMotion 10 // ms，匹配 CAN3 底盘实测反馈
#define SEND_DURATION_GroundRobotPosition   10// ms
#define SEND_DURATION_RfidStatus   10// ms
#define SEND_DURATION_RobotStatus  10// ms
#define SEND_DURATION_JointState   10// ms
#define SEND_DURATION_Buff         10// ms
#define SEND_DURATION_SentryInfo   10// ms
#define SEND_DURATION_MapCommand 10 // ms
#define SEND_DURATION_InitSpeed 10 // ms
#define SEND_DURATION_NavigationStatus 50 // ms

// clang-format on

#define HEADER_SIZE 4         // byte

#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - last_send_time.send_name) >= SEND_DURATION_##send_name) {   \
            last_send_time.send_name = HAL_GetTick();                                    \
            Send##send_name();                                                           \
        }                                                                                \
    } while (0)

// 模块单例
Usb usb;

/**
 * @brief      Run one full cycle: send + receive + offline check
 * @param[in]  none
 * @retval     None
 */
void Usb::Update(void)
{
    SendAll();
    Receive();
    UpdateVirtualRc();
    UpdateSentryCmd();

    if (HAL_GetTick() - receive_time > USB_LINK_TIMEOUT_MS) {
        offline = true;
        continue_receive_cnt = 0;
    } else if (continue_receive_cnt > USB_CONNECT_CNT) {
        offline = false;
    } else {
        continue_receive_cnt++;
    }

    LuojiaFox::system_state.updateUsbLink({offline != 0});
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
void Usb::Init(void)
{
    rx_size = 0U;
    // 数据置零
    memset(&last_send_time, 0, sizeof(LastSendTime_t));
    memset(&receive_robot_cmd, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&receive_pid_debug, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&receive_virtual_rc, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&robot_cmd, 0, sizeof(RobotCmdData_t));
    memset(&virtual_rc, 0, sizeof(RcControl));
    robot_cmd_receive_time = 0;

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/
    
    // 1.初始化调试数据包
    // 帧头部分
    send_debug.frame_header.sof = SEND_SOF;
    send_debug.frame_header.len = (uint8_t)(sizeof(SendDataDebug_s) - 6);
    send_debug.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_debug.frame_header), sizeof(send_debug.frame_header));
    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        send_debug.packages[i].type = 1;
        send_debug.packages[i].name[0] = '\0';
    }
    
    // 2.初始化IMU数据包
    send_imu.frame_header.sof = SEND_SOF;
    send_imu.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    send_imu.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_imu.frame_header), sizeof(send_imu.frame_header));
    /*******************************************************************************/
    /* Referee                                                                     */
    /*******************************************************************************/
    
    // 3.初始化机器人信息数据包
    // 帧头部分
    send_robot_state_info.frame_header.sof = SEND_SOF;
    send_robot_state_info.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    send_robot_state_info.frame_header.id = ROBOT_STATE_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_robot_state_info.frame_header), sizeof(send_robot_state_info.frame_header));
    // 数据部分
    #ifdef GIMBAL_TYPE
    send_robot_state_info.data.type.chassis = CHASSIS_OMNI_WHEEL;
    #else
    send_robot_state_info.data.type.chassis = CHASSIS_TYPE;
    #endif
    send_robot_state_info.data.type.gimbal = GIMBAL_TYPE;
    send_robot_state_info.data.type.shoot = SHOOT_TYPE;
    send_robot_state_info.data.type.arm = MECHANICAL_ARM_TYPE;
    
    // 4.初始化事件数据包
    send_event.frame_header.sof = SEND_SOF;
    send_event.frame_header.len = (uint8_t)(sizeof(SendDataEvent_s) - 6);
    send_event.frame_header.id = EVENT_DATA_SEND_ID;
    append_CRC8_check_sum
        ((uint8_t *)(&send_event.frame_header), sizeof(send_event.frame_header));

    // 5.初始化pid调参数据
    send_pid.frame_header.sof = SEND_SOF;
    send_pid.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    send_pid.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_pid.frame_header), sizeof(send_pid.frame_header));

    // 6.初始化所有机器人血量数据
    send_all_robot_hp.frame_header.sof = SEND_SOF;
    send_all_robot_hp.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    send_all_robot_hp.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_all_robot_hp.frame_header),
        sizeof(send_all_robot_hp.frame_header));

    // 7.初始化比赛状态数据
    send_game_status.frame_header.sof = SEND_SOF;
    send_game_status.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    send_game_status.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_game_status.frame_header),
        sizeof(send_game_status.frame_header));
     // 8.初始化机器人运动数据
    send_robot_motion.frame_header.sof = SEND_SOF;
    send_robot_motion.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    send_robot_motion.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_robot_motion.frame_header),
        sizeof(send_robot_motion.frame_header));
    // 9.初始化地面机器人位置数据
    send_ground_robot_position.frame_header.sof = SEND_SOF;
    send_ground_robot_position.frame_header.len =(uint8_t)(sizeof(SendDataGroundRobotPosition_s) - 6);
    send_ground_robot_position.frame_header.id = GROUND_ROBOT_POSITION_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_ground_robot_position.frame_header),
        sizeof(send_ground_robot_position.frame_header));

    // 10.初始化RFID状态数据
    send_rfid_status.frame_header.sof = SEND_SOF;
    send_rfid_status.frame_header.len = (uint8_t)(sizeof(SendDataRfidStatus_s) - 6);
    send_rfid_status.frame_header.id = RFID_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_rfid_status.frame_header),
        sizeof(send_rfid_status.frame_header));

    // 11.初始化机器人状态数据
    send_robot_status.frame_header.sof = SEND_SOF;
    send_robot_status.frame_header.len = (uint8_t)(sizeof(SendDataRobotStatus_s) - 6);
    send_robot_status.frame_header.id = ROBOT_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_robot_status.frame_header),
        sizeof(send_robot_status.frame_header));
    
    // 12.初始化云台状态数据
    send_joint_state.frame_header.sof = SEND_SOF;
    send_joint_state.frame_header.len = (uint8_t)(sizeof(SendDataJointState_s) - 6);
    send_joint_state.frame_header.id = JOINT_STATE_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_joint_state.frame_header),
        sizeof(send_joint_state.frame_header));

    // 13.初始化机器人增益和底盘能量数据
    send_buff.frame_header.sof = SEND_SOF;
    send_buff.frame_header.len = (uint8_t)(sizeof(SendDataBuff_s) - 6);
    send_buff.frame_header.id = BUFF_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_buff.frame_header),
        sizeof(send_buff.frame_header));

    // 14.初始化哨兵决策信息数据
    send_sentry_info.frame_header.sof = SEND_SOF;
    send_sentry_info.frame_header.len = (uint8_t)(sizeof(SendDataSentryInfo_s) - 6);
    send_sentry_info.frame_header.id = SENTRYINFO_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_sentry_info.frame_header),
        sizeof(send_sentry_info.frame_header));

    // 15.初始化地图命令数据
    send_map_command.frame_header.sof = SEND_SOF;
    send_map_command.frame_header.len = (uint8_t)(sizeof(SendDataMapCmd_s) - 6);
    send_map_command.frame_header.id = MAP_COMMAND_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_map_command.frame_header),
        sizeof(send_map_command.frame_header));

    // 16.初始化弹速数据
    send_init_speed.frame_header.sof = SEND_SOF;
    send_init_speed.frame_header.len = (uint8_t)(sizeof(SendInitSpeed_s) - 6);
    send_init_speed.frame_header.id = INIT_SPEED_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&send_init_speed.frame_header),
        sizeof(send_init_speed.frame_header));

    send_navigation_status.frame_header.sof = SEND_SOF;
    send_navigation_status.frame_header.len = sizeof(SendDataNavigationStatus_s) - 6;
    send_navigation_status.frame_header.id = NAVIGATION_STATUS_SEND_ID;
    append_CRC8_check_sum(
        reinterpret_cast<uint8_t*>(&send_navigation_status.frame_header),
        sizeof(send_navigation_status.frame_header));
}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
void Usb::SendAll(void)
{
    if (HAL_GetTick() - last_send_time.NavigationStatus >= SEND_DURATION_NavigationStatus) {
        SendNavigationStatus();
    }
    // 发送Debug数据
    CheckDurationAndSend(Debug);
    // 发送Imu数据
    CheckDurationAndSend(Imu);
    // 发送RobotStateInfo数据
    CheckDurationAndSend(RobotStateInfo);
    // 发送Event数据
    CheckDurationAndSend(Event);
    // 发送PidDebug数据
    // CheckDurationAndSend(Pid);
    // 发送AllRobotHp数据
    CheckDurationAndSend(AllRobotHp); 
    // 发送GameStatus数据
    CheckDurationAndSend(GameStatus);
    // 发送RobotMotion数据
    CheckDurationAndSend(RobotMotion);
    // 发送GroundRobotPosition数据
    CheckDurationAndSend(GroundRobotPosition);
    // 发送RfidStatus数据
    CheckDurationAndSend(RfidStatus);
    // 发送RobotStatus数据
    CheckDurationAndSend(RobotStatus);
    // 发送JointState数据
    CheckDurationAndSend(JointState);
    // 发送Buff数据
    CheckDurationAndSend(Buff);

    CheckDurationAndSend(SentryInfo);

    CheckDurationAndSend(MapCommand);

    CheckDurationAndSend(InitSpeed);

}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
void Usb::Receive(void)
{
    uint32_t received = sizeof(rx_buf) - rx_size;
    if (USB_Receive(rx_buf + rx_size, &received) != USBD_OK) {
        rx_size = 0U;
        return;
    }
    rx_size += received;
    uint32_t consumed = 0U;
    while (rx_size - consumed >= HEADER_SIZE) {
        uint8_t* frame = rx_buf + consumed;
        if (frame[0] != RECEIVE_SOF || !verify_CRC8_check_sum(frame, HEADER_SIZE)) {
            ++consumed;
            continue;
        }
        const uint32_t frame_size = HEADER_SIZE + frame[1] + sizeof(uint16_t);
        const uint8_t id = frame[2];
        const bool known_size =
            (id == ROBOT_CMD_DATA_RECEIVE_ID && frame_size == sizeof(receive_robot_cmd)) ||
            (id == PID_DEBUG_DATA_RECEIVE_ID && frame_size == sizeof(receive_pid_debug)) ||
            (id == VIRTUAL_RC_DATA_RECEIVE_ID && frame_size == sizeof(receive_virtual_rc)) ||
            (id == SENTRY_CMD_RECEIVE_ID && frame_size == sizeof(receive_sentry_cmd));
        if (!known_size || frame_size > sizeof(rx_buf)) {
            ++consumed;
            continue;
        }
        if (rx_size - consumed < frame_size) { break; }
        if (!verify_CRC16_check_sum(frame, frame_size)) {
            ++consumed;
            continue;
        }
        consumed += frame_size;
        uint32_t timestamp;
        memcpy(&timestamp, frame + HEADER_SIZE, sizeof(timestamp));
        switch (id) {
        case ROBOT_CMD_DATA_RECEIVE_ID: {
            ReceiveDataRobotCmd_s candidate{};
            memcpy(&candidate, frame, sizeof(candidate));
            if (!AcceptRobotCmd(candidate)) { continue; }
            receive_time = robot_cmd_receive_time;
            break;
        }
        case PID_DEBUG_DATA_RECEIVE_ID:
            memcpy(&receive_pid_debug, frame, frame_size);
            break;
        case VIRTUAL_RC_DATA_RECEIVE_ID:
            memcpy(&receive_virtual_rc, frame, frame_size);
            break;
        case SENTRY_CMD_RECEIVE_ID:
            memcpy(&receive_sentry_cmd, frame, frame_size);
            break;
        default:
            break;
        }
        if (timestamp > latest_rx_timestamp) {
            latest_rx_timestamp = timestamp;
            receive_time = HAL_GetTick();
        }
    }
    rx_size -= consumed;
    memmove(rx_buf, rx_buf + consumed, rx_size);
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/


/**
 * @brief 发送DEBUG数据
 * @param duration 发送周期
 */
void Usb::SendDebug(void)
{
    send_debug.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&send_debug, sizeof(SendDataDebug_s));
    USB_Transmit((uint8_t *)&send_debug, sizeof(SendDataDebug_s));
}

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
void Usb::SendImu(void)
{
    const auto imu = LuojiaFox::system_state.read().muzzle_imu;
    const uint32_t now = HAL_GetTick();
    if (imu.last_update_ms == 0U || now - imu.last_update_ms >= GIMBAL_IMU_TIMEOUT_MS) {
        return;
    }

    send_imu.time_stamp = now;

    send_imu.data.yaw = imu.yaw_deg;
    send_imu.data.pitch = imu.pitch_deg;
    send_imu.data.roll = imu.roll_deg;

    send_imu.data.yaw_vel = imu.yaw_rate_dps;
    send_imu.data.pitch_vel = imu.pitch_rate_dps;
    send_imu.data.roll_vel = imu.roll_rate_dps;

    append_CRC16_check_sum((uint8_t *)&send_imu, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&send_imu, sizeof(SendDataImu_s));
}

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
void Usb::SendRobotStateInfo(void)
{
    send_robot_state_info.time_stamp = HAL_GetTick();

    send_robot_state_info.data.referee.id = referee.robot_status.robot_id;
    send_robot_state_info.data.referee.hp = referee.robot_status.current_HP;
    send_robot_state_info.data.referee.color = referee.GetTeamColor();       // 0-红 1-蓝 2-未知
    send_robot_state_info.data.referee.attacked = referee.IsAttacked();

    append_CRC16_check_sum((uint8_t *)&send_robot_state_info, sizeof(SendDataRobotStateInfo_s));
    USB_Transmit((uint8_t *)&send_robot_state_info, sizeof(SendDataRobotStateInfo_s));
}


/**
 * @brief 发送事件数据
 * @param duration 发送周期
 */
void Usb::SendEvent(void)
{
    send_event.time_stamp = HAL_GetTick();
    send_event.data.event_data = referee.field_event.event_data;
    append_CRC16_check_sum((uint8_t *)&send_event, sizeof(SendDataEvent_s));
    USB_Transmit((uint8_t *)&send_event, sizeof(SendDataEvent_s));
}

/**
 * @brief 发送PidDebug数据
 * @param duration 发送周期
 */
// static void SendPidDebug(void)
// {
//     send_pid.time_stamp = HAL_GetTick();
//     append_CRC16_check_sum((uint8_t *)&send_pid, sizeof(SendDataPidDebug_s));
// }

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
void Usb::SendAllRobotHp(void)
{
    send_all_robot_hp.time_stamp = HAL_GetTick();

    send_all_robot_hp.data.ally_1_robot_HP = referee.game_robot_HP.ally_1_robot_HP;
    send_all_robot_hp.data.ally_2_robot_HP = referee.game_robot_HP.ally_2_robot_HP;
    send_all_robot_hp.data.ally_3_robot_HP = referee.game_robot_HP.ally_3_robot_HP;
    send_all_robot_hp.data.ally_4_robot_HP = referee.game_robot_HP.ally_4_robot_HP;
    send_all_robot_hp.data.damage_difference = referee.game_robot_HP.damage_difference;
    send_all_robot_hp.data.ally_7_robot_HP = referee.game_robot_HP.ally_7_robot_HP;
    send_all_robot_hp.data.ally_outpost_HP = referee.game_robot_HP.ally_outpost_HP;
    send_all_robot_hp.data.ally_base_HP = referee.game_robot_HP.ally_base_HP;
    send_all_robot_hp.data.enemy_outpost_HP = referee.game_robot_HP.enemy_outpost_HP;
    send_all_robot_hp.data.enemy_base_HP = referee.game_robot_HP.enemy_base_HP;

    append_CRC16_check_sum((uint8_t *)&send_all_robot_hp, sizeof(SendDataAllRobotHp_s));
    USB_Transmit((uint8_t *)&send_all_robot_hp, sizeof(SendDataAllRobotHp_s));
}

/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
void Usb::SendGameStatus(void)
{
    send_game_status.time_stamp = HAL_GetTick();

    send_game_status.data.game_progress = referee.game_status.game_progress;
    send_game_status.data.stage_remain_time = referee.game_status.stage_remain_time;

    append_CRC16_check_sum((uint8_t *)&send_game_status, sizeof(SendDataGameStatus_s));
    USB_Transmit((uint8_t *)&send_game_status, sizeof(SendDataGameStatus_s));
}

/**
 * @brief 发送机器人运动数据
 * @param duration 发送周期
 */
void Usb::SendRobotMotion(void)
{
    const auto fdb_speed_vector = LuojiaFox::system_state.read().chassis;
    // RobotMotion has no validity bit; do not turn stale feedback into a new sample.
    if (!fdb_speed_vector.valid || HAL_GetTick() - fdb_speed_vector.last_update_ms >=
        board_link::can3::kChassisFeedbackTimeoutMs) { return; }

    send_robot_motion.time_stamp = fdb_speed_vector.last_update_ms;

    send_robot_motion.data.speed_vector.vx = fdb_speed_vector.vx;
    send_robot_motion.data.speed_vector.vy = fdb_speed_vector.vy;
    send_robot_motion.data.speed_vector.wz = fdb_speed_vector.wz;

    append_CRC16_check_sum((uint8_t *)&send_robot_motion, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&send_robot_motion, sizeof(SendDataRobotMotion_s));
}

/**
 * @brief 发送地面机器人位置数据
 * @param duration 发送周期
 */
void Usb::SendGroundRobotPosition(void)
{
    const ground_robot_position_t &pos = referee.ground_robot_pos_t;

    send_ground_robot_position.time_stamp = HAL_GetTick();
    send_ground_robot_position.data.hero_x = pos.hero_x;
    send_ground_robot_position.data.hero_y = pos.hero_y;
    send_ground_robot_position.data.engineer_x = pos.engineer_x;
    send_ground_robot_position.data.engineer_y = pos.engineer_y;
    send_ground_robot_position.data.standard_3_x = pos.standard_3_x;
    send_ground_robot_position.data.standard_3_y = pos.standard_3_y;
    send_ground_robot_position.data.standard_4_x = pos.standard_4_x;
    send_ground_robot_position.data.standard_4_y = pos.standard_4_y;
    send_ground_robot_position.data.reserved1 = pos.reserved1;
    send_ground_robot_position.data.reserved2 = pos.reserved2;

    append_CRC16_check_sum((uint8_t *)&send_ground_robot_position, sizeof(SendDataGroundRobotPosition_s));
    USB_Transmit((uint8_t *)&send_ground_robot_position, sizeof(SendDataGroundRobotPosition_s));
}

/**
 * @brief 发送RFID状态数据
 * @param duration 发送周期
 */
void Usb::SendRfidStatus(void)
{
    send_rfid_status.time_stamp = HAL_GetTick();
    send_rfid_status.data.rfid_status = referee.RFID_STATUS.rfid_status;
    send_rfid_status.data.rfid_status_2 = referee.RFID_STATUS.rfid_status_2;

    // send_rfid_status.data.base_gain_point = referee.RFID_STATUS.rfid_status & 0x01;
    // send_rfid_status.data.circular_highland_gain_point = (referee.RFID_STATUS.rfid_status >> 1) & 0x01;
    // send_rfid_status.data.enemy_circular_highland_gain_point = (referee.RFID_STATUS.rfid_status >> 2) & 0x01;
    

    append_CRC16_check_sum((uint8_t *)&send_rfid_status, sizeof(SendDataRfidStatus_s));
    USB_Transmit((uint8_t *)&send_rfid_status, sizeof(SendDataRfidStatus_s));
}

/**
 * @brief 发送机器人状态数据
 * @param duration 发送周期
 */
void Usb::SendRobotStatus(void)
{
    const robot_status_t &rs = referee.robot_status;

    send_robot_status.time_stamp = HAL_GetTick();

    send_robot_status.data.robot_id = rs.robot_id;
    send_robot_status.data.robot_level = rs.robot_level;
    send_robot_status.data.current_hp = rs.current_HP;
    send_robot_status.data.maximum_hp = rs.maximum_HP;
    send_robot_status.data.shooter_barrel_cooling_value = rs.shooter_barrel_cooling_value;
    send_robot_status.data.shooter_barrel_heat_limit = rs.shooter_barrel_heat_limit;
    send_robot_status.data.chassis_power_limit = rs.chassis_power_limit;
    send_robot_status.data.bullet_speed_limit = rs.bullet_speed_limit;

    send_robot_status.data.shooter_17mm_barrel_heat = referee.power_heat_data.shooter_17mm_barrel_heat;
    send_robot_status.data.shooter_42mm_barrel_heat = referee.power_heat_data.shooter_42mm_barrel_heat;

    send_robot_status.data.robot_pos_x = referee.game_robot_pos_t.x;
    send_robot_status.data.robot_pos_y = referee.game_robot_pos_t.y;
    send_robot_status.data.robot_pos_angle = referee.game_robot_pos_t.angle;

    send_robot_status.data.armor_id = referee.robot_hurt_t.armor_id;
    send_robot_status.data.hp_deduction_reason = referee.robot_hurt_t.HP_deduction_reason;

    send_robot_status.data.projectile_allowance_17mm = referee.projectile_allowance.projectile_allowance_17mm;
    send_robot_status.data.projectile_allowance_42mm = referee.projectile_allowance.projectile_allowance_42mm;
    send_robot_status.data.remaining_gold_coin = referee.projectile_allowance.remaining_gold_coin;

    append_CRC16_check_sum((uint8_t *)&send_robot_status, sizeof(SendDataRobotStatus_s));
    USB_Transmit((uint8_t *)&send_robot_status, sizeof(SendDataRobotStatus_s));
}


/**
 * @brief 发送云台状态数据
 * @param duration 发送周期
 */
void Usb::SendJointState(void)
{
    const auto state = LuojiaFox::system_state.read().gimbal;
    const uint32_t now = HAL_GetTick();
    if (!state.online || now - state.updated_at_ms >= GIMBAL_IMU_TIMEOUT_MS ||
        !isfinite(state.joint_pitch_rad) || !isfinite(state.joint_yaw_rad)) { return; }
    send_joint_state.time_stamp = state.updated_at_ms;
    send_joint_state.data.pitch = state.joint_pitch_rad;
    send_joint_state.data.yaw = state.joint_yaw_rad;
    append_CRC16_check_sum((uint8_t *)&send_joint_state, sizeof(SendDataJointState_s));
    USB_Transmit((uint8_t *)&send_joint_state, sizeof(SendDataJointState_s));
}

/**
 * @brief 发送机器人增益和底盘能量数据
 * @param duration 发送周期
 */
void Usb::SendBuff(void)
{
    const buff_t &buff = referee.buff_musk_t;

    send_buff.time_stamp = HAL_GetTick();
    send_buff.data.recovery_buff = buff.recovery_buff;
    send_buff.data.cooling_buff = buff.cooling_buff;
    send_buff.data.defence_buff = buff.defence_buff;
    send_buff.data.vulnerability_buff = buff.vulnerability_buff;
    send_buff.data.attack_buff = buff.attack_buff;
    send_buff.data.remaining_energy = buff.remaining_energy;

    append_CRC16_check_sum((uint8_t *)&send_buff, sizeof(SendDataBuff_s));
    USB_Transmit((uint8_t *)&send_buff, sizeof(SendDataBuff_s));
}

void Usb::SendSentryInfo(void)
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
    uint64_t sentry_info_3;

    send_sentry_info.time_stamp = HAL_GetTick();
    referee.GetSentryInfo(&sentry_info, &sentry_info_2, &sentry_info_3);
    send_sentry_info.data.sentry_info = sentry_info;
    send_sentry_info.data.sentry_info_2 = sentry_info_2;
    send_sentry_info.data.sentry_info_3 = sentry_info_3;
    append_CRC16_check_sum((uint8_t *)&send_sentry_info, sizeof(SendDataSentryInfo_s));
    USB_Transmit((uint8_t *)&send_sentry_info, sizeof(SendDataSentryInfo_s));
}

void Usb::SendMapCommand(void)
{
    send_map_command.time_stamp = HAL_GetTick();
    float x, y;
    uint8_t cmd_keyboard, target_robot_id;
    uint16_t cmd_source;
    referee.GetMapCommandData(&x, &y, &cmd_keyboard, &target_robot_id, &cmd_source);
    send_map_command.data.target_position_x = x;
    send_map_command.data.target_position_y = y;
    send_map_command.data.cmd_keyboard = cmd_keyboard;
    send_map_command.data.target_robot_id = target_robot_id;
    send_map_command.data.cmd_source = cmd_source;
    append_CRC16_check_sum((uint8_t *)&send_map_command, sizeof(SendDataMapCmd_s));
    USB_Transmit((uint8_t *)&send_map_command, sizeof(SendDataMapCmd_s));
}

void Usb::SendInitSpeed(void)
{
    send_init_speed.time_stamp = HAL_GetTick();
    send_init_speed.data.initial_speed = referee.shoot_data.initial_speed;
    append_CRC16_check_sum((uint8_t *)&send_init_speed, sizeof(SendInitSpeed_s));
    USB_Transmit((uint8_t *)&send_init_speed, sizeof(SendInitSpeed_s));
}

void Usb::SendNavigationStatus(void)
{
    using LuojiaFox::GimbalShape;
    using LuojiaFox::GimbalTransformState;
    const uint32_t now = HAL_GetTick();
    const auto state = LuojiaFox::system_state.read();
    const auto& imu = state.imu;
    const auto& joints = state.gimbal;
    const auto power = referee.GetPowerSupplySnapshot();
    board_link::BigYawFeedback big_yaw{};
    board_link::getPeerBigYawFeedback(big_yaw);

    send_navigation_status.time_stamp = now;
    auto& data = send_navigation_status.data;
    data = {};
    data.session = receive_robot_cmd.data.navigation.session;
    data.command_time_stamp = receive_robot_cmd.time_stamp;
    data.transformed = static_cast<uint8_t>(joints.transformed);

    const bool imu_valid = imu.last_update_ms != 0U &&
        now - imu.last_update_ms < GIMBAL_IMU_TIMEOUT_MS &&
        isfinite(imu.yaw_deg) && isfinite(imu.pitch_deg) && isfinite(imu.roll_deg);
    if (imu_valid) {
        constexpr float radians_per_degree = 0.017453292519943295f;
        data.flags |= NAV_STATUS_IMU_VALID;
        data.main_imu = {imu.last_update_ms, imu.yaw_deg * radians_per_degree};
    }
    if (big_yaw.motor_online && isfinite(big_yaw.joint_rad)) {
        data.flags |= NAV_STATUS_BIG_YAW_VALID;
        data.big_yaw = {big_yaw.rx_ms, big_yaw.joint_rad};
    }
    const bool joints_valid = joints.online && now - joints.updated_at_ms < GIMBAL_IMU_TIMEOUT_MS &&
        isfinite(joints.joint_yaw_rad) && isfinite(joints.joint_pitch_rad) && isfinite(joints.joint_roll_rad);
    if (power.online) {
        data.flags |= NAV_STATUS_REFEREE_ONLINE;
        if (power.power_received && now - power.power_tick < 1000U) {
            data.flags |= NAV_STATUS_POWER_VALID;  // 功率值由 RobotStatus 上报
        }
        if (power.buffer_received && now - power.buffer_tick < 1000U) {
            data.flags |= NAV_STATUS_BUFFER_VALID;
            data.buffer_energy_j = power.buffer_energy_j;
        }
    }

    const auto& cmd = state.robot_cmd;
    const bool motion_valid = state.chassis.valid &&
        now - state.chassis.last_update_ms < board_link::can3::kChassisFeedbackTimeoutMs;
    auto transform_state = joints.transform_state;
    if (Chassis_GetMode() != CHASSIS_MODE_AUTO_VISION || !cmd.chassis_enabled ||
        cmd.last_update_ms == 0U || now - cmd.last_update_ms >= chassis_app::kAutoCommandTimeoutMs) {
        transform_state = GimbalTransformState::Disabled;
    } else if (transform_state != GimbalTransformState::Fault &&
               (!motion_valid || !joints_valid || !big_yaw.ready ||
                (cmd.passage_requested && (!cmd.heading_valid || !imu_valid)))) {
        transform_state = GimbalTransformState::Recovering;
    } else if (transform_state == GimbalTransformState::Ready) {
        // The command may change before the gimbal task publishes its next state.
        const auto requested_shape = cmd.passage_requested ? GimbalShape::Folded : GimbalShape::Upright;
        if (joints.transformed != requested_shape) {
            transform_state = cmd.passage_requested
                ? GimbalTransformState::Preparing : GimbalTransformState::Restoring;
        }
    }
    data.transform_state = static_cast<uint8_t>(transform_state);

    auto* bytes = reinterpret_cast<uint8_t*>(&send_navigation_status);
    append_CRC16_check_sum(bytes, sizeof(send_navigation_status));
    if (USB_Transmit(bytes, sizeof(send_navigation_status)) == USBD_OK) {
        last_send_time.NavigationStatus = now;
    }
}


/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/


bool Usb::AcceptRobotCmd(const ReceiveDataRobotCmd_s& frame)
{
    const auto& nav = frame.data.navigation;
    const auto& speed = frame.data.speed_vector;
    const bool chassis_enabled = (nav.flags & ROBOT_CMD_CHASSIS_ENABLED) != 0U;
    const bool heading_valid = (nav.flags & ROBOT_CMD_HEADING_VALID) != 0U;
    const bool aim_valid = (nav.flags & ROBOT_CMD_AIM_VALID) != 0U;
    constexpr uint8_t allowed_flags =
        ROBOT_CMD_CHASSIS_ENABLED | ROBOT_CMD_HEADING_VALID | ROBOT_CMD_AIM_VALID;
    if (nav.session == 0U || (nav.flags & ~allowed_flags) != 0 ||
        nav.mode > ROBOT_CMD_NAV_UPRIGHT ||
        (chassis_enabled && nav.mode == ROBOT_CMD_NAV_FOLD && !heading_valid) ||
        !isfinite(speed.vx) || !isfinite(speed.vy) || !isfinite(speed.wz) ||
        (heading_valid && (!isfinite(nav.heading) || fabsf(nav.heading) > 3.141593f)) ||
        (aim_valid && (!isfinite(frame.data.gimbal.yaw) || !isfinite(frame.data.gimbal.pitch)))) {
        return false;
    }
    if (nav.session != receive_robot_cmd.data.navigation.session) {
        // A host restart establishes a session with a neutral command.
        if (nav.flags != 0U || nav.mode != ROBOT_CMD_NAV_NORMAL ||
            speed.vx != 0.0f || speed.vy != 0.0f || speed.wz != 0.0f ||
            frame.data.shoot.fire != 0U || frame.data.shoot.fric_on != 0U) {
            return false;
        }
    } else if (static_cast<int32_t>(frame.time_stamp - receive_robot_cmd.time_stamp) <= 0) {
        return false;
    }
    receive_robot_cmd = frame;
    UpdateRobotCmd();
    return true;
}

void Usb::UpdateRobotCmd(void)
{
    LuojiaFox::RobotCmdState cmd {};
    const auto state = LuojiaFox::system_state.read();
    robot_cmd_receive_time = HAL_GetTick();

    robot_cmd.speed_vector.vx = receive_robot_cmd.data.speed_vector.vx;
    robot_cmd.speed_vector.vy = receive_robot_cmd.data.speed_vector.vy;
    robot_cmd.speed_vector.wz = receive_robot_cmd.data.speed_vector.wz;

    robot_cmd.chassis.yaw = receive_robot_cmd.data.chassis.yaw;
    robot_cmd.chassis.pitch = receive_robot_cmd.data.chassis.pitch;
    robot_cmd.chassis.roll = receive_robot_cmd.data.chassis.roll;
    robot_cmd.chassis.leg_length = receive_robot_cmd.data.chassis.leg_lenth;

    const auto& nav = receive_robot_cmd.data.navigation;
    cmd.chassis_enabled = (nav.flags & ROBOT_CMD_CHASSIS_ENABLED) != 0U;
    cmd.heading_valid = (nav.flags & ROBOT_CMD_HEADING_VALID) != 0U;
    cmd.big_yaw_heading_rad = cmd.heading_valid ? nav.heading : 0.0f;
    cmd.passage_requested = state.robot_cmd.passage_requested;
    if (cmd.chassis_enabled && nav.mode == ROBOT_CMD_NAV_FOLD) { cmd.passage_requested = true; }
    if (cmd.chassis_enabled && nav.mode == ROBOT_CMD_NAV_UPRIGHT) { cmd.passage_requested = false; }

    const bool aim_valid = (nav.flags & ROBOT_CMD_AIM_VALID) != 0U;
    const float yaw_target = aim_valid ? receive_robot_cmd.data.gimbal.yaw : 0.0f;
    const float pitch_target = aim_valid ? receive_robot_cmd.data.gimbal.pitch : 0.0f;
    if (aim_valid) {
        // Aim activity follows changes in the received target.
        const bool changed = state.robot_cmd.gimbal_aim_changed_ms == 0U ||
                             yaw_target != robot_cmd.gimbal.yaw ||
                             pitch_target != robot_cmd.gimbal.pitch;
        cmd.gimbal_aim_changed_ms = changed ? robot_cmd_receive_time :
                                             state.robot_cmd.gimbal_aim_changed_ms;
    }
    robot_cmd.gimbal.yaw = yaw_target;
    robot_cmd.gimbal.pitch = pitch_target;

    robot_cmd.shoot.fire = receive_robot_cmd.data.shoot.fire;
    robot_cmd.shoot.fric_on = receive_robot_cmd.data.shoot.fric_on;

    cmd.vx = robot_cmd.speed_vector.vx;
    cmd.vy = robot_cmd.speed_vector.vy;
    cmd.wz = robot_cmd.speed_vector.wz;

    cmd.chassis_roll = robot_cmd.chassis.roll;
    cmd.chassis_pitch = robot_cmd.chassis.pitch;
    cmd.chassis_yaw = robot_cmd.chassis.yaw;
    cmd.chassis_leg_length = robot_cmd.chassis.leg_length;

    // SENTRY RobotCmd carries absolute targets in the gimbal IMU frame.
    cmd.gimbal_yaw = yaw_target;
    cmd.gimbal_pitch = pitch_target;

    cmd.fire = robot_cmd.shoot.fire;
    cmd.fric_on = robot_cmd.shoot.fric_on;
    cmd.last_update_ms = robot_cmd_receive_time;

    LuojiaFox::system_state.updateRobotCmd(cmd);
}

void Usb::UpdateVirtualRc(void)
{
    memcpy(&virtual_rc, &receive_virtual_rc.data, sizeof(RcControl));

    LuojiaFox::system_state.updateVirtualRc({virtual_rc});
}

void Usb::UpdateSentryCmd(void)
{
    sentry_cmd = receive_sentry_cmd.sentry_cmd;
}

uint32_t Usb::GetSentryCmd(void)
{
    return sentry_cmd;
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/



/**
 * @brief 修改调试数据包
 * @param index 索引
 * @param data  发送数据
 * @param name  数据名称
 */
void Usb::ModifyDebugData(uint8_t index, float data, const char * name)
{
    send_debug.packages[index].data = data;

    if (send_debug.packages[index].name[0] != '\0') {
        return;
    }

    uint8_t i = 0;
    while (name[i] != '\0' && i < 10) {
        send_debug.packages[index].name[i] = name[i];
        i++;
    }

    //TODO:添加对数据名称的一些检查工作
}

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
float Usb::GetGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return robot_cmd.gimbal.yaw;
    } else if (axis == AX_PITCH) {
        return robot_cmd.gimbal.pitch;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
#if __SELF_BOARD_ID == C_BOARD_OMNI_SENTRY_ROT_GIMBAL
#else
float Usb::GetChassisSpeed(uint8_t axis)
{
    if (axis == AX_X)
    {
        return robot_cmd.speed_vector.vx;
    } 
    else if (axis == AX_Y) 
    {
        return robot_cmd.speed_vector.vy;
    }
    // 角速度置零
    else if (axis == AX_Z)
    {
        return robot_cmd.speed_vector.wz;
    }
    return 0.0f;
}
#endif
/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
float Usb::GetChassisHeight(void)
{
    return robot_cmd.chassis.leg_length;
}

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
bool Usb::GetFire(void)
{
    return robot_cmd.shoot.fire;
}


/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
bool Usb::GetFricOn(void)
{
    return robot_cmd.shoot.fric_on;
}

/**
 * @brief 获取USB链路是否离线
 * @param void
 * @return true:离线，false:在线
 */
bool Usb::IsOffline(void)
{
    return offline;
}

/**
 * @brief 获取上位机控制指令：底盘姿态角
 * @param axis 轴id，可配合定义好的轴id宏 AX_ROLL,AX_PITCH,AX_YAW 使用
 * @return (rad) 底盘姿态角
 */
float Usb::GetChassisAngle(uint8_t axis)
{
    if (axis == AX_ROLL) {
        return robot_cmd.chassis.roll;
    } else if (axis == AX_PITCH) {
        return robot_cmd.chassis.pitch;
    } else if (axis == AX_YAW) {
        return robot_cmd.chassis.yaw;
    }
    return 0.0f;
}

/**
 * @brief 获取虚拟遥控器通道值
 * @param channel 通道id
 * @return 通道值，范围为 [-1,1]
 */
float Usb::GetVirtualRcCh(uint8_t channel)
{
    return virtual_rc.rc.ch[channel] * RC_TO_ONE;
}

/**
 * @brief 获取虚拟遥控器拨杆值，可配合 switch_is_xxx 系列宏函数使用
 * @param channel 拨杆id
 * @return 拨杆值，范围为 {1,2,3}
 */
char Usb::GetVirtualRcSw(uint8_t channel)
{
    return virtual_rc.rc.s[channel];
}

/*------------------------------ End of File ------------------------------*/
