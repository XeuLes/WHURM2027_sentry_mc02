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

// clang-format on

#define USB_RECEIVE_LEN 150   // byte
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

}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
void Usb::SendAll(void)
{
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
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = rx_buf;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = rx_buf;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        const auto frame_size = HEADER_SIZE + data_len + sizeof(uint16_t);
                        const bool legacy = frame_size ==
                            sizeof(receive_robot_cmd) - sizeof(receive_robot_cmd.gimbal_cmd);
                        if (!legacy && frame_size != sizeof(receive_robot_cmd)) { break; }

                        uint32_t command_timestamp;
                        memcpy(&command_timestamp, &sof_address[4], sizeof(command_timestamp));
                        if (has_robot_cmd_timestamp &&
                            static_cast<int32_t>(command_timestamp - latest_robot_cmd_timestamp) <= 0) {
                            break;
                        }

                        memcpy(&receive_robot_cmd, sof_address, frame_size - sizeof(uint16_t));
                        if (legacy) {
                            receive_robot_cmd.gimbal_cmd = 0U;
                        } else if (receive_robot_cmd.gimbal_cmd > 1U) {
                            break;
                        }
                        latest_robot_cmd_timestamp = command_timestamp;
                        has_robot_cmd_timestamp = true;
                        UpdateRobotCmd();
                        receive_time = robot_cmd_receive_time;
                    } break;
                    case PID_DEBUG_DATA_RECEIVE_ID: {
                        memcpy(&receive_pid_debug, sof_address, sizeof(ReceiveDataPidDebug_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(
                            &receive_virtual_rc, sof_address, sizeof(ReceiveDataVirtualRc_s));
                    } break;
                    case SENTRY_CMD_RECEIVE_ID: {
                        memcpy(
                            &receive_sentry_cmd, sof_address, sizeof(ReceiveDataSentryCmd_s));
                    } break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > latest_rx_timestamp) {
                    latest_rx_timestamp = *((uint32_t *)(&sof_address[4]));
                    receive_time = HAL_GetTick();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = rx_buf;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = rx_buf + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(rx_buf, sof_address, remaining_data_len);
    }
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
    if (!state.online ||
        now - state.updated_at_ms >= 20U) { return; }
    send_joint_state.time_stamp = now;
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


/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

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

    const float yaw_target = receive_robot_cmd.data.gimbal.yaw;
    const float pitch_target = receive_robot_cmd.data.gimbal.pitch;
    const bool aim_finite = isfinite(yaw_target) && isfinite(pitch_target);
    cmd.passage_requested = receive_robot_cmd.gimbal_cmd == 1U;
    if (aim_finite) {
        // Aim activity follows changes in the received target.
        const bool changed = yaw_target != robot_cmd.gimbal.yaw ||
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
