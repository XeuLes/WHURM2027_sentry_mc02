/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.cpp/hpp
  * @brief      Updated the referee system Communication Protocol 1.6.1
  * @note       裁判系统通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026/3/6         YZX             1.更新裁判系统通信协议至1.6.1（不包含半自动步兵和UI）
  *  v2.0.0     2026/10/15      XeuLes           1.将原有的裁判系统通信协议解析代码封装为Referee类，方便在C++中使用
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "referee.hpp"

#include "CRC8_CRC16.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "string.h"

Referee referee;

void Referee::Init(void)
{
    memset(&receive_header, 0, sizeof(frame_header_struct_t));
    memset(&send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

    memset(&field_event, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));

    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&robot_hurt_t, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));

    memset(&RFID_STATUS, 0, sizeof(rfid_status_t));
    memset(&ground_robot_pos_t, 0, sizeof(ground_robot_position_t));
    memset(&sentry_decision_info_t, 0, sizeof(sentry_info_t));

    memset(&student_interactive_data_t, 0, sizeof(robot_interaction_data_t));
    memset(&CUSTOM_CONTROLLER_DATA, 0, sizeof(CustomControllerData_t));
    memset(&MAP_COMMAND_DATA, 0, sizeof(MapCommandData_t));

    /* 解包状态机也必须复位，否则 unpack_step/index 是随机值 */
    memset(&unpack_obj, 0, sizeof(unpack_data_t));

    online_time = 0;
    receive_count = 0;
    power_tick_ = buffer_tick_ = 0;
    power_received_ = buffer_received_ = false;
    last_hp_ = 0;
    attacked_time_ = 0;
}

void Referee::DataSolve(uint8_t *frame)
{
    if (HAL_GetTick() - online_time > kTimeout) {
        receive_count = 0;
    }
    receive_count++;

    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATE_CMD_ID: {
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            online_time = HAL_GetTick();
        } break;
        case GAME_RESULT_CMD_ID: {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            online_time = HAL_GetTick();
        } break;
        case GAME_ROBOT_HP_CMD_ID: {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            online_time = HAL_GetTick();
        } break;

        case FIELD_EVENTS_CMD_ID: {
            memcpy(&field_event, frame + index, sizeof(event_data_t));
            online_time = HAL_GetTick();
        } break;
        case REFEREE_WARNING_CMD_ID: {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
            online_time = HAL_GetTick();
        } break;

        case ROBOT_STATE_CMD_ID: {
            if (receive_header.data_length != sizeof(robot_status_t)) {
                --receive_count;
                break;
            }
            const uint32_t mask = __get_PRIMASK();
            __disable_irq();
            __DMB();
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            online_time = HAL_GetTick();
            power_tick_ = online_time;
            power_received_ = true;
            __DMB();
            __set_PRIMASK(mask);

            uint16_t hp = robot_status.current_HP;
            /* 开局第一次收到时 last_hp_ 还是 0，不能把"从 0 涨到满血"当成掉血 */
            if (last_hp_ != 0 && hp < last_hp_) {
                attacked_time_ = online_time;
            }
            last_hp_ = hp;
        } break;
        case POWER_HEAT_DATA_CMD_ID: {
            if (receive_header.data_length != sizeof(power_heat_data_t)) {
                --receive_count;
                break;
            }
            const uint32_t mask = __get_PRIMASK();
            __disable_irq();
            __DMB();
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            online_time = HAL_GetTick();
            buffer_tick_ = online_time;
            buffer_received_ = true;
            __DMB();
            __set_PRIMASK(mask);
        } break;
        case ROBOT_POS_CMD_ID: {
            memcpy(&game_robot_pos_t, frame + index, sizeof(robot_pos_t));
            online_time = HAL_GetTick();
        } break;
        case BUFF_MUSK_CMD_ID: {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_t));
            online_time = HAL_GetTick();
        } break;
        case ROBOT_HURT_CMD_ID: {
            memcpy(&robot_hurt_t, frame + index, sizeof(hurt_data_t));
            online_time = HAL_GetTick();
        } break;
        case SHOOT_DATA_CMD_ID: {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            online_time = HAL_GetTick();
        } break;
        case PROJECTILE_ALLOWANCE_CMD_ID: {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            online_time = HAL_GetTick();
        } break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID: {
            memcpy(&student_interactive_data_t, frame + index, sizeof(robot_interaction_data_t));
            online_time = HAL_GetTick();
        } break;
        case CUSTOM_CONTROLLER_CMD_ID: {
            memcpy(&CUSTOM_CONTROLLER_DATA, frame + index, sizeof(CustomControllerData_t));
            online_time = HAL_GetTick();
        } break;
        case MAP_COMMAND_CMD_ID: {
            memcpy(&MAP_COMMAND_DATA, frame + index, sizeof(MapCommandData_t));
            online_time = HAL_GetTick();
        } break;
        case RFID_STATUS_ID: {
            memcpy(&RFID_STATUS, frame + index, sizeof(rfid_status_t));
            online_time = HAL_GetTick();
        } break;
        case GROUND_ROBOT_POS_ID: {
            /* 目标是 40 字节的 ground_robot_position_t，
               旧代码这里写成了 sizeof(robot_pos_t)（12 字节），后 7 个 float 永远收不到 */
            memcpy(&ground_robot_pos_t, frame + index, sizeof(ground_robot_position_t));
            online_time = HAL_GetTick();
        } break;
        case SENTRY_DECISION_INFO_ID: {
            memcpy(&sentry_decision_info_t, frame + index, sizeof(sentry_info_t));
            online_time = HAL_GetTick();
        } break;
        default: {
            receive_count--;
            break;
        }
    }
}

void Referee::FeedBytes(const char *data, uint16_t len)
{
    fifo_s_puts(&fifo, (char *)data, len);
}

void Referee::UnpackFifoData(void)
{
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &unpack_obj;

    while (fifo_s_used(&fifo)) {
        byte = fifo_s_get(&fifo);
        switch (p_obj->unpack_step) {
            case RSTEP_HEADER_SOF: {
                if (byte == sof) {
                    p_obj->unpack_step = RSTEP_LENGTH_LOW;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else {
                    p_obj->index = 0;
                }
            } break;

            case RSTEP_LENGTH_LOW: {
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = RSTEP_LENGTH_HIGH;
            } break;

            case RSTEP_LENGTH_HIGH: {
                p_obj->data_len |= (byte << 8);
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
                    p_obj->unpack_step = RSTEP_FRAME_SEQ;
                } else {
                    p_obj->unpack_step = RSTEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            } break;
            case RSTEP_FRAME_SEQ: {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = RSTEP_HEADER_CRC8;
            } break;

            case RSTEP_HEADER_CRC8: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                        p_obj->unpack_step = RSTEP_DATA_CRC16;
                    } else {
                        p_obj->unpack_step = RSTEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            } break;

            case RSTEP_DATA_CRC16: {
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->unpack_step = RSTEP_HEADER_SOF;
                    p_obj->index = 0;

                    if (verify_CRC16_check_sum(
                            p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                        DataSolve(p_obj->protocol_packet);
                    }
                }
            } break;

            default: {
                p_obj->unpack_step = RSTEP_HEADER_SOF;
                p_obj->index = 0;
            } break;
        }
    }
}

float Referee::GetBuffer(void) { return power_heat_data.buffer_energy; }

RefereePowerSnapshot Referee::GetPowerSupplySnapshot()
{
    // 裁判解析在任务上下文运行；短临界区阻止任务切换，避免两帧数据混读。
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    const RefereePowerSnapshot result = {
        robot_status.chassis_power_limit, power_heat_data.buffer_energy,
        power_tick_, buffer_tick_, power_received_, buffer_received_, !IsOffline(),
        robot_status.shooter_barrel_cooling_value, robot_status.shooter_barrel_heat_limit,
        power_heat_data.shooter_17mm_barrel_heat
    };
    __DMB();
    __set_PRIMASK(mask);
    return result;
}

void Referee::GetChassisPowerLimit(uint16_t *power_limit) { *power_limit = robot_status.chassis_power_limit; }

uint8_t Referee::GetRobotId(void) { return robot_status.robot_id; }

uint16_t Referee::GetShootHeat(void)
{
    return power_heat_data.shooter_17mm_barrel_heat;
}

void Referee::GetShootHeat17LimitAndHeat17(uint16_t *cooling_value, uint16_t *heat_limit, uint16_t *heat)
{
    *cooling_value = robot_status.shooter_barrel_cooling_value;
    *heat_limit = robot_status.shooter_barrel_heat_limit;
    *heat = power_heat_data.shooter_17mm_barrel_heat;
}

void Referee::GetShootSpeedFre(uint8_t *shoot_fre, float *shoot_speed)
{
    *shoot_fre = shoot_data.launching_frequency;
    *shoot_speed = shoot_data.initial_speed;
}

/**
 * @brief 反馈机器人颜色
 * @param  none
 * @return 0为红色,1为蓝色,2为未知
 */
uint8_t Referee::GetTeamColor(void)  // 谨防“哨兵在打我”
{
    uint8_t id = robot_status.robot_id;
    if (id >= 1 && id <= 11) {
        return 0;  //红方
    }
    if (id >= 101 && id <= 111) {
        return 1;  //蓝方
    }
    return 2;  //未知
}

CustomControllerData_t *Referee::GetCustomControllerDataPoint(void) { return &CUSTOM_CONTROLLER_DATA; }

void Referee::GetMapCommandData(float *x, float *y, uint8_t *cmd_keyboard, uint8_t *target_robot_id, uint16_t *cmd_source)
{
    *x = MAP_COMMAND_DATA.target_position_x;
    *y = MAP_COMMAND_DATA.target_position_y;
    *cmd_keyboard = MAP_COMMAND_DATA.cmd_keyboard;
    *target_robot_id = MAP_COMMAND_DATA.target_robot_id;
    *cmd_source = MAP_COMMAND_DATA.cmd_source;
}

void Referee::GetSentryInfo(uint32_t *sentry_info, uint16_t *sentry_info_2, uint64_t *sentry_info_3)
{
    *sentry_info = sentry_decision_info_t.sentry_info;
    *sentry_info_2 = sentry_decision_info_t.sentry_info_2;
    *sentry_info_3 = sentry_decision_info_t.sentry_info_3;
}

uint16_t Referee::GetCurrentHP(void)
{
    return robot_status.current_HP;
}

/*========== API ==========*/

bool Referee::IsOffline(void)
{
    uint32_t current_time = HAL_GetTick();
    return !((receive_count > 5) && (current_time - online_time < kTimeout));
}

bool Referee::IsAttacked(void)
{
    if (attacked_time_ == 0) {
        return false;
    }
    return (HAL_GetTick() - attacked_time_) < kAttackedHoldMs;
}

/**
 * @brief 获取自定义控制器数据
 * @param  index 数据索引
 * @return float 数据
 */
float Referee::GetCustomControllerPos(uint8_t index)
{
    float data = 0;
    memcpy(&data, &CUSTOM_CONTROLLER_DATA.data[index * 4], 4);
    return data;
    // return *((float *)(&CUSTOM_CONTROLLER_DATA.data[index * 4]));
}

/*------------------------------ End of File ------------------------------*/
