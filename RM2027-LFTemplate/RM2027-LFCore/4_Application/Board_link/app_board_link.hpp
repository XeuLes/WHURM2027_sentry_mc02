/**
 * @file app_board_link.hpp
 * @brief 双板应用层通信：定义板间 Payload、接收状态和 FreeRTOS 任务接口。
 */

#pragma once

#include <stdint.h>

#include "communication_protocol.hpp"

namespace board_link {

/** Payload 协议版本，两端必须保持一致。 */
constexpr uint8_t kProtocolVersion = 2U;

/** 周期发送本板状态的间隔，单位毫秒。 */
constexpr uint32_t kTxPeriodMs = 10U;

/** 通信任务轮询间隔，单位毫秒。 */
constexpr uint32_t kTaskPeriodMs = 2U;

/**
 * @brief 底盘板发送给云台板的数据。
 * @note 直接在线路上传输，不使用 bool，并保证结构体布局固定。
 */
struct __attribute__((packed)) ChassisToGimbalPayload {
    uint8_t protocol_version;
    uint8_t sender_board_id;
    uint8_t receiver_board_id;
    uint32_t sequence;

    int16_t rc_ch[5];
    uint8_t rc_switch[4];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint16_t keyboard;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t rc_online;

    float vx;
    float vy;
    float wz;

    float imu_yaw_deg;
    float imu_pitch_deg;
    float imu_roll_deg;
    float imu_yaw_rate_dps;
    float imu_pitch_rate_dps;
    float imu_roll_rate_dps;

    uint16_t robot_id;
    uint16_t remaining_hp;
    uint8_t referee_online;
    uint16_t power_limit_w;
    uint16_t buffer_energy_j;
    uint32_t power_age_ms;
    uint32_t buffer_age_ms;
    uint8_t power_received;
    uint8_t buffer_received;

};

/**
 * @brief 云台板发送给底盘板的数据。
 * @note 迷你 PC 离线时，控制量会清零，防止底盘继续使用旧指令。
 */
struct __attribute__((packed)) GimbalToChassisPayload {
    uint8_t protocol_version;
    uint8_t sender_board_id;
    uint8_t receiver_board_id;
    uint32_t sequence;

    float vx;
    float vy;
    float wz;

    uint8_t usb_online;
    float small_yaw_offset_rad;
    // Distinguishes continuous SCAN from threshold-following AUTO_AIM.
    uint8_t gimbal_mode;

};

static_assert(
    sizeof(float) == 4U,
    "board link payload requires 32-bit IEEE-754 float"
);
static_assert(
    sizeof(ChassisToGimbalPayload) <= communication_protocol::kMaxPayloadSize,
    "ChassisToGimbalPayload exceeds protocol payload limit"
);
static_assert(
    sizeof(GimbalToChassisPayload) <= communication_protocol::kMaxPayloadSize,
    "GimbalToChassisPayload exceeds protocol payload limit"
);

/** 板间通信统计，用于上板确认收发是否正常。 */
struct Statistics {
    uint32_t tx_attempts;
    uint32_t tx_failures;
    uint32_t valid_rx_frames;
    uint32_t invalid_rx_frames;
    uint32_t sequence_gaps;
};

/**
 * @brief 供调试器实时观察的本板发送快照。
 * @note 每次发送前更新 Payload 镜像，发送后更新成功和失败计数。
 */
struct TxDebugState {
    uint32_t tx_attempts;
    uint32_t tx_successes;
    uint32_t tx_failures;
    uint32_t last_tx_ms;
    uint32_t last_sequence;
    uint32_t last_payload_length;
    ChassisToGimbalPayload last_chassis_payload;
    GimbalToChassisPayload last_gimbal_payload;
};

/** 当前板的发送调试镜像，可在 EmberProbe Live Watch 中直接展开。 */
extern TxDebugState g_tx_debug;

/**
 * @brief 复制最近一次收到的底盘遥测。
 * @param output 输出数据，不能为空。
 * @return true 表示至少收到过一帧有效底盘数据。
 */
bool getRemoteChassisData(ChassisToGimbalPayload* output);

/**
 * @brief 复制最近一次收到的云台命令。
 * @param output 输出数据，不能为空。
 * @return true 表示至少收到过一帧有效云台数据。
 */
bool getRemoteGimbalData(GimbalToChassisPayload* output);

/**
 * @brief 判断对端是否在配置的超时时间内发送过有效 Payload。
 * @param now_ms 当前毫秒时间。
 * @return true 表示对端在线。
 */
bool isPeerOnline(uint32_t now_ms);


/**
 * @brief 获取板间通信统计快照。
 * @return 当前统计值。
 */
Statistics getStatistics();

} // namespace board_link

/**
 * @brief 板间通信任务。
 * @param argument 未使用。
 * @note 由 Core/Src/freertos.c 创建，负责初始化、接收和周期发送。
 */
extern "C" void BoardLink_Task(void const* argument);
