#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include <stdint.h>

#include "attribute_typedef.h"

// LFSentryCommon27 Gimbal/NUC USB CDC. All multibyte fields are little-endian.
enum {
    USB_SYNC_REQUEST_SOF = 0xA0,
    USB_ROBO_CMD_SOF = 0xA1,
    USB_SHOOT_CMD_SOF = 0xA2,
    USB_RFR_CMD_SOF = 0xA3,
    USB_GLOBAL_PATH_SOF = 0xA4,
    USB_DECISION_GIMBAL_CMD_SOF = 0xA5,
    USB_SYNC_RESPONSE_SOF = 0xB0,
    USB_RAW_GIMBAL_IMU_SOF = 0xB1,
    USB_GIMBAL_STATUS_SOF = 0xB2,
    USB_NAV_STATUS_SOF = 0xB3,
    USB_BIG_YAW_HEADING_SOF = 0xB4,
    USB_COMP_STAGE_SOF = 0xC0,
    USB_COMP_EVENT_SOF = 0xC1,
    USB_ALLIED_HEALTH_SOF = 0xC2,
    USB_ALLIED_POS_SOF = 0xC3,
    USB_ROBOT_STATUS_SOF = 0xC4,
    USB_ROBOT_POS_SOF = 0xC5,
    USB_RFID_STATUS_SOF = 0xC6,
    USB_HEALTH_REDUCTION_SOF = 0xC7,
    USB_SENTRY_INFO_SOF = 0xC8,
    USB_ENEMY_POS_SOF = 0xC9,
    USB_OPERATOR_CMD_SOF = 0xCA,
    USB_ENEMY_HEALTH_SOF = 0xCB,
    USB_ENEMY_BUFF_SOF = 0xCC,
};

enum {
    USB_ROBO_MODE_NORMAL = 0,
    USB_ROBO_MODE_SPIN_SLOW = 1,
    USB_ROBO_MODE_SPIN_FAST = 2,
    USB_ROBO_MODE_FOLD = 3,
    USB_ROBO_MODE_RESTORE = 4,
};

enum {
    USB_SHOOT_MODE_NONE = 0,
    USB_SHOOT_MODE_SINGLE = 1,
    USB_SHOOT_MODE_BURST = 2,
};

typedef struct {
    uint8_t sof;  // 帧类型
    uint8_t len;  // 包含帧头和 CRC8 的总长
} __packed__ FrameHeader_t;

// NUC -> Gimbal
typedef struct {
    FrameHeader_t frame_header;
    uint32_t sequence;
    uint8_t checksum;
} __packed__ ReceiveSyncRequest_s;

// 底盘运动与过洞请求。
typedef struct {
    FrameHeader_t frame_header;
    float velocity;  // 底盘前向速度，m/s
    float omega;     // 底盘绕 z 轴角速度，rad/s
    uint8_t mode;
    uint8_t hole_dist;  // cm，仅 mode 3/4 有效
    float lateral_velocity;  // 底盘横向速度，左为正，m/s
    uint8_t heading_valid;
    float big_yaw_heading;  // 主板 BMI088 惯性系，rad
    uint8_t checksum;
} __packed__ ReceiveRoboCmd_s;

// 自瞄目标与发弹请求，角度为电控世界系下的期望云台欧拉角。
typedef struct {
    FrameHeader_t frame_header;
    float yaw;    // -pi~pi，rad
    float pitch;  // -pi/2~pi/2，rad
    uint8_t shoot_mode;  // 0 不发弹，1 单发，2 连发
    uint8_t checksum;
} __packed__ ReceiveShootCmd_s;

typedef struct {
    FrameHeader_t frame_header;
    uint8_t stance;
    uint8_t checksum;
} __packed__ ReceiveRfrCmd_s;

typedef struct {
    FrameHeader_t frame_header;
    uint8_t reserved;
    int16_t start_x, start_y;   // dm
    int8_t delta_x[28], delta_y[28];
    uint8_t checksum;
} __packed__ ReceiveGlobalPath_s;

// 决策云台目标。当前只校验长度和 CRC8，不参与控制。
typedef struct {
    FrameHeader_t frame_header;
    float yaw;    // 电控世界系，rad
    float pitch;  // 电控世界系，rad
    uint8_t checksum;
} __packed__ ReceiveDecisionGimbalCmd_s;

// Gimbal -> NUC
typedef struct {
    FrameHeader_t frame_header;
    uint32_t sequence;
    uint32_t t2_ticks;  // 通过 CRC 校验的本地时刻，100 us
    uint32_t t3_ticks;  // 提交 CDC 发送的本地时刻，100 us
    uint8_t checksum;
} __packed__ SendSyncResponse_s;

typedef struct {
    FrameHeader_t frame_header;
    uint32_t sample_ticks;
    float accel_x, accel_y, accel_z;  // IMU 机体系，m/s²
    float gyro_x, gyro_y, gyro_z;     // IMU 机体系，rad/s
    uint8_t checksum;
} __packed__ SendRawGimbalImu_s;

typedef struct {
    FrameHeader_t frame_header;
    uint32_t sample_ticks;
    float x, y, z, w;  // 四元数，电控世界系
    float pitch;
    float yaw;
    float pitch_ref;
    float yaw_ref;
    float bullet_speed;
    float big_yaw;
    float roll;
    uint8_t transformed;  // 已完成的形态
    uint8_t checksum;
} __packed__ SendGimbalStatus_s;

typedef struct {
    FrameHeader_t frame_header;
    float vx, vy, omega;  // 底盘实测速度，m/s、m/s、rad/s
    uint8_t transform_state;  // 执行状态
    int16_t remaining_energy_supercap;
    int16_t remaining_energy_buffercap;
    int16_t curr_chassis_pwr;
    int16_t rfr_pwr_limit;
    uint8_t checksum;
} __packed__ SendNavStatus_s;

typedef struct {
    FrameHeader_t frame_header;
    uint32_t sample_ticks;  // 主板 IMU 样本时间，100 us
    float yaw;  // 主板 BMI088 惯性系，rad
    uint8_t checksum;
} __packed__ SendBigYawHeading_s;

typedef struct {
    FrameHeader_t frame_header;
    uint8_t game_progress;
    uint16_t stage_remain_time;
    uint8_t checksum;
} __packed__ SendCompStage_s;

typedef struct {
    FrameHeader_t frame_header;
    uint32_t event_data;
    uint8_t checksum;
} __packed__ SendCompEvent_s;

typedef struct {
    FrameHeader_t frame_header;
    uint16_t ally_1_robot_hp, ally_2_robot_hp, ally_3_robot_hp, ally_4_robot_hp;
    uint16_t ally_outpost_hp, ally_base_hp, enemy_outpost_hp, enemy_base_hp;
    uint8_t checksum;
} __packed__ SendAlliedHealth_s;

typedef struct {
    FrameHeader_t frame_header;
    float hero_x, hero_y, engineer_x, engineer_y;
    float standard_3_x, standard_3_y, standard_4_x, standard_4_y;  // m
    uint8_t checksum;
} __packed__ SendAlliedPos_s;

typedef struct {
    FrameHeader_t frame_header;
    uint8_t robot_id;
    uint16_t current_hp, maximum_hp;
    uint16_t projectile_allowance_17mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
    uint8_t checksum;
} __packed__ SendRobotStatus_s;

typedef struct {
    FrameHeader_t frame_header;
    float self_x, self_y;  // m
    uint8_t checksum;
} __packed__ SendRobotPos_s;

typedef struct {
    FrameHeader_t frame_header;
    uint32_t rfid_status;
    uint8_t rfid_status_2;
    uint8_t checksum;
} __packed__ SendRfidStatus_s;

typedef struct {
    FrameHeader_t frame_header;
    uint8_t armor_and_reason;  // 低 4 位为装甲 ID，高 4 位为扣血原因
    uint8_t checksum;
} __packed__ SendHealthReduction_s;

typedef struct {
    FrameHeader_t frame_header;
    uint32_t sentry_info;
    uint16_t sentry_info_2;
    uint64_t sentry_info_3;
    uint8_t checksum;
} __packed__ SendSentryInfo_s;

typedef struct {
    FrameHeader_t frame_header;
    float target_position_x, target_position_y;  // 裁判小地图坐标，m
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t checksum;
} __packed__ SendOperatorCmd_s;

// 雷达站数据仅保留协议帧号和线长，当前暂不收发。
typedef struct {
    FrameHeader_t frame_header;
    uint8_t payload[40];
    uint8_t checksum;
} __packed__ SendEnemyPos_s;

typedef struct {
    FrameHeader_t frame_header;
    uint8_t payload[10];
    uint8_t checksum;
} __packed__ SendEnemyHealth_s;

typedef struct {
    FrameHeader_t frame_header;
    uint8_t payload[15];
    uint8_t checksum;
} __packed__ SendEnemyBuff_s;

#ifdef __cplusplus
static_assert(sizeof(FrameHeader_t) == 2, "USB header wire size");
static_assert(sizeof(ReceiveSyncRequest_s) == 7, "SyncRequest wire size");
static_assert(sizeof(ReceiveRoboCmd_s) == 22, "RoboCmd wire size");
static_assert(sizeof(ReceiveShootCmd_s) == 12, "ShootCmd wire size");
static_assert(sizeof(ReceiveRfrCmd_s) == 4, "RfrCmd wire size");
static_assert(sizeof(ReceiveGlobalPath_s) == 64, "GlobalPath wire size");
static_assert(sizeof(ReceiveDecisionGimbalCmd_s) == 11, "DecisionGimbalCmd wire size");
static_assert(sizeof(SendSyncResponse_s) == 15, "SyncResponse wire size");
static_assert(sizeof(SendRawGimbalImu_s) == 31, "RawGimbalIMU wire size");
static_assert(sizeof(SendGimbalStatus_s) == 52, "GimbalStatus wire size");
static_assert(sizeof(SendNavStatus_s) == 24, "NavStatus wire size");
static_assert(sizeof(SendBigYawHeading_s) == 11, "BigYawHeading wire size");
static_assert(sizeof(SendCompStage_s) == 6, "CompStage wire size");
static_assert(sizeof(SendCompEvent_s) == 7, "CompEvent wire size");
static_assert(sizeof(SendAlliedHealth_s) == 19, "AlliedHealth wire size");
static_assert(sizeof(SendAlliedPos_s) == 35, "AlliedPos wire size");
static_assert(sizeof(SendRobotStatus_s) == 14, "RobotStatus wire size");
static_assert(sizeof(SendRobotPos_s) == 11, "RobotPos wire size");
static_assert(sizeof(SendRfidStatus_s) == 8, "RfidStatus wire size");
static_assert(sizeof(SendHealthReduction_s) == 4, "HealthReduction wire size");
static_assert(sizeof(SendSentryInfo_s) == 17, "SentryInfo wire size");
static_assert(sizeof(SendEnemyPos_s) == 43, "EnemyPos wire size");
static_assert(sizeof(SendOperatorCmd_s) == 13, "OperatorCommand wire size");
static_assert(sizeof(SendEnemyHealth_s) == 13, "EnemyHealth wire size");
static_assert(sizeof(SendEnemyBuff_s) == 18, "EnemyBuff wire size");
#endif

#endif  // USB_TYPEDEF_H
