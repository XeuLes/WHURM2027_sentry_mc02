#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

#define DEBUG_PACKAGE_NUM 10

#define DATA_DOMAIN_OFFSET 0x08

// clang-format off
#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define DEBUG_DATA_SEND_ID        ((uint8_t)0x01)
#define IMU_DATA_SEND_ID          ((uint8_t)0x02)
#define ROBOT_STATE_INFO_DATA_SEND_ID   ((uint8_t)0x03)
#define EVENT_DATA_SEND_ID        ((uint8_t)0x04)
#define PID_DEBUG_DATA_SEND_ID    ((uint8_t)0x05)
#define ALL_ROBOT_HP_SEND_ID      ((uint8_t)0x06)
#define GAME_STATUS_SEND_ID       ((uint8_t)0x07) 
#define ROBOT_MOTION_DATA_SEND_ID ((uint8_t)0x08)
#define GROUND_ROBOT_POSITION_SEND_ID ((uint8_t)0x09)
#define RFID_STATUS_SEND_ID       ((uint8_t)0x0A)
#define ROBOT_STATUS_SEND_ID      ((uint8_t)0x0B)
#define JOINT_STATE_SEND_ID       ((uint8_t)0x0C)
#define BUFF_SEND_ID              ((uint8_t)0x0D)
#define SENTRYINFO_SEND_ID        ((uint8_t)0x0E)
#define MAP_COMMAND_SEND_ID       ((uint8_t)0x0F)
#define INIT_SPEED_SEND_ID        ((uint8_t)0x10)
#define NAVIGATION_STATUS_SEND_ID ((uint8_t)0x21)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define PID_DEBUG_DATA_RECEIVE_ID  ((uint8_t)0x02)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
#define SENTRY_CMD_RECEIVE_ID      ((uint8_t)0x04)
// clang-format on

//后续迁移到其他文件中整体定义
#ifndef CHASSIS_OMNI_WHEEL
#define CHASSIS_NONE           0  // 无底盘
#define CHASSIS_MECANUM_WHEEL  1  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL     2  // 全向轮底盘
#define CHASSIS_STEERING_WHEEL 3  // 舵轮底盘
#define CHASSIS_BALANCE        4  // 平衡底盘

#define GIMBAL_NONE                 0  // 无云台
#define GIMBAL_YAW_PITCH_DIRECT     1  // yaw-pitch 电机直连云台
#define GIMBAL_BIG_YAW              2  // 大 yaw 环云台

#define SHOOT_NONE              0  // 无发射机构
#define SHOOT_FRIC_TRIGGER      1  // 摩擦轮 + 拨弹盘
#define SHOOT_PNEUMATIC_TRIGGER 2  // 气动 + 拨弹盘

#define MECHANICAL_ARM_NONE             0  // 无机械臂
#define MECHANICAL_ARM_PENGUIN_MINI_ARM 1  // 企鹅 mini 机械臂
#define MECHANICAL_ARM_ENGINEER_ARM     2  // 工程机械臂
#endif

/* 本车实际构型，改这里即可 */
#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL
#endif
#ifndef GIMBAL_TYPE
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT
#endif
#ifndef SHOOT_TYPE
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER
#endif
#ifndef MECHANICAL_ARM_TYPE
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE
#endif

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/

// 串口调试数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        uint8_t name[10];
        uint8_t type;
        float data;
    } __packed__ packages[DEBUG_PACKAGE_NUM];
    uint16_t checksum;
} __packed__ SendDataDebug_s;

// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float yaw;    // 度
        float pitch;  // 度
        float roll;   // 度

        float yaw_vel;    // 度/秒
        float pitch_vel;  // 度/秒
        float roll_vel;   // 度/秒

        // float x_accel;  // m/s^2
        // float y_accel;  // m/s^2
        // float z_accel;  // m/s^2
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

// 机器人信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    struct
    {
        /// @brief 机器人部位类型 2 bytes
        struct
        {
            uint16_t chassis : 3;
            uint16_t gimbal : 3;
            uint16_t shoot : 3;
            uint16_t arm : 3;
            uint16_t custom_controller : 3;
            uint16_t reserve : 1;
        } __packed__ type;
        /// @brief 机器人部位状态 1 byte
        /// @note 0: 正常，1: 错误
        struct
        {
            uint8_t chassis : 1;
            uint8_t gimbal : 1;
            uint8_t shoot : 1;
            uint8_t arm : 1;
            uint8_t custom_controller : 1;
            uint8_t reserve : 3;
        } __packed__ state;
        /// @brief 机器人裁判系统信息 7 bytes
        struct
        {
            uint8_t id;
            uint8_t color;  // 0-red 1-blue 2-unknown
            bool attacked;
            uint16_t hp;
            uint16_t heat;
        } __packed__ referee;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotStateInfo_s;

// 事件数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x04
    uint32_t time_stamp;

    struct
    {
        /* 场地事件原始值，位定义与裁判系统 0x0101 一致，
           见《RoboMaster 2026 通信协议 V2.0.0》表 1-9。
           上位机自行按位拆解，避免板子侧再做一次位域映射导致语义漂移。 */
        uint32_t event_data;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataEvent_s;

// PID调参数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x05
    uint32_t time_stamp;
    struct
    {
        float fdb;
        float ref;
        float pid_out;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataPidDebug_s;

// 全场机器人hp信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x06
    uint32_t time_stamp;
    struct
    {
        uint16_t ally_1_robot_HP;
        uint16_t ally_2_robot_HP;
        uint16_t ally_3_robot_HP;
        uint16_t ally_4_robot_HP;
        int16_t damage_difference;  // 己方全队总伤害与对方全队总伤害之差
        uint16_t ally_7_robot_HP;
        uint16_t ally_outpost_HP;
        uint16_t ally_base_HP;
        uint16_t enemy_outpost_HP;  // 对方前哨站血量
        uint16_t enemy_base_HP;     // 对方基地血量
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataAllRobotHp_s;

// 比赛信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x07
    uint32_t time_stamp;
    struct
    {
        uint8_t game_progress;
        uint16_t stage_remain_time;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGameStatus_s;

// 地面机器人位置数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x09
    uint32_t time_stamp;
    
    struct
    {
        float hero_x;
        float hero_y;

        float engineer_x;
        float engineer_y;

        float standard_3_x;
        float standard_3_y;

        float standard_4_x;
        float standard_4_y;

        float reserved1;  // 裁判系统 0x020B 的保留位
        float reserved2;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGroundRobotPosition_s;

// 机器人运动数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotMotion_s;

// RFID状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0A
    uint32_t time_stamp;

    //   struct
    // {
    //     bool base_gain_point;                     // 己方基地增益点
    //     bool circular_highland_gain_point;        // 己方环形高地增益点
    //     bool enemy_circular_highland_gain_point;  // 对方环形高地增益点
    //     bool friendly_r3_b3_gain_point;           // 己方 R3/B3 梯形高地增益点
    //     bool enemy_r3_b3_gain_point;              // 对方 R3/B3 梯形高地增益点
    //     bool friendly_r4_b4_gain_point;           // 己方 R4/B4 梯形高地增益点
    //     bool enemy_r4_b4_gain_point;              // 对方 R4/B4 梯形高地增益点
    //     bool energy_mechanism_gain_point;         // 己方能量机关激活点
    //     bool friendly_fly_ramp_front_gain_point;  // 己方飞坡增益点（靠近己方一侧飞坡前）
    //     bool friendly_fly_ramp_back_gain_point;  // 己方飞坡增益点（靠近己方一侧飞坡后）
    //     bool enemy_fly_ramp_front_gain_point;  // 对方飞坡增益点（靠近对方一侧飞坡前）
    //     bool enemy_fly_ramp_back_gain_point;   // 对方飞坡增益点（靠近对方一侧飞坡后）
    //     bool friendly_outpost_gain_point;      // 己方前哨站增益点
    //     bool friendly_healing_point;           // 己方补血点（检测到任一均视为激活）
    //     bool friendly_sentry_patrol_area;      // 己方哨兵巡逻区
    //     bool enemy_sentry_patrol_area;         // 对方哨兵巡逻区
    //     bool friendly_big_resource_island;     // 己方大资源岛增益点
    //     bool enemy_big_resource_island;        // 对方大资源岛增益点
    //     bool friendly_exchange_area;           // 己方兑换区
    //     bool center_gain_point;  // 中心增益点 RFID 卡状态（仅 RMUL 适用），1 为已检测到
    // } __attribute__((packed)) data;
    struct
    {
        uint32_t rfid_status;
        uint8_t rfid_status_2;
    } __attribute__((packed)) data;
    uint16_t crc;            
} __packed__ SendDataRfidStatus_s;

// 机器人状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0B
    uint32_t time_stamp;

    struct
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_hp;
        uint16_t maximum_hp;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        float bullet_speed_limit;  // 射击初速度上限

        uint16_t shooter_17mm_barrel_heat;
        uint16_t shooter_42mm_barrel_heat;

        float robot_pos_x;
        float robot_pos_y;
        float robot_pos_angle;

        uint8_t armor_id : 4;
        uint8_t hp_deduction_reason : 4;

        uint16_t projectile_allowance_17mm;
        uint16_t projectile_allowance_42mm;
        uint16_t remaining_gold_coin;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotStatus_s;

// 云台状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0C
    uint32_t time_stamp;
    struct
    {
        float pitch;
        float yaw;

    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataJointState_s;

// 机器人增益和底盘能量数据包
typedef struct 
{
    FrameHeader_t frame_header;
    uint32_t time_stamp;

    struct
    {
        uint8_t recovery_buff;   // 回血增益，百分比
        uint16_t cooling_buff;   // 射击热量冷却增益具体值
        uint8_t defence_buff;    // 防御增益，百分比
        uint8_t vulnerability_buff;  // 负防御增益，百分比
        uint16_t attack_buff;    // 攻击增益，百分比
        uint8_t remaining_energy;    // 剩余能量值反馈，各位含义见协议表 1-15
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataBuff_s;

// 哨兵信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0E
    uint32_t time_stamp;

    struct
    {
        uint32_t sentry_info;    // bit 0-31，兑换次数 / 复活 / 立即复活金币数
        uint16_t sentry_info_2;  // bit 0-15，脱战状态 / 可兑换数 / 当前姿态 / 能量机关 / 强化姿态
        uint64_t sentry_info_3;  // bit 0-63，六种姿态弱化前剩余可持续时长，每 8 bit 一个
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataSentryInfo_s;

typedef struct 
{
    FrameHeader_t frame_header;  // 数据段id = 0x0F
    uint32_t time_stamp;
    struct
    {
        float target_position_x;
        float target_position_y;
        uint8_t cmd_keyboard;
        uint8_t target_robot_id;
        uint16_t cmd_source;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataMapCmd_s;

typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x10
    uint32_t time_stamp;
    struct
    {
        float initial_speed;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendInitSpeed_s;

enum {
    NAV_STATUS_IMU_VALID      = 1U << 0,
    NAV_STATUS_BIG_YAW_VALID  = 1U << 1,
    NAV_STATUS_POWER_VALID    = 1U << 2,
    NAV_STATUS_BUFFER_VALID   = 1U << 3,
    NAV_STATUS_REFEREE_ONLINE = 1U << 4,
};

// 导航补充反馈。底盘速度、小 Yaw/Pitch 关节及其他裁判数据使用现有发送帧。
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x21
    uint32_t time_stamp;
    struct
    {
        uint8_t transform_state;
        uint8_t transformed;
        uint8_t flags;
        uint32_t session;
        uint32_t command_time_stamp;
        struct
        {
            uint32_t time_stamp;
            float yaw;
        } __packed__ main_imu;   // 大 Yaw 上的主控 BMI088 惯性航向，rad
        struct
        {
            uint32_t time_stamp;
            float angle;
        } __packed__ big_yaw;    // 大 Yaw 相对底盘的机械角，rad
        uint16_t buffer_energy_j;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataNavigationStatus_s;

/*-------------------- Receive --------------------*/
enum {
    ROBOT_CMD_CHASSIS_ENABLED = 1U << 0,
    ROBOT_CMD_HEADING_VALID    = 1U << 1,
    ROBOT_CMD_AIM_VALID        = 1U << 2,
};

enum {
    ROBOT_CMD_NAV_NORMAL = 0,
    ROBOT_CMD_NAV_SLOW_SPIN = 1,
    ROBOT_CMD_NAV_FAST_SPIN = 2,
    ROBOT_CMD_NAV_FOLD = 3,
    ROBOT_CMD_NAV_UPRIGHT = 4,
};

typedef struct RobotCmdData
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;  // chassis_link，m/s、m/s、rad/s
        struct
        {
            float roll;
            float pitch;
            float yaw;
            float leg_lenth;
        } __packed__ chassis;
        struct
        {
            float pitch;
            float yaw;
        } __packed__ gimbal;
        struct
        {
            uint8_t fire;
            uint8_t fric_on;
        } __packed__ shoot;
        struct
        {
            uint8_t mode;    // ROBOT_CMD_NAV_*
            uint8_t flags;   // ROBOT_CMD_* 有效位
            uint32_t session;
            float heading;   // 主控 BMI088 惯性参考系内的大 Yaw 绝对航向，rad
        } __packed__ navigation;
    } __packed__ data;
    uint16_t checksum;
} __packed__ ReceiveDataRobotCmd_s;

#ifdef __cplusplus
static_assert(sizeof(ReceiveDataRobotCmd_s) == 58, "RobotCmd wire size");
static_assert(sizeof(SendDataNavigationStatus_s) == 39, "navigation status wire size");
#endif

// PID调参数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float kp;
        float ki;
        float kd;
        float max_out;
        float max_iout;
    } __packed__ data;
    uint16_t crc;
} __packed__ ReceiveDataPidDebug_s;

// 虚拟遥控器数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    RcControl data;
    uint16_t crc;
} __packed__ ReceiveDataVirtualRc_s;

typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x04
    uint32_t time_stamp;
    uint32_t sentry_cmd;
    uint16_t crc;
} __packed__ ReceiveDataSentryCmd_s;


#endif  // USB_TYPEDEF_H
