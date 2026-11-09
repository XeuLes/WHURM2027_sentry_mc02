/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.cpp/hpp
  * @brief      Updated the referee system Communication Protocol V2.0.0
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/3/6         YZX             1.更新裁判系统通信协议至1.6.1（不包含半自动步兵和UI）
  *  V2.0.0     2026/10/15      XeuLes          1.更新裁判系统通信协议至 V2.0.0（20260626）
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#ifndef REFEREE_HPP
#define REFEREE_HPP

#include "main.h"
#include "protocol.h"
#include "fifo.h"
#include "stdbool.h"

struct RefereePowerSnapshot {
    uint16_t power_limit_w;
    uint16_t buffer_energy_j;
    uint32_t power_tick;
    uint32_t buffer_tick;
    bool power_received;
    bool buffer_received;
    bool online;
    uint16_t shooter_cooling_value;
    uint16_t shooter_heat_limit;
    uint16_t shooter_17mm_heat;
};

#define USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH 1024

// clang-format off
/* ID 编号见《通信协议 V2.0.0》附录二，蓝方是 101~111 */
typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    RED_DART        = 8,
    RED_RADAR       = 9,
    RED_BASE        = 10,
    RED_OUTPOST     = 11,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
    BLUE_DART       = 108,
    BLUE_RADAR      = 109,
    BLUE_BASE       = 110,
    BLUE_OUTPOST    = 111,
    REFEREE_SERVER  = 0x8080,  //裁判系统服务器，哨兵/雷达自主决策指令的接收者
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
// clang-format on
typedef struct __packed  //0x0001
{
    uint8_t game_type : 4;       //比赛类型
    uint8_t game_progress : 4;   //当前比赛阶段
    uint16_t stage_remain_time;  //当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp;  //UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_status_t;

typedef struct __packed  //0x0002
{
    uint8_t winner;  //0：平局 1：红方胜利 2：蓝方胜利
} game_result_t;
typedef struct __packed  //0x0003  共 20 字节
{
    uint16_t ally_1_robot_HP;   //己方 1 号英雄机器人血量
    uint16_t ally_2_robot_HP;   //己方 2 号工程机器人血量
    uint16_t ally_3_robot_HP;   //己方 3 号步兵机器人血量
    uint16_t ally_4_robot_HP;   //己方 4 号步兵机器人血量
    int16_t damage_difference;  //己方全队总伤害与对方全队总伤害之差（旧版此处是保留位）
    uint16_t ally_7_robot_HP;    //己方 7 号哨兵机器人血量
    uint16_t ally_outpost_HP;    //己方前哨站血量
    uint16_t ally_base_HP;       //己方基地血量
    uint16_t enemy_outpost_HP;   //对方前哨站血量（V2.0.0 新增）
    uint16_t enemy_base_HP;      //对方基地血量（V2.0.0 新增）
} game_robot_HP_t;
typedef struct __packed  //0x0101
{
    uint32_t event_data;  //场地事件，各 bit 含义见《通信协议 V2.0.0》表 1-9
} event_data_t;

/* 0x0102 补给站动作 / 0x0103 补给站预约 已在《通信协议 V2.0.0》中删除 */

typedef struct __packed  //0x0104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

typedef struct __packed  //0x105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

typedef struct __packed  //0x0120
{
    uint32_t sentry_cmd;
} sentry_cmd_t;

typedef struct __packed  //0x0201  共 17 字节
{
    uint8_t robot_id;                       //本机器人 ID
    uint8_t robot_level;                    //机器人等级
    uint16_t current_HP;                    //机器人当前血量
    uint16_t maximum_HP;                    //机器人血量上限
    uint16_t shooter_barrel_cooling_value;  //机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;     //机器人枪口热量上限
    uint16_t chassis_power_limit;           //机器人底盘功率上限
    float bullet_speed_limit;               //机器人射击初速度上限（V2.0.0 新增，旧结构体缺此 4 字节）
    uint8_t power_management_gimbal_output : 1;  //电源管理模块的输出情况：gimbal 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_chassis_output : 1;  //chassis 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_shooter_output : 1;  //shooter 口输出：0 为无输出，1 为 24V 输出
} robot_status_t;

typedef struct __packed  //0x0202  共 14 字节
{
    /* 前 8 字节在 V2.0.0 中已改为保留位。旧版此处是 chassis_voltage / chassis_current /
       chassis_power，2026 赛季起裁判系统不再下发实际底盘功率，功率控制需自行采样。 */
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;              //缓冲能量（单位：J）
    uint16_t shooter_17mm_barrel_heat;   //17mm 发射机构的枪口热量
    uint16_t shooter_42mm_barrel_heat;   //42mm 发射机构的枪口热量
} power_heat_data_t;

typedef struct __packed  //0x0203
{
    float x;      //本机器人位置 x 坐标，单位：m
    float y;      //本机器人位置 y 坐标，单位：m
    float angle;  //本机器人测速模块的朝向，单位：度。正北为 0 度
} robot_pos_t;

typedef struct __packed  //0x0204  共 8 字节
{
    uint8_t recovery_buff;       //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint16_t cooling_buff;       //机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
                                 //注意：V2.0.0 起是 16 位，旧结构体写成了 uint8_t，导致后面全部错位
    uint8_t defence_buff;        //机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff;  //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff;        //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
    uint8_t remaining_energy;  //机器人剩余能量值反馈，各位含义见《通信协议 V2.0.0》表 1-15
} buff_t;

/* 0x0205 空中机器人能量 已在《通信协议 V2.0.0》中删除 */

typedef struct __packed  //0x0206
{
    uint8_t
        armor_id : 4;  //当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t HP_deduction_reason : 4;  //血量变化类型
} hurt_data_t;

typedef struct __packed  //0x0207
{
    uint8_t bullet_type;          //弹丸类型
    uint8_t shooter_number;       //发射机构 ID：
    uint8_t launching_frequency;  //弹丸射速（单位：Hz）
    float initial_speed;          //弹丸初速度（单位：m/s）
} shoot_data_t;
typedef struct __packed  //0x0208
{
    uint16_t projectile_allowance_17mm;  //17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;  //42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin;        //剩余金币数量
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;
typedef struct __packed  //0x0209
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} rfid_status_t;
typedef struct __packed
{
    uint8_t bullet_remaining_num;
} ext_bullet_remaining_t;
typedef struct __packed  //0x020A
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;
typedef struct __packed  //0x020B  共 40 字节
{
    float hero_x;        //己方英雄机器人位置 x 轴坐标，单位：m
    float hero_y;        //己方英雄机器人位置 y 轴坐标，单位：m
    float engineer_x;    //己方工程机器人位置 x 轴坐标，单位：m
    float engineer_y;    //己方工程机器人位置 y 轴坐标，单位：m
    float standard_3_x;  //己方 3 号步兵机器人位置 x 轴坐标，单位：m
    float standard_3_y;  //己方 3 号步兵机器人位置 y 轴坐标，单位：m
    float standard_4_x;  //己方 4 号步兵机器人位置 x 轴坐标，单位：m
    float standard_4_y;  //己方 4 号步兵机器人位置 y 轴坐标，单位：m
    float reserved1;     //保留位（旧结构体误标为 standard_5_x）
    float reserved2;     //保留位（旧结构体误标为 standard_5_y）
} ground_robot_position_t;

/* 0x020C 雷达标记进度：V2.0.0 已改为单个 uint16_t mark_progress，
   且发送方是“服务器→己方雷达机器人”，哨兵收不到，故不解析 */

typedef struct __packed  //0x020D  共 14 字节
{
    uint32_t sentry_info;    //bit 0-31，见《通信协议 V2.0.0》表 1-23
    uint16_t sentry_info_2;  //bit 0-15，脱战状态 / 可兑换数 / 当前姿态 / 能量机关 / 强化姿态
    uint64_t sentry_info_3;  //bit 0-63，六种姿态弱化前剩余可持续时长，每 8 bit 一个
} sentry_info_t;
typedef struct __packed  //0x020E
{
    uint8_t radar_info;
} radar_info_t;
typedef struct __packed  //0x0301
{
    uint16_t data_cmd_id;  //子内容 ID，取值见《通信协议 V2.0.0》表 1-25
    uint16_t sender_id;    //发送者 ID
    uint16_t receiver_id;  //接收者 ID
    uint8_t user_data[112];  //内容数据段，整包 127 字节减去 9 字节帧头尾和 6 字节数据段头
} robot_interaction_data_t;

/**
 * @brief 裁判系统发送的数据, 0x0301图形交互信息, 用户自主发送
 *
 */
typedef struct __packed
{
  uint8_t figure_name[3];
  uint32_t operate_tpye:3;
  uint32_t figure_tpye:3;
  uint32_t layer:4;
  uint32_t color:4;
  uint32_t details_a:9;
  uint32_t details_b:9;
  uint32_t width:10;
  uint32_t start_x:11;
  uint32_t start_y:11;
  uint32_t details_c:10;
  uint32_t details_d:11;
  uint32_t details_e:11;
}interaction_figure_t;

/**
 * @brief 自定义控制器数据
 */
typedef struct __packed __CustomControllerData  //0x0302
{
    uint8_t data[30];
} CustomControllerData_t;

typedef struct __packed __MapCommandData  //0x0303
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
} MapCommandData_t;

typedef struct __packed
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;

typedef struct __packed
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef struct __packed
{
    uint8_t data[32];
} ext_download_stream_data_t;

/* 0x0304 机器人指令（鼠标/键盘）已在《通信协议 V2.0.0》中删除 */

/**
 * @brief 裁判系统模块
 * @note  协议结构体保持 __packed 位域布局不变，直接 memcpy 收发。
 */
class Referee
{
public:
    /* 裁判系统协议数据 */
    game_status_t game_status;
    game_result_t game_result;
    game_robot_HP_t game_robot_HP;

    event_data_t field_event;
    referee_warning_t referee_warning;

    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    robot_pos_t game_robot_pos_t;
    buff_t buff_musk_t;
    hurt_data_t robot_hurt_t;
    shoot_data_t shoot_data;
    projectile_allowance_t projectile_allowance;
    robot_interaction_data_t student_interactive_data_t;
    CustomControllerData_t CUSTOM_CONTROLLER_DATA;
    MapCommandData_t MAP_COMMAND_DATA;

    rfid_status_t RFID_STATUS;
    ground_robot_position_t ground_robot_pos_t;
    sentry_info_t sentry_decision_info_t;

    /* 接收链路 */
    uint8_t uart_buf[2][USART_RX_BUF_LENGHT];
    fifo_s_t fifo;
    uint8_t fifo_buf[REFEREE_FIFO_BUF_LENGTH];
    unpack_data_t unpack_obj;

    /* 生命周期 */
    void Init(void);

    /* 协议解析 */
    void DataSolve(uint8_t *frame);
    void UnpackFifoData(void);
    void FeedBytes(const char *data, uint16_t len);

    /* 查询接口
       注意：《通信协议 V2.0.0》起 0x0202 不再下发实际底盘功率，原先的
       GetChassisPower / GetChassisPowerAndBuffer 已删除，功率需自行采样。 */
    bool IsOffline(void);
    /* 最近 kAttackedHoldMs 内血量掉过，用于上位机判"正在挨打"。
       用血量差值判定而不是 0x0206 的 HP_deduction_reason，
       因为 reason 0 是合法值（被弹丸击打），无法区分"没挨打"和"被弹丸打"。 */
    bool IsAttacked(void);
    float GetBuffer(void);
    RefereePowerSnapshot GetPowerSupplySnapshot();
    void GetChassisPowerLimit(uint16_t *power_limit);
    void GetShootHeat17LimitAndHeat17(uint16_t *cooling_value, uint16_t *heat_limit, uint16_t *heat);
    void GetSentryInfo(uint32_t *sentry_info, uint16_t *sentry_info_2, uint64_t *sentry_info_3);
    uint16_t GetCurrentHP(void);
    uint8_t GetRobotId(void);
    uint8_t GetTeamColor(void);
    uint16_t GetShootHeat(void);
    void GetShootSpeedFre(uint8_t *shoot_fre, float *shoot_speed);
    CustomControllerData_t *GetCustomControllerDataPoint(void);
    void GetMapCommandData(float *x, float *y, uint8_t *cmd_keyboard, uint8_t *target_robot_id, uint16_t *cmd_source);
    float GetCustomControllerPos(uint8_t index);

private:
    frame_header_struct_t receive_header;
    frame_header_struct_t send_header;

    uint32_t online_time;
    uint32_t receive_count;
    uint32_t power_tick_ = 0;
    uint32_t buffer_tick_ = 0;
    bool power_received_ = false;
    bool buffer_received_ = false;

    uint16_t last_hp_ = 0;
    uint32_t attacked_time_ = 0;

    static constexpr uint32_t kTimeout = 300;
    static constexpr uint32_t kAttackedHoldMs = 1000;
};

extern Referee referee;

/* 中断向量表按符号名找入口，必须保持 C 链接 */
extern "C" void USART1_IRQHandler(void);

#endif
