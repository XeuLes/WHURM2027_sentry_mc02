/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb.cpp/hpp
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#ifndef USB_HPP
#define USB_HPP

#include <stdint.h>
#include <stdbool.h>

#include "usb_typdef.h"

#define USB_RX_DATA_SIZE 256  // byte

/**
 * @brief 发送数据间隔时间
 */
typedef struct
{
    uint32_t Debug;
    uint32_t Imu;
    uint32_t RobotStateInfo;
    uint32_t Event;
    uint32_t Pid;
    uint32_t AllRobotHp;
    uint32_t GameStatus;
    uint32_t RobotMotion;
    uint32_t GroundRobotPosition;
    uint32_t RfidStatus;
    uint32_t RobotStatus;
    uint32_t JointState;
    uint32_t Buff;
    uint32_t SentryInfo;
    uint32_t MapCommand;
    uint32_t InitSpeed;
    uint32_t NavigationStatus;
} LastSendTime_t;

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

/**
 * @brief 上位机下发的控制指令
 */
typedef struct
{
    ChassisSpeedVector_t speed_vector;
    struct
    {
        float roll;
        float pitch;
        float yaw;
        float leg_length;
    } chassis;

    struct
    {
        float pitch;
        float yaw;
    } gimbal;

    struct
    {
        bool fire;
        bool fric_on;
    } shoot;

} RobotCmdData_t;

/**
 * @brief 上位机下发的详细哨兵指令
 */
typedef struct
{
    bool confirm_revival;
    bool instant_revival;
    unsigned int purchase_ammo;
    unsigned int remote_purchase_ammo_times;
    unsigned int remote_purchase_HP_time;
    unsigned int stance; //哨兵姿态 1 为进攻姿态，2 为防御姿态，3 为移动姿态，默认为 3；修改此值即可改变哨兵姿态。
    bool energy_motivated;
    unsigned int reserved;
} Detailed_SentryCmd_t;

/**
 * @brief 上位机通信模块
 * @note  收发协议结构体保持 __packed 位域布局不变，直接 memcpy 收发。
 */
class Usb
{
public:
    /* 初始化 */
    void Init(void);

    /* 收发 + 解包 + 离线判定 */
    void Update(void);

    /* 链路状态 */
    bool IsOffline(void);

    /* 上位机控制指令 */
    float GetChassisSpeed(uint8_t axis);
    float GetChassisAngle(uint8_t axis);
    float GetChassisHeight(void);
    float GetGimbalAngle(uint8_t axis);
    bool GetFire(void);
    bool GetFricOn(void);

    /* 虚拟遥控器 */
    float GetVirtualRcCh(uint8_t channel);
    char GetVirtualRcSw(uint8_t channel);

    /* 哨兵指令 */
    uint32_t GetSentryCmd(void);

    /* 调试数据通道 */
    void ModifyDebugData(uint8_t index, float data, const char *name);


    /**
     * @brief 哨兵决策状态
     * @note  与通信无关，放在这里暂存，后续应独立成决策模块
     */
    struct SccState
    {
        float cmd_yaw_last;    // 上一帧的上位机目标角度
        uint16_t aimlosttime;  // 目标丢失时间
        bool scan;
        uint8_t scan_dir_yaw;  // 扫描方向
    } scc;

private:
    /* 链路状态  */
    uint8_t rx_buf[USB_RX_DATA_SIZE];
    uint32_t rx_size = 0U;
    bool offline = true;
    uint32_t receive_time = 0;
    uint32_t robot_cmd_receive_time = 0;
    uint32_t latest_rx_timestamp = 0;
    uint32_t continue_receive_cnt = 0;

    /* 数据发送结构体 */
    // clang-format off
    SendDataDebug_s       send_debug;
    SendDataImu_s         send_imu;
    SendDataRobotStateInfo_s   send_robot_state_info;
    SendDataEvent_s       send_event;
    SendDataPidDebug_s    send_pid;
    SendDataAllRobotHp_s  send_all_robot_hp;
    SendDataGameStatus_s  send_game_status;
    SendDataRobotMotion_s send_robot_motion;
    SendDataGroundRobotPosition_s send_ground_robot_position;
    SendDataRfidStatus_s  send_rfid_status;
    SendDataRobotStatus_s send_robot_status;
    SendDataJointState_s  send_joint_state;
    SendDataBuff_s        send_buff;
    SendDataSentryInfo_s  send_sentry_info;
    SendDataMapCmd_s      send_map_command;
    SendInitSpeed_s       send_init_speed;
    SendDataNavigationStatus_s send_navigation_status;
    // clang-format on

    /* 数据接收结构体 */
    ReceiveDataRobotCmd_s receive_robot_cmd;
    ReceiveDataPidDebug_s receive_pid_debug;
    ReceiveDataVirtualRc_s receive_virtual_rc;
    ReceiveDataSentryCmd_s receive_sentry_cmd;


    /* 控制指令数据 */
    RobotCmdData_t robot_cmd;
    RcControl virtual_rc;
    uint32_t sentry_cmd;
    Detailed_SentryCmd_t sentry_cmd_detailed;

    /* 发送节流  */
    LastSendTime_t last_send_time;

    /* 收发主流程 */
    void SendAll(void);
    void Receive(void);

    /* 各数据包发送 */
    void SendDebug(void);
    void SendImu(void);
    void SendRobotStateInfo(void);
    void SendEvent(void);
    void SendAllRobotHp(void);
    void SendGameStatus(void);
    void SendRobotMotion(void);
    void SendGroundRobotPosition(void);
    void SendRfidStatus(void);
    void SendRobotStatus(void);
    void SendJointState(void);
    void SendBuff(void);
    void SendSentryInfo(void);
    void SendMapCommand(void);
    void SendInitSpeed(void);
    void SendNavigationStatus(void);

    /* 接收数据解包 */
    bool AcceptRobotCmd(const ReceiveDataRobotCmd_s& frame);
    void UpdateRobotCmd(void);
    void UpdateVirtualRc(void);
    void UpdateSentryCmd(void);
};

extern Usb usb;

#endif
/*------------------------------ End of File ------------------------------*/
