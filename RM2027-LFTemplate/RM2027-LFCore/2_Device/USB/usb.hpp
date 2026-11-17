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

#include "usb_typdef.h"

class Usb {
public:
    void Init(void);
    void Update(void);

private:
    bool Receive(void);
    void UpdateRoboCmd(const ReceiveRoboCmd_s& frame);
    void UpdateShootCmd(const ReceiveShootCmd_s& frame);
    void SendAll(void);
    bool SendRawGimbalImu(uint32_t now);
    bool SendGimbalStatus(uint32_t now);
    bool SendNavStatus(uint32_t now);
    bool SendBigYawHeading(uint32_t now);
    bool SendCompStage(void);
    bool SendCompEvent(void);
    bool SendAlliedHealth(void);
    bool SendAlliedPos(void);
    bool SendRobotStatus(void);
    bool SendRobotPos(void);
    bool SendRfidStatus(void);
    bool SendSentryInfo(void);
    void SendRefereeEvent(void);

    uint8_t rx_buf[256]{};
    uint32_t rx_size = 0;
    SendSyncResponse_s sync_response{};
    bool sync_pending = false;

    uint32_t last_robot_cmd_ms = 0;
    bool has_robot_cmd = false;
    bool has_shoot_cmd = false;
    bool transform_blocked_pending = false;
    uint32_t last_raw_imu_ms = 0;
    uint32_t last_gimbal_status_ms = 0;
    uint32_t last_nav_status_ms = 0;
    uint32_t last_big_yaw_heading_ms = 0;
    uint32_t last_comp_stage_ms = 0;
    uint32_t last_comp_event_ms = 0;
    uint32_t last_allied_health_ms = 0;
    uint32_t last_allied_pos_ms = 0;
    uint32_t last_robot_status_ms = 0;
    uint32_t last_robot_pos_ms = 0;
    uint32_t last_rfid_status_ms = 0;
    uint32_t last_sentry_info_ms = 0;
    uint32_t last_gyro_revision = 0;
    uint32_t last_attitude_sample_ms = 0;
};

extern Usb usb;

#endif  // USB_HPP
