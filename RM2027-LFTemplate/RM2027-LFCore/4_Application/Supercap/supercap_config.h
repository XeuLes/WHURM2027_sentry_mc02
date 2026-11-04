#pragma once

// 默认总线参考旧工程 robot_param_omni_sentry_chassis.h 的 CHASSIS_CAN=1。
#ifndef SUPERCAP_CAN_INDEX
#define SUPERCAP_CAN_INDEX 1U
#endif

#define SUPERCAP_TX_PERIOD_MS 1U
#define SUPERCAP_FEEDBACK_TIMEOUT_MS 100U
#define SUPERCAP_REFEREE_TIMEOUT_MS 1000U
#define SUPERCAP_CHARGING_LIMIT_RATIO 229U
// 协议字段上界，不代表某一赛季允许值；可按机器人协议收紧。
#define SUPERCAP_MAX_REFEREE_POWER_W 65535.0f
#define SUPERCAP_MAX_BUFFER_J 65535.0f
