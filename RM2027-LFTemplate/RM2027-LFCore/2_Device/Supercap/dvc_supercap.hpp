/** @file dvc_supercap.hpp @brief 超级电容协议；按用户提供的 superpower_task 和超电接口文档移植。 */
#pragma once
#include "bsp_can.h"

struct SupercapConfig {
    FDCAN_HandleTypeDef* bus = nullptr;
    uint32_t offline_timeout_ms = 100U;
    float max_referee_power_w = 65535.0f;
    float max_buffer_j = 65535.0f;
};

struct SupercapFeedback {
    uint8_t status_code = 0; // 新版原始状态字，位定义未提供，不推断具体故障。
    float chassis_power_w = 0.0f;
    float referee_power_w = 0.0f;
    float chassis_power_limit_w = 0.0f;
    uint8_t cap_energy_raw = 0; // 文档未提供量纲，不假定百分比。
    // 接收状态与反馈一起保存；online 在 data() 读取时按超时重新计算。
    uint32_t last_rx_tick = 0;
    uint32_t rx_count = 0;
    bool online = false;
};

struct SupercapControl {
    bool enable_dcdc = false;
    bool restart = false;
    bool clear_error = false;
    bool active_charging_limit = true;
    uint8_t charging_limit_ratio = 229U;
};

class Supercap {
public:
    Supercap() = default;
    Supercap(const Supercap&) = delete;
    Supercap& operator=(const Supercap&) = delete;
    /** @brief 初始化并接入 CAN 观察者，对象须为 static，注册后不允许重新 init。 */
    bool init(const SupercapConfig& config);
    // Only the confirmed 0x52 standard classic CAN feedback is accepted.
    bool update(uint32_t identifier, const uint8_t* data, uint8_t length);
    /** @brief 返回反馈副本，包含接收计数、最近接收时间和当前在线状态。 */
    SupercapFeedback data() const;
    void resetFeedback();
    /** @brief 两字段独立过滤；非正数/非有限/越界值保留各自最近有效值。 */
    bool updateReferee(float power_w, float buffer_j);
    /** @brief 只支持新版；首次收到两项有效裁判数据之前不发送。 */
    bool sendControl(const SupercapControl& control);
    /** @brief 无需注册反馈 ID 的控制帧发送；供固定数据 CAN 调试复用协议打包。 */
    static bool sendControlFrame(FDCAN_HandleTypeDef* bus, const SupercapControl& control,
                                 uint16_t power_w, uint16_t buffer_j);
    /** @brief 仅按超电格式解码，不判断数据来源；来源由总线/ID 匹配确认。 */
    static bool decodeFeedback(const uint8_t* data, uint8_t length, SupercapFeedback* feedback);
    /** @brief 新版、在线且上限非零才返回可用预算，否则返回 false 并置零。 */
    bool chassisPowerLimit(float* watts) const;

private:
    static void onCanFrame(FDCAN_HandleTypeDef*, const FDCAN_RxHeaderTypeDef*,
                           const uint8_t*, uint8_t);
    static Supercap* devices_[3];
    static uint8_t device_count_;
    static bool observer_registered_;
    SupercapConfig config_;
    SupercapFeedback feedback_;
    bool initialized_ = false;
    bool received_ = false;
    bool power_valid_ = false;
    bool buffer_valid_ = false;
    uint16_t power_w_ = 0;
    uint16_t buffer_j_ = 0;
};
