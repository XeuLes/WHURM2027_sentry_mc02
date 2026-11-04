#include "dvc_supercap.hpp"
#include <math.h>

namespace {
class IrqLock {
public:
    IrqLock() : mask_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~IrqLock() { __DMB(); __set_PRIMASK(mask_); }
private:
    uint32_t mask_;
};
uint16_t readLe16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
bool validInput(float value, float maximum)
{
    return isfinite(value) && value >= 1.0f && value <= maximum && value <= 65535.0f;
}
}

Supercap* Supercap::devices_[3] = {};
uint8_t Supercap::device_count_ = 0;
bool Supercap::observer_registered_ = false;

bool Supercap::init(const SupercapConfig& config)
{
    if (initialized_ || config.bus == nullptr
        || !validInput(config.max_referee_power_w, 65535.0f)
        || !validInput(config.max_buffer_j, 65535.0f)) { return false; }
    IrqLock lock;
    if (device_count_ >= 3U) { return false; }
    for (uint8_t i = 0; i < device_count_; ++i) {
        if (devices_[i]->config_.bus == config.bus) {
            // 控制 ID 固定 0x61，同一路总线只注册一块超电板。
            return false;
        }
    }
    if (!observer_registered_) {
        if (!bspFdcanAddObserver(onCanFrame)) { return false; }
        observer_registered_ = true;
    }
    config_ = config;
    initialized_ = true;
    devices_[device_count_++] = this;
    return true;
}

void Supercap::onCanFrame(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* header,
                         const uint8_t* data, uint8_t length)
{
    if (header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME
        || header->FDFormat != FDCAN_CLASSIC_CAN) { return; }
    for (uint8_t i = 0; i < device_count_; ++i) {
        if (devices_[i]->config_.bus == bus) {
            (void)devices_[i]->update(header->Identifier, data, length);
        }
    }
}

bool Supercap::update(uint32_t identifier, const uint8_t* data, uint8_t length)
{
    if (!initialized_ || identifier != 0x52U || data == nullptr || length != 8U) {
        return false;
    }
    SupercapFeedback parsed;
    if (!decodeFeedback(data, length, &parsed)) { return false; }
    IrqLock lock;
    parsed.last_rx_tick = HAL_GetTick();
    parsed.rx_count = feedback_.rx_count + 1U;
    parsed.online = true;
    feedback_ = parsed;
    received_ = true;
    return true;
}

SupercapFeedback Supercap::data() const
{
    IrqLock lock;
    SupercapFeedback result = feedback_;
    result.online = received_
        && (HAL_GetTick() - result.last_rx_tick) <= config_.offline_timeout_ms;
    return result;
}

void Supercap::resetFeedback()
{
    IrqLock lock;
    feedback_ = SupercapFeedback{};
    received_ = false;
}

bool Supercap::updateReferee(float power_w, float buffer_j)
{
    IrqLock lock;
    if (!initialized_) { return false; }
    if (validInput(power_w, config_.max_referee_power_w)) {
        power_w_ = (uint16_t)power_w;
        power_valid_ = true;
    }
    if (validInput(buffer_j, config_.max_buffer_j)) {
        buffer_j_ = (uint16_t)buffer_j;
        buffer_valid_ = true;
    }
    return power_valid_ && buffer_valid_;
}

bool Supercap::sendControl(const SupercapControl& control)
{
    uint16_t power_w;
    uint16_t buffer_j;
    {
        IrqLock lock;
        if (!initialized_ || !power_valid_ || !buffer_valid_) { return false; }
        power_w = power_w_;
        buffer_j = buffer_j_;
    }
    return sendControlFrame(config_.bus, control, power_w, buffer_j);
}

bool Supercap::sendControlFrame(FDCAN_HandleTypeDef* bus, const SupercapControl& control,
                                uint16_t power_w, uint16_t buffer_j)
{
    if (bus == nullptr || power_w == 0U || buffer_j == 0U) { return false; }
    // 直接复用正式协议，保留位为零；不复制 packed 位域结构体。
    uint8_t data[8] = {};
    data[0] = (control.enable_dcdc ? 0x01U : 0U)
        | (control.restart ? 0x02U : 0U) | (control.clear_error ? 0x20U : 0U)
        | (control.active_charging_limit ? 0x40U : 0U) | 0x80U;
    data[1] = (uint8_t)power_w;
    data[2] = (uint8_t)(power_w >> 8);
    data[3] = (uint8_t)buffer_j;
    data[4] = (uint8_t)(buffer_j >> 8);
    data[5] = control.charging_limit_ratio;
    return bspFdcanSend(bus, 0x61U, data, 8U);
}

bool Supercap::decodeFeedback(const uint8_t* data, uint8_t length, SupercapFeedback* feedback)
{
    if (data == nullptr || feedback == nullptr || length != 8U) { return false; }
    feedback->status_code = data[0];
    feedback->chassis_power_w = ((float)readLe16(data + 1) - 16384.0f) / 64.0f;
    feedback->referee_power_w = ((float)readLe16(data + 3) - 16384.0f) / 64.0f;
    feedback->chassis_power_limit_w = (float)readLe16(data + 5);
    feedback->cap_energy_raw = data[7];
    return true;
}

bool Supercap::chassisPowerLimit(float* watts) const
{
    if (watts == nullptr) { return false; }
    const auto feedback = data();
    *watts = 0.0f;
    if (!feedback.online || feedback.chassis_power_limit_w <= 0.0f) { return false; }
    *watts = feedback.chassis_power_limit_w;
    return true;
}
