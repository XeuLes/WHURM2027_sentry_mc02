#include "app_supercap.h"
#include "supercap_config.h"
#include "dvc_supercap.hpp"
#include "referee.hpp"
#include "cmsis_os.h"

namespace {
class IrqLock {
public:
    IrqLock() : mask_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~IrqLock() { __DMB(); __set_PRIMASK(mask_); }
private:
    uint32_t mask_;
};
Supercap capacitor;
SupercapAppStatus status = SUPERCAP_NOT_CONFIGURED;
bool initialized = false;
// 与提供的 superpower_task.c 一致，默认请求使能 DCDC。
bool enabled_request = true;
}

void superpower_task(void const* argument)
{
    (void)argument;
    for (;;) {
        const auto referee_data = referee.GetPowerSupplySnapshot();
        const bool values_ready = capacitor.updateReferee(
            referee_data.power_received ? (float)referee_data.power_limit_w : 0.0f,
            referee_data.buffer_received ? (float)referee_data.buffer_energy_j : 0.0f);
        const uint32_t now = HAL_GetTick();
        const bool fresh = referee_data.online && referee_data.power_received
            && referee_data.buffer_received
            && now - referee_data.power_tick <= SUPERCAP_REFEREE_TIMEOUT_MS
            && now - referee_data.buffer_tick <= SUPERCAP_REFEREE_TIMEOUT_MS;
        const auto feedback = capacitor.data();
        SupercapControl command;
        bool requested;
        { IrqLock lock; requested = enabled_request; }
        // 先发送使能再等待反馈，避免需要控制帧才回报的超电板无法启动。
        command.enable_dcdc = requested && values_ready && fresh;
        command.charging_limit_ratio = SUPERCAP_CHARGING_LIMIT_RATIO;
        // 无首帧有效裁判值时不发送；离线后沿用有效值并发送失能。
        if (values_ready) { (void)capacitor.sendControl(command); }
        {
            IrqLock lock;
            status = !values_ready ? SUPERCAP_WAIT_REFEREE
                : !fresh ? SUPERCAP_REFEREE_OFFLINE
                : !feedback.online ? SUPERCAP_WAIT_FEEDBACK : SUPERCAP_READY;
        }
        osDelay(SUPERCAP_TX_PERIOD_MS);
    }
}

bool supercapInit(void)
{
    if (initialized) { return true; }
    SupercapConfig config;
    config.bus = bspFdcanGetHandle(SUPERCAP_CAN_INDEX);
    config.offline_timeout_ms = SUPERCAP_FEEDBACK_TIMEOUT_MS;
    config.max_referee_power_w = SUPERCAP_MAX_REFEREE_POWER_W;
    config.max_buffer_j = SUPERCAP_MAX_BUFFER_J;
    if (!capacitor.init(config)) { status = SUPERCAP_INIT_FAILED; return false; }
    status = SUPERCAP_WAIT_REFEREE;
    initialized = true;
    return true;
}

void supercapSetEnabled(bool enabled)
{
    IrqLock lock;
    enabled_request = enabled;
}

SupercapAppStatus supercapGetStatus(void)
{
    IrqLock lock;
    if (status == SUPERCAP_READY && !capacitor.data().online) { return SUPERCAP_WAIT_FEEDBACK; }
    return status;
}

SupercapFeedback supercapGetFeedback(void)
{
    return capacitor.data();
}

bool supercapGetChassisPowerLimit(float* watts)
{
    if (watts == nullptr) { return false; }
    *watts = 0.0f;
    if (supercapGetStatus() != SUPERCAP_READY) { return false; }
    return capacitor.chassisPowerLimit(watts);
}
