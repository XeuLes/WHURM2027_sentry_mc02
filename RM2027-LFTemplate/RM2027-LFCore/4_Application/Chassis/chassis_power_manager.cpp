/**
 * @file chassis_power_manager.cpp
 * @brief 底盘功率控制适配实现。
 */

#include "chassis_power_manager.hpp"

#include "app_supercap.h"
#include "chassis_config.hpp"
#include "referee.hpp"

namespace chassis_app {

ChassisPowerManager g_chassisPowerManager;

/**
 * @brief 初始化四轮 C620 功率控制器和安全快照。
 */
void ChassisPowerManager::init()
{
    power_control::powerControllerInit(
        &controller_,
        &power_control::kDefaultC620Coefficients,
        kFallbackChassisPowerLimitW
    );
    controller_.margin = kPowerSafetyMargin;
    controller_.posture_scale = {1.0f, 1.0f, 1.0f, 1.0f};
    power_control::powerControllerSetCurrentLimit(
        &controller_,
        kC620MaxCurrent
    );

    effective_limit_w = kFallbackChassisPowerLimitW;
    estimated_power_w = 0.0f;
    consumed_power_w = 0.0f;
    status = power_control::PowerControlStatus::Disabled;
    enabled_ = true;
}

/**
 * @brief 解析裁判系统和超级电容共同约束下的功率上限。
 * @note 裁判功率 0 表示禁止输出；超级电容仅在提供正上限时参与取最小值。
 */
float ChassisPowerManager::resolveLimit() const
{
    const RefereePowerSnapshot snapshot = referee.GetPowerSupplySnapshot();
    float limit = kFallbackChassisPowerLimitW;

    if (snapshot.online && snapshot.power_received) {
        // 0x0201 已包含机器人类型、等级、姿态和现有增益效果。
        limit = static_cast<float>(snapshot.power_limit_w);
    }

    float supercap_limit = 0.0f;
    if (supercapGetChassisPowerLimit(&supercap_limit) && supercap_limit > 0.0f) {
        if (limit <= 0.0f || supercap_limit < limit) {
            limit = supercap_limit;
        }
    }
    return limit > 0.0f ? limit : 0.0f;
}

/**
 * @brief 更新功率上限并衰减四轮目标电流。
 * @note 输入非法或功率预算不可达时输出零电流并返回 false。
 */
bool ChassisPowerManager::apply(
    const float wheel_omega[kWheelCount],
    const float target_current[kWheelCount],
    float output_current[kWheelCount]
)
{
    if (!enabled_ || wheel_omega == nullptr || target_current == nullptr
        || output_current == nullptr) {
        return false;
    }

    effective_limit_w = resolveLimit();
    power_control::powerControllerSetPowerLimit(
        &controller_,
        effective_limit_w
    );

    power_control::MotorCommand motors[kWheelCount] = {};
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        motors[i].target_current = target_current[i];
        motors[i].velocity = wheel_omega[i];
    }

    power_control::powerControllerUpdate(
        &controller_,
        motors,
        kWheelCount
    );
    status = controller_.status;
    estimated_power_w = controller_.origin_power;
    consumed_power_w = controller_.consume_power;

    const bool valid_result =
        status == power_control::PowerControlStatus::Ready
        || status == power_control::PowerControlStatus::Limited;
    if (!valid_result) {
        for (uint8_t i = 0U; i < kWheelCount; ++i) {
            output_current[i] = 0.0f;
        }
        return false;
    }

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        output_current[i] = power_control::powerControllerLimitCurrent(
            &controller_,
            i
        );
    }
    return true;
}

} // namespace chassis_app
