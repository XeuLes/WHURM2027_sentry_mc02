/**
 * @file chassis_power_manager.hpp
 * @brief 底盘功率上限仲裁与电流衰减接口。
 */

#ifndef CHASSIS_POWER_MANAGER_HPP
#define CHASSIS_POWER_MANAGER_HPP

#include "alg_power_control.hpp"
#include "chassis_config.hpp"

namespace chassis_app {

class ChassisPowerManager {
public:
    /**
     * @brief 初始化 C620 功率模型、安全裕度和调试快照。
     * @note 初始化使用台架回退功率上限，实际上限由 apply() 每周期更新。
     */
    void init();

    /**
     * @brief 按裁判系统和超级电容限制衰减四轮目标电流。
     * @param[in] wheel_omega 四轮输出轴角速度，单位 rad/s。
     * @param[in] target_current 力控计算出的四轮目标电流，单位 A。
     * @param[out] output_current 功率衰减后的四轮电流，单位 A。
     * @return 控制结果可信且满足功率预算时返回 true。
     * @note 失败时 output_current 全部置零，调用方还应停止底盘控制链。
     */
    bool apply(
        const float wheel_omega[kWheelCount],
        const float target_current[kWheelCount],
        float output_current[kWheelCount]
    );

    /** @brief 查询功率管理器是否已完成初始化。 */
    bool enabled() const
    {
        return enabled_;
    }

    // 调试快照：可直接在 Live Watch 中展开 g_chassisPowerManager。
    float effective_limit_w = 0.0f;
    float estimated_power_w = 0.0f;
    float consumed_power_w = 0.0f;
    power_control::PowerControlStatus status =
        power_control::PowerControlStatus::Disabled;

private:
    /**
     * @brief 解析当前允许的底盘功率上限。
     * @return 裁判系统与超级电容约束中的较小值，单位 W。
     */
    float resolveLimit() const;

    power_control::PowerController controller_ {};
    bool enabled_ = false;
};

extern ChassisPowerManager g_chassisPowerManager;

} // namespace chassis_app

#endif // CHASSIS_POWER_MANAGER_HPP
