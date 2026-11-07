/**
 * @file alg_gravity_compensation.cpp
 * @brief 底盘坡道重力前馈计算实现。
 */

#include "alg_gravity_compensation.hpp"

#include <cmath>

namespace gravity_compensation {

namespace {

constexpr float kDegToRad = 0.01745329251994329577f;

/**
 * @brief 将数值按最大步长移动到目标值。
 * @param current 当前值。
 * @param target 目标值。
 * @param maximum_delta 本次允许的最大变化量。
 * @return 移动后的值。
 */
float moveToward(float current, float target, float maximum_delta)
{
    if (target > current + maximum_delta) {
        return current + maximum_delta;
    }
    if (target < current - maximum_delta) {
        return current - maximum_delta;
    }
    return target;
}

} // namespace

/**
 * @brief 检查质量、重力、滤波和限幅参数。
 */
bool GravityCompensator::configValid(const Config& config)
{
    return std::isfinite(config.mass_kg)
        && config.mass_kg >= 0.0f
        && std::isfinite(config.gravity_mps2)
        && config.gravity_mps2 > 0.0f
        && std::isfinite(config.scale_x)
        && std::isfinite(config.scale_y)
        && std::isfinite(config.filter_alpha)
        && config.filter_alpha >= 0.0f
        && config.filter_alpha < 1.0f
        && std::isfinite(config.slew_rate_nps)
        && config.slew_rate_nps >= 0.0f
        && std::isfinite(config.max_force_n)
        && config.max_force_n >= 0.0f
        && std::isfinite(config.max_tilt_deg)
        && config.max_tilt_deg > 0.0f;
}

/**
 * @brief 保存参数并重置重力投影、滤波和斜坡状态。
 */
bool GravityCompensator::init(const Config& config)
{
    if (!configValid(config)) {
        configured_ = false;
        reset();
        return false;
    }

    config_ = config;
    configured_ = true;
    reset();
    return true;
}

/**
 * @brief 清空重力投影、前馈力以及滤波/斜坡就绪标志。
 */
void GravityCompensator::reset()
{
    gravity_accel_x_filtered_ = 0.0f;
    gravity_accel_y_filtered_ = 0.0f;
    force_x_ = 0.0f;
    force_y_ = 0.0f;
    filter_ready_ = false;
    slew_ready_ = false;
}

/**
 * @brief 根据姿态计算重力投影和经过滤波、限幅、斜坡处理的前馈力。
 */
Output GravityCompensator::update(
    float pitch_deg,
    float roll_deg,
    bool attitude_valid,
    float dt_s
)
{
    Output output {};
    if (!configured_ || !config_.enabled || !attitude_valid
        || !std::isfinite(pitch_deg) || !std::isfinite(roll_deg)
        || !std::isfinite(dt_s) || dt_s <= 0.0f
        || std::fabs(pitch_deg) > config_.max_tilt_deg
        || std::fabs(roll_deg) > config_.max_tilt_deg) {
        reset();
        return output;
    }

    const float pitch_rad = pitch_deg * kDegToRad;
    const float roll_rad = roll_deg * kDegToRad;

    // x 向前、y 向左、z 向上时的重力在底盘 x/y 平面投影。
    const float gravity_x =
        config_.gravity_mps2 * std::sin(pitch_rad);
    const float gravity_y =
        -config_.gravity_mps2 * std::cos(pitch_rad) * std::sin(roll_rad);

    if (!filter_ready_) {
        gravity_accel_x_filtered_ = gravity_x;
        gravity_accel_y_filtered_ = gravity_y;
        filter_ready_ = true;
    } else {
        const float alpha = config_.filter_alpha;
        gravity_accel_x_filtered_ =
            alpha * gravity_accel_x_filtered_ + (1.0f - alpha) * gravity_x;
        gravity_accel_y_filtered_ =
            alpha * gravity_accel_y_filtered_ + (1.0f - alpha) * gravity_y;
    }

    const float target_force_x = -config_.mass_kg
        * gravity_accel_x_filtered_ * config_.scale_x;
    const float target_force_y = -config_.mass_kg
        * gravity_accel_y_filtered_ * config_.scale_y;

    if (!slew_ready_) {
        force_x_ = 0.0f;
        force_y_ = 0.0f;
        slew_ready_ = true;
    }

    const float maximum_delta = config_.slew_rate_nps * dt_s;
    force_x_ = moveToward(force_x_, target_force_x, maximum_delta);
    force_y_ = moveToward(force_y_, target_force_y, maximum_delta);
    force_x_ = std::fmax(
        -config_.max_force_n,
        std::fmin(config_.max_force_n, force_x_)
    );
    force_y_ = std::fmax(
        -config_.max_force_n,
        std::fmin(config_.max_force_n, force_y_)
    );

    output.gravity_accel_x_mps2 = gravity_accel_x_filtered_;
    output.gravity_accel_y_mps2 = gravity_accel_y_filtered_;
    output.force_x_n = force_x_;
    output.force_y_n = force_y_;
    output.valid = true;
    output.applied = config_.mass_kg > 0.0f
        && (config_.scale_x != 0.0f || config_.scale_y != 0.0f);
    return output;
}

/** @brief 返回当前重力前馈参数副本。 */
Config GravityCompensator::config() const
{
    return config_;
}

} // namespace gravity_compensation
