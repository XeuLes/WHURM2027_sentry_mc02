/**
 * @file alg_gravity_compensation.hpp
 * @brief 底盘坡道重力前馈计算接口。
 */

#ifndef ALG_GRAVITY_COMPENSATION_HPP
#define ALG_GRAVITY_COMPENSATION_HPP

#include <stdint.h>

namespace gravity_compensation {

/**
 * @brief 底盘重力前馈参数。
 */
struct Config {
    /** @brief 是否启用重力前馈计算。 */
    bool enabled;

    /** @brief 整车质量，kg；0 表示尚未标定，输出保持为零。 */
    float mass_kg;

    /** @brief 当地重力加速度，m/s²。 */
    float gravity_mps2;

    /** @brief 车体 x 方向重力增益和符号修正。 */
    float scale_x;

    /** @brief 车体 y 方向重力增益和符号修正。 */
    float scale_y;

    /** @brief 重力投影一阶低通中的旧值权重，范围为 0~1。 */
    float filter_alpha;

    /** @brief 前馈力变化率上限，N/s。 */
    float slew_rate_nps;

    /** @brief 单个水平方向的前馈力绝对值上限，N。 */
    float max_force_n;

    /** @brief 允许计算前馈的最大 Roll/Pitch 绝对值，deg。 */
    float max_tilt_deg;
};

/**
 * @brief 当前周期的重力前馈结果。
 */
struct Output {
    /** @brief 底盘坐标系中的重力加速度 x/y 分量，m/s²。 */
    float gravity_accel_x_mps2 = 0.0f;
    float gravity_accel_y_mps2 = 0.0f;

    /** @brief 用于抵消重力的广义力 Fx/Fy，N。 */
    float force_x_n = 0.0f;
    float force_y_n = 0.0f;

    /** @brief 姿态输入有效且未超过倾角限制。 */
    bool valid = false;

    /** @brief 质量或增益已配置，前馈功能实际启用。 */
    bool applied = false;
};

/**
 * @brief 底盘坡道重力前馈器。
 */
class GravityCompensator {
public:
    /**
     * @brief 初始化重力前馈参数并清空滤波状态。
     * @param[in] config 重力前馈参数。
     * @return 参数合法时返回 true。
     */
    bool init(const Config& config);

    /**
     * @brief 清空姿态滤波和输出斜坡状态，保留参数。
     */
    void reset();

    /**
     * @brief 根据底盘姿态计算重力投影和前馈力。
     * @param[in] pitch_deg 底盘俯仰角，deg。
     * @param[in] roll_deg 底盘横滚角，deg。
     * @param[in] attitude_valid IMU 姿态数据是否新鲜有效。
     * @param[in] dt_s 控制周期，s。
     * @return 本周期重力投影、前馈力及有效状态。
     * @note 质量为零时仍计算重力投影用于标定，但输出力保持为零。
     */
    Output update(
        float pitch_deg,
        float roll_deg,
        bool attitude_valid,
        float dt_s
    );

    /** @brief 获取当前参数副本。 */
    Config config() const;

private:
    /**
     * @brief 检查重力前馈参数是否在合法范围内。
     * @param[in] config 待检查参数。
     * @return 参数有限、非负且逻辑一致时返回 true。
     */
    static bool configValid(const Config& config);

    Config config_ {};
    float gravity_accel_x_filtered_ = 0.0f;
    float gravity_accel_y_filtered_ = 0.0f;
    float force_x_ = 0.0f;
    float force_y_ = 0.0f;
    bool filter_ready_ = false;
    bool slew_ready_ = false;
    bool configured_ = false;
};

} // namespace gravity_compensation

#endif // ALG_GRAVITY_COMPENSATION_HPP
