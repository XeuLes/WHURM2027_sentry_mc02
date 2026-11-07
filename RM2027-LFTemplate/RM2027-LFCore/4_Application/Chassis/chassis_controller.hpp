/**
 * @file chassis_controller.hpp
 * @brief 四轮全向底盘反馈估计、速度闭环和力分配接口。
 */

#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include "alg_force.hpp"
#include "alg_gravity_compensation.hpp"
#include "alg_motion_compensation.hpp"
#include "alg_pid.h"
#include "chassis_command.hpp"
#include "chassis_config.hpp"
#include "dvc_dji_motor.hpp"
#include "lf_core.hpp"

#include <stdint.h>

namespace chassis_app {

struct ChassisFeedback {
    float wheel_omega[kWheelCount] {};
    float wheel_linear[kWheelCount] {};
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    uint8_t valid_wheel_mask = 0U;
    bool all_wheels_online = false;
    bool velocity_estimate_valid = false;
    bool acceleration_valid = false;
    bool motion_updated = false;
    /** @brief 轮速校正前的预测底盘速度，m/s。 */
    float predicted_vx = 0.0f;
    float predicted_vy = 0.0f;
    /** @brief 逐轮残差和归一化打滑分数。 */
    float wheel_slip_velocity[kWheelCount] {};
    float wheel_slip_score[kWheelCount] {};
    float wheel_confidence[kWheelCount] {};
    /** @brief 独立于传感器有效性的打滑轮位图与检测状态。 */
    uint8_t slip_wheel_mask = 0U;
    bool slip_detection_valid = false;
    bool slip_detected = false;
    /** @brief 姿态投影到车体平面的重力加速度，m/s²。 */
    float gravity_accel_x = 0.0f;
    float gravity_accel_y = 0.0f;
    /** @brief 重力前馈广义力，N；参数未标定时为零。 */
    float gravity_force_x = 0.0f;
    float gravity_force_y = 0.0f;
    bool gravity_valid = false;
    bool gravity_applied = false;
};

class ChassisController {
public:
    /**
     * @brief 初始化四轮电机、运动补偿器、速度环和力分配器。
     * @return 电机配置及注册全部成功时返回 true。
     * @note 必须在 FDCAN 初始化完成后、FreeRTOS 调度器启动前调用。
     */
    bool init();

    /**
     * @brief 读取一次电机和 IMU 状态，并生成速度反馈。
     * @param[in] imu 最新的 INS 姿态和去重力线性加速度。
     * @param[in] now_ms 当前系统时间，单位 ms。
     * @return 本周期轮速、底盘速度、有效位和估计状态。
     * @note 正常情况下轮速逆解只在运动补偿器内部执行一次。
     */
    ChassisFeedback readFeedback(
        const LuojiaFox::ImuState& imu,
        uint32_t now_ms
    );

    /**
     * @brief 根据命令和反馈计算四轮目标电流。
     * @param[in] command 已通过来源选择的速度命令。
     * @param[in] feedback 本周期读取并估计后的底盘反馈。
     * @param[out] current 四轮目标电流，单位 A。
     * @return 所有安全条件满足且成功生成输出时返回 true。
     * @note 返回 false 时会清除速度环积分并关闭力分配输出。
     */
    bool calculate(
        const ChassisCommand& command,
        const ChassisFeedback& feedback,
        float current[kWheelCount]
    );

    /**
     * @brief 将四轮电流命令换算为 M3508 原始控制量并发送。
     * @param[in] current 四轮目标电流，单位 A；空指针按零电流处理。
     */
    void sendCurrents(const float current[kWheelCount]);

    /** @brief 向四个轮电机发送零电流命令。 */
    void sendZero();

    /** @brief 清除闭环历史状态并发送零电流命令。 */
    void stop();

    // 调试快照：可直接在 Live Watch 中展开 g_chassisController。
    /** @brief 查询四轮力分配矩阵是否构建成功。 */
    bool forceReady() const
    {
        return force_ready_;
    }

    /** @brief 查询轮速/IMU 运动补偿器是否初始化成功。 */
    bool motionReady() const
    {
        return motion_ready_;
    }

    /** @brief 查询重力前馈器是否初始化成功。 */
    bool gravityReady() const
    {
        return gravity_ready_;
    }

    /** @brief 查询本周期是否使用了有效 IMU 加速度。 */
    bool motionUsingAcceleration() const
    {
        return motion_using_acceleration_;
    }

    /** @brief 获取运动补偿器估计的底盘 x 向加速度，单位 m/s²。 */
    float motionAccelerationX() const
    {
        return motion_acceleration_x_;
    }

    /** @brief 获取运动补偿器估计的底盘 y 向加速度，单位 m/s²。 */
    float motionAccelerationY() const
    {
        return motion_acceleration_y_;
    }

    /** @brief 最近一次反馈快照，可在调试器中查看打滑分数和置信度。 */
    ChassisFeedback latest_feedback;

private:
    /** @brief 清空速度环积分并关闭力分配输出，不发送 CAN 命令。 */
    void resetControlState();

    DjiMotor wheel_motor_[kWheelCount] {};
    ForceSolver force_solver_ {};
    PID vx_pid_ {};
    PID vy_pid_ {};
    PID wz_pid_ {};
    motion_compensation::OmniMotionCompensator motion_compensator_ {};
    gravity_compensation::GravityCompensator gravity_compensator_ {};

    float vx_filtered_ = 0.0f;
    float vy_filtered_ = 0.0f;
    float wz_filtered_ = 0.0f;
    float motion_acceleration_x_ = 0.0f;
    float motion_acceleration_y_ = 0.0f;
    bool force_ready_ = false;
    bool motion_ready_ = false;
    bool gravity_ready_ = false;
    bool motion_using_acceleration_ = false;
};

extern ChassisController g_chassisController;

} // namespace chassis_app

#endif // CHASSIS_CONTROLLER_HPP
