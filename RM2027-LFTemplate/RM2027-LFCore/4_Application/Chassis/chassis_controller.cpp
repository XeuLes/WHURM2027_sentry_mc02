/**
 * @file chassis_controller.cpp
 * @brief 四轮全向底盘控制实现。
 */

#include "chassis_controller.hpp"

#include "alg_basic.hpp"
#include "chassis_config.hpp"
#include "fdcan.h"

#include <cmath>

namespace chassis_app {

ChassisController g_chassisController;

namespace {

/**
 * @brief 由有效轮速估计底盘旋转角速度。
 * @param[in] feedback 包含轮缘线速度和有效轮位图的反馈快照。
 * @return 底盘旋转角速度，单位 rad/s；无有效轮时返回 0。
 * @note 全向轮纯旋转时各轮轮缘线速度约为 -R*wz。
 */
float wheelAngularVelocity(
    const ChassisFeedback& feedback
)
{
    float sum = 0.0f;
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        if ((feedback.valid_wheel_mask & static_cast<uint8_t>(1U << i)) != 0U) {
            sum += feedback.wheel_linear[i];
            ++count;
        }
    }
    if (count == 0U) {
        return 0.0f;
    }

    // 纯旋转时各轮轮缘线速度为 -R*wz。
    return -sum / (static_cast<float>(count) * kWheelRadius);
}

} // namespace

/**
 * @brief 初始化电机、运动估计、速度 PID 和力分配链路。
 * @return 四个 M3508 均配置并注册成功时返回 true。
 */
bool ChassisController::init()
{
    ForceActuatorParam params[kWheelCount] = {};
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        if (!wheel_motor_[i].init(
                MotorType::DjiM3508,
                kWheelMotorId[i],
                &hfdcan1,
                kWheelDirection[i]
            )
            || !MotorBase::registerMotor(wheel_motor_[i])) {
            return false;
        }
        wheel_motor_[i].setOfflineTimeout(kOfflineTimeoutMs);

        params[i].torque_constant = kWheelTorquePerAmp;
        params[i].reduction_ratio = kForceReductionRatio;
        params[i].direction = kForceDirection;
        params[i].torque_max = kWheelTorqueMax;
        params[i].inertia = kForceInertia;
        params[i].viscous_friction = kForceViscousFriction;
        params[i].coulomb_friction = kForceCoulombFriction;
        params[i].gravity_amplitude = kForceGravityAmplitude;
        params[i].gravity_offset = kForceGravityOffset;
    }

    force_solver_.init(params, kWheelCount, kForceDof, kControlPeriodS);
    force_solver_.setVelocityFilterAlpha(kForceVelocityFilterAlpha);

    float distribution[kWheelCount * kForceDof] = {};
    force_ready_ = ForceSolver::buildOmniDistribution(
        kWheelRadius,
        kWheelCenterDistance,
        distribution
    );
    if (force_ready_) {
        force_solver_.setDistribution(distribution);
    }

    motion_compensation::Config motion_config =
        motion_compensation::defaultConfig();
    motion_config.slip_detection_enabled = kEnableSlipDetection;
    motion_config.slip_min_speed_mps = kSlipMinSpeedMps;
    motion_config.slip_score_enter = kSlipScoreEnter;
    motion_config.slip_score_exit = kSlipScoreExit;
    motion_config.slip_score_filter_alpha = kSlipScoreFilterAlpha;
    motion_config.slip_enter_hold_s = kSlipEnterHoldS;
    motion_config.slip_exit_hold_s = kSlipExitHoldS;
    motion_ready_ = motion_compensator_.init(motion_config);
    motion_using_acceleration_ = false;
    motion_acceleration_x_ = 0.0f;
    motion_acceleration_y_ = 0.0f;

    gravity_compensation::Config gravity_config {};
    gravity_config.enabled = kEnableGravityCompensation;
    gravity_config.mass_kg = kGravityCompensationMassKg;
    gravity_config.gravity_mps2 = kGravityMps2;
    gravity_config.scale_x = kGravityScaleX;
    gravity_config.scale_y = kGravityScaleY;
    gravity_config.filter_alpha = kGravityFilterAlpha;
    gravity_config.slew_rate_nps = kGravitySlewRateNps;
    gravity_config.max_force_n = kGravityMaxForceN;
    gravity_config.max_tilt_deg = kGravityMaxTiltDeg;
    gravity_ready_ = gravity_compensator_.init(gravity_config);

    vx_pid_.init(
        kPidVxKp, kPidVxKi, 0.0f, 0.0f,
        20.0f, kMaxFx, kControlPeriodS,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );
    vy_pid_.init(
        kPidVyKp, kPidVyKi, 0.0f, 0.0f,
        20.0f, kMaxFy, kControlPeriodS,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );
    wz_pid_.init(
        kPidWzKp, kPidWzKi, 0.0f, 0.0f,
        4.0f, kMaxMz, kControlPeriodS,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );

    vx_filtered_ = 0.0f;
    vy_filtered_ = 0.0f;
    wz_filtered_ = 0.0f;
    sendZero();
    return true;
}

/**
 * @brief 读取轮电机和 IMU，生成本周期底盘速度反馈。
 * @note 融合可用时直接采用运动补偿结果，否则退回轮速一阶低通。
 */
ChassisFeedback ChassisController::readFeedback(
    const LuojiaFox::ImuState& imu,
    uint32_t now_ms
)
{
    ChassisFeedback feedback {};
    feedback.all_wheels_online = true;

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        const DjiMotorData data = wheel_motor_[i].data();
        const bool online = wheel_motor_[i].isOnline(now_ms);
        feedback.wheel_omega[i] = data.velocity_rad_s;
        feedback.wheel_linear[i] = kWheelRadius * data.velocity_rad_s;
        feedback.all_wheels_online = feedback.all_wheels_online && online;

        // 在线仅代表 CAN 反馈未超时；有限值检查阻止 NaN/Inf 进入轮速逆解。
        // bit i 对应逻辑轮序 i，供运动补偿器筛掉不可用轮速。
        if (online && std::isfinite(data.velocity_rad_s)) {
            feedback.valid_wheel_mask |=
                static_cast<uint8_t>(1U << i);
        }
    }

    const bool imu_fresh =
        imu.last_update_ms != 0U
        && now_ms - imu.last_update_ms <= kImuDataTimeoutMs;

    const gravity_compensation::Output gravity =
        gravity_compensator_.update(
            imu.pitch_deg,
            imu.roll_deg,
            imu_fresh,
            kControlPeriodS
        );
    feedback.gravity_accel_x = gravity.gravity_accel_x_mps2;
    feedback.gravity_accel_y = gravity.gravity_accel_y_mps2;
    feedback.gravity_force_x = gravity.force_x_n;
    feedback.gravity_force_y = gravity.force_y_n;
    feedback.gravity_valid = gravity.valid;
    feedback.gravity_applied = gravity.applied;

    motion_compensation::Measurement measurement {};
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        measurement.wheel_linear_velocity_mps[i] = feedback.wheel_linear[i];
    }
    measurement.valid_wheel_mask = feedback.valid_wheel_mask;
    measurement.yaw_rad = 0.0f;
    measurement.yaw_rate_rad_s =
        imu_fresh ? imu.yaw_rate_dps * BASIC_MATH_DEG_TO_RAD : 0.0f;
    measurement.yaw_rate_valid = imu_fresh;
    measurement.wheel_radius_m = kWheelRadius;
    measurement.acceleration_x_mps2 =
        imu_fresh ? imu.acceleration_x_mps2 : 0.0f;
    measurement.acceleration_y_mps2 =
        imu_fresh ? imu.acceleration_y_mps2 : 0.0f;
    measurement.acceleration_valid =
        imu_fresh && imu.acceleration_valid;
    measurement.dt_s = kControlPeriodS;
    feedback.acceleration_valid = measurement.acceleration_valid;

    feedback.motion_updated =
        motion_ready_ && motion_compensator_.update(measurement);
    if (feedback.motion_updated) {
        const motion_compensation::Estimate estimate =
            motion_compensator_.estimate();
        feedback.vx = estimate.vx_mps;
        feedback.vy = estimate.vy_mps;
        feedback.velocity_estimate_valid =
            estimate.velocity_measurement_valid;
        motion_using_acceleration_ = measurement.acceleration_valid;
        motion_acceleration_x_ = estimate.ax_mps2;
        motion_acceleration_y_ = estimate.ay_mps2;
        feedback.predicted_vx = estimate.predicted_vx_mps;
        feedback.predicted_vy = estimate.predicted_vy_mps;
        feedback.slip_wheel_mask = estimate.slip_wheel_mask;
        feedback.slip_detection_valid = estimate.slip_detection_valid;
        feedback.slip_detected = estimate.slip_detected;
        for (uint8_t i = 0U; i < kWheelCount; ++i) {
            feedback.wheel_slip_velocity[i] =
                estimate.slip_velocity_mps[i];
            feedback.wheel_slip_score[i] = estimate.slip_score[i];
            feedback.wheel_confidence[i] = estimate.slip_confidence[i];
        }
    } else {
        float raw_vx = 0.0f;
        float raw_vy = 0.0f;
        feedback.velocity_estimate_valid =
            motion_compensation::OmniMotionCompensator::solveVelocity(
                feedback.wheel_linear,
                feedback.valid_wheel_mask,
                0.0f,
                &raw_vx,
                &raw_vy
            );

        if (feedback.velocity_estimate_valid) {
            feedback.vx = kVelocityFilterAlpha * vx_filtered_
                + (1.0f - kVelocityFilterAlpha) * raw_vx;
            feedback.vy = kVelocityFilterAlpha * vy_filtered_
                + (1.0f - kVelocityFilterAlpha) * raw_vy;
        }
        motion_using_acceleration_ = false;
        motion_acceleration_x_ = 0.0f;
        motion_acceleration_y_ = 0.0f;
    }

    if (feedback.velocity_estimate_valid) {
        feedback.wz = kVelocityFilterAlpha * wz_filtered_
            + (1.0f - kVelocityFilterAlpha) * wheelAngularVelocity(feedback);
    }

    vx_filtered_ = feedback.vx;
    vy_filtered_ = feedback.vy;
    wz_filtered_ = feedback.wz;
    latest_feedback = feedback;
    return feedback;
}

/**
 * @brief 清除闭环动态状态，防止重新使能时继承旧积分或力矩。
 */
void ChassisController::resetControlState()
{
    vx_pid_.setIntegralError(0.0f);
    vy_pid_.setIntegralError(0.0f);
    wz_pid_.setIntegralError(0.0f);
    force_solver_.setEnable(false);
}

/**
 * @brief 执行速度闭环、目标补偿和四轮力分配。
 * @note 任一安全条件失败时输出零电流并清除闭环状态。
 */
bool ChassisController::calculate(
    const ChassisCommand& command,
    const ChassisFeedback& feedback,
    float current[kWheelCount]
)
{
    if (current == nullptr) {
        resetControlState();
        return false;
    }

    if (!command.ready || !feedback.velocity_estimate_valid
        || !feedback.all_wheels_online || !force_ready_) {
        resetControlState();
        for (uint8_t i = 0U; i < kWheelCount; ++i) {
            current[i] = 0.0f;
        }
        return false;
    }

    float target_vx = command.vx;
    float target_vy = command.vy;
    if (kEnableMotionTargetCompensation
        && motion_compensator_.isInitialized()) {
        const motion_compensation::CorrectedVelocity corrected =
            motion_compensator_.compensate(target_vx, target_vy);
        target_vx = basic_math::constrain(corrected.vx_mps, -kMaxVx, kMaxVx);
        target_vy = basic_math::constrain(corrected.vy_mps, -kMaxVy, kMaxVy);
    }

    vx_pid_.setTarget(target_vx);
    vy_pid_.setTarget(target_vy);
    wz_pid_.setTarget(command.wz);

    vx_pid_.setNow(feedback.vx);
    vy_pid_.setNow(feedback.vy);
    wz_pid_.setNow(feedback.wz);

    vx_pid_.timCalculatePeriodElapsedCallback();
    vy_pid_.timCalculatePeriodElapsedCallback();
    wz_pid_.timCalculatePeriodElapsedCallback();

    const float generalized_force[kForceDof] = {
        basic_math::constrain(
            vx_pid_.getOut() + feedback.gravity_force_x,
            -kMaxFx,
            kMaxFx
        ),
        basic_math::constrain(
            vy_pid_.getOut() + feedback.gravity_force_y,
            -kMaxFy,
            kMaxFy
        ),
        basic_math::constrain(wz_pid_.getOut(), -kMaxMz, kMaxMz)
    };

    force_solver_.setTarget(generalized_force);
    force_solver_.setNow(nullptr, feedback.wheel_omega);
    force_solver_.setEnable(true);
    force_solver_.timCalculatePeriodElapsedCallback();

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        current[i] = basic_math::constrain(
            force_solver_.getOut(i),
            -kC620MaxCurrent,
            kC620MaxCurrent
        );
    }
    return true;
}

/**
 * @brief 将安培电流换算为 C620 原始控制量并发送共享 CAN 帧。
 * @note 空指针会安全回退为零电流发送。
 */
void ChassisController::sendCurrents(const float current[kWheelCount])
{
    if (current == nullptr) {
        sendZero();
        return;
    }

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        const float raw = basic_math::constrain(
            current[i] * (16384.0f / kC620MaxCurrent),
            -16384.0f,
            16384.0f
        );
        wheel_motor_[i].setCommand(static_cast<int16_t>(raw));
    }
    (void)DjiMotor::sendGroup(&hfdcan1, kChassisTxId);
}

/** @brief 发送零目标电流。 */
void ChassisController::sendZero()
{
    const float zero[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    sendCurrents(zero);
}

/** @brief 清除控制历史并发送零电流。 */
void ChassisController::stop()
{
    resetControlState();
    sendZero();
}

} // namespace chassis_app
