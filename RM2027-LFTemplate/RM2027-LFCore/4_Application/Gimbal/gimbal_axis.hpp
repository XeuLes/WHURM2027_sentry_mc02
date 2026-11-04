#pragma once
#include "gimbal_config.h"
#include "dvc_dji_motor.hpp"
#include "alg_smc.hpp"
#include "remote_control.h"
#include <math.h>

namespace gimbal { namespace detail {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kDt = GIMBAL_PERIOD_MS * 0.001f;
static_assert(GIMBAL_PERIOD_MS > 0U, "gimbal period must be positive");
static_assert(GIMBAL_FOLLOW_EXIT_RAD < GIMBAL_FOLLOW_ENTER_RAD &&
    GIMBAL_FOLLOW_ENTER_RAD < GIMBAL_SMALL_YAW_MAX_RAD &&
    GIMBAL_FOLLOW_ENTER_RAD < -GIMBAL_SMALL_YAW_MIN_RAD,
    "follow hysteresis must lie inside small yaw limits");
static_assert(GIMBAL_PITCH_MIN_RAD < 0 && GIMBAL_PITCH_MAX_RAD > 0,
    "pitch zero must lie inside measured limits");
static_assert(GIMBAL_PASS_ROLL_RAD >= GIMBAL_ROLL_MIN_RAD &&
    GIMBAL_PASS_ROLL_RAD <= GIMBAL_ROLL_MAX_RAD &&
    GIMBAL_ROLL_UPRIGHT_RAD >= GIMBAL_ROLL_MIN_RAD &&
    GIMBAL_ROLL_UPRIGHT_RAD <= GIMBAL_ROLL_MAX_RAD,
    "roll poses must lie inside measured limits");
static_assert(GIMBAL_PASS_PITCH_RAD >= GIMBAL_PITCH_MIN_RAD &&
    GIMBAL_PASS_PITCH_RAD <= GIMBAL_PITCH_MAX_RAD,
    "passage pitch must lie inside measured limits");
static_assert(GIMBAL_ROLL_SPEED_RAD_S > 0 && GIMBAL_ROLL_ACCEL_RAD_S2 > 0 &&
    GIMBAL_PASS_JOINT_SPEED_RAD_S > 0 && GIMBAL_PASS_JOINT_ACCEL_RAD_S2 > 0 &&
    GIMBAL_PASS_SETTLE_MS > 0 && GIMBAL_PASS_PHASE_TIMEOUT_MS > GIMBAL_PASS_SETTLE_MS,
    "passage motion parameters must be positive");

float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(x, hi)); }
float wrap(float x) { return remainderf(x, 2.0f * kPi); }
float stick(int16_t raw) {
    const float x = clamp(static_cast<float>(raw) * RC_TO_ONE, -1.0f, 1.0f);
    return fabsf(x) < GIMBAL_RC_DEADZONE ? 0.0f : x;
}

// Joint-space reference with bounded speed and acceleration. Reset from
// measured position/velocity when reversing a partially completed movement.
struct JointTrajectory {
    float position = 0, velocity = 0;
    void reset(float p, float v = 0) { position = p; velocity = v; }
    float advance(float goal, float speed, float accel) {
        const float delta = goal - position;
        // Discrete braking margin; do not snap velocity on reversal/arrival.
        const float desired = copysignf(fminf(speed, fmaxf(0.0f,
            sqrtf(accel * accel * kDt * kDt + 2 * accel * fabsf(delta)) - accel * kDt)), delta);
        const float old_velocity = velocity;
        velocity += clamp(desired - velocity, -accel * kDt, accel * kDt);
        position += (old_velocity + velocity) * 0.5f * kDt;
        return position;
    }
};

struct Axis {
    DjiMotor motor;
    Sliding smc{};
    uint16_t mid = 0;
    float joint = 0, origin = 0, target = 0, output_max = 0;
    bool aligned = false;
    bool init(MotorType type, FDCAN_HandleTypeDef* bus, uint8_t id,
              int8_t direction, float ratio, uint16_t midpoint,
              const float (&params)[9], Rmode law, float max_output) {
        if (midpoint >= kDjiEncoderRange || !isfinite(max_output) ||
            max_output <= 0 || max_output > 25000.0f) { return false; }
        mid = midpoint;
        output_max = max_output;
        smcInit(&smc, params, law, GIMBAL_SMC_SAT_LIMIT,
                max_output, GIMBAL_POSITION_EPS);
        return motor.init(type, id, bus, direction, ratio) &&
               MotorBase::registerMotor(motor);
    }
    bool observe(uint32_t now) {
        const DjiMotorData d = motor.data();
        if (!motor.isOnline(now) || !d.initialized || d.error_code != 0U ||
            !isfinite(d.total_angle_rad) || !isfinite(d.velocity_rad_s)) {
            return false;
        }
        if (!aligned) {
            const float raw = (static_cast<float>(d.encoder) - mid) *
                              (2.0f * kPi / kDjiEncoderRange);
            origin = wrap(raw) * motor.direction() / motor.reductionRatio() -
                     d.total_angle_rad;
            aligned = true;
        }
        joint = origin + d.total_angle_rad;
        return isfinite(joint);
    }
    void reset(float position) {
        // smcClear also erases tolerances; reconstruct to preserve config.
        const SlidingParam p = smc.param;
        const Rmode law = smc.flag;
        const float params[9] = {p.J,p.K,p.c,p.c1,p.c2,p.p,p.q,p.beta,p.epsilon};
        smc = Sliding{};
        smcInit(&smc, params, law, GIMBAL_SMC_SAT_LIMIT,
                output_max, GIMBAL_POSITION_EPS);
        target = position;
        smc.error.tar_last = position;
        smc.error.tar_now = position;
        motor.setCommand(0);
    }
    bool control(float position, float rate, bool limited, float lo, float hi) {
        smcPosErrorUpdate(&smc, target, position, rate, kDt);
        smc.error.p_error_integral = clamp(smc.error.p_error_integral,
            -GIMBAL_INTEGRAL_MAX, GIMBAL_INTEGRAL_MAX);
        float u = smcCalc(&smc);
        if (!isfinite(u)) { motor.setCommand(0); return false; }
        if (limited && ((joint <= lo && u < 0) || (joint >= hi && u > 0))) {
            u = 0;
            smcIntgValClear(&smc);
        }
        motor.setCommand(static_cast<int16_t>(clamp(u, -output_max, output_max)));
        return true;
    }
};

} } // namespace gimbal::detail
