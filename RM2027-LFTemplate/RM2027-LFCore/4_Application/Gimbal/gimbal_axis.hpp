#pragma once
#include "gimbal_config.h"
#include "dvc_dji_motor.hpp"
#include "dvc_dm_motor.hpp"
#include "alg_smc.hpp"
#include "remote_control.h"
#include <math.h>

namespace gimbal { namespace detail {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kDt = GIMBAL_PERIOD_MS * 0.001f;
inline float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(x, hi)); }
inline float wrap(float x) { return remainderf(x, 2.0f * kPi); }
inline float stick(int16_t raw) {
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
    Sliding smc{};
    float joint = 0, velocity = 0, target = 0, output = 0, output_max = 0;
    void initController(const float (&params)[9], Rmode law, float max_output) {
        output_max = max_output;
        smcInit(&smc, params, law, GIMBAL_SMC_SAT_LIMIT,
                max_output, GIMBAL_POSITION_EPS);
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
        output = 0;
    }
    bool control(float position, float rate, bool limited, float lo, float hi,
                 bool circular = false, float feedforward = 0.0f) {
        if (circular) {
            // Use the nearest equivalent angles for both error and target rate.
            position = target + wrap(position - target);
            smc.error.tar_last = target - wrap(target - smc.error.tar_last);
        }
        smcPosErrorUpdate(&smc, target, position, rate, kDt);
        smc.error.p_error_integral = clamp(smc.error.p_error_integral,
            -GIMBAL_INTEGRAL_MAX, GIMBAL_INTEGRAL_MAX);
        float u = smcCalc(&smc) + feedforward;
        if (!isfinite(u)) { output = 0; return false; }
        if (limited && ((joint <= lo && u < 0) || (joint >= hi && u > 0))) {
            u = 0;
            smcIntgValClear(&smc);
        }
        output = clamp(u, -output_max, output_max);
        return true;
    }
};

inline bool jointSettled(const Axis& axis, float goal) {
    return fabsf(axis.joint - goal) < GIMBAL_PASS_POSITION_TOL_RAD &&
           fabsf(axis.velocity) < GIMBAL_PASS_SPEED_TOL_RAD_S;
}

struct DjiAxis : Axis {
    DjiMotor motor;
    uint16_t mid = 0;

    bool init(FDCAN_HandleTypeDef* bus, uint8_t id, int8_t direction, uint16_t midpoint,
              const float (&params)[9], Rmode law, float max_output) {
        if (midpoint >= kDjiEncoderRange || !isfinite(max_output) ||
            max_output <= 0 || max_output > 25000.0f) { return false; }
        mid = midpoint;
        initController(params, law, max_output);
        if (!motor.init(MotorType::DjiGm6020, id, bus, direction, 1.0f)) { return false; }
        // This task owns the entire DJI group frame.
        for (uint8_t i = 0; i < MotorBase::motorCount(); ++i) {
            const auto* other = MotorBase::motorAt(i);
            if (other->bus() == bus && other->txIdentifier() == motor.txIdentifier()) {
                return false;
            }
        }
        return MotorBase::registerMotor(motor);
    }

    bool observe(uint32_t now) {
        const DjiMotorData d = motor.data();
        if (!motor.isOnline(now) || !d.initialized || d.error_code != 0U ||
            d.encoder >= kDjiEncoderRange || !isfinite(d.velocity_rad_s)) { return false; }
        const float offset = (static_cast<float>(d.encoder) - mid) *
                             (2.0f * kPi / kDjiEncoderRange);
        joint = wrap(offset) * motor.direction();
        velocity = d.velocity_rad_s;
        return true;
    }
};

struct DmAxis : Axis {
    DmMotor motor;
    float mid = 0;
    bool ready = false;
    bool command_sent = false, enable_requested = false;
    uint32_t command_tick = 0, enable_rx_count = 0;

    bool init(FDCAN_HandleTypeDef* bus, uint8_t id, uint32_t master_id, int8_t direction,
              float midpoint, const float (&params)[9], Rmode law, float max_torque) {
        const DmParamRange range{GIMBAL_DM_POS_MAX_RAD, GIMBAL_DM_VEL_MAX_RAD_S,
                                 GIMBAL_DM_TORQUE_MAX_NM};
        if (!isfinite(midpoint) || fabsf(midpoint) > range.position_max ||
            !isfinite(max_torque) || max_torque <= 0 || max_torque > range.torque_max) {
            return false;
        }
        if (!motor.init(id, bus, DmControlMode::Mit, range, id, master_id, direction)) {
            return false;
        }
        // DM compares the low eight bits of the CAN identifier.
        if ((master_id & 0xffU) == id) { return false; }
        for (uint8_t i = 0; i < MotorBase::motorCount(); ++i) {
            const auto* other = MotorBase::motorAt(i);
            if (other->bus() != bus) { continue; }
            if ((other->txIdentifier() & 0xffU) == id ||
                (other->rxIdentifier() & 0xffU) == id ||
                (other->type() == MotorType::Dm &&
                 (master_id & 0xffU) == (other->txIdentifier() & 0xffU))) { return false; }
        }
        mid = midpoint * motor.direction();
        initController(params, law, max_torque);
        return MotorBase::registerMotor(motor);
    }

    bool observe(uint32_t now) {
        const DmMotorData d = motor.data();
        const bool healthy = motor.isOnline(now) && d.initialized && d.state <= 1U &&
                             isfinite(d.position_rad) && isfinite(d.velocity_rad_s);
        ready = healthy && d.state == 1U && enable_requested &&
                (ready || motor.rxCount() != enable_rx_count);
        if (healthy) {
            joint = wrap(d.position_rad - mid);
            velocity = d.velocity_rad_s;
        }
        return healthy;
    }

    bool send(uint32_t now, bool enable, float torque) {
        if (enable && ready) {
            DmControl command{};
            command.torque_nm = torque;
            return motor.sendMit(command);
        }
        if (!enable) { ready = false; }
        if (command_sent && enable == enable_requested &&
            now - command_tick < GIMBAL_DM_COMMAND_PERIOD_MS) { return true; }
        // Disabled drives also need a command to return status and position.
        const uint32_t rx_count = motor.rxCount();
        const auto command = enable ? DmCommand::Enable : DmCommand::Disable;
        if (!motor.sendCommand(command)) { return false; }
        if (enable && !enable_requested) { enable_rx_count = rx_count; }
        enable_requested = enable;
        command_tick = now;
        command_sent = true;
        return true;
    }
};

} } // namespace gimbal::detail
