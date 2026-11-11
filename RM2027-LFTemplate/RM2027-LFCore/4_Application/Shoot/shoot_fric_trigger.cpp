// 发射任务移植自 WHURM2026_SENTRY 的 shoot_task.c / shoot_fric_trigger.c。
// 参考实现 Copyright (C) 2024 Polarbear, 2026 LuojiaFox。
#include "shoot_config.hpp"

#include "alg_basic.hpp"
#include "alg_smc.hpp"
#include "app_board_link.hpp"
#include "app_gimbal.h"
#include "dvc_dji_motor.hpp"
#include "lf_core.hpp"
#include "referee.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

namespace {
using namespace shoot_config;

constexpr size_t kFricCount = sizeof(kFricIds) / sizeof(kFricIds[0]);
constexpr uint32_t kTxIdentifier = 0x200U;
constexpr float kTwoPi = 6.28318530718f;
constexpr float kAnglePerBullet = kTwoPi / kBulletNum;
constexpr float kFricSmcParams[] = SHOOT_FRIC_SMC_PARAMS;
constexpr float kTriggerSmcParams[] = SHOOT_TRIGGER_SMC_PARAMS;

enum LoadMode { LOAD_STOP, LOAD_BULLET, LOAD_NO_LIMIT, LOAD_BURSTFIRE, LOAD_BLOCK };
enum FricState { FRIC_NOT_READY, FRIC_ON, FRIC_READY };

struct ShootFeedback {
    float trigger_angle;
    float trigger_speed;
    float fric_speed[kFricCount];
};

struct SpeedLoop {
    Sliding smc{};
    float applied_ref = 0.0f;
};

struct ShootState {
    DjiMotor trigger_motor;
    DjiMotor fric_motor[kFricCount];
    SpeedLoop trigger_loop;
    SpeedLoop fric_loop[kFricCount];
    ShootFeedback fdb{};
    ShootFeedback ref{};

    LoadMode mode = LOAD_STOP;
    FricState state = FRIC_NOT_READY;
    bool motors_online = false;
    bool inhibited = true;
    bool stop_requested = true;
    bool automatic = false;
    bool fire = false;
    bool sentry_fric_cmd = false;
    bool fric_at_speed = false;
    bool heat_fresh = false;
    bool move_flag = false;
    uint16_t cooling_value = 0U;
    uint16_t heat_limit = 0U;
    uint16_t heat = 0U;
    uint32_t aim_lost_time = 0U;
    uint32_t block_time = 0U;
    uint32_t reverse_time = 0U;
    uint32_t shoot_time = 0U;
    float shoot_window_ms = 0.0f;
    float shoot_speed = 0.0f; // 发/s
    uint32_t tx_failures = 0U;
};

ShootState SHOOT;

void InitSpeedLoop(SpeedLoop& loop, const float (&params)[9])
{
    smcInit(&loop.smc, params, VELSMC, kSmcSatLimit, DJI_M2006_MAX_COMMAND, 0.0f);
}

void ResetSpeedLoop(SpeedLoop& loop)
{
    smcClear(&loop.smc);
    loop.smc.s = 0.0f;
    loop.smc.u = 0.0f;
    loop.applied_ref = 0.0f;
}

bool ShootInit()
{
    auto* bus = motorBus();
    if (!SHOOT.trigger_motor.init(MotorType::DjiM2006, kTriggerId, bus,
                                 kTriggerDirection, kTriggerReduction)) {
        return false;
    }
    for (size_t i = 0; i < kFricCount; ++i) {
        if (!SHOOT.fric_motor[i].init(MotorType::DjiM2006, kFricIds[i], bus,
                                     kFricDirections[i], kFricReduction)) {
            return false;
        }
    }
    InitSpeedLoop(SHOOT.trigger_loop, kTriggerSmcParams);
    for (SpeedLoop& loop : SHOOT.fric_loop) {
        InitSpeedLoop(loop, kFricSmcParams);
    }

    // 四个电机独占同一组发送帧。先检查整组，避免注册到一半发生冲突。
    DjiMotor* const motors[] = {
        &SHOOT.trigger_motor, &SHOOT.fric_motor[0], &SHOOT.fric_motor[1], &SHOOT.fric_motor[2]
    };
    MotorCriticalSection lock;
    const uint8_t registered = MotorBase::motorCount();
    if (registered + sizeof(motors) / sizeof(motors[0]) > MotorBase::kMaxMotors) {
        return false;
    }
    uint8_t occupied_slots = 0U;
    for (const DjiMotor* motor : motors) {
        if (motor->txIdentifier() != kTxIdentifier) {
            return false;
        }
        const uint8_t slot = 1U << motor->txSlot();
        if (occupied_slots & slot) {
            return false;
        }
        occupied_slots |= slot;
        for (uint8_t i = 0U; i < registered; ++i) {
            const MotorBase* other = MotorBase::motorAt(i);
            if (other->bus() == bus &&
                (other->txIdentifier() == kTxIdentifier || other->rxIdentifier() == kTxIdentifier ||
                 other->rxIdentifier() == motor->rxIdentifier() ||
                 other->txIdentifier() == motor->rxIdentifier())) {
                return false;
            }
        }
    }
    for (DjiMotor* motor : motors) {
        if (!MotorBase::registerMotor(*motor)) {
            return false;
        }
    }
    return true;
}

bool sentry_fric_ready()
{
    for (size_t i = 0; i < kFricCount; ++i) {
        if (fabsf(SHOOT.fdb.fric_speed[i] - kFricSpeedRadS[i]) > kFricSpeedErrorRadS) {
            return false;
        }
    }
    return true;
}

void ShootObserver()
{
    const auto system = LuojiaFox::system_state.read();
    const auto power = referee.GetPowerSupplySnapshot();
    uint32_t now;
    const bool peer_online = board_link::isPeerOnline();
    bool rc_online;
    char sw;
    {
        MotorCriticalSection lock;
        now = HAL_GetTick();
        rc_online = !getRcOffline();
        sw = getDt7RcSw(SHOOT_MODE_CHANNEL);
        SHOOT.stop_requested = rc_online &&
            (SWITCH_IS_DOWN(getDt7RcSw(RC_SW_RIGHT)) || (!SWITCH_IS_UP(sw) && !SWITCH_IS_MID(sw)));

        const auto trigger = SHOOT.trigger_motor.data();
        SHOOT.fdb.trigger_angle = trigger.total_angle_rad;
        SHOOT.fdb.trigger_speed = trigger.velocity_rad_s;
        SHOOT.motors_online = SHOOT.trigger_motor.isOnline(now);
        SHOOT.fric_at_speed = true;
        for (size_t i = 0; i < kFricCount; ++i) {
            SHOOT.fdb.fric_speed[i] = SHOOT.fric_motor[i].data().velocity_rad_s;
            SHOOT.motors_online = SHOOT.fric_motor[i].isOnline(now) && SHOOT.motors_online;
            SHOOT.fric_at_speed = SHOOT.fdb.fric_speed[i] >= kFricSpeedLimitRadS &&
                                  SHOOT.fric_at_speed;
        }
    }
    // 过通道时小云台会折叠，发射任务让出机构。
    SHOOT.stop_requested = SHOOT.stop_requested ||
        system.gimbal.mode == static_cast<uint8_t>(gimbal::Mode::kPassage) ||
        system.gimbal.mode == static_cast<uint8_t>(gimbal::Mode::kFault);
    SHOOT.automatic = !rc_online || SWITCH_IS_MID(sw);
    SHOOT.fire = !system.usb_link.offline && system.robot_cmd.last_update_ms != 0U &&
        now - system.robot_cmd.last_update_ms < kRobotCmdTimeoutMs && system.robot_cmd.fire != 0U;

    // 中档与遥控离线共用 SENTRY 的自动分支，丢火控后维持摩擦轮 3 s。
    if (SHOOT.automatic && !SHOOT.fire) {
        if (SHOOT.aim_lost_time < kAimLostMs) {
            SHOOT.aim_lost_time += kControlMs;
        }
    } else {
        SHOOT.aim_lost_time = 0U;
    }
    SHOOT.sentry_fric_cmd = SHOOT.aim_lost_time < kAimLostMs;

    SHOOT.heat_fresh = peer_online && power.online && power.power_received &&
        power.buffer_received && now - power.power_tick < kRefereeTimeoutMs &&
        now - power.buffer_tick < kRefereeTimeoutMs && power.shooter_heat_limit != 0U;
    SHOOT.cooling_value = power.shooter_cooling_value;
    SHOOT.heat_limit = power.shooter_heat_limit;
    SHOOT.heat = power.shooter_17mm_heat;
}

void ShootHandleException()
{
    const bool inhibited = !SHOOT.motors_online;
    if (inhibited && !SHOOT.inhibited) {
        ResetSpeedLoop(SHOOT.trigger_loop);
        for (SpeedLoop& loop : SHOOT.fric_loop) {
            ResetSpeedLoop(loop);
        }
    }
    SHOOT.inhibited = inhibited;
}

void ShootSetMode()
{
    SHOOT.state = FRIC_NOT_READY;
    SHOOT.mode = LOAD_STOP;
    if (SHOOT.inhibited || SHOOT.stop_requested) {
        return;
    }

    if (SHOOT.automatic) {
        if (SHOOT.sentry_fric_cmd) {
            SHOOT.state = sentry_fric_ready() ? FRIC_READY : FRIC_ON;
        }
        if (SHOOT.fire && SHOOT.state == FRIC_READY) {
            SHOOT.mode = LOAD_BURSTFIRE;
        }
    } else {
        // 上档清弹。沿用 SENTRY 的最低轮速门槛，允许清弹时不等待精确到速。
        SHOOT.state = FRIC_READY;
        SHOOT.mode = LOAD_BURSTFIRE;
    }
    if (!SHOOT.fric_at_speed || !SHOOT.heat_fresh) {
        SHOOT.mode = LOAD_STOP;
    }
}

float FireControl()
{
    const float a = SHOOT.cooling_value;
    const float m = static_cast<float>(SHOOT.heat_limit) - SHOOT.heat;
    constexpr float d = 10.0f;

    if (m > 100.0f) {
        SHOOT.shoot_speed = 20.0f;
        return SHOOT.shoot_speed * kAnglePerBullet;
    }
    if (m <= 50.0f) {
        return a / d * kAnglePerBullet;
    }

    if (SHOOT.shoot_time == 0U) {
        SHOOT.shoot_window_ms = (m + 2.0f * a) * 10.0f;
        const float reserve = m < 100.0f ? 3.0f * d : 5.0f * d;
        SHOOT.shoot_speed = (10.0f * m - a - reserve) /
                            (d * (SHOOT.shoot_window_ms / 100.0f)) + a / d;
    }
    const bool bursting = SHOOT.shoot_time < SHOOT.shoot_window_ms;
    if (bursting) {
        SHOOT.shoot_time += kControlMs;
    }
    const float rate = bursting ? SHOOT.shoot_speed : a / d;
    return fminf(fmaxf(rate * kAnglePerBullet, 0.0f), -kTriggerSpeedRadS);
}

void HandleBlock(float forward_speed)
{
    if (SHOOT.block_time >= kBlockTimeMs) {
        if (SHOOT.reverse_time < kReverseTimeMs) {
            SHOOT.reverse_time += kControlMs;
            SHOOT.mode = LOAD_BLOCK;
            return;
        }
        SHOOT.block_time = 0U;
        SHOOT.reverse_time = 0U;
    }
    // 低射频时用实际目标限住堵转阈值，零速指令不计入堵转。
    const float threshold = fminf(kBlockTriggerSpeedRadS, fabsf(forward_speed) * 0.5f);
    if (fabsf(SHOOT.fdb.trigger_speed) < threshold) {
        SHOOT.block_time += kControlMs;
    } else {
        SHOOT.block_time = 0U;
    }
}

float SingleShotSpeed()
{
    const float error = SHOOT.ref.trigger_angle - SHOOT.fdb.trigger_angle;
    const float remaining = fmaxf(fabsf(error) - kSingleAngleToleranceRad, 0.0f);
    // 按剩余制动距离生成单发速度参考，速度闭环仍使用同一个 VELSMC。
    const float braking_speed = sqrtf(2.0f * kTriggerSlewRadS2 * remaining);
    return copysignf(fminf(braking_speed, -kTriggerSpeedRadS), error);
}

void ShootReference()
{
    for (size_t i = 0; i < kFricCount; ++i) {
        SHOOT.ref.fric_speed[i] = SHOOT.state == FRIC_NOT_READY ? 0.0f : kFricSpeedRadS[i];
    }

    // 当前 SENTRY 拨杆只选停机或限热连发。保留其单发与不限热分支。
    switch (SHOOT.mode) {
    case LOAD_BULLET:
        if (!SHOOT.move_flag) {
            SHOOT.ref.trigger_angle = SHOOT.fdb.trigger_angle - kAnglePerBullet;
            SHOOT.move_flag = true;
        }
        SHOOT.ref.trigger_speed = SingleShotSpeed();
        HandleBlock(SHOOT.ref.trigger_speed);
        break;
    case LOAD_NO_LIMIT:
        SHOOT.ref.trigger_speed = kTriggerSpeedRadS;
        SHOOT.shoot_time = 0U;
        HandleBlock(SHOOT.ref.trigger_speed);
        break;
    case LOAD_BURSTFIRE:
        SHOOT.ref.trigger_speed = -FireControl();
        HandleBlock(SHOOT.ref.trigger_speed);
        break;
    case LOAD_STOP:
        SHOOT.ref.trigger_speed = 0.0f;
        SHOOT.shoot_time = SHOOT.block_time = SHOOT.reverse_time = 0U;
        break;
    case LOAD_BLOCK:
        break;
    }

    if (SHOOT.mode == LOAD_BLOCK) {
        SHOOT.ref.trigger_speed = kReverseSpeedRadS;
        SHOOT.shoot_time = 0U;
    }
    if (SHOOT.mode != LOAD_BULLET) {
        SHOOT.move_flag = false;
    }
}

int16_t SpeedCommand(SpeedLoop& loop, float feedback, float reference, float slew)
{
    // 停止或反转时清除旧目标和积分，避免斜坡继续沿进弹方向输出。
    if (reference == 0.0f || reference * loop.applied_ref < 0.0f) {
        ResetSpeedLoop(loop);
    }
    const float max_step = slew * kControlSeconds;
    loop.applied_ref += basic_math::constrain(reference - loop.applied_ref, -max_step, max_step);
    smcVolErrorUpdate(&loop.smc, loop.applied_ref, feedback, kControlSeconds);

    const float command = smcCalc(&loop.smc);
    return isfinite(command) ? static_cast<int16_t>(command) : 0;
}

void ShootConsole()
{
    if (SHOOT.inhibited) {
        for (DjiMotor& motor : SHOOT.fric_motor) {
            motor.setCommand(0);
        }
        SHOOT.trigger_motor.setCommand(0);
        return;
    }
    for (size_t i = 0; i < kFricCount; ++i) {
        SHOOT.fric_motor[i].setCommand(SpeedCommand(
            SHOOT.fric_loop[i], SHOOT.fdb.fric_speed[i], SHOOT.ref.fric_speed[i], kFricSlewRadS2));
    }
    SHOOT.trigger_motor.setCommand(SpeedCommand(
        SHOOT.trigger_loop, SHOOT.fdb.trigger_speed, SHOOT.ref.trigger_speed, kTriggerSlewRadS2));
}

void ShootSendCmd()
{
    if (!DjiMotor::sendGroup(motorBus(), kTxIdentifier)) {
        ++SHOOT.tx_failures;
    }
}

void ShootPublish()
{
    float mean_speed = 0.0f;
    for (float speed : SHOOT.fdb.fric_speed) {
        mean_speed += fabsf(speed) / kFricCount;
    }
    const uint16_t rpm = static_cast<uint16_t>(mean_speed * 60.0f / kTwoPi);
    const bool ready = !SHOOT.inhibited && SHOOT.heat_fresh && SHOOT.fric_at_speed &&
        SHOOT.state == FRIC_READY && SHOOT.mode != LOAD_BLOCK;
    LuojiaFox::system_state.updateShooter({rpm, 0U, ready});
}

} // namespace

// 覆盖 CubeMX 的弱实现，沿用现有 Start_Shoot_Task。
extern "C" void Shoot_Task(void const* argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(kInitMs));
    if (!ShootInit()) {
        vTaskDelete(nullptr);
        return;
    }

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        ShootObserver();
        ShootHandleException();
        ShootSetMode();
        ShootReference();
        ShootConsole();
        ShootSendCmd();
        ShootPublish();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kControlMs));
    }
}
