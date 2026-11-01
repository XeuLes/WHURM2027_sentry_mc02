/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_chassis.cpp
  * @brief          : 四轮全向底盘应用实现
  * @author         : Xiaoyuhaoran
  * @date           : 2026/10/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 本文件负责底盘电机初始化、遥控速度闭环、力控分配和状态发布。
  *                    Chassis_Init() 之前必须完成三路 FDCAN 初始化。
  *                    电机反馈由 DjiMotor 和 MotorBase 的统一 CAN 分发自动处理。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_chassis.h"
#include "board_cfg.hpp"

// 数学、力控分配、PID、CAN、电机、系统状态和遥控器接口。
#include "alg_basic.hpp"
#include "alg_force.hpp"
#include "alg_motion_compensation.hpp"
#include "alg_pid.h"
#include "alg_power_control.hpp"
#include "app_supercap.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "dvc_dji_motor.hpp"
#include "lf_core.hpp"
#include "referee.hpp"
#include "remote_control.h"

namespace {

// 当前底盘固定为四轮全向轮结构。
constexpr uint8_t kWheelCount = 4U;

// 逻辑轮序与 ForceSolver/buildOmniDistribution 的矩阵行顺序一致。
// 电机 ID 和方向必须按实车接线校准；修改顺序会影响力控分配结果。
constexpr uint8_t kWheelMotorId[kWheelCount] = {2U, 3U, 4U, 1U};
constexpr int8_t kWheelDirection[kWheelCount] = {1, 1, -1, -1};

// M3508/M2006 ID1~4 共用 0x200 电流控制帧。
constexpr uint32_t kChassisTxId = 0x200U;

// 轮电机连续超过该时间没有有效反馈时，底盘立即停止输出。
constexpr uint32_t kOfflineTimeoutMs = 10U;

// 底盘机械参数，当前沿用 26 哨兵旧底盘的调试值。
constexpr float kWheelRadius = 0.050f;          // 轮半径，m
constexpr float kWheelCenterDistance = 0.250f;  // 轮中心到车体中心的距离，m
constexpr float kWheelGearRatio = 19.0f;        // 电机转子到轮输出轴减速比

// ForceSolver 的力矩是轮子输出轴力矩。
// ratio 设为 1，TorqueConstant 使用输出轴 N*m/A，
// 因此 getOut() 直接得到电机电流 A。
constexpr float kWheelTorquePerAmp = 0.150f;  // 轮输出轴 N*m/A
constexpr float kWheelTorqueMax = 2.500f;     // 单轮输出轴力矩上限，N*m
constexpr float kC620MaxCurrent = 20.0f;      // C620 满量程电流，A

// 裁判数据不可用时用于台架调试的保守上限，对应半自动哨兵基础功率。
constexpr float kFallbackChassisPowerLimitW = 60.0f;

// 规则允许短暂使用缓冲能量；控制侧保留 10% 裕量，避免模型误差持续消耗缓冲。
constexpr float kPowerSafetyMargin = 0.90f;

// 遥控摇杆满量程对应的底盘目标速度。
constexpr float kMaxVx = 1.0f;  // 最大前后速度，m/s
constexpr float kMaxVy = 1.0f;  // 最大横移速度，m/s
constexpr float kMaxWz = 2.0f;  // 最大旋转角速度，rad/s

// 速度环输出经过力控分配前允许的最大广义力。
constexpr float kMaxFx = 70.0f;  // 前后方向合力上限，N
constexpr float kMaxFy = 70.0f;  // 横移方向合力上限，N
constexpr float kMaxMz = 12.0f;  // 旋转力矩上限，N*m

// 遥控器输入处理和车体坐标方向修正。
constexpr float kRcDeadzone = 0.05f;  // 归一化摇杆死区
constexpr float kRcXSign = 1.0f;      // 前后方向符号
constexpr float kRcYSign = -1.0f;     // 横移方向符号
constexpr float kRcWzSign = -1.0f;    // 旋转方向符号

// 车体速度一阶低通系数；越大越平滑，但反馈延迟越明显。
constexpr float kVelocityFilterAlpha = 0.75f;

// 电机对象必须具有静态生命周期，注册后其地址会保存在 MotorBase 注册表中。
DjiMotor g_wheelMotor[kWheelCount];

// 将三个车体方向的速度误差转换为四个轮子的输出力矩。
ForceSolver g_chassisForce;

// 车体 vx、vy、wz 三个方向的速度闭环。
PID g_vxPid;
PID g_vyPid;
PID g_wzPid;

// 四轮功率衰减器。输入为未限幅电流，输出为满足功率预算的电流。
power_control::PowerController g_powerController;

// 调试器可直接观察的功率控制快照。
float g_effectivePowerLimitW = kFallbackChassisPowerLimitW;
float g_estimatedPowerW = 0.0f;
float g_consumedPowerW = 0.0f;
power_control::PowerControlStatus g_powerStatus =
    power_control::PowerControlStatus::Disabled;

// 经过一阶低通后的底盘速度，用于 PID 反馈和系统状态发布。
float g_vxFiltered = 0.0f;
float g_vyFiltered = 0.0f;
float g_wzFiltered = 0.0f;

// 力控分配矩阵构建成功后才允许底盘输出。
bool g_forceReady = false;

/**
 * @brief 将当前底盘速度快照发布到任务间共享状态。
 */
void PublishChassisVelocity(void) {
    LuojiaFox::system_state.updateChassis({
        g_vxFiltered,
        g_vyFiltered,
        g_wzFiltered
    });
}

/**
 * @brief 对归一化遥控器输入应用死区。
 * @param value 遥控器归一化输入。
 * @return 死区外保持原值，死区内返回 0。
 */
float ApplyDeadzone(float value) {
    return (basic_math::abs(value) < kRcDeadzone) ? 0.0f : value;
}

/**
 * @brief 清除三个速度环的积分项。
 * @note 底盘重新使能时调用，避免停车期间积分饱和造成起步冲击。
 */
void ResetVelocityPids(void) {
    g_vxPid.setIntegralError(0.0f);
    g_vyPid.setIntegralError(0.0f);
    g_wzPid.setIntegralError(0.0f);
}

/**
 * @brief 检查四个轮电机是否均在反馈超时时间内。
 * @return 四个轮电机全部在线时返回 true。
 */
bool AllWheelsOnline(void) {
    const uint32_t now = HAL_GetTick();

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        if (!g_wheelMotor[i].isOnline(now)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 获取当前有效的底盘功率上限。
 *
 * 裁判系统在 0x0201 中下发的 chassis_power_limit 已包含机器人类型、
 * 等级、姿态和已有功率增益，是比赛中的权威动态上限。超级电容模块在线时，
 * 还必须同时满足超级电容允许的功率上限。
 *
 * 裁判数据尚未建立时使用 60 W 台架回退值，避免上电阶段无限制输出。
 */
float ResolveChassisPowerLimit(void) {
    const RefereePowerSnapshot snapshot = referee.GetPowerSupplySnapshot();
    float limit = kFallbackChassisPowerLimitW;

    if (snapshot.online && snapshot.power_received) {
        // 收到有效 0x0201 后以裁判系统为准；0 W 表示当前禁止底盘输出。
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
 * @brief 按当前动态功率上限衰减四轮目标电流。
 * @param wheelOmega 四轮输出轴角速度，rad/s。
 * @param targetCurrent 力控计算得到的四轮目标电流，A。
 * @param outputCurrent 功率控制器衰减后的四轮电流，A。
 * @return 功率控制结果可信且满足预算时返回 true。
 *
 * 该函数每 1 ms 运行一次。裁判系统以 10 Hz 更新功率上限，
 * 因此这里每周期重新读取并应用最新上限。
 */
bool ApplyPowerControl(
    const float wheelOmega[kWheelCount],
    const float targetCurrent[kWheelCount],
    float outputCurrent[kWheelCount]
) {
    g_effectivePowerLimitW = ResolveChassisPowerLimit();
    power_control::powerControllerSetPowerLimit(
        &g_powerController,
        g_effectivePowerLimitW
    );

    power_control::MotorCommand motors[kWheelCount] = {};
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        motors[i].target_current = targetCurrent[i];
        motors[i].velocity = wheelOmega[i];
    }

    power_control::powerControllerUpdate(
        &g_powerController,
        motors,
        kWheelCount
    );
    g_powerStatus = g_powerController.status;
    g_estimatedPowerW = g_powerController.origin_power;
    g_consumedPowerW = g_powerController.consume_power;

    const bool valid_result =
        g_powerStatus == power_control::PowerControlStatus::Ready
        || g_powerStatus == power_control::PowerControlStatus::Limited;

    if (!valid_result) {
        for (uint8_t i = 0U; i < kWheelCount; ++i) {
            outputCurrent[i] = 0.0f;
        }
        return false;
    }

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        outputCurrent[i] = power_control::powerControllerLimitCurrent(
            &g_powerController,
            i
        );
    }
    return true;
}

/**
 * @brief 写入四个轮电机的控制量并发送一帧 0x200。
 * @param command 与逻辑轮序一致的 M3508 原始控制量，不是安培值。
 * @note DjiMotor::sendGroup() 会自动应用安装方向并填充对应 CAN 槽位。
 */
void SendCurrentCommand(const float command[kWheelCount]) {
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        float raw = command[i];
        // M3508/C620 的原始控制量范围为 -16384~16384。
        raw = basic_math::constrain(raw, -16384.0f, 16384.0f);
        g_wheelMotor[i].setCommand(static_cast<int16_t>(raw));
    }

    // 四个轮电机共享同一发送帧，因此只能在此处统一发送一次。
    (void)DjiMotor::sendGroup(&hfdcan1, kChassisTxId);
}

/**
 * @brief 关闭底盘输出，向四个轮电机发送零控制量。
 */
void SendZeroCurrent(void) {
    const float zero[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    SendCurrentCommand(zero);
}

/**
 * @brief 由四个轮子的输出轴角速度反算车体速度。
 * @param wheelOmega 逻辑轮序对应的轮角速度，rad/s。
 * @param relativeYawRad 期望坐标系相对底盘坐标系的 Yaw 角，rad。
 * @param vx 输出前后速度，m/s。
 * @param vy 输出横移速度，m/s。
 * @param wz 输出旋转角速度，rad/s。
 * @return 四轮速度逆解和 wz 计算均成功时返回 true。
 */
bool EstimateBodyVelocity(
    const float wheelOmega[kWheelCount],
    float relativeYawRad,
    float *vx,
    float *vy,
    float *wz
) {
    float wheelLinear[kWheelCount];

    // 将角速度换算为轮缘线速度，供车体逆运动学使用。
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        wheelLinear[i] = kWheelRadius * wheelOmega[i];
    }

    // vx/vy 复用运动补偿算法层的四轮逆解；当前仍要求四轮全部有效。
    if (!motion_compensation::OmniMotionCompensator::solveVelocity(
            wheelLinear,
            motion_compensation::kAllWheelsValidMask,
            relativeYawRad,
            vx,
            vy
        )) {
        *vx = 0.0f;
        *vy = 0.0f;
        *wz = 0.0f;
        return false;
    }

    // 旋转速度继续使用底盘现有的轮距公式，避免改变当前 wz 控制行为。
    *wz = -(
        wheelLinear[0] + wheelLinear[1] +
        wheelLinear[2] + wheelLinear[3]
    ) / (4.0f * kWheelCenterDistance);

    return true;
}

}  // namespace

/**
 * @brief 初始化四个轮电机、力控分配器、速度环和功率控制器。
 * @note 必须在 bspFdcanInit() 之后、FreeRTOS 启动之前调用。
 *       任一电机配置或注册失败时进入 Error_Handler()。
 */
void Chassis_Init(void) {
    if (!board_cfg::kConfig.has_chassis) { return; }
    ForceActuatorParam params[kWheelCount] = {};

    // 1. 配置并注册四个 M3508；减速比和方向在此阶段一次性确定。
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        const uint32_t motorId = kWheelMotorId[i];

        if (!g_wheelMotor[i].init(
                MotorType::DjiM3508,
                static_cast<uint8_t>(motorId),
                &hfdcan1,
                kWheelDirection[i],
                kWheelGearRatio
            ) ||
            !MotorBase::registerMotor(g_wheelMotor[i])) {
            Error_Handler();
        }
        g_wheelMotor[i].setOfflineTimeout(kOfflineTimeoutMs);

        // 力控参数使用轮输出轴坐标；电机安装方向已由 DjiMotor 处理。
        params[i].torque_constant = kWheelTorquePerAmp;
        params[i].reduction_ratio = 1.0f;
        // 电机方向由 DjiMotor 统一处理，力控输出使用机构输出轴坐标。
        params[i].direction = 1;
        params[i].torque_max = kWheelTorqueMax;

        // 调试阶段先关闭模型补偿。
        params[i].inertia = 0.0f;
        params[i].viscous_friction = 0.0f;
        params[i].coulomb_friction = 0.0f;
        params[i].gravity_amplitude = 0.0f;
        params[i].gravity_offset = 0.0f;
    }

    // 2. 初始化 4 执行器、3 自由度力控求解器。
    g_chassisForce.init(params, kWheelCount, 3U, 0.001f);
    g_chassisForce.setVelocityFilterAlpha(0.8f);

    // 构建四轮全向底盘分配矩阵；失败时 g_forceReady 保持 false。
    float distribution[kWheelCount * 3U] = {};

    g_forceReady = ForceSolver::buildOmniDistribution(
        kWheelRadius,
        kWheelCenterDistance,
        distribution
    );

    if (g_forceReady) {
        g_chassisForce.setDistribution(distribution);
    }

    // 3. 初始化 vx、vy、wz 三个速度环。
    g_vxPid.init(
        25.0f, 5.0f, 0.0f, 0.0f,
        20.0f, kMaxFx, 0.001f,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );

    g_vyPid.init(
        25.0f, 5.0f, 0.0f, 0.0f,
        20.0f, kMaxFy, 0.001f,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );

    g_wzPid.init(
        4.0f, 0.5f, 0.0f, 0.0f,
        4.0f, kMaxMz, 0.001f,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );

    // 4. 初始化功率模型。最终动态上限在控制周期中读取裁判系统后设置，
    //    这里先使用保守回退值；裁判系统 0x0201 已包含姿态和增益效果，
    //    因此禁用算法层内部的二次姿态缩放。
    power_control::powerControllerInit(
        &g_powerController,
        &power_control::kDefaultC620Coefficients,
        kFallbackChassisPowerLimitW
    );
    g_powerController.margin = kPowerSafetyMargin;
    g_powerController.posture_scale = {1.0f, 1.0f, 1.0f, 1.0f};
    power_control::powerControllerSetCurrentLimit(
        &g_powerController,
        kC620MaxCurrent
    );
    g_effectivePowerLimitW = kFallbackChassisPowerLimitW;
    g_estimatedPowerW = 0.0f;
    g_consumedPowerW = 0.0f;
    g_powerStatus = power_control::PowerControlStatus::Disabled;

    // 5. 清空反馈状态，并确保初始化结束时电机输出为零。
    g_vxFiltered = 0.0f;
    g_vyFiltered = 0.0f;
    g_wzFiltered = 0.0f;

    SendZeroCurrent();
}

/**
 * @brief 执行一次底盘控制周期。
 * @note 正常周期约为 1 ms：读取反馈 -> 估计车体速度 -> 安全检查
 *       -> 遥控速度闭环 -> 力控分配 -> CAN 电流命令。
 */
void Chassis_Control(void) {
    if (!board_cfg::kConfig.has_chassis) { return; }
    float wheelOmega[kWheelCount];

    // 1. 读取新电机驱动换算后的轮输出轴角速度。
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        wheelOmega[i] = g_wheelMotor[i].data().velocity_rad_s;
    }

    // 2. 由四轮反馈估计车体速度，并做一阶低通滤波。
    //    云台与底盘相对 Yaw 尚未接入，当前使用 0 rad 保持原坐标方向。
    float vxRaw = 0.0f;
    float vyRaw = 0.0f;
    float wzRaw = 0.0f;
    constexpr float kRelativeYawRad = 0.0f;
    const bool velocityEstimateValid = EstimateBodyVelocity(
        wheelOmega,
        kRelativeYawRad,
        &vxRaw,
        &vyRaw,
        &wzRaw
    );

    g_vxFiltered = kVelocityFilterAlpha * g_vxFiltered +
                   (1.0f - kVelocityFilterAlpha) * vxRaw;

    g_vyFiltered = kVelocityFilterAlpha * g_vyFiltered +
                   (1.0f - kVelocityFilterAlpha) * vyRaw;

    g_wzFiltered = kVelocityFilterAlpha * g_wzFiltered +
                   (1.0f - kVelocityFilterAlpha) * wzRaw;

    PublishChassisVelocity();

    // 3. 任一安全条件不满足时，关闭力控并发送零电流。
    //    安全条件包括遥控器掉线、右拨杆下档、轮电机掉线和分配矩阵无效。
    const bool rcOnline = !getRcOffline();
    const bool locked =
        SWITCH_IS_DOWN(getDt7RcSw(RC_SW_RIGHT));

    if ((!velocityEstimateValid) || (!rcOnline) || locked ||
        (!AllWheelsOnline()) || (!g_forceReady)) {
        g_vxFiltered = vxRaw;
        g_vyFiltered = vyRaw;
        g_wzFiltered = wzRaw;
        PublishChassisVelocity();

        ResetVelocityPids();
        g_chassisForce.setEnable(false);
        SendZeroCurrent();
        return;
    }

    // 4. 读取 DT7 左摇杆前后/横移和右摇杆水平旋转输入。
    const float forward = ApplyDeadzone(
        getDt7RcCh(RC_CH_LEFT_VERTICAL)
    );
    const float lateral = ApplyDeadzone(
        getDt7RcCh(RC_CH_LEFT_HORIZONTAL)
    );
    const float rotate = ApplyDeadzone(
        getDt7RcCh(RC_CH_RIGHT_HORIZONTAL)
    );

    // 5. 将归一化摇杆量映射为车体目标速度。
    g_vxPid.setTarget(kRcXSign * forward * kMaxVx);
    g_vyPid.setTarget(kRcYSign * lateral * kMaxVy);
    g_wzPid.setTarget(kRcWzSign * rotate * kMaxWz);

    // 更新速度环反馈并计算三个方向的广义力。
    g_vxPid.setNow(g_vxFiltered);
    g_vyPid.setNow(g_vyFiltered);
    g_wzPid.setNow(g_wzFiltered);

    g_vxPid.timCalculatePeriodElapsedCallback();
    g_vyPid.timCalculatePeriodElapsedCallback();
    g_wzPid.timCalculatePeriodElapsedCallback();

    const float generalizedForce[3] = {
        basic_math::constrain(g_vxPid.getOut(), -kMaxFx, kMaxFx),
        basic_math::constrain(g_vyPid.getOut(), -kMaxFy, kMaxFy),
        basic_math::constrain(g_wzPid.getOut(), -kMaxMz, kMaxMz)
    };

    // 6. 使用当前轮速和广义力计算每个轮子的输出轴力矩。
    g_chassisForce.setTarget(generalizedForce);
    g_chassisForce.setNow(nullptr, wheelOmega);
    g_chassisForce.setEnable(true);
    g_chassisForce.timCalculatePeriodElapsedCallback();

    // 7. 先按规则功率预算衰减目标电流，再换算成 C620 原始控制量。
    float targetCurrent[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    float limitedCurrent[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        targetCurrent[i] = basic_math::constrain(
            g_chassisForce.getOut(i),
            -kC620MaxCurrent,
            kC620MaxCurrent
        );
    }

    if (!ApplyPowerControl(wheelOmega, targetCurrent, limitedCurrent)) {
        ResetVelocityPids();
        g_chassisForce.setEnable(false);
        SendZeroCurrent();
        return;
    }

    constexpr float currentToCommand =
        16384.0f / kC620MaxCurrent;
    float currentCommand[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        currentCommand[i] =
            limitedCurrent[i] * currentToCommand;
    }

    // 8. 更新四个电机的控制量并发送共享 CAN 帧。
    SendCurrentCommand(currentCommand);
}

/**
 * @brief 底盘控制 FreeRTOS 任务入口。
 * @param argument 未使用。
 * @note 任务以约 1 ms 周期调用 Chassis_Control()，与 PID 和力控配置周期一致。
 */
extern "C" void Control_Task(void const *argument) {
    (void)argument;

    for (;;) {
        Chassis_Control();
        osDelay(1);
    }
}
