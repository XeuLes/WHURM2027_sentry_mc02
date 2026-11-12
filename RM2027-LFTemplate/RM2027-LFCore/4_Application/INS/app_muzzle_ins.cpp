#include "app_muzzle_ins.hpp"

#include "app_ins.h"
#include "alg_lpf.hpp"
#include "alg_quaternion.hpp"
#include "dm_imu.hpp"
#include "ins_config.hpp"
#include "lf_core.hpp"

#include <cmath>

namespace muzzle_ins {
namespace {
// Same EKF setup and INS processing as app_ins.cpp, with independent state.
InsInfo muzzle_info{};
QuaternionInfo muzzle_quaternion_info;
LowPassFilter2p ins_accel_pf2p[3];
static float ins_lpf2p_alpha[3] = { 1.929454039488895f,
                                    -0.93178349823448126f,
                                    0.002329458745586203f };

static float quaternion_ekf_a_data[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                           0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

static float quaternion_ekf_p_data[36] = { 100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,   0.1f,   100000,
                                           0.1f,   0.1f, 0.1f, 0.1f, 0.1f, 0.1f,   100000, 0.1f,
                                           0.1f,   0.1f, 0.1f, 0.1f, 0.1f, 100000, 0.1f,   0.1f,
                                           0.1f,   0.1f, 0.1f, 0.1f, 100,  0.1f,   0.1f,   0.1f,
                                           0.1f,   0.1f, 0.1f, 100 };

uint32_t last_gyro_revision = 0;
uint32_t last_gyro_ms = 0;
bool have_gyro = false;
constexpr uint32_t kSampleTimeoutMs = 10U;

bool init() {
    for (unsigned axis = 0; axis < 3; ++axis) {
        lowPassFilter2pInit(&ins_accel_pf2p[axis], ins_lpf2p_alpha);
    }
    quaternionEkfInit(
        &muzzle_quaternion_info,
        10.f,
        0.001f,
        1000000.f,
        quaternion_ekf_a_data,
        quaternion_ekf_p_data
    );
    return muzzle_quaternion_info.initialized;
}

static void UpdateLinearAcceleration(void) {
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    bool attitude_ready = false;

    attitude_ready = muzzle_quaternion_info.initialized;
    q0 = muzzle_quaternion_info.quat[0];
    q1 = muzzle_quaternion_info.quat[1];
    q2 = muzzle_quaternion_info.quat[2];
    q3 = muzzle_quaternion_info.quat[3];

    // DM 加速度已换算为 g，因此范围判断也在 g 域完成。
    const float accel_norm_squared_g =
        muzzle_info.accel[0] * muzzle_info.accel[0]
        + muzzle_info.accel[1] * muzzle_info.accel[1]
        + muzzle_info.accel[2] * muzzle_info.accel[2];
    constexpr float minimum_norm_g = 0.5f;
    constexpr float maximum_norm_g = 2.0f;
    const bool accel_in_range =
        accel_norm_squared_g >= minimum_norm_g * minimum_norm_g
        && accel_norm_squared_g <= maximum_norm_g * maximum_norm_g;

    if (!attitude_ready || !accel_in_range) {
        muzzle_info.linear_accel[0] = 0.0f;
        muzzle_info.linear_accel[1] = 0.0f;
        muzzle_info.linear_accel[2] = 0.0f;
        muzzle_info.accel_valid = false;
        return;
    }

    // 与四元数 EKF 观测模型一致：静止水平时重力方向为 +Z。
    const float gravity_body[3] = {
        2.0f * (q1 * q3 - q0 * q2),
        2.0f * (q0 * q1 + q2 * q3),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    };

    // 先在 g 域消除重力，再统一换算为 m/s²，供枪口 IMU 状态发布使用。
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        muzzle_info.linear_accel[axis] =
            (muzzle_info.accel[axis] - gravity_body[axis]) * GRAVITY_ACCEL;
    }

    muzzle_info.accel_valid =
        std::isfinite(muzzle_info.linear_accel[0])
        && std::isfinite(muzzle_info.linear_accel[1])
        && std::isfinite(muzzle_info.linear_accel[2]);
    if (!muzzle_info.accel_valid) {
        muzzle_info.linear_accel[0] = 0.0f;
        muzzle_info.linear_accel[1] = 0.0f;
        muzzle_info.linear_accel[2] = 0.0f;
    }
}

} // namespace

void update() {
    constexpr uint8_t required = (1U << (QUAT_DATA - ACCEL_DATA)) |
                                 (1U << (QUAT_DATA - GYRO_DATA));
    imu_t sample{};
    if (!dm_imu_get_data(&sample) || (sample.received_mask & required) != required) { return; }
    const uint32_t now = HAL_GetTick();
    if (now - sample.accel_rx_ms >= kSampleTimeoutMs ||
        now - sample.gyro_rx_ms >= kSampleTimeoutMs ||
        (have_gyro && sample.gyro_revision == last_gyro_revision)) { return; }
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(sample.gyro[axis]) || !std::isfinite(sample.accel[axis])) { return; }
    }
    const float norm2 = sample.accel[0]*sample.accel[0] +
        sample.accel[1]*sample.accel[1] + sample.accel[2]*sample.accel[2];
    if (!std::isfinite(norm2) || norm2 <= 0.0f) { return; }
    const bool first = !have_gyro;
    const uint32_t elapsed = sample.gyro_rx_ms - last_gyro_ms;
    if (!first && elapsed == 0U) { return; }
    if (!muzzle_quaternion_info.initialized && !init()) { return; }
    last_gyro_revision = sample.gyro_revision;
    last_gyro_ms = sample.gyro_rx_ms;
    have_gyro = true;
    if (!first && elapsed >= kSampleTimeoutMs) {
        // Do not integrate motion across an unknown CAN outage. Restart LPFs
        // on the next sample; retain the existing attitude and yaw reference.
        for (unsigned axis = 0; axis < 3; ++axis) {
            lowPassFilter2pInit(&ins_accel_pf2p[axis], ins_lpf2p_alpha);
        }
        return;
    }
    // DM reports m/s^2; app_ins.cpp feeds acceleration in g to this EKF.
    for (unsigned axis = 0; axis < 3; ++axis) {
        muzzle_info.accel[axis] = lowPassFilter2pUpdate(
            &ins_accel_pf2p[axis], sample.accel[axis] / GRAVITY_ACCEL);
        muzzle_info.gyro[axis] = sample.gyro[axis];
    }
    const float filtered_norm2 = muzzle_info.accel[0]*muzzle_info.accel[0] +
        muzzle_info.accel[1]*muzzle_info.accel[1] + muzzle_info.accel[2]*muzzle_info.accel[2];
    if (!std::isfinite(filtered_norm2) || filtered_norm2 <= 0.0f) { return; }
    const float dt = first ? 0.001f : elapsed * 0.001f;
    const uint32_t oldest_tick = now - sample.accel_rx_ms >= now - sample.gyro_rx_ms
        ? sample.accel_rx_ms : sample.gyro_rx_ms;

    /* Update the QuaternionEKF */
    quaternionEkfUpdate(&muzzle_quaternion_info, muzzle_info.gyro, muzzle_info.accel, dt);

    if (muzzle_quaternion_info.quaternion_ekf.mat_status != ARM_MATH_SUCCESS) { return; }

    memcpy(muzzle_info.angle, muzzle_quaternion_info.euler_angle, sizeof(muzzle_info.angle));

    /* Update the Euler angle in degrees. */
    muzzle_info.pitch_angle =
        muzzle_quaternion_info.euler_angle[IMU_ANGLE_INDEX_PITCH] * BASIC_MATH_RAD_TO_DEG;
    muzzle_info.yaw_angle =
        muzzle_quaternion_info.euler_angle[IMU_ANGLE_INDEX_YAW] * BASIC_MATH_RAD_TO_DEG;
    muzzle_info.roll_angle =
        muzzle_quaternion_info.euler_angle[IMU_ANGLE_INDEX_ROLL] * BASIC_MATH_RAD_TO_DEG;

    /* Update the yaw total angle */
    if (muzzle_info.yaw_angle - muzzle_info.last_yaw_angle < -180.f) {
        muzzle_info.yaw_round_count++;
    } else if (muzzle_info.yaw_angle - muzzle_info.last_yaw_angle > 180.f) {
        muzzle_info.yaw_round_count--;
    }
    muzzle_info.last_yaw_angle = muzzle_info.yaw_angle;

    muzzle_info.yaw_tol_angle = muzzle_info.yaw_angle + muzzle_info.yaw_round_count * 360.f;

    /* Update the INS gyro in degrees */
    muzzle_info.pitch_gyro = muzzle_info.gyro[IMU_GYRO_INDEX_PITCH] * RADIANS_TO_DEGREES;
    muzzle_info.yaw_gyro = muzzle_info.gyro[IMU_GYRO_INDEX_YAW] * RADIANS_TO_DEGREES;
    muzzle_info.roll_gyro = muzzle_info.gyro[IMU_GYRO_INDEX_ROLL] * RADIANS_TO_DEGREES;

    UpdateLinearAcceleration();

    LuojiaFox::system_state.updateMuzzleImu(
        { muzzle_info.yaw_angle,
          muzzle_info.pitch_angle,
          muzzle_info.roll_angle,
          muzzle_info.yaw_gyro,
          muzzle_info.pitch_gyro,
          muzzle_info.roll_gyro,
          muzzle_info.linear_accel[0],
          muzzle_info.linear_accel[1],
          muzzle_info.accel_valid,
          oldest_tick }
    );
}
} // namespace muzzle_ins
