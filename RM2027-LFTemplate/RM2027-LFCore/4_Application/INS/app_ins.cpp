/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_ins.cpp
  * @brief          : INS task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_ins.h"

//#include "detect.h"

#include "alg_basic.hpp"
#include "alg_lpf.hpp"
#include "alg_pid.h"
#include "alg_quaternion.hpp"
#include "cmsis_os.h"
#include "dvc_bmi088.hpp"
#include "ins_config.hpp"
#include "lf_core.hpp"
#include "tim.h"

#include <cmath>

/* Attitude solver selection ------------------------------------------------*/
/**
 * @brief 选择解算方式
 *          0 = QuaternionEKF   (Algorithm/Quatertion/alg_quaternion.cpp)
 *          1 = Mahony          (Algorithm/Mahony/alg_mahony.c)
 */
#ifndef INS_USE_MAHONY
#define INS_USE_MAHONY 0
#endif

#if INS_USE_MAHONY
#include "alg_mahony.h"
#endif

/* Heater ------------------------------------------------------------------- */
/**
 * @brief 加热目标温度（摄氏度）
 * @note  同时作为暖机零偏标定的温度门槛
 */
#ifndef BMI088_Target_Temperature
#define BMI088_Target_Temperature 35.0f
#endif

/**
 * @brief 加热 PWM 上限，对应 TIM3_CH4 的 ARR 值
 */
#ifndef BMI088_Heater_PWM_Max
#define BMI088_Heater_PWM_Max 18000.0f
#endif

/**
 * @brief yaw 轴陀螺死区（rad/s），低于此值视为静止直接清零
 */
#ifndef BMI088_GYRO_Z_DEADBAND
#define BMI088_GYRO_Z_DEADBAND 0.012f
#endif

/**
 * @brief 暖机零偏标定开关
 * @note  融合前这个值是 0，yaw 用的是 dvc_bmi088.cpp 里的实测零偏；
 *        改成 1 会在温度稳定后重新标定并覆盖那组零偏
 */
#ifndef BMI088_Warm_Calib_Enable
#define BMI088_Warm_Calib_Enable 0
#endif

/* Globals ------------------------------------------------------------------*/
/**
  * @brief the structure that contains the information for the INS.
  */
InsInfo ins_info;

/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
static float ins_lpf2p_alpha[3] = { 1.929454039488895f,
                                    -0.93178349823448126f,
                                    0.002329458745586203f };

/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p ins_accel_pf2p[3];

/**
  * @brief the Initialize data of state transition matrix.
  */
static float quaternion_ekf_a_data[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                           0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

/**
  * @brief the Initialize data of posteriori covariance matrix.
  */
static float quaternion_ekf_p_data[36] = { 100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,   0.1f,   100000,
                                           0.1f,   0.1f, 0.1f, 0.1f, 0.1f, 0.1f,   100000, 0.1f,
                                           0.1f,   0.1f, 0.1f, 0.1f, 0.1f, 100000, 0.1f,   0.1f,
                                           0.1f,   0.1f, 0.1f, 0.1f, 100,  0.1f,   0.1f,   0.1f,
                                           0.1f,   0.1f, 0.1f, 100 };

TickType_t ins_task_sys_tick = 0;

static PID TempCtrl_PID;

#if BMI088_Warm_Calib_Enable

float Warm_Gyro_Offset[3] = { 0.0f, 0.0f, 0.0f };
float Warm_Gyro_Offset_Average[3] = { 0.0f, 0.0f, 0.0f };

uint8_t Warm_Count = 0;

volatile uint8_t Warm_Calib_State = 0;
#endif

/**
 * @brief Initializes the INS_Task.
 */
static void insTaskInit(void);

/**
 * @brief Control the BMI088 temperature.
 * @param temp  measure of the BMI088 temperature.
 */
static void BMI088_Temp_Control(float temp);

/**
 * @brief 由当前姿态扣除重力，得到机体坐标系线性加速度。
 * @note BMI088 驱动按 ±6 g 量程输出的 accel[] 单位仍为 g；
 *       本函数先在 g 域去除重力，再统一换算为 m/s²。
 */
static void UpdateLinearAcceleration(void);

#if BMI088_Warm_Calib_Enable
/**
 * @brief Drive the one-off warm gyro-bias calibration.
 */
static void BMI088_Warm_Calib_Update(void);
#endif

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
extern "C" void INS_Task(void const* argument) {
    /* USER CODE BEGIN INS_Task */

    /* Initialize BMI088 IMU sensor - moved here from bspInit() to avoid HardFault
     * The sensor initialization requires Delay_ms() which uses HAL_GetTick().
     * This must be called after FreeRTOS scheduler has started to ensure proper timing. */
    static bool bmi088_initialized = false;
    if (!bmi088_initialized) {
        bmi088Init();
        bmi088_initialized = true;
    }

    /* Initializes the INS_Task. */
    insTaskInit();

    /* 加热控制的分频计数器。用独立计数器而不是 sys_tick % 5，
       避免 osDelayUntil 抖动导致 PID 的等效 dt 不准。 */
    uint32_t heater_div = 0;

    /* Infinite loop */
    for (;;) {
        ins_task_sys_tick = osKernelSysTick();

        /* Update the BMI088 measurement */
        bmi088InfoUpdate(&bmi088_info);
        //detect_hook(DETECT_BMI_GYRO);
        //detect_hook(DETECT_BMI_ACCEL);

        /* Accel measurement LPF2p */
        ins_info.accel[0] = lowPassFilter2pUpdate(&ins_accel_pf2p[0], bmi088_info.accel[0]);
        ins_info.accel[1] = lowPassFilter2pUpdate(&ins_accel_pf2p[1], bmi088_info.accel[1]);
        ins_info.accel[2] = lowPassFilter2pUpdate(&ins_accel_pf2p[2], bmi088_info.accel[2]);

        /* Update the INS gyro in radians */
        ins_info.gyro[0] = bmi088_info.gyro[0];
        ins_info.gyro[1] = bmi088_info.gyro[1];
        ins_info.gyro[2] = bmi088_info.gyro[2];

        /* yaw 轴死区 */
        if (fabsf(ins_info.gyro[IMU_GYRO_INDEX_YAW]) < BMI088_GYRO_Z_DEADBAND) {
            ins_info.gyro[IMU_GYRO_INDEX_YAW] = 0.0f;
        }

#if INS_USE_MAHONY
        /* Seed the Mahony filter from the first accelerometer sample */
        if (!Mahony_Info.Init) {
            Mahony_Init(&Mahony_Info, ins_info.accel[0], ins_info.accel[1], ins_info.accel[2]);
        }

        /* Update the Mahony complementary filter */
        Mahony_Update(
            &Mahony_Info,
            ins_info.gyro[0],
            ins_info.gyro[1],
            ins_info.gyro[2],
            ins_info.accel[0],
            ins_info.accel[1],
            ins_info.accel[2]
        );
        Mahony_ComputeAngles(&Mahony_Info);

        memcpy(ins_info.angle, Mahony_Info.EulerAngle, sizeof(ins_info.angle));

        /* Update the Euler angle in degrees. */
        ins_info.pitch_angle =
            Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH] * BASIC_MATH_RAD_TO_DEG;
        ins_info.yaw_angle = Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_YAW] * BASIC_MATH_RAD_TO_DEG;
        ins_info.roll_angle = Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL] * BASIC_MATH_RAD_TO_DEG;
#else
        /* Update the QuaternionEKF */
        quaternionEkfUpdate(&quaternion_info, ins_info.gyro, ins_info.accel, 0.001f);

        memcpy(ins_info.angle, quaternion_info.euler_angle, sizeof(ins_info.angle));

        /* Update the Euler angle in degrees. */
        ins_info.pitch_angle =
            quaternion_info.euler_angle[IMU_ANGLE_INDEX_PITCH] * BASIC_MATH_RAD_TO_DEG;
        ins_info.yaw_angle =
            quaternion_info.euler_angle[IMU_ANGLE_INDEX_YAW] * BASIC_MATH_RAD_TO_DEG;
        ins_info.roll_angle =
            quaternion_info.euler_angle[IMU_ANGLE_INDEX_ROLL] * BASIC_MATH_RAD_TO_DEG;
#endif

        /* ---- 以下与解算方式无关，两套算法共用 ---- */

        /* Update the yaw total angle */
        if (ins_info.yaw_angle - ins_info.last_yaw_angle < -180.f) {
            ins_info.yaw_round_count++;
        } else if (ins_info.yaw_angle - ins_info.last_yaw_angle > 180.f) {
            ins_info.yaw_round_count--;
        }
        ins_info.last_yaw_angle = ins_info.yaw_angle;

        ins_info.yaw_tol_angle = ins_info.yaw_angle + ins_info.yaw_round_count * 360.f;

        /* Update the INS gyro in degrees */
        ins_info.pitch_gyro = ins_info.gyro[IMU_GYRO_INDEX_PITCH] * RADIANS_TO_DEGREES;
        ins_info.yaw_gyro = ins_info.gyro[IMU_GYRO_INDEX_YAW] * RADIANS_TO_DEGREES;
        ins_info.roll_gyro = ins_info.gyro[IMU_GYRO_INDEX_ROLL] * RADIANS_TO_DEGREES;

        UpdateLinearAcceleration();

        LuojiaFox::system_state.updateImu(
            { ins_info.yaw_angle,
              ins_info.pitch_angle,
              ins_info.roll_angle,
              ins_info.yaw_gyro,
              ins_info.pitch_gyro,
              ins_info.roll_gyro,
              ins_info.linear_accel[0],
              ins_info.linear_accel[1],
              ins_info.accel_valid,
              HAL_GetTick() }
        );

        /* 温控每 5 ms 跑一次，与 PID 初始化时的 dt = 0.005f 对应 */
        if (++heater_div >= 5) {
            heater_div = 0;
            BMI088_Temp_Control(bmi088_info.temperature);
        }

#if BMI088_Warm_Calib_Enable
        BMI088_Warm_Calib_Update();
#endif

        osDelayUntil(&ins_task_sys_tick, 1);
    }
    /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
static void insTaskInit(void) {
    ins_info.pitch_angle = 0.f;
    ins_info.yaw_angle = 0.f;
    ins_info.roll_angle = 0.f;
    ins_info.yaw_tol_angle = 0.f;
    ins_info.last_yaw_angle = 0.f;
    ins_info.yaw_round_count = 0;
    ins_info.pitch_gyro = 0.f;
    ins_info.yaw_gyro = 0.f;
    ins_info.roll_gyro = 0.f;
    ins_info.angle[0] = 0.f;
    ins_info.angle[1] = 0.f;
    ins_info.angle[2] = 0.f;
    ins_info.gyro[0] = 0.f;
    ins_info.gyro[1] = 0.f;
    ins_info.gyro[2] = 0.f;
    ins_info.accel[0] = 0.f;
    ins_info.accel[1] = 0.f;
    ins_info.accel[2] = 0.f;
    ins_info.linear_accel[0] = 0.f;
    ins_info.linear_accel[1] = 0.f;
    ins_info.linear_accel[2] = 0.f;
    ins_info.accel_valid = false;

    /* Initializes the Second order lowpass filter  */
    lowPassFilter2pInit(&ins_accel_pf2p[0], ins_lpf2p_alpha);
    lowPassFilter2pInit(&ins_accel_pf2p[1], ins_lpf2p_alpha);
    lowPassFilter2pInit(&ins_accel_pf2p[2], ins_lpf2p_alpha);

    /* 温控 PID：dt 必须和实际调用周期一致，否则 D 项会算错 */
    TempCtrl_PID.init(
        2000.0f,
        2000.0f,
        3.6f,
        0.0f,
        BMI088_Heater_PWM_Max,
        BMI088_Heater_PWM_Max,
        0.005f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        PidDFirstDisable
    );

    /* 上电先断加热 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

#if INS_USE_MAHONY
    Mahony_Init(&Mahony_Info, ins_info.accel[0], ins_info.accel[1], ins_info.accel[2]);
#else
    /* Initializes the Quaternion EKF */
    quaternionEkfInit(
        &quaternion_info,
        10.f,
        0.001f,
        1000000.f,
        quaternion_ekf_a_data,
        quaternion_ekf_p_data
    );
#endif
}

//------------------------------------------------------------------------------
static void UpdateLinearAcceleration(void) {
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    bool attitude_ready = false;

#if INS_USE_MAHONY
    attitude_ready = Mahony_Info.Init;
    q0 = Mahony_Info.q0;
    q1 = Mahony_Info.q1;
    q2 = Mahony_Info.q2;
    q3 = Mahony_Info.q3;
#else
    attitude_ready = quaternion_info.initialized;
    q0 = quaternion_info.quat[0];
    q1 = quaternion_info.quat[1];
    q2 = quaternion_info.quat[2];
    q3 = quaternion_info.quat[3];
#endif

    // BMI088_ACCEL_6G_SEN 的换算结果为 g，因此范围判断也在 g 域完成。
    const float accel_norm_squared_g =
        ins_info.accel[0] * ins_info.accel[0]
        + ins_info.accel[1] * ins_info.accel[1]
        + ins_info.accel[2] * ins_info.accel[2];
    constexpr float minimum_norm_g = 0.5f;
    constexpr float maximum_norm_g = 2.0f;
    const bool accel_in_range =
        accel_norm_squared_g >= minimum_norm_g * minimum_norm_g
        && accel_norm_squared_g <= maximum_norm_g * maximum_norm_g;

    if (!attitude_ready || !accel_in_range) {
        ins_info.linear_accel[0] = 0.0f;
        ins_info.linear_accel[1] = 0.0f;
        ins_info.linear_accel[2] = 0.0f;
        ins_info.accel_valid = false;
        return;
    }

    // 与四元数 EKF 观测模型一致：静止水平时重力方向为 +Z。
    const float gravity_body[3] = {
        2.0f * (q1 * q3 - q0 * q2),
        2.0f * (q0 * q1 + q2 * q3),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    };

    // 先在 g 域消除重力，再统一换算为 m/s²，供底盘速度融合使用。
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        ins_info.linear_accel[axis] =
            (ins_info.accel[axis] - gravity_body[axis]) * GRAVITY_ACCEL;
    }

    ins_info.accel_valid =
        std::isfinite(ins_info.linear_accel[0])
        && std::isfinite(ins_info.linear_accel[1])
        && std::isfinite(ins_info.linear_accel[2]);
    if (!ins_info.accel_valid) {
        ins_info.linear_accel[0] = 0.0f;
        ins_info.linear_accel[1] = 0.0f;
        ins_info.linear_accel[2] = 0.0f;
    }
}

//------------------------------------------------------------------------------
/**
 * @brief Control the BMI088 temperature.
 * @param temp  measure of the BMI088 temperature.
 */
static void BMI088_Temp_Control(float temp) {
    TempCtrl_PID.setTarget(BMI088_Target_Temperature);
    TempCtrl_PID.setNow(temp);
    TempCtrl_PID.timCalculatePeriodElapsedCallback();

    float output = TempCtrl_PID.getOut();
    output = basic_math::constrain(output, 0.0f, BMI088_Heater_PWM_Max);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)output);
}

//------------------------------------------------------------------------------
/**
  * @brief  Drive the one-off warm gyro-bias calibration
  * @retval none
  */
#if BMI088_Warm_Calib_Enable
static void BMI088_Warm_Calib_Update(void) {
    static const float Temp_Band = 0.02f;
    static const uint16_t Window_Samples = 5000;
    static const uint8_t Window_Num = 500;

    static uint16_t SampleCnt = 0;
    static float Gyro_Sum[3] = { 0.0f, 0.0f, 0.0f };

    if (Warm_Calib_State == 2) {
        return; /* already done */
    }

    if ((fabsf(bmi088_info.temperature - BMI088_Target_Temperature) >= Temp_Band)) {
        return;
    }

    Warm_Calib_State = 1; /* sampling has started, for the debugger */

    Gyro_Sum[0] += bmi088_info.gyro[0];
    Gyro_Sum[1] += bmi088_info.gyro[1];
    Gyro_Sum[2] += bmi088_info.gyro[2];

    if (++SampleCnt >= Window_Samples) {
        Warm_Gyro_Offset[0] = Gyro_Sum[0] / (float)Window_Samples + bmi088_info.offsets_gyro_x;
        Warm_Gyro_Offset[1] = Gyro_Sum[1] / (float)Window_Samples + bmi088_info.offsets_gyro_y;
        Warm_Gyro_Offset[2] = Gyro_Sum[2] / (float)Window_Samples + bmi088_info.offsets_gyro_z;

        Warm_Gyro_Offset_Average[0] += Warm_Gyro_Offset[0];
        Warm_Gyro_Offset_Average[1] += Warm_Gyro_Offset[1];
        Warm_Gyro_Offset_Average[2] += Warm_Gyro_Offset[2];
        Warm_Count++;

        if (Warm_Count >= Window_Num) {
            Warm_Gyro_Offset_Average[0] /= (float)Window_Num;
            Warm_Gyro_Offset_Average[1] /= (float)Window_Num;
            Warm_Gyro_Offset_Average[2] /= (float)Window_Num;
            bmi088_info.offsets_gyro_x = Warm_Gyro_Offset_Average[0];
            bmi088_info.offsets_gyro_y = Warm_Gyro_Offset_Average[1];
            bmi088_info.offsets_gyro_z = Warm_Gyro_Offset_Average[2];
            Warm_Calib_State = 2;
        }

        Gyro_Sum[0] = 0.0f;
        Gyro_Sum[1] = 0.0f;
        Gyro_Sum[2] = 0.0f;
        SampleCnt = 0;
    }
}
#endif
//------------------------------------------------------------------------------
