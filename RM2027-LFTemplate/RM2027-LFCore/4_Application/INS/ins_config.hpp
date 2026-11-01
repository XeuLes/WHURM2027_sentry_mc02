/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Config.c
  * @brief          : Configuare the Robot Functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "alg_basic.hpp"
#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

/* General physics and mathematics constants ---------------------------------*/

/**
 * @brief the value of local gravity acceleration
 */

#define GRAVITY_ACCEL 9.718f

/**
  * @brief Euler's Number
  */
#define EULER_NUMBER 2.718281828459045f

/**
 * @brief radian system rotation degrees system , 180.f/PI
 */
#define RADIANS_TO_DEGREES BASIC_MATH_RAD_TO_DEG

/**
 * @brief degrees system rotation radian system , PI/180.f
 */
#define DEGREES_TO_RADIANS BASIC_MATH_DEG_TO_RAD

/* Vision reslove constants -------------------------------------------------*/

/**
 * @brief  Decision Marking mode
 *         0: select the minimum yaw armor
 *         1: select the minimum distance armor
 */
#define YAW_DISTANCE_DECISION 0

/**
 * @brief ballistic coefficient
 * @note  17mm: 0.038
 *        42mm: 0.019
 */
#define BULLET_COEFFICIENT 0.038f

/**
 * @brief the half width of little armor
 */
#define LITTLE_ARMOR_HALF_WIDTH 0.07f

/**
 * @brief the half width of Large armor
 */
#define LARGE_ARMOR_HALF_WIDTH 0.1175f

/* IMU reslove constants ---------------------------------------------------*/
/**
 * @brief the flag of bmi088 Calibration
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_CALIBRATION_ENABLE 0U

/**
 * @brief the index of pitch angle update
 */
#define IMU_ANGLE_INDEX_PITCH 1U
/**
 * @brief the index of yaw angle update
 */
#define IMU_ANGLE_INDEX_YAW 0U
/**
 * @brief the index of roll angle update
 */
#define IMU_ANGLE_INDEX_ROLL 2U

/**
 * @brief the index of pitch gyro update
 */
#define IMU_GYRO_INDEX_PITCH 0U
/**
 * @brief the index of yaw gyro update
 */
#define IMU_GYRO_INDEX_YAW 2U
/**
 * @brief the index of roll gyro update
 */
#define IMU_GYRO_INDEX_ROLL 1U

/**
 * @brief the index of pitch accel update
 */
#define IMU_ACCEL_INDEX_PITCH 0U
/**
 * @brief the index of yaw accel update
 */
#define IMU_ACCEL_INDEX_YAW 2U
/**
 * @brief the index of roll accel update
 */
#define IMU_ACCEL_INDEX_ROLL 1U

/* Remote reslove constants -----------------------------------------------*/
/**
 * @brief the flag of remote control receive frame data
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN 0U

#endif //ROBOT_CONFIG_H
