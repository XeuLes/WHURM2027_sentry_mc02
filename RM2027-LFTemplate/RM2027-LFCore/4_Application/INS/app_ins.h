/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_INS_H
#define APP_INS_H

#include "core_linkage.h"

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the INS.
 */
typedef struct {
    float pitch_angle;
    float yaw_angle;
    float yaw_tol_angle;
    float roll_angle;

    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    float angle[3];
    float gyro[3];
    float accel[3];
    float linear_accel[3];
    bool accel_valid;

    float last_yaw_angle;
    int16_t yaw_round_count;

} InsInfo;

/* Externs---------------------------------------------------------*/
LF_EXTERN_C InsInfo ins_info;

LF_EXTERN_C void INS_Task(void const* argument);

#endif //APP_INS_H
