/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.h
  * @brief          : The header file of bsptick.h
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DRV_TICK_H
#define DRV_TICK_H

#include "core_linkage.h"

#include "stdint.h"

LF_EXTERN_C void delayUs(uint32_t us);

LF_EXTERN_C void delayMs(uint32_t ms);

// HAL 启动后的本地时间，单位 100 us。供任务上下文调用。
LF_EXTERN_C uint32_t getTick100us(void);

#endif
