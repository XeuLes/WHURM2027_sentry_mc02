/**
 * @file app_chassis.h
 * @brief 四轮全向底盘应用对外接口。
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 底盘控制模式。
 */
typedef enum {
    CHASSIS_MODE_DISABLED = 0,    /**< 禁止输出。 */
    CHASSIS_MODE_DIRECT_RC = 1,   /**< 直接遥控调试模式。 */
    CHASSIS_MODE_AUTO_VISION = 2, /**< 上位机/视觉自动模式。 */
} ChassisModeTypedef;

/**
 * @brief 初始化底盘命令、功率管理、电机、力控分配器和速度环。
 * @note 必须在 bspFdcanInit() 之后、FreeRTOS 调度器启动之前调用。
 *       任一电机配置或注册失败时会进入 Error_Handler()。
 */
void Chassis_Init(void);

/**
 * @brief 执行一次底盘控制周期，串联反馈、命令、控制、功率和发送。
 * @note 应在底盘控制任务中周期调用，不要在中断中调用。
 */
void Chassis_Control(void);

/**
 * @brief 获取当前底盘控制模式。
 * @return 当前模式，可用于调试和状态上报。
 */
ChassisModeTypedef Chassis_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif
