/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_ins.h"
#include "app_supercap.h"
#include "app_gimbal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId userTaskHandle;
static uint32_t Gimbal_TaskBuffer[1024];
static osStaticThreadDef_t Gimbal_TaskControlBlock;
osThreadId gimbal_task_handle;
osThreadId insTaskHandle;
uint32_t BoardLink_TaskBuffer[512];
osStaticThreadDef_t BoardLink_TaskControlBlock;
osThreadId BoardLink_TaskHandle;
osThreadId superpower_handle;
uint32_t SUPERPOWER_TaskBuffer[256];
osStaticThreadDef_t SUPERPOWER_TaskControlBlock;
/* USER CODE END Variables */
osThreadId Start_INS_TaskHandle;
uint32_t Start_INS_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_INS_TaskControlBlock;
osThreadId Start_Control_TaskHandle;
uint32_t Start_Control_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Control_TaskControlBlock;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void BoardLink_Task(void const * argument);
/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void Control_Task(void const * argument);
void Detect_Task(void const * argument);
void MiniPC_Task(void const * argument);
void Shoot_Task(void const * argument);
void referee_task(void const * argument);
void usb_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Start_INS_Task */
  osThreadStaticDef(Start_INS_Task, INS_Task, osPriorityHigh, 0, 2048, Start_INS_TaskBuffer, &Start_INS_TaskControlBlock);
  Start_INS_TaskHandle = osThreadCreate(osThread(Start_INS_Task), NULL);

  /* definition and creation of Start_Control_Task */
  osThreadStaticDef(Start_Control_Task, Control_Task, osPriorityAboveNormal, 0, 1024, Start_Control_TaskBuffer, &Start_Control_TaskControlBlock);
  Start_Control_TaskHandle = osThreadCreate(osThread(Start_Control_Task), NULL);




  /* USER CODE BEGIN RTOS_THREADS */
  osThreadStaticDef(Start_Gimbal_Task, Gimbal_Task, osPriorityAboveNormal, 0, 1024, Gimbal_TaskBuffer, &Gimbal_TaskControlBlock);
  gimbal_task_handle = osThreadCreate(osThread(Start_Gimbal_Task), NULL);
  
  /* 超电本地执行；裁判快照通过 CAN3 接收。 */
  if (supercapInit()) {
    osThreadStaticDef(SUPERPOWER, superpower_task, osPriorityAboveNormal, 0, 256, SUPERPOWER_TaskBuffer, &SUPERPOWER_TaskControlBlock);
    superpower_handle = osThreadCreate(osThread(SUPERPOWER), NULL);
    if (superpower_handle == NULL) {
      Error_Handler();
    }
  }
  /* 静态任务声明 */


  osThreadStaticDef(BoardLink, BoardLink_Task, osPriorityAboveNormal, 0, 512, BoardLink_TaskBuffer, &BoardLink_TaskControlBlock);
  BoardLink_TaskHandle = osThreadCreate(osThread(BoardLink), NULL);
   /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the Start_INS_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the Start_Control_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the Start_Detect_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* USER CODE BEGIN Header_MiniPC_Task */
/**
* @brief Function implementing the Start_MiniPC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MiniPC_Task */
__weak void MiniPC_Task(void const * argument)
{
  /* USER CODE BEGIN MiniPC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MiniPC_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Start_Shoot_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
