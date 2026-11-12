#include "dvc_bsp.h"

#include "dvc_buzzer.hpp"




void bspInit(void)
{
  (void)buzzer.init();
  // BMI088_Init() moved to INS_Task to avoid HardFault during pre-scheduler initialization
  // The issue was Delay_ms() using HAL_GetTick() before FreeRTOS scheduler starts
}
