#include "dvc_bsp.h"

#if BOARD_ROLE == 1
#include "dvc_buzzer.hpp"
#endif




void bspInit(void)
{
#if BOARD_ROLE == 1
  (void)buzzer.init();
#endif
  // BMI088_Init() moved to INS_Task to avoid HardFault during pre-scheduler initialization
  // The issue was Delay_ms() using HAL_GetTick() before FreeRTOS scheduler starts
}
