/*
 * task_Controller.c
 *
 *  Created on: 11.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"

//#include "stm32l4xx_hal_gpio.h"
#include "usb.h"
#include "bus_spi.h"

#include "task_Controller.h"


extern EventGroupHandle_t   extiEventGroupHandle;
extern osSemaphoreId        usbToHostBinarySemHandle;
extern EventGroupHandle_t   spiEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;


static void controllerInit(void)
{

}


/* Tasks */

void controllerTaskInit(void)
{
  osDelay(500UL);
  controllerInit();
}

void controllerTaskLoop(void)
{
  const uint32_t  eachMs              = 100UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 100UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);
}

