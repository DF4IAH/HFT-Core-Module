/*
 * tcxo_20MHz.c
 *
 *  Created on: 30.08.2018
 *      Author: DF4IAH
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "usb.h"

#include "i2c.h"


/* Tasks */

void tcxo20MhzTaskInit(void)
{
  osDelay(500);
  i2cI2c4Tcxo20MhzDacInit();

  /* Preload-value of TCXO */
  i2cI2c4Tcxo20MhzDacSet(TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VALUE);
}

void tcxo20MhzTaskLoop(void)
{
  osDelay(100);
}
