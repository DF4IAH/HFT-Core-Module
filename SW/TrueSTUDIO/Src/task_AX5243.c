/*
 * task_AX5243.c
 *
 *  Created on: 10.09.2018
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

#include "task_AX5243.h"


extern EventGroupHandle_t   extiEventGroupHandle;
extern osSemaphoreId        usbToHostBinarySemHandle;
extern EventGroupHandle_t   spiEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;

extern uint8_t              spi3TxBuffer[SPI3_BUFFERSIZE];
extern uint8_t              spi3RxBuffer[SPI3_BUFFERSIZE];


uint8_t spiDetectAx5243(void)
{
  /* Request RD-address 0x000 SILICONREV */
  {
    uint8_t axVersion = 0;

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x00;
    if (HAL_OK == spiProcessSpi3Msg(SPI3_AX, 2)) {
      axVersion = spi3RxBuffer[1];
    }

    if (axVersion != 0x51) {                                                                    // AX5243
      /* We can handle Version  0x51 (AX5243) only */
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}


/* Tasks */

