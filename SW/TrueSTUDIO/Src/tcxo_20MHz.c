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
#include "adc.h"

#include "tcxo_20MHz.h"


extern EventGroupHandle_t             adcEventGroupHandle;

extern uint16_t                       g_adc_v_solar;
extern uint16_t                       g_adc_v_pull_tcxo;



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
  const uint32_t  eachMs        = 1000UL;
  int   dbgLen;
  char  dbgBuf[128];

  static uint32_t previousWakeTime = 0UL;
  if (!previousWakeTime) {
    previousWakeTime = osKernelSysTick();
  }

  /* Repeat each time period ADC conversion */
  osDelayUntil(&previousWakeTime, eachMs);
  adcStartConv(ADC_PORT_V_PULL_TCXO);

  BaseType_t egBits = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC__CONV_AVAIL_V_PULL_TCXO, EG_ADC__CONV_AVAIL_V_PULL_TCXO, pdFALSE, 100 / portTICK_PERIOD_MS);
  if (egBits & EG_ADC__CONV_AVAIL_V_PULL_TCXO) {
    dbgLen = sprintf(dbgBuf, "ADC: value = %4d mV\r\n", (int16_t) (0.5f + ADC_V_OFFS_PULL_TCXO_mV + g_adc_v_pull_tcxo * (ADC_REFBUF_mV / 65535.0f)));
    usbLogLen(dbgBuf, dbgLen);
  }
}
