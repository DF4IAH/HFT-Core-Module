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


/* Tasks */

void tcxo20MhzTaskInit(void)
{
  osDelay(900UL);
  i2cI2c4Tcxo20MhzDacInit();

  /* Preload-value of TCXO */
  i2cI2c4Tcxo20MhzDacSet(TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VALUE);
}

void tcxo20MhzTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;
  int             dbgLen;
  char            dbgBuf[128];


  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 900UL;
  }

  /* Repeat each time period ADC conversion */
  osDelayUntil(&sf_previousWakeTime, eachMs);
  adcStartConv(ADC_PORT_V_PULL_TCXO);

  BaseType_t egBits = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC__CONV_AVAIL_V_PULL_TCXO, EG_ADC__CONV_AVAIL_V_PULL_TCXO, pdFALSE, 100 / portTICK_PERIOD_MS);
  if (egBits & EG_ADC__CONV_AVAIL_V_PULL_TCXO) {
    uint16_t l_adc_v_pull_tcxo = adcGetVpullTcxo();

    dbgLen = sprintf(dbgBuf, "ADC: Vpull  = %4d mV\r\n", (int16_t) (0.5f + ADC_V_OFFS_PULL_TCXO_mV + l_adc_v_pull_tcxo * (ADC_REFBUF_mV / 65535.0f)));
    usbLogLen(dbgBuf, dbgLen);
  }
}
