/*
 * adc.c
 *
 *  Created on: 30.08.2018
 *      Author: DF4IAH
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "adc.h"


extern ADC_HandleTypeDef              hadc1;
extern ADC_HandleTypeDef              hadc3;

extern EventGroupHandle_t             adcEventGroupHandle;

extern uint16_t                       g_adc_v_solar;
extern uint16_t                       g_adc_v_pull_tcxo;


void adcStartConv(ADC_PORT_ENUM_t adc)
{
  switch (adc) {
  case ADC_PORT_V_SOLAR:
    HAL_ADC_Start_IT(&hadc1);
    break;

  case ADC_PORT_V_PULL_TCXO:
    HAL_ADC_Start_IT(&hadc3);
    break;

  default:
    { }
  }
}

void adcStopConv(ADC_PORT_ENUM_t adc)
{
  switch (adc) {
  case ADC_PORT_V_SOLAR:
    HAL_ADC_Stop_IT(&hadc1);
    break;

  case ADC_PORT_V_PULL_TCXO:
    HAL_ADC_Stop_IT(&hadc3);
    break;

  default:
    { }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  BaseType_t      pxHigherPriorityTaskWoken = pdFALSE;
  const uint16_t  adcVal                    = HAL_ADC_GetValue(hadc);

  if (hadc == &hadc1) {
    g_adc_v_solar = adcVal;
    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC__CONV_AVAIL_V_SOLAR, &pxHigherPriorityTaskWoken);

  } else if (hadc == &hadc3) {
    g_adc_v_pull_tcxo = adcVal;
    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC__CONV_AVAIL_V_PULL_TCXO, &pxHigherPriorityTaskWoken);
  }
}
