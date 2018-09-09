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

static uint16_t                       s_adc1_dma_buf[4]       = { 0U };
static uint16_t                       s_adc3_dma_buf[1]       = { 0U };

static float                          s_adc1_refint_val       = 0.f;
static float                          s_adc1_vref_mv           = 0.f;
static float                          s_adc1_solar_mv          = 0.f;
static float                          s_adc1_bat_mv            = 0.f;
static float                          s_adc1_temp             = 0.f;

static float                          s_adc3_v_pull_tcxo      = 0.f;


float adcGetVal(ADC_ENUM_t channel)
{
  switch (channel) {
  case ADC_ADC1_REFINT_VAL:
    return s_adc1_refint_val;

  case ADC_ADC1_V_VDDA:
    return s_adc1_vref_mv;

  case ADC_ADC1_INT8_V_SOLAR:
    return s_adc1_solar_mv;

  case ADC_ADC1_V_BAT:
    return s_adc1_bat_mv;

  case ADC_ADC1_TEMP:
    return s_adc1_temp;

  case ADC_ADC3_V_PULL_TCXO:
    return s_adc3_v_pull_tcxo;

  default:
    return 0.f;
  }
}


void adcStartConv(ADC_ENUM_t adc)
{
  switch (adc) {
  case ADC_ADC1_REFINT_VAL:
  case ADC_ADC1_INT8_V_SOLAR:
  case ADC_ADC1_V_BAT:
  case ADC_ADC1_TEMP:
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_V_REFINT | EG_ADC1__CONV_AVAIL_V_SOLAR | EG_ADC1__CONV_AVAIL_V_BAT | EG_ADC1__CONV_AVAIL_TEMP);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) s_adc1_dma_buf, (sizeof(s_adc1_dma_buf) / sizeof(uint16_t)));
    break;

  case ADC_ADC3_V_PULL_TCXO:
    xEventGroupClearBits(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_V_PULL_TCXO);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*) s_adc3_dma_buf, (sizeof(s_adc3_dma_buf) / sizeof(uint16_t)));
    break;

  default:
    { }
  }
}

void adcStopConv(ADC_ENUM_t adc)
{
  switch (adc) {
  case ADC_ADC1_INT8_V_SOLAR:
    HAL_ADC_Stop_DMA(&hadc1);
    break;

  case ADC_ADC3_V_PULL_TCXO:
    HAL_ADC_Stop_IT(&hadc3);
    break;

  default:
    { }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  BaseType_t      pxHigherPriorityTaskWoken = pdFALSE;

  if (hadc == &hadc1) {
    s_adc1_refint_val = s_adc1_dma_buf[0] / 16.0f;
    s_adc1_vref_mv    = (((float)VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR)) / s_adc1_refint_val) - ADC_V_OFFS_VREF_mV;
    s_adc1_solar_mv   = (s_adc1_dma_buf[1] * s_adc1_vref_mv / 65535.0f) + ADC_V_OFFS_SOLAR_mV;
    s_adc1_bat_mv     = ADC_V_MUL_BAT * ((s_adc1_dma_buf[2] * s_adc1_vref_mv / 65535.0f) + ADC_V_OFFS_BAT_mV);
    s_adc1_temp       = (s_adc1_dma_buf[3] * s_adc1_vref_mv / 65535.0f);

    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC1__CONV_AVAIL_V_REFINT | EG_ADC1__CONV_AVAIL_V_SOLAR | EG_ADC1__CONV_AVAIL_V_BAT | EG_ADC1__CONV_AVAIL_TEMP, &pxHigherPriorityTaskWoken);

  } else if (hadc == &hadc3) {
    s_adc3_v_pull_tcxo = (s_adc3_dma_buf[0] * s_adc1_vref_mv / 65535.0f) + ADC_V_OFFS_PULL_TCXO_mV;
    xEventGroupSetBitsFromISR(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_V_PULL_TCXO, &pxHigherPriorityTaskWoken);
  }
}
