/*
 * adc.h
 *
 *  Created on: 30.08.2018
 *      Author: DF4IAH
 */

#ifndef ADC_H_
#define ADC_H_


#define ADC_V_OFFS_VREF_mV            38.8f
#define ADC_V_OFFS_REFINT_mV          38.0f
#define ADC_V_OFFS_SOLAR_mV           38.f
#define ADC_V_MUL_BAT                 3.076f
#define ADC_V_OFFS_BAT_mV             38.0f
#define ADC_V_OFFS_TEMP_mC            -273150L

#define ADC_V_OFFS_PULL_TCXO_mV       38.0f


typedef enum ADC_ENUM {

  ADC_ADC1_REFINT_VAL                 = 0x00,
  ADC_ADC1_INT8_V_SOLAR,
  ADC_ADC1_V_BAT,
  ADC_ADC1_TEMP,
  ADC_ADC1_V_VDDA                     = 0x08,

  ADC_ADC3_V_PULL_TCXO                = 0x10,

} ADC_ENUM_t;


/* Event Group ADC */
typedef enum EG_ADC_ENUM {

  EG_ADC1__CONV_AVAIL_V_REFINT        = 0x0001U,
  EG_ADC1__CONV_AVAIL_V_SOLAR         = 0x0002U,
  EG_ADC1__CONV_AVAIL_V_BAT           = 0x0004U,
  EG_ADC1__CONV_AVAIL_TEMP            = 0x0008U,

  EG_ADC3__CONV_AVAIL_V_PULL_TCXO     = 0x0010U,

} EG_ADC_ENUM_t;


float adcGetVal(ADC_ENUM_t channel);

void adcStartConv(ADC_ENUM_t adc);
void adcStopConv(ADC_ENUM_t adc);

#endif /* ADC_H_ */
