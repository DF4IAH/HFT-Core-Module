/*
 * adc.h
 *
 *  Created on: 30.08.2018
 *      Author: DF4IAH
 */

#ifndef ADC_H_
#define ADC_H_


#define ADC_REFBUF_mV                 2048UL
#define ADC_V_OFFS_PULL_TCXO_mV       38L


typedef enum ADC_PORT_ENUM {

  ADC_PORT_V_SOLAR                    = 1,
  ADC_PORT_V_PULL_TCXO                = 3,

} ADC_PORT_ENUM_t;


/* Event Group ADC */
typedef enum EG_ADC_ENUM {

  EG_ADC__CONV_AVAIL_V_SOLAR          = 0x0001U,
  EG_ADC__CONV_AVAIL_V_PULL_TCXO      = 0x0002U,

} EG_ADC_ENUM_t;


void adcStartConv(ADC_PORT_ENUM_t adc);
void adcStopConv(ADC_PORT_ENUM_t adc);

#endif /* ADC_H_ */
