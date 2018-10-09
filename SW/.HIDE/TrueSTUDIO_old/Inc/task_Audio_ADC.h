/*
 * task_Audio_ADC.h
 *
 *  Created on: 03.10.2018
 *      Author: DF4IAH
 */

#ifndef TASK_AUDIO_ADC_H_
#define TASK_AUDIO_ADC_H_



typedef enum AUDIO_ADC_GET_TYPE_ENUM {

  AUDIO_ADC_GET_TYPE__L                                       = 1,
  AUDIO_ADC_GET_TYPE__R,

} AUDIO_ADC_GET_TYPE_t;


typedef enum audioAdcMsgAudioAdcCmds_ENUM {

  MsgAudioAdc__InitDo                                         = 0x01U,
  MsgAudioAdc__InitDone,

//MsgAudioAdc__SetVar01_x                                     = 0x41U,

  MsgAudioAdc__GetVar01_L_and_R                               = 0x81U,

  MsgAudioAdc__CallFunc01_DoMeasure                           = 0xc1U,
  MsgAudioAdc__CallFunc02_CyclicTimerEvent,
  MsgAudioAdc__CallFunc03_CyclicTimerStart,
  MsgAudioAdc__CallFunc04_CyclicTimerStop,

} audioAdcMsgAudioAdcCmds_t;



int32_t audioAdcGetValue(AUDIO_ADC_GET_TYPE_t type);

void audioAdcTimerCallback(void const *argument);


/* Task */

void audioAdcTaskInit(void);
void audioAdcTaskLoop(void);

#endif /* TASK_AUDIO_ADC_H_ */
