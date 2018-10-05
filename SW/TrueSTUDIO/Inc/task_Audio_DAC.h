/*
 * task_Audio_DAC.h
 *
 *  Created on: 03.10.2018
 *      Author: DF4IAH
 */

#ifndef TASK_AUDIO_DAC_H_
#define TASK_AUDIO_DAC_H_



typedef enum AUDIO_DAC_GET_TYPE_ENUM {

  AUDIO_DAC_PUT_TYPE__L_and_R                                 = 1,
  AUDIO_DAC_PUT_TYPE__L,
  AUDIO_DAC_PUT_TYPE__R,

} AUDIO_DAC_GET_TYPE_t;


typedef enum audioDacMsgAudioDacCmds_ENUM {

  MsgAudioDac__InitDo                                         = 0x01U,
  MsgAudioDac__InitDone,

  MsgAudioDac__SetVar01_L_and_R                               = 0x41U,
  MsgAudioDac__SetVar02_L,
  MsgAudioDac__SetVar03_R,

//MsgAudioDac__GetVar01_y                                     = 0x81U,

  MsgAudioDac__CallFunc01_DoExport                            = 0xc1U,
  MsgAudioDac__CallFunc02_CyclicTimerEvent,
  MsgAudioDac__CallFunc03_CyclicTimerStart,
  MsgAudioDac__CallFunc04_CyclicTimerStop,

} audioDacMsgAudioDacCmds_t;



void audioDacPutValue(AUDIO_DAC_GET_TYPE_t type, uint16_t val_L, uint16_t val_R);
void audioDacExport(void);

void audioDacTimerCallback(void const *argument);


/* Task */

void audioDacTaskInit(void);
void audioDacTaskLoop(void);

#endif /* TASK_AUDIO_DAC_H_ */
