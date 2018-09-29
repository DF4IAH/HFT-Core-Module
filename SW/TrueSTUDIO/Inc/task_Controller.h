/*
 * task_Controller.h
 *
 *  Created on: 11.09.2018
 *      Author: DF4IAH
 */

#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include "stm32l4xx_hal.h"
#include "main.h"


#define CONTROLLER_MSG_Q_LEN                                  8


/* Bit-mask for the controllerEventGroup */
typedef enum EG_Controller_BM {

  Controller__QUEUE_IN                                        = 0x00000001UL,
  Controller__QUEUE_OUT                                       = 0x00000002UL,

  Controller__CTRL_IS_RUNNING                                 = 0x00800000UL,

} EG_Controller_BM_t;



typedef enum ControllerMsgDestinations_ENUM {

  Destinations__Unspec                                        = 0,
  Destinations__Controller,
  Destinations__Main_Default,

  Destinations__Osc_TCXO,
  Destinations__Osc_Si5338,

  Destinations__Actor_LCD,

  Destinations__Sensor_Baro,
  Destinations__Sensor_Hygro,
  Destinations__Sensor_Gyro,

  Destinations__Radio_AX5243,
  Destinations__Radio_SX1276,

  Destinations__Audio_ADC,
  Destinations__Audio_DAC,

} ControllerMsgDestinations_t;


typedef struct ControllerMods {

  uint8_t                           main_default;
  uint8_t                           osc_TCXO;
  uint8_t                           osc_Si5338;
  uint8_t                           actor_LCD;
  uint8_t                           sensor_Baro;
  uint8_t                           sensor_Hygro;
  uint8_t                           sensor_Gyro;
  uint8_t                           radio_AX5243;
  uint8_t                           radio_SX1276;
  uint8_t                           audio_ADC;
  uint8_t                           audio_DAC;

} ControllerMods_t;


typedef enum ControllerMsgControllerCmds_ENUM {

  MsgController__InitDo                                       = 0x01U,
  MsgController__InitDone,

  MsgController__SetVar01                                     = 0x41U,
  MsgController__SetVar02,
  MsgController__SetVar03,
  MsgController__SetVar04,
  MsgController__SetVar05,

  MsgController__GetVar01                                     = 0x81U,
  MsgController__GetVar02,
  MsgController__GetVar03,
  MsgController__GetVar04,
  MsgController__GetVar05,

  MsgController__CallFunc01                                   = 0xc1U,
  MsgController__CallFunc02,
  MsgController__CallFunc03,
  MsgController__CallFunc04,
  MsgController__CallFunc05,

} ControllerMsgControllerCmds_t;


typedef struct ControllerMsg2Proc {

  ControllerMsgDestinations_t         msgSrc;
  ControllerMsgDestinations_t         msgDst;                                                         // Not in use yet
  ControllerMsgControllerCmds_t       msgCmd;
  uint8_t                             msgLen;

  uint8_t                             bRemain;

  uint8_t                             optLen;
  uint8_t                             optAry[CONTROLLER_MSG_Q_LEN << 2];

} ControllerMsg2Proc_t;



uint32_t controllerCalcMsg(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd);

uint32_t controllerMsgPushToQueueIn(uint8_t msgLen, uint32_t* msgAry, uint32_t waitMs);
uint32_t controllerMsgPullFromQueueOut(uint32_t* msgAry, ControllerMsgDestinations_t dst, uint32_t waitMs);

void controllerTaskInit(void);
void controllerTaskLoop(void);

#endif /* TASK_CONTROLLER_H_ */
