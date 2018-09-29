/*
 * task_Controller.c
 *
 *  Created on: 11.09.2018
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

#include "task_Controller.h"


extern osMessageQId         controllerInQueueHandle;
extern osMessageQId         controllerOutQueueHandle;
extern osSemaphoreId        usbToHostBinarySemHandle;
extern osMutexId            controllerQueueInMutexHandle;
extern osMutexId            controllerQueueOutMutexHandle;
extern EventGroupHandle_t   controllerEventGroupHandle;
extern EventGroupHandle_t   extiEventGroupHandle;
extern EventGroupHandle_t   spiEventGroupHandle;
extern osSemaphoreId        c2Default_BSemHandle;
extern osSemaphoreId        c2Tcxo_BSemHandle;
extern osSemaphoreId        c2Si5338_BSemHandle;
extern osSemaphoreId        c2Lcd_BSemHandle;
extern osSemaphoreId        c2Baro_BSemHandle;
extern osSemaphoreId        c2Hygro_BSemHandle;
extern osSemaphoreId        c2Gyro_BSemHandle;
extern osSemaphoreId        c2Ax5243_BSemHandle;
extern osSemaphoreId        c2Sx1276_BSemHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;

static ControllerMsg2Proc_t s_msg_in                          = { 0 };

static ControllerMods_t     s_mod_start                       = { 0 };
static ControllerMods_t     s_mod_rdy                         = { 0 };


uint32_t controllerCalcMsg(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd)
{
  return ((uint32_t)dst << 24U) | ((uint32_t)src << 16U) | ((uint32_t)lengthBytes << 8U) | ((uint32_t)cmd);
}

static uint32_t controllerCalcMsgInit(uint32_t* ary, ControllerMsgDestinations_t dst, uint32_t startDelayMs)
{
  ary[0] = controllerCalcMsg(dst, Destinations__Controller, sizeof(uint32_t), MsgController__InitDo);
  ary[1] = startDelayMs;
  return 2UL;
}

static void controllerMsgQueueOut(uint8_t msgLen, uint32_t* msgAry)
{
  /* Sanity checks */
  if (!msgLen || !msgAry) {
    return;
  }

  /* Get mutex to queue in */
  osMutexWait(controllerQueueOutMutexHandle, osWaitForever);

  uint8_t idx = 0U;
  while (msgLen--) {
    osMessagePut(controllerOutQueueHandle, msgAry[idx++], osWaitForever);
  }

  /* Inform destination to pull from*/
  osSemaphoreId semId = 0;
  switch ((ControllerMsgDestinations_t) (msgAry[0] >> 24)) {
  case Destinations__Main_Default:
    semId = c2Default_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Osc_TCXO:
    semId = c2Tcxo_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Osc_Si5338:
    semId = c2Si5338_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Actor_LCD:
    semId = c2Lcd_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Sensor_Baro:
    semId = c2Baro_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Sensor_Hygro:
    semId = c2Hygro_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Sensor_Gyro:
    semId = c2Gyro_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Radio_AX5243:
    semId = c2Ax5243_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Radio_SX1276:
    semId = c2Sx1276_BSemHandle;
    osSemaphoreRelease(semId);
    break;

  case Destinations__Audio_ADC:
    // TODO
    //osSemaphoreRelease(c2);
    break;

  case Destinations__Audio_DAC:
    // TODO
    //osSemaphoreRelease(c2);
    break;

  default: { }
   }

  /* Return mutex */
  osMutexRelease(controllerQueueOutMutexHandle);

  /* Door bell */
  xEventGroupSetBits(  controllerEventGroupHandle, Controller__QUEUE_OUT);
  xEventGroupClearBits(controllerEventGroupHandle, Controller__QUEUE_OUT);

  /* Request semaphore back */
  if (semId) {
    osSemaphoreWait(semId, osWaitForever);
  }
}

uint32_t controllerMsgPushToQueueIn(uint8_t msgLen, uint32_t* msgAry, uint32_t waitMs)
{
  /* Sanity checks */
  if (!msgLen || (msgLen > CONTROLLER_MSG_Q_LEN) || !msgAry) {
    return 0UL;
  }

  /* Get mutex to queue in */
  osMutexWait(controllerQueueInMutexHandle, waitMs);

  uint8_t idx = 0U;
  while (idx < msgLen) {
    osMessagePut(controllerInQueueHandle, msgAry[idx++], osWaitForever);
  }

  /* Return mutex */
  osMutexRelease(controllerQueueInMutexHandle);

  /* Door bell */
  xEventGroupSetBits(controllerEventGroupHandle, Controller__QUEUE_IN);

  return idx;
}

uint32_t controllerMsgPullFromQueueOut(uint32_t* msgAry, ControllerMsgDestinations_t dst, uint32_t waitMs)
{
  uint32_t len = 0UL;

  /* Get mutex to queue out */
  osMutexWait(controllerQueueOutMutexHandle, waitMs);

  osEvent ev = osMessagePeek(controllerOutQueueHandle, osWaitForever);
  if (ev.status == osEventMessage) {
    const uint32_t  hdr       = ev.value.v;
    uint32_t        lenBytes  = 0xffUL & (hdr >> 8U);

    if (dst == (0xffUL & (hdr >> 24))) {
      (void) osMessageGet(controllerOutQueueHandle, osWaitForever);
      msgAry[len++] = hdr;

      while (lenBytes) {
        /* Push token into array */
        ev = osMessageGet(controllerOutQueueHandle, osWaitForever);
        const uint32_t opt = ev.value.v;
        msgAry[len++] = opt;

        /* Count off bytes transferred */
        if (lenBytes > 4) {
          lenBytes -= 4U;
        } else {
          lenBytes = 0U;
        }
      }

    } else {
      /* Strip off unused message */
      const uint8_t cnt = (lenBytes + 3U) / 4U;

      for (uint8_t idx = 0; idx < cnt; idx++) {
        (void) osMessageGet(controllerOutQueueHandle, osWaitForever);
      }
    }
  }

  /* Return mutex */
  osMutexRelease(controllerQueueOutMutexHandle);

  return len;
}

static uint32_t controllerMsgPullFromQueue(void)
{
  /* Process each message token */
  do {
    const osEvent evt = osMessageGet(controllerInQueueHandle, osWaitForever);
    if (osEventMessage == evt.status) {
      const uint32_t token = evt.value.v;

      /* Starting new message or go ahead with option bytes */
      if (!s_msg_in.bRemain) {
        /* Message type */
        s_msg_in.msgDst   = (ControllerMsgDestinations_t)   (0xffUL & (token >> 24U));
        s_msg_in.msgSrc   = (ControllerMsgDestinations_t)   (0xffUL & (token >> 16U));
        s_msg_in.msgLen   =                                  0xffUL & (token >>  8U) ;
        s_msg_in.msgCmd   = (ControllerMsgControllerCmds_t) (0xffUL &  token        );

        /* Init option fields */
        s_msg_in.bRemain  = s_msg_in.msgLen;
        s_msg_in.optLen   = 0U;
        memset(s_msg_in.optAry, 0, sizeof(s_msg_in.optAry));

      } else {
        /* Option fields */
        const uint8_t cnt = min(4U, s_msg_in.bRemain);

        for (uint8_t idx = 0; idx < cnt; ++idx) {
          const uint8_t byte = (uint8_t) (0xffUL & (token >> ((3U - idx) << 3U)));
          s_msg_in.optAry[s_msg_in.optLen++] = byte;
          s_msg_in.bRemain--;
        }
      }

    } else {
      return osErrorResource;
    }
  } while (s_msg_in.bRemain);

  return osOK;
}

static void controllerMsgProcessor(void)
{
  /* Discard message that are not for us (yet) */
  if (s_msg_in.msgDst > Destinations__Controller) {
    /* Drop message */
    memset(&s_msg_in, 0, sizeof(s_msg_in));

  } else {
    /* Message is for us */

    /* Register all ready modules */
    switch (s_msg_in.msgCmd) {
    case MsgController__InitDone:
      {
        switch (s_msg_in.msgSrc) {
        case Destinations__Main_Default:
          s_mod_rdy.main_default  = 1U;
          break;

        case Destinations__Osc_TCXO:
          s_mod_rdy.osc_TCXO      = 1U;
          break;

        case Destinations__Osc_Si5338:
          s_mod_rdy.osc_Si5338    = 1U;
          break;

        case Destinations__Actor_LCD:
          s_mod_rdy.actor_LCD     = 1U;
          break;

        case Destinations__Sensor_Baro:
          s_mod_rdy.sensor_Baro   = 1U;
          break;

        case Destinations__Sensor_Hygro:
          s_mod_rdy.sensor_Hygro  = 1U;
          break;

        case Destinations__Sensor_Gyro:
          s_mod_rdy.sensor_Gyro   = 1U;
          break;

        case Destinations__Radio_AX5243:
          s_mod_rdy.radio_AX5243  = 1U;
          break;

        case Destinations__Radio_SX1276:
          s_mod_rdy.radio_SX1276  = 1U;
          break;

        case Destinations__Audio_ADC:
          s_mod_rdy.audio_ADC     = 1U;
          break;

        case Destinations__Audio_DAC:
          s_mod_rdy.audio_DAC     = 1U;
          break;

        default:
          Error_Handler();
        }  // switch (s_msg_in.msgSrc)
      }  // case MsgController__InitDone { }
    break;

    default:
      Error_Handler();
    }  // switch (s_msg_in.msgCmd)
  }  // else
}


static void controllerInit(void)
{
  /* Load configuration */

  /* Prepare all semaphores */
  {
    osSemaphoreWait(c2Default_BSemHandle, osWaitForever);
    osSemaphoreWait(c2Tcxo_BSemHandle,    osWaitForever);
    osSemaphoreWait(c2Si5338_BSemHandle,  osWaitForever);
    osSemaphoreWait(c2Lcd_BSemHandle,     osWaitForever);
    osSemaphoreWait(c2Baro_BSemHandle,    osWaitForever);
    osSemaphoreWait(c2Hygro_BSemHandle,   osWaitForever);
    osSemaphoreWait(c2Gyro_BSemHandle,    osWaitForever);
    osSemaphoreWait(c2Ax5243_BSemHandle,  osWaitForever);
    osSemaphoreWait(c2Sx1276_BSemHandle,  osWaitForever);
  }

  /* Read FLASH data */
  {

  }

  /* Set-up runtime environment */
  {
    memset(&s_msg_in,   0, sizeof(s_msg_in));
    memset(&s_mod_rdy,  0, sizeof(s_mod_rdy));

    s_mod_start.main_default                                  = 1U;

    s_mod_start.osc_TCXO                                      = 0U;
    s_mod_start.osc_Si5338                                    = 0U;
    s_mod_start.actor_LCD                                     = 1U;
    s_mod_start.sensor_Baro                                   = 1U;
    s_mod_start.sensor_Hygro                                  = 1U;
    s_mod_start.sensor_Gyro                                   = 0U;
    s_mod_start.radio_AX5243                                  = 0U;
    s_mod_start.radio_SX1276                                  = 0U;
    s_mod_start.audio_ADC                                     = 0U;
    s_mod_start.audio_DAC                                     = 0U;
  }

  /* Signaling controller is up and running */
  xEventGroupSetBits(controllerEventGroupHandle, Controller__CTRL_IS_RUNNING);
  osDelay(10UL);

  /* Send INIT message for modules that should be active */
  {
    uint32_t msgAry[2];

    /* main_default */
    if (s_mod_start.main_default) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Main_Default,
          0UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* osc_TCXO */
    if (s_mod_start.osc_TCXO) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Osc_TCXO,
          0UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* actor_LCD */
    if (s_mod_start.actor_LCD) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Actor_LCD,
          50UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* sensor_Baro */
    if (s_mod_start.sensor_Baro) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Sensor_Baro,
          200UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* sensor_Hygro */
    if (s_mod_start.sensor_Hygro) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Sensor_Hygro,
          250UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* sensor_Gyro */
    if (s_mod_start.sensor_Gyro) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Sensor_Gyro,
          300UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* radio_AX5243 */
    if (s_mod_start.radio_AX5243) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Radio_AX5243,
          400UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* radio_SX1276 */
    if (s_mod_start.radio_SX1276) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Radio_SX1276,
          450UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* osc_Si5338 */
    if (s_mod_start.osc_Si5338) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Osc_Si5338,
          600UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* audio_ADC */
    if (s_mod_start.audio_ADC) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Audio_ADC,
          700UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }

    /* audio_DAC */
    if (s_mod_start.audio_DAC) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Audio_DAC,
          750UL);
      controllerMsgQueueOut(msgLen, msgAry);
    }
  }

  //#define I2C4_BUS_ADDR_SCAN
  #ifdef I2C4_BUS_ADDR_SCAN
  i2cBusAddrScan(&hi2c4, i2c4MutexHandle);
  #endif
}


/* Tasks */

void controllerTaskInit(void)
{
  controllerInit();
}

void controllerTaskLoop(void)
{
  const EventBits_t eb = xEventGroupWaitBits(controllerEventGroupHandle,
      Controller__QUEUE_IN,
      Controller__QUEUE_IN,
      0, HAL_MAX_DELAY);

  if (Controller__QUEUE_IN & eb) {
    /* Get next complete messages */
    if (osOK != controllerMsgPullFromQueue()) {
      /* Message Queue corrupt */
      Error_Handler();
    }

    /* Process message */
    controllerMsgProcessor();
  }
}
