/*
 * task_Controller.c
 *
 *  Created on: 11.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <task_USB.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"

//#include "stm32l4xx_hal_gpio.h"
#include "bus_spi.h"
#include "task_TCXO_20MHz.h"
#include "task_Si5338.h"
#include "task_LCD.h"
#include "task_Baro.h"
#include "task_Hygro.h"
#include "task_Gyro.h"
#include "task_Audio_ADC.h"
#include "task_Audio_DAC.h"
#include "task_AX5243.h"
#include "task_SX1276.h"

#include "task_Controller.h"


extern osMessageQId         controllerInQueueHandle;
extern osMessageQId         controllerOutQueueHandle;
extern osTimerId            controllerTimerHandle;
extern osSemaphoreId        c2AudioAdc_BSemHandle;
extern osSemaphoreId        c2AudioDac_BSemHandle;
extern osSemaphoreId        c2Ax5243_BSemHandle;
extern osSemaphoreId        c2Baro_BSemHandle;
extern osSemaphoreId        c2Default_BSemHandle;
extern osSemaphoreId        c2Gyro_BSemHandle;
extern osSemaphoreId        c2Hygro_BSemHandle;
extern osSemaphoreId        c2Lcd_BSemHandle;
extern osSemaphoreId        c2Si5338_BSemHandle;
extern osSemaphoreId        c2Sx1276_BSemHandle;
extern osSemaphoreId        c2Tcxo_BSemHandle;
extern osSemaphoreId        cQin_BSemHandle;
extern osSemaphoreId        cQout_BSemHandle;
extern osSemaphoreId        c2usbToHost_BSemHandle;
extern osSemaphoreId        c2usbFromHost_BSemHandle;
extern osSemaphoreId        usb_BSemHandle;

extern EventGroupHandle_t   globalEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;

static ControllerMsg2Proc_t s_msg_in                          = { 0 };

static ControllerMods_t     s_mod_start                       = { 0 };
static ControllerMods_t     s_mod_rdy                         = { 0 };
static uint8_t              s_controller_doCycle;
static DefaultMcuClocking_t s_controller_McuClocking          = DefaultMcuClocking_04MHz_MSI;


uint32_t controllerCalcMsgHdr(ControllerMsgDestinations_t dst, ControllerMsgDestinations_t src, uint8_t lengthBytes, uint8_t cmd)
{
  return ((uint32_t)dst << 24U) | ((uint32_t)src << 16U) | ((uint32_t)lengthBytes << 8U) | ((uint32_t)cmd);
}

static uint32_t controllerCalcMsgInit(uint32_t* ary, ControllerMsgDestinations_t dst, uint32_t startDelayMs)
{
  ary[0] = controllerCalcMsgHdr(dst, Destinations__Controller, sizeof(uint32_t), MsgController__InitDo);
  ary[1] = startDelayMs;
  return 2UL;
}


uint32_t controllerMsgPushToInQueue(uint8_t msgLen, uint32_t* msgAry, uint32_t waitMs)
{
  uint8_t idx = 0U;

  /* Sanity checks */
  if (!msgLen || (msgLen > CONTROLLER_MSG_Q_LEN) || !msgAry) {
    return 0UL;
  }

  /* Get semaphore to queue in */
  osSemaphoreWait(cQin_BSemHandle, waitMs);

  /* Push to the queue */
  while (idx < msgLen) {
    osMessagePut(controllerInQueueHandle, msgAry[idx++], osWaitForever);
  }

  /* Return semaphore */
  osSemaphoreRelease(cQin_BSemHandle);

  /* Door bell */
  xEventGroupSetBits(globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_IN);

  return idx;
}

void controllerMsgPushToOutQueue(uint8_t msgLen, uint32_t* msgAry, uint32_t waitMs)
{
  osSemaphoreId semId = 0;

  /* Get semaphore to queue out */
  if(osOK != osSemaphoreWait(cQout_BSemHandle, waitMs)) {
    return;
  }

  /* Push to the queue */
  uint8_t idx = 0U;
  while (idx < msgLen) {
    osMessagePut(controllerOutQueueHandle, msgAry[idx++], osWaitForever);
  }

  /* Ring bell at the destination */
  switch ((ControllerMsgDestinations_t) (msgAry[0] >> 24)) {
  case Destinations__Main_Default:
    semId = c2Default_BSemHandle;
    break;

  case Destinations__Network_USBtoHost:
    semId = c2usbToHost_BSemHandle;
    break;

  case Destinations__Network_USBfromHost:
    semId = c2usbFromHost_BSemHandle;
    break;

  case Destinations__Osc_TCXO:
    semId = c2Tcxo_BSemHandle;
    break;

  case Destinations__Osc_Si5338:
    semId = c2Si5338_BSemHandle;
    break;

  case Destinations__Actor_LCD:
    semId = c2Lcd_BSemHandle;
    break;

  case Destinations__Sensor_Baro:
    semId = c2Baro_BSemHandle;
    break;

  case Destinations__Sensor_Hygro:
    semId = c2Hygro_BSemHandle;
    break;

  case Destinations__Sensor_Gyro:
    semId = c2Gyro_BSemHandle;
    break;

  case Destinations__Audio_ADC:
    semId = c2AudioAdc_BSemHandle;
    break;

  case Destinations__Audio_DAC:
    semId = c2AudioDac_BSemHandle;
    break;

  case Destinations__Radio_AX5243:
    semId = c2Ax5243_BSemHandle;
    break;

  case Destinations__Radio_SX1276:
    semId = c2Sx1276_BSemHandle;
    break;

  default:
    semId = 0;
  }  // switch ()

  /* Wakeup of module */
  if (semId) {
    /* Grant blocked module to request the queue */
    osSemaphoreRelease(semId);                                                                        // Give semaphore token to launch module
  }

  /* Big Ben for the public */
  xEventGroupSetBits(  globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_OUT);
  xEventGroupClearBits(globalEventGroupHandle, EG_GLOBAL__Controller_QUEUE_OUT);

  /* Hand over ctrlQout */
  osSemaphoreRelease(cQout_BSemHandle);
}


static uint32_t controllerMsgPullFromInQueue(void)
{
  /* Prepare all fields */
  memset(&s_msg_in, 0, sizeof(s_msg_in));

  /* Process each message token */
  do {
    const osEvent evt = osMessageGet(controllerInQueueHandle, osWaitForever);
    if (osEventMessage == evt.status) {
      const uint32_t token = evt.value.v;

      /* Starting new message or go ahead with option bytes */
      if (!s_msg_in.bRemain) {
        if (!token) {
          return osErrorResource;
        }

        /* Store raw 32bit tokens */
        s_msg_in.rawLen = 0U;
        s_msg_in.rawAry[s_msg_in.rawLen++] = token;

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

        /* Store raw 32bit tokens */
        s_msg_in.rawAry[s_msg_in.rawLen++] = token;

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

uint32_t controllerMsgPullFromOutQueue(uint32_t* msgAry, ControllerMsgDestinations_t dst, uint32_t waitMs)
{
  uint32_t len = 0UL;

  /* Get semaphore to queue out */
  osSemaphoreWait(cQout_BSemHandle, waitMs);

  do {
    osEvent ev = osMessagePeek(controllerOutQueueHandle, 1UL);
    if (ev.status == osEventMessage) {
      const uint32_t  hdr       = ev.value.v;
      uint32_t        lenBytes  = 0xffUL & (hdr >> 8U);

      if (dst == (0xffUL & (hdr >> 24))) {
        (void) osMessageGet(controllerOutQueueHandle, 1UL);
        msgAry[len++] = hdr;

        while (lenBytes) {
          /* Push token into array */
          ev = osMessageGet(controllerOutQueueHandle, 1UL);
          const uint32_t opt = ev.value.v;
          msgAry[len++] = opt;

          /* Count off bytes transferred */
          if (lenBytes > 4) {
            lenBytes -= 4U;
          } else {
            lenBytes = 0U;
          }
        }
        break;

      } else {
        /* Strip off unused message */
        const uint8_t cnt = 1U + ((lenBytes + 3U) / 4U);

        for (uint8_t idx = 0; idx < cnt; idx++) {
          (void) osMessageGet(controllerOutQueueHandle, 1UL);
        }
      }
    }
  } while (1);

  /* Return semaphore */
  osSemaphoreRelease(cQout_BSemHandle);

  return len;
}


/* Timer functions */

static void controllerCyclicStart(uint32_t period_ms)
{
  osTimerStart(controllerTimerHandle, period_ms);
}

static void controllerCyclicStop(void)
{
  osTimerStop(controllerTimerHandle);
}

void controllerTimerCallback(void const *argument)
{
  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Controller, Destinations__Controller, 0U, MsgController__CallFunc01_CyclicTimerEvent);
  controllerMsgPushToInQueue(msgLen, msgAry, 1UL);
}

static void controllerCyclicTimerEvent(void)
{
  /* Cyclic job to do */
  // TODO: coding
  __asm volatile( "nop" );
}


static void controllerMsgProcessor(void)
{
  uint32_t msgAry[CONTROLLER_MSG_Q_LEN] = { 0 };

  if (s_msg_in.msgDst > Destinations__Controller) {
    /* Forward message to the destination via the ctrlQout */
    const uint8_t cnt                     = s_msg_in.rawLen;
    uint8_t msgLen                        = 0U;

    /* Copy message header and option entries to the target */
    for (uint8_t idx = 0; idx < cnt; ++idx) {
      msgAry[msgLen++] = s_msg_in.rawAry[idx];
    }

    /* Push message out */
    controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);

  } else {
    /* Message is for us */

    /* Register all ready modules */
    switch (s_msg_in.msgCmd) {
    case MsgController__InitDone:
      {
        switch (s_msg_in.msgSrc) {
        case Destinations__Main_Default:
          s_mod_rdy.rtos_Default = 1U;

          /* Send MCU clocking set-up request */
          {
            uint8_t msgLen    = 0U;

            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Main_Default, Destinations__Controller, 1U, MsgDefault__SetVar02_Clocking);
            msgAry[msgLen++]  = (s_controller_McuClocking << 24U);
            controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
          }
          break;

        case Destinations__Network_USBtoHost:
          s_mod_rdy.network_USBtoHost = 1U;
          break;

        case Destinations__Network_USBfromHost:
          s_mod_rdy.network_USBfromHost = 1U;
          break;

        case Destinations__Osc_TCXO:
          s_mod_rdy.osc_TCXO = 1U;

          /* Activate MCU HSE clocking */
          if (s_mod_rdy.osc_Si5338) {
            uint8_t  msgLen                       = 0U;

            s_controller_McuClocking = DefaultMcuClocking_80MHz_HSE20_PLL;

            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Main_Default, Destinations__Controller, 1U, MsgDefault__SetVar02_Clocking);
            msgAry[msgLen++]  = (s_controller_McuClocking << 24U);
            controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
          }
          break;

        case Destinations__Osc_Si5338:
          s_mod_rdy.osc_Si5338 = 1U;

          /* Switch Si5338 to the desired mode */
          {
            /* Set variant */
            {
              uint8_t msgLen    = 0U;
              msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Osc_Si5338, Destinations__Controller, 1U, MsgSi5338__SetVar01_Variant);
              if (s_mod_rdy.osc_TCXO) {
                msgAry[msgLen++] = (I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ << 24U);
              } else {
                msgAry[msgLen++] = (I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ << 24U);
              }
              controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
            }

            /* Execute */
            {
              uint8_t msgLen = 0U;
              msgAry[msgLen++] = controllerCalcMsgHdr(Destinations__Osc_Si5338, Destinations__Controller, 0U, MsgSi5338__CallFunc04_Execute);
              controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
            }
          }

          /* Activate MCU HSE clocking */
          if (s_mod_rdy.osc_TCXO) {
            s_controller_McuClocking = DefaultMcuClocking_80MHz_HSE20_PLL;

            uint8_t msgLen    = 0U;
            msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Main_Default, Destinations__Controller, 1U, MsgDefault__SetVar02_Clocking);
            msgAry[msgLen++]  = (s_controller_McuClocking << 24U);
            controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
          }
          break;

        case Destinations__Actor_LCD:
          s_mod_rdy.actor_LCD     = 1U;

          /* Welcome Text */
          {
            const uint8_t strBuf[]                  = "*HFT-CoreModule*";
            const uint8_t strLen                    = strlen((const char*) strBuf);
            uint8_t   msgLen                        = 0U;

            msgAry[msgLen++] = controllerCalcMsgHdr(Destinations__Actor_LCD, Destinations__Controller, 1U + strLen, MsgLcd__CallFunc05_WriteString);
            uint32_t word = 0x00U << 24U;                                                             // Display @ Row=0, Col=0
            uint8_t wordPos = 2;

            for (uint8_t strIdx = 0U; strIdx < strLen; ++strIdx) {
              word |= (0xffU & strBuf[strIdx]) << (wordPos << 3U);

              if (wordPos) {
                --wordPos;

              } else {
                msgAry[msgLen++] = word;

                word = 0UL;
                wordPos = 3U;
              }
            }

            /* Add last fraction */
            if ((1U + strLen) % 4) {
              msgAry[msgLen++] = word;
            }

            /* Put message into ControllerQueueOut */
            controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
          }
          break;

        case Destinations__Sensor_Baro:
          {
            s_mod_rdy.sensor_Baro   = 1U;

            /* Start cyclic measurements */
            {
              uint8_t msgLen    = 0U;
              msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Sensor_Baro, Destinations__Controller, 4U, MsgBaro__CallFunc02_CyclicTimerStart);
              msgAry[msgLen++]  = 1000UL;                                                             // Period in ms
              controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
            }
          }
          break;

        case Destinations__Sensor_Hygro:
          {
            s_mod_rdy.sensor_Hygro  = 1U;

            /* Start cyclic measurements */
            {
              uint8_t msgLen    = 0U;
              msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Sensor_Hygro, Destinations__Controller, 4U, MsgHygro__CallFunc02_CyclicTimerStart);
              msgAry[msgLen++]  = 1000UL;                                                             // Period in ms
              controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
            }
          }
          break;

        case Destinations__Sensor_Gyro:
          s_mod_rdy.sensor_Gyro   = 1U;
          break;

        case Destinations__Audio_ADC:
          {
            s_mod_rdy.audio_ADC     = 1U;

            /* Start cyclic measurements */
            {
              uint8_t msgLen    = 0U;
              msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Audio_ADC, Destinations__Controller, 4U, MsgAudioAdc__CallFunc02_CyclicTimerStart);
              msgAry[msgLen++]  = 1000UL;                                                               // Period in ms
              controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
            }
          }
          break;

        case Destinations__Audio_DAC:
          {
            s_mod_rdy.audio_DAC     = 1U;

            /* Start cyclic measurements */
            {
              uint8_t msgLen    = 0U;
              msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Audio_DAC, Destinations__Controller, 4U, MsgAudioDac__CallFunc02_CyclicTimerStart);
              msgAry[msgLen++]  = 1000UL;                                                               // Period in ms
              controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
            }
          }
          break;

        case Destinations__Radio_AX5243:
          s_mod_rdy.radio_AX5243  = 1U;
          break;

        case Destinations__Radio_SX1276:
          s_mod_rdy.radio_SX1276  = 1U;
          break;

        default:
          Error_Handler();
        }  // switch (s_msg_in.msgSrc)
      }  // case MsgController__InitDone { }
    break;

    /* Process own command */
    case MsgController__CallFunc01_CyclicTimerEvent:
    {
      controllerCyclicTimerEvent();
    }
    break;

    default:
      Error_Handler();
    }  // switch (s_msg_in.msgCmd)
  }  // else

  /* Discard message */
  memset(&s_msg_in, 0, sizeof(s_msg_in));
}


static void controllerInit(void)
{
  /* Load configuration */

  /* Prepare all semaphores */
  {
    osSemaphoreWait(c2Default_BSemHandle,     osWaitForever);
    osSemaphoreWait(c2AudioAdc_BSemHandle,    osWaitForever);
    osSemaphoreWait(c2AudioDac_BSemHandle,    osWaitForever);
    osSemaphoreWait(c2Ax5243_BSemHandle,      osWaitForever);
    osSemaphoreWait(c2Baro_BSemHandle,        osWaitForever);
    osSemaphoreWait(c2Gyro_BSemHandle,        osWaitForever);
    osSemaphoreWait(c2Hygro_BSemHandle,       osWaitForever);
    osSemaphoreWait(c2Lcd_BSemHandle,         osWaitForever);
    osSemaphoreWait(c2Si5338_BSemHandle,      osWaitForever);
    osSemaphoreWait(c2Sx1276_BSemHandle,      osWaitForever);
    osSemaphoreWait(c2Tcxo_BSemHandle,        osWaitForever);
    osSemaphoreWait(c2usbToHost_BSemHandle,   osWaitForever);
    osSemaphoreWait(c2usbFromHost_BSemHandle, osWaitForever);
  }

  /* Read FLASH data */
  {

  }

  /* Set-up runtime environment */
  {
    memset(&s_msg_in,   0, sizeof(s_msg_in));
    memset(&s_mod_rdy,  0, sizeof(s_mod_rdy));

    s_controller_McuClocking                                  = DefaultMcuClocking_16MHz_MSI;

    s_controller_doCycle                                      = 0U;

    s_mod_start.rtos_Default                                  = 1U;
    s_mod_start.network_USBtoHost                             = 1U;
    s_mod_start.network_USBfromHost                           = 1U;
    s_mod_start.osc_TCXO                                      = 0U;
    s_mod_start.osc_Si5338                                    = 0U;
    s_mod_start.actor_LCD                                     = 1U;
    s_mod_start.sensor_Baro                                   = 1U;
    s_mod_start.sensor_Hygro                                  = 1U;
    s_mod_start.sensor_Gyro                                   = 0U;
    s_mod_start.audio_ADC                                     = 0U;
    s_mod_start.audio_DAC                                     = 0U;
    s_mod_start.radio_AX5243                                  = 0U;
    s_mod_start.radio_SX1276                                  = 0U;
  }

  /* Signaling controller is up and running */
  xEventGroupSetBits(globalEventGroupHandle, EG_GLOBAL__Controller_CTRL_IS_RUNNING);
  osDelay(10UL);

  /* Send INIT message for modules that should be active */
  {
    uint32_t msgAry[2];

    /* main_default */
    if (s_mod_start.rtos_Default) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Main_Default,
          0UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_USBtoHost */
    if (s_mod_start.network_USBtoHost) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_USBtoHost,
          50UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* network_USBfromHost */
    if (s_mod_start.network_USBfromHost) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Network_USBfromHost,
          75UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* osc_TCXO */
    if (s_mod_start.osc_TCXO) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Osc_TCXO,
          100UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* actor_LCD */
    if (s_mod_start.actor_LCD) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Actor_LCD,
          150UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* sensor_Baro */
    if (s_mod_start.sensor_Baro) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Sensor_Baro,
          250UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* sensor_Hygro */
    if (s_mod_start.sensor_Hygro) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Sensor_Hygro,
          400UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* sensor_Gyro */
    if (s_mod_start.sensor_Gyro) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Sensor_Gyro,
          450UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* osc_Si5338 */
    if (s_mod_start.osc_Si5338) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Osc_Si5338,
          600UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* audio_ADC */
    if (s_mod_start.audio_ADC) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Audio_ADC,
          750UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* audio_DAC */
    if (s_mod_start.audio_DAC) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Audio_DAC,
          800UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* radio_AX5243 */
    if (s_mod_start.radio_AX5243) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Radio_AX5243,
          850UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }

    /* radio_SX1276 */
    if (s_mod_start.radio_SX1276) {
      const uint32_t msgLen = controllerCalcMsgInit(msgAry,
          Destinations__Radio_SX1276,
          950UL);
      controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
    }
  }

  /* Enable service cycle */
  if (s_controller_doCycle) {
    controllerCyclicStart(1000UL);
  } else {
    controllerCyclicStop();
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
  const EventBits_t eb = xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_QUEUE_IN,
      EG_GLOBAL__Controller_QUEUE_IN,
      0, HAL_MAX_DELAY);

  if (EG_GLOBAL__Controller_QUEUE_IN & eb) {
    /* Get next complete messages */
    if (osOK != controllerMsgPullFromInQueue()) {
      /* Message Queue corrupt */
      Error_Handler();
    }

    /* Process message */
    controllerMsgProcessor();
  }
}
