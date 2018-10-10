/*
 * task_Audio_DAC.c
 *
 *  Created on: 03.10.2018
 *      Author: DF4IAH
 */

#include <string.h>
#include <math.h>
#include <task_USB.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "bus_spi.h"
#include "task_Controller.h"

#include "task_Audio_DAC.h"


extern osTimerId            audioDacTimerHandle;
extern osSemaphoreId        c2AudioDac_BSemHandle;
extern osSemaphoreId        spi3_BSemHandle;

//extern EventGroupHandle_t   extiEventGroupHandle;
extern EventGroupHandle_t   spiEventGroupHandle;
extern EventGroupHandle_t   globalEventGroupHandle;

extern uint8_t              spi3TxBuffer[SPI3_BUFFERSIZE];
extern uint8_t              spi3RxBuffer[SPI3_BUFFERSIZE];


static uint8_t              s_audio_dac_enable;
static uint32_t             s_audioDacStartTime;
static uint16_t             s_audioDacValue_L;
static uint16_t             s_audioDacValue_R;


void audioDacPutValue(AUDIO_DAC_GET_TYPE_t type, uint16_t val_L, uint16_t val_R)
{
  if (s_audio_dac_enable) {
    switch (type) {
    case AUDIO_DAC_PUT_TYPE__L_and_R:
      s_audioDacValue_L = val_L;
      s_audioDacValue_R = val_R;
      break;

    case AUDIO_DAC_PUT_TYPE__L:
      s_audioDacValue_L = val_L;
      break;

    case AUDIO_DAC_PUT_TYPE__R:
      s_audioDacValue_R = val_R;
      break;

    default:
      { }
    }
  }
}

void audioDacExport(void)
{
  // TODO: coding
  #if 0
  /* DAC */
  {
    const uint8_t txMsgL[3] = { 0x31U, s_audioDacValue_L, 0 };
    const uint8_t txMsgR[3] = { 0x32U, s_audioDacValue_R, 0 };

    spiProcessSpi3MsgTemplate(SPI3_DAC, sizeof(txMsgL), txMsgL);
    spiProcessSpi3MsgTemplate(SPI3_DAC, sizeof(txMsgR), txMsgR);
  }

  /* Stimulus */
  s_audioDacValue_L += 0x10U;
  s_audioDacValue_R -= 0x10U;
  #endif
}

static void audioDacDistributor(void)
{
  /* USB Logging */
  {
    int   dbgLen = 0;
    char  dbgBuf[128];

    dbgLen = sprintf(dbgBuf, "\r\nAUDIO_DAC:\t Left = %5u\t Right = %5u\r\n", s_audioDacValue_L, s_audioDacValue_R);
    usbLogLen(dbgBuf, dbgLen);
  }
}


/* Timer functions */

static void audioDacCyclicTimerStart(uint32_t period_ms)
{
  osTimerStart(audioDacTimerHandle, period_ms);
}

static void audioDacCyclicTimerStop(void)
{
  osTimerStop(audioDacTimerHandle);
}

void audioDacTimerCallback(void const *argument)
{
  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Audio_DAC, Destinations__Audio_DAC, 0U, MsgAudioDac__CallFunc01_CyclicTimerEvent);
  controllerMsgPushToOutQueue(msgLen, msgAry, 1UL);
}

static void audioDacCyclicTimerEvent(void)
{
  audioDacExport();
  audioDacDistributor();
}


static void audioDacInit(void)
{
  usbLog("< AudioDacInit -\r\n");

  // TODO: coding
  /* Check if Si5338 is active first */

  #if 0
  PowerSwitchDo(POWERSWITCH__3V3_HICUR, 1U);
  __HAL_RCC_GPIOC_CLK_ENABLE();
  osDelay(5UL);
  #endif

  usbLog("- AudioDacInit >\r\n\r\n");
}


static void audioDacMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                        msgIdx  = 0UL;
  const uint32_t                  hdr     = msgAry[msgIdx++];
  const audioDacMsgAudioDacCmds_t cmd     = (audioDacMsgAudioDacCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgAudioDac__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_audioDacStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      audioDacInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Audio_DAC, 0U, MsgAudioDac__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgAudioDac__SetVar01_L_and_R:
    {
      /* Put the values */
      audioDacPutValue(AUDIO_DAC_PUT_TYPE__L_and_R, msgAry[1], msgAry[2]);
    }
    break;

  case MsgAudioDac__SetVar02_L:
    {
      /* Put the value */
      audioDacPutValue(AUDIO_DAC_PUT_TYPE__L, msgAry[1], 0U);
    }
    break;

  case MsgAudioDac__SetVar03_R:
    {
      /* Put the value */
      audioDacPutValue(AUDIO_DAC_PUT_TYPE__R, 0U, msgAry[1]);
    }
    break;

  case MsgAudioDac__CallFunc01_CyclicTimerEvent:
    {
      audioDacCyclicTimerEvent();
    }
    break;

  case MsgAudioDac__CallFunc02_CyclicTimerStart:
    {
      /* Start cyclic measurements with that period in ms */
      audioDacCyclicTimerStart(msgAry[msgIdx++]);
    }
    break;

  case MsgAudioDac__CallFunc03_CyclicTimerStop:
    {
      /* Stop cyclic measurements */
      audioDacCyclicTimerStop();
    }
    break;

  case MsgAudioDac__CallFunc04_DoExport:
    {
      /* Get the values */
      audioDacExport();
    }
    break;

  default: { }
  }  // switch (cmd)
}


/* Task */

void audioDacTaskInit(void)
{
  s_audio_dac_enable = 0U;

  /* Out values */
  s_audioDacValue_L = 0U;
  s_audioDacValue_R = 0U;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_audioDacStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void audioDacTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2AudioDac_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Audio_DAC, 1UL);                     // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    audioDacMsgProcess(msgLen, msgAry);
  }
}
