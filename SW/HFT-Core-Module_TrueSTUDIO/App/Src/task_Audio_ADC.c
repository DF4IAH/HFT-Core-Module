/*
 * task_Audio_ADC.c
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

#include "task_Audio_ADC.h"


extern osTimerId            audioAdcTimerHandle;
extern osSemaphoreId        c2AudioAdc_BSemHandle;
extern osSemaphoreId        spi3_BSemHandle;

//extern EventGroupHandle_t   extiEventGroupHandle;
extern EventGroupHandle_t   spiEventGroupHandle;
extern EventGroupHandle_t   globalEventGroupHandle;

extern SPI_HandleTypeDef    hspi3;
extern DMA_HandleTypeDef    hdma_spi3_rx;
extern DMA_HandleTypeDef    hdma_spi3_tx;

extern uint8_t              spi3TxBuffer[SPI3_BUFFERSIZE];
extern uint8_t              spi3RxBuffer[SPI3_BUFFERSIZE];


static uint8_t              s_audio_adc_enable;
static uint32_t             s_audioAdcStartTime;
static uint16_t             s_audioAdcValue_L;
static uint16_t             s_audioAdcValue_R;


int32_t audioAdcGetValue(AUDIO_ADC_GET_TYPE_t type)
{
  if (s_audio_adc_enable) {
    switch (type) {
    case AUDIO_ADC_GET_TYPE__L:
      return s_audioAdcValue_L;

    case AUDIO_ADC_GET_TYPE__R:
      return s_audioAdcValue_R;

    default:
      { }
    }
  }

  return 0L;
}

static void audioAdcFetch(void)
{
  // TODO: coding
  #if 0
  const uint8_t txMsg_0x00_RdAdcs[7] = { ((0x00U << 1U) | 0x01U),                                   // Read address 0x00
      0U
  };
  spiProcessSpi3MsgTemplateLocked(SPI3_ADC, sizeof(txMsg_0x00_RdAdcs), txMsg_0x00_RdAdcs, 1U);
  s_audioAdcValue_L = ((uint16_t)spi3RxBuffer[1] << 8U) | spi3RxBuffer[2];
  s_audioAdcValue_R = ((uint16_t)spi3RxBuffer[3] << 8U) | spi3RxBuffer[4];
  osSemaphoreRelease(spi3_BSemHandle);
  #endif
}

static void audioAdcCalc(void)
{
}

static void audioAdcDistributor(void)
{
  /* USB Logging */
  {
    int   dbgLen = 0;
    char  dbgBuf[128];

    dbgLen = sprintf(dbgBuf, "\r\nAUDIO_ADC:\t Left = %5u\t Right = %5u\r\n", s_audioAdcValue_L, s_audioAdcValue_R);
    usbLogLen(dbgBuf, dbgLen);
  }
}


void audioAdcDoMeasure(void)
{
  audioAdcFetch();
  audioAdcCalc();
}


/* Timer functions */

static void audioAdcCyclicTimerStart(uint32_t period_ms)
{
  osTimerStart(audioAdcTimerHandle, period_ms);
}

static void audioAdcCyclicTimerStop(void)
{
  osTimerStop(audioAdcTimerHandle);
}

void audioAdcTimerCallback(void const *argument)
{
  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Audio_ADC, Destinations__Audio_ADC, 0U, MsgAudioAdc__CallFunc01_CyclicTimerEvent);
  controllerMsgPushToOutQueue(msgLen, msgAry, 1UL);
}

static void audioAdcCyclicTimerEvent(void)
{
  audioAdcDoMeasure();
  audioAdcDistributor();
}


static void audioAdcInit(void)
{
  usbLog("< AudioAdcInit -\r\n");

  /* SPI3 init */
  spix_Init(&hspi3, spi3_BSemHandle);

  // TODO: coding
  /* Check if Si5338 is active first */

  #if 0
  /* Disable RESET of AUDIO_ADC */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_GPIO_WritePin(MCU_OUT_AUDIO_ADC_nRESET_GPIO_Port, MCU_OUT_AUDIO_ADC_nRESET_Pin, GPIO_PIN_SET);
  osDelay(5UL);

  /* Init conversions */
  {
    const uint8_t txMsg_0x0d_Reset[2]     = { ((0x0dU << 1U) | 0x00U),                                // Write address 0x0D
        0xc2U                                                                                         // Reset both ADCs
    };
    const uint8_t txMsg_0x07_Config[14]   = { ((0x07U << 1U) | 0x00U),                                // Write address 0x07
        0x00U, 0x00U,                                                                                 // No phase delay between ADC1 and ADC2
        0b10101101U,                                                                                  // Gain=32 and Boost=1x (0b10)
        0b00010011U, 0b10100100U,                                                                     // DR:PP, 16bit, EN_OFFCAL=1
        0b00011110U, 0b00000010U,                                                                     // AMCLK=MCLK, OSR=256,
        0xfdU, 0x70U, 0x00U,                                                                          // Offset CH0
        0x00U, 0x00U, 0x00U                                                                           // Offset CH1
    };

    spiProcessSpi3MsgTemplate(SPI3_ADC, sizeof(txMsg_0x0d_Reset),   txMsg_0x0d_Reset);
    spiProcessSpi3MsgTemplate(SPI3_ADC, sizeof(txMsg_0x07_Config),  txMsg_0x07_Config);
  }
  #endif

  usbLog("- AudioAdcInit >\r\n\r\n");
}


static void audioAdcMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                        msgIdx  = 0UL;
  const uint32_t                  hdr     = msgAry[msgIdx++];
  const audioAdcMsgAudioAdcCmds_t cmd     = (audioAdcMsgAudioAdcCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgAudioAdc__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_audioAdcStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      audioAdcInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Audio_ADC, 0U, MsgAudioAdc__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgAudioAdc__GetVar01_L_and_R:
    {
      /* Send them to the controller */
      {
        uint32_t cmdBack[3];

        cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Audio_ADC, sizeof(cmdBack) - 4U, MsgAudioAdc__CallFunc04_DoMeasure);
        cmdBack[1] = s_audioAdcValue_L;
        cmdBack[2] = s_audioAdcValue_R;

        controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
      }
    }
    break;

  case MsgAudioAdc__CallFunc01_CyclicTimerEvent:
    {
      audioAdcCyclicTimerEvent();
    }
    break;

  case MsgAudioAdc__CallFunc02_CyclicTimerStart:
    {
      /* Start cyclic measurements with that period in ms */
      audioAdcCyclicTimerStart(msgAry[msgIdx++]);
    }
    break;

  case MsgAudioAdc__CallFunc03_CyclicTimerStop:
    {
      /* Stop cyclic measurements */
      audioAdcCyclicTimerStop();
    }
    break;

  case MsgAudioAdc__CallFunc04_DoMeasure:
    {
      /* Get the values */
      audioAdcDoMeasure();

      /* Send them to the controller */
      {
        uint32_t cmdBack[3];

        cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Audio_ADC, sizeof(cmdBack) - 4U, MsgAudioAdc__CallFunc04_DoMeasure);
        cmdBack[1] = s_audioAdcValue_L;
        cmdBack[2] = s_audioAdcValue_R;

        controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
      }
    }
    break;

  default: { }
  }  // switch (cmd)
}


/* Task */

void audioAdcTaskInit(void)
{
  s_audio_adc_enable = 0U;

  /* Out values */
  s_audioAdcValue_L = 0U;
  s_audioAdcValue_R = 0U;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_audioAdcStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void audioAdcTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2AudioAdc_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Audio_ADC, 1UL);                     // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    audioAdcMsgProcess(msgLen, msgAry);
  }
}
