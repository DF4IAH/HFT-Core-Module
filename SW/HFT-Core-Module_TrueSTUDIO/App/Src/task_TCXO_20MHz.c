/*
 * task_TCXO_20MHz.c
 *
 *  Created on: 30.08.2018
 *      Author: DF4IAH
 */

#include <device_adc.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "bus_i2c.h"
#include "usb.h"
#include "task_Controller.h"

#include "task_TCXO_20MHz.h"


extern osTimerId            tcxoTimerHandle;
extern osThreadId           tcxo20MhzTaskHandle;
extern osSemaphoreId        c2Tcxo_BSemHandle;
extern osSemaphoreId        i2c4_BSemHandle;
extern I2C_HandleTypeDef    hi2c4;

extern EventGroupHandle_t   globalEventGroupHandle;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

static uint8_t              s_tcxo_enable;
static uint16_t             s_tcxo20MhzDacStatus;
static uint32_t             s_tcxoVoltage_uV;
static uint32_t             s_tcxoStartTime;


static uint16_t tcxoCalcVoltage2DacValue(uint32_t voltage_uV)
{
  return (uint16_t) (((uint64_t)voltage_uV * TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VALUE) / TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VOLTAGE_UV);
}

static void tcxo20MhzDacInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  usbLog("< Tcxo20MhzDacInit -\r\n");

  {
    const uint8_t i2cWriteLongAry[2] = {
      0x00, 0x20                                                                                      // 0x0020: clear value Mid, nAUX disable, DAC enabled
    };

    uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_USER_CONFIG, sizeof(i2cWriteLongAry), i2cWriteLongAry);
    if (i2cErr == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". Tcxo20MhzDacInit: ERROR DAC does not respond\r\n");
      goto tcxo20MhzDacInit_out;
    }
  }

  {
    const uint8_t i2cWriteLongAry[2] = {
      0x00, 0x00                                                                                      // DC
    };

    i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_SW_CLEAR, sizeof(i2cWriteLongAry), i2cWriteLongAry);
  }

  /* Read ID and status */
  {
    uint8_t regQry[1] = { I2C_SLAVE_20MHZ_DAC_REG_RD_STATUS };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_tcxo20MhzDacStatus = ((uint16_t)i2c4RxBuffer[0] << 8U) | i2c4RxBuffer[1];

      dbgLen = sprintf(dbgBuf, ". Tcxo20MhzDacInit: Status=0x%04X\r\n", s_tcxo20MhzDacStatus);
      usbLogLen(dbgBuf, dbgLen);
    }
  }

tcxo20MhzDacInit_out:
  usbLog("- Tcxo20MhzDacInit >\r\n\r\n");
}

static void tcxo20MhzDacSet(uint16_t dac)
{
  const uint8_t dacHi = (uint8_t) (dac >> 8U);
  const uint8_t dacLo = (uint8_t) (dac & 0xFFU);
  const uint8_t i2cWriteLongAry[2] = {
    dacHi, dacLo
  };

  i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_CODELOAD, sizeof(i2cWriteLongAry), i2cWriteLongAry);
}


/* Timer functions */

static void tcxoCyclicStart(uint32_t period_ms)
{
  osTimerStart(tcxoTimerHandle, period_ms);
}

static void tcxoCyclicStop(void)
{
  osTimerStop(tcxoTimerHandle);
}

void tcxoTimerCallback(void const *argument)
{

}


static void tcxo20MhzInit(void)
{
  /* Enable clock of Port C */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Switch on TCXO and high-current circuits */
  {
    mainPowerSwitchDo(POWERSWITCH__3V3_XO, 1U);
    osDelay(10);
  }

  /* TCXO is being powered */
  s_tcxo_enable = 1U;

  /* Init after power-up */
  {
    /* Prepare TCXO-DAC */
    tcxo20MhzDacInit();

    /* Tune VCTCXO for its mid-range voltage */
    s_tcxoVoltage_uV = TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VOLTAGE_UV;
    tcxo20MhzDacSet(tcxoCalcVoltage2DacValue(s_tcxoVoltage_uV));
  }

  /* Disable clock of Port C, again */
  __HAL_RCC_GPIOC_CLK_DISABLE();
}

static void tcxo20MHzMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                      msgIdx  = 0UL;
  const uint32_t                hdr     = msgAry[msgIdx++];
  const Tcxo20MHzMsgTcxoCmds_t  cmd     = (Tcxo20MHzMsgTcxoCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgTcxo__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_tcxoStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      tcxo20MhzInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Osc_TCXO, 0U, MsgTcxo__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgTcxo__SetVar01_Voltage:
    {
      s_tcxoVoltage_uV = msgAry[msgIdx++];
    }
    break;

  case MsgTcxo__GetVar01_Voltage:
    {
      uint32_t cmdBack[2];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Osc_TCXO, 0U, MsgTcxo__GetVar01_Voltage);
      cmdBack[1] = s_tcxoVoltage_uV;
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgTcxo__CallFunc01_SetDAC:
    {
      tcxo20MhzDacSet(tcxoCalcVoltage2DacValue(s_tcxoVoltage_uV));
    }
    break;

  case MsgTcxo__CallFunc02_CyclicMeasurements:
    {
      /* Start cyclic measurements with that period in ms */
      tcxoCyclicStart(msgAry[msgIdx++]);
    }
    break;

  case MsgTcxo__CallFunc03_StopCycles:
    {
      /* Stop cyclic measurements */
      tcxoCyclicStop();
    }
    break;

  default: { }
  }  // switch (cmd)

#if 0
  if (s_tcxo_enable) {
    adcStartConv(ADC_ADC3_V_PULL_TCXO);

    BaseType_t egBits = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_V_PULL_TCXO, EG_ADC3__CONV_AVAIL_V_PULL_TCXO, pdFALSE, 100 / portTICK_PERIOD_MS);
    if (egBits & EG_ADC3__CONV_AVAIL_V_PULL_TCXO) {
      int       dbgLen;
      char      dbgBuf[128];
      uint16_t  l_adc_v_pull_tcxo = adcGetVal(ADC_ADC3_V_PULL_TCXO);

      dbgLen = sprintf(dbgBuf, "ADC: Vpull  = %4d mV\r\n",  (int16_t) (l_adc_v_pull_tcxo + 0.5f));
      usbLogLen(dbgBuf, dbgLen);
    }
  }
#endif
}


/* Tasks */

void tcxo20MhzTaskInit(void)
{
  s_tcxo_enable         = 0U;
  s_tcxo20MhzDacStatus  = 0U;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_tcxoStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void tcxo20MhzTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2Tcxo_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Osc_TCXO, 1UL);                      // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    tcxo20MHzMsgProcess(msgLen, msgAry);
  }
}
