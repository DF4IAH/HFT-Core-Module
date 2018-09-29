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
#include <task_TCXO_20MHz.h>


extern I2C_HandleTypeDef    hi2c4;
extern osThreadId           tcxo20MhzTaskHandle;
extern osMutexId            i2c4MutexHandle;
extern EventGroupHandle_t   adcEventGroupHandle;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

static uint8_t              s_tcxo_enable                     = 0U;
static uint16_t             s_tcxo20MhzDacStatus              = 0U;


void tcxo20MhzDacInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  usbLog("< Tcxo20MhzDacInit -\r\n");

  {
    const uint8_t i2cWriteLongAry[2] = {
      0x00, 0x20                                                                                      // 0x0020: clear value Mid, nAUX disable, DAC enabled
    };

    uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_USER_CONFIG, sizeof(i2cWriteLongAry), i2cWriteLongAry);
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

    i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_SW_CLEAR, sizeof(i2cWriteLongAry), i2cWriteLongAry);
  }

  /* Read ID and status */
  {
    uint8_t regQry[1] = { I2C_SLAVE_20MHZ_DAC_REG_RD_STATUS };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_tcxo20MhzDacStatus = ((uint16_t)i2c4RxBuffer[0] << 8U) | i2c4RxBuffer[1];

      dbgLen = sprintf(dbgBuf, ". Tcxo20MhzDacInit: Status=0x%04X\r\n", s_tcxo20MhzDacStatus);
      usbLogLen(dbgBuf, dbgLen);
    }
  }

tcxo20MhzDacInit_out:
  usbLog("- Tcxo20MhzDacInit >\r\n\r\n");
}

void tcxo20MhzDacSet(uint16_t dac)
{
  const uint8_t dacHi = (uint8_t) (dac >> 8U);
  const uint8_t dacLo = (uint8_t) (dac & 0xFFU);
  const uint8_t i2cWriteLongAry[2] = {
    dacHi, dacLo
  };

  i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_CODELOAD, sizeof(i2cWriteLongAry), i2cWriteLongAry);
}


/* Tasks */

void tcxo20MhzTaskInit(void)
{
  osDelay(900UL);

  if (GPIO_PIN_SET == HAL_GPIO_ReadPin(MCU_OUT_20MHZ_EN_GPIO_Port, MCU_OUT_20MHZ_EN_Pin)) {
    tcxo20MhzDacInit();

    /* Preload-value of TCXO */
    tcxo20MhzDacSet(TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VALUE);

  } else {
    s_tcxo_enable = 0U;
  }
}

void tcxo20MhzTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;
  int             dbgLen;
  char            dbgBuf[128];

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 900UL;
  }

  /* Repeat each time period ADC conversion */
  osDelayUntil(&sf_previousWakeTime, eachMs);

  if (s_tcxo_enable) {
    adcStartConv(ADC_ADC3_V_PULL_TCXO);

    BaseType_t egBits = xEventGroupWaitBits(adcEventGroupHandle, EG_ADC3__CONV_AVAIL_V_PULL_TCXO, EG_ADC3__CONV_AVAIL_V_PULL_TCXO, pdFALSE, 100 / portTICK_PERIOD_MS);
    if (egBits & EG_ADC3__CONV_AVAIL_V_PULL_TCXO) {
      uint16_t l_adc_v_pull_tcxo = adcGetVal(ADC_ADC3_V_PULL_TCXO);

      dbgLen = sprintf(dbgBuf, "ADC: Vpull  = %4d mV\r\n",  (int16_t) (l_adc_v_pull_tcxo + 0.5f));
      usbLogLen(dbgBuf, dbgLen);
    }
  }
}
