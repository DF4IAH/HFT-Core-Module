/*
 * task_Si5338.c
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"
//#include "stm32l4xx_hal_gpio.h"

#include "usb.h"
#include "bus_i2c.h"

#include "task_Si5338_RegMap_MCU_8MHz.h"
#include "task_Si5338_RegMap_MCU_12MHz.h"
#include "task_Si5338_RegMap_TCXO.h"

#include "task_Si5338.h"


extern osMutexId            i2c4MutexHandle;
extern I2C_HandleTypeDef    hi2c4;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

extern const Reg_Data_t     si5338_Reg_Store_MCU_8MHz[];
extern const Reg_Data_t     si5338_Reg_Store_MCU_12MHz[];
extern const Reg_Data_t     si5338_Reg_Store_TCXO[];


static uint8_t              s_si5338_enable                   = 0U;
static uint8_t              s_si5338_waitMask                 = 0U;
static uint8_t              s_si5338_variant                  = I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ;
static uint8_t              ls_si5338_variant                 = 0U;


void si5338VariantSet(I2C_SI5338_CLKIN_VARIANT_t v)
{
  PowerSwitchDo(POWERSWITCH__3V3_HICUR, 1);
  osDelay(10);

  s_si5338_variant  = v;
  s_si5338_enable   = 1U;
}


static void si5338SettingStep1(void)
{
  /* Figure 9 of DS: reg230[4] = 1, reg241[7] = 1 */
  const Reg_Data_t rdAry[2] = {
      { 230, 0x10, 0x10 },
      { 241, 0x80, 0x80 }
  };

  uint32_t i2cErr = i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  if (i2cErr == HAL_I2C_ERROR_AF) {
    /* Chip not responding */
    usbLog(". si5338: ERROR chip does not respond\r\n");
  }
}

static void si5338SettingStep3(void)
{
  /* Wait for input getting stable */
  while (1) {
    uint8_t regQry[1] = { 218 };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
        /* Leave when ready */
        if (!(i2c4RxBuffer[0] & s_si5338_waitMask)) {
          break;
        }
        osDelay(1);
    }
  }

  /* Figure 9 of DS: reg49[7] = 0, reg246[1] = 1 */
  {
    const Reg_Data_t rdAry[2] = {
        {  49, 0x00, 0x80 },
        { 246, 0x02, 0x02 }
    };

    i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }

  osDelay(25);

  /* Figure 9 of DS: reg241[7] = 0, reg241 = 0x65 */
  {
    const Reg_Data_t rdAry[2] = {
        { 241, 0x00, 0x80 },
        { 241, 0x65, 0xFF }
    };

    i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }

  /* Wait for PLL getting stable */
  while (1) {
    uint8_t regQry[1] = { 218 };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
        /* Leave when ready */
        if (!(i2c4RxBuffer[0] & 0x11)) {
          break;
        }
        osDelay(1);
    }
  }

  /* FCAL values */
  {
    uint8_t val = 0U;

    {
      uint8_t regQry[1] = { 237 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = i2c4RxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[1] = {
            {  47, cVal, 0x03 }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 1, rdAry);
      }
    }

    {
      uint8_t regQry[1] = { 236 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = i2c4RxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[1] = {
            {  46, cVal, 0xFF }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 1, rdAry);
      }
    }

    {
      uint8_t regQry[1] = { 235 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = i2c4RxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[2] = {
            {  45, cVal, 0xFF },
            {  47, 0x14, 0xFC }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
      }
    }
  }

  /* Figure 9 of DS: reg49[7] = 1, reg230[4] = 0 */
  {
    const Reg_Data_t rdAry[2] = {
        {  49, 0x80, 0x80 },
        { 230, 0x00, 0x10 }
    };
    i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }
}


static void si5338Init(void)
{
  if (s_si5338_enable) {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(MCU_OUT_HICUR_EN_GPIO_Port, MCU_OUT_HICUR_EN_Pin)) {
      switch (s_si5338_variant) {
      case I2C_SI5338_CLKIN_VARIANT__MCU_MCO_8MHZ:
        si5338SettingStep1();
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_MCU_8MHZ_NUM_REGS_MAX, si5338_Reg_Store_MCU_8MHz);
        s_si5338_waitMask = 0x04U;
        si5338SettingStep3();
        break;

      case I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ:
        si5338SettingStep1();
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_MCU_12MHZ_NUM_REGS_MAX, si5338_Reg_Store_MCU_12MHz);
        s_si5338_waitMask = 0x04U;
        si5338SettingStep3();
        break;

      case I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ:
        si5338SettingStep1();
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_TCXO_NUM_REGS_MAX, si5338_Reg_Store_TCXO);
        s_si5338_waitMask = 0x08U;
        si5338SettingStep3();
        break;

      default:
        s_si5338_enable = 0U;
      }
    } else {
      s_si5338_enable = 0U;
    }
  }
}


/* Tasks */

void si5338TaskInit(void)
{
  osDelay(100UL);

  ls_si5338_variant = s_si5338_variant;
  si5338Init();
}

void si5338TaskLoop(void)
{
  const uint32_t  eachMs              = 100UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 100UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);

  if (s_si5338_enable) {
    /* Change of variant detected */
    if (ls_si5338_variant != s_si5338_variant) {
      ls_si5338_variant = s_si5338_variant;
      si5338Init();
    }
  }
}
