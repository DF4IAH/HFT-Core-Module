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


static uint8_t              s_si5338_enable                   = 1U;
static uint8_t              s_si5338_variant                  = I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ;


static void si5338Init(void)
{
  if (s_si5338_enable) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(MCU_OUT_HICUR_EN_GPIO_Port, MCU_OUT_HICUR_EN_Pin)) {
      switch (s_si5338_variant) {
      case I2C_SI5338_CLKIN_VARIANT__MCU_MCO_8MHZ:
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_MCU_8MHZ_NUM_REGS_MAX, si5338_Reg_Store_MCU_8MHz);
        break;

      case I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ:
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_MCU_12MHZ_NUM_REGS_MAX, si5338_Reg_Store_MCU_12MHz);
        break;

      case I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ:
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_TCXO_NUM_REGS_MAX, si5338_Reg_Store_TCXO);
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
  osDelay(400UL);
  si5338Init();
}

void si5338TaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 400UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);

  if (s_si5338_enable) {
    // TODO: code here
  }
}
