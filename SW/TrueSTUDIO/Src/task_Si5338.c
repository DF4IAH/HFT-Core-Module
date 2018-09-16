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

#include "task_Si5338.h"


extern osMutexId            i2c4MutexHandle;
extern I2C_HandleTypeDef    hi2c4;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

static uint8_t              s_si5338_enable                   = 0U;


uint32_t i2cSequenceRead(I2C_HandleTypeDef* dev, osMutexId mutexHandle, uint8_t addr, uint8_t i2cRegLen, uint8_t i2cReg[], uint16_t readLen)
{
  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  for (uint8_t regIdx = 0; regIdx < i2cRegLen; regIdx++) {
    i2c4TxBuffer[regIdx] = i2cReg[regIdx];
  }
  if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) i2c4TxBuffer, min(i2cRegLen, I2C_TXBUFSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
    /* Error_Handler() function is called when error occurs. */
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1);
  }
  if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
    /* Return mutex */
    osSemaphoreRelease(i2c4MutexHandle);

    /* Chip not responding */
    usbLog("i2cSequenceRead: ERROR chip does not respond\r\n");
    return HAL_I2C_ERROR_AF;
  }

  memset(i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
  if (HAL_I2C_Master_Sequential_Receive_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) i2c4RxBuffer, min(readLen, I2C_RXBUFSIZE), I2C_OTHER_FRAME) != HAL_OK) {
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1);
  }

  /* Return mutex */
  osSemaphoreRelease(i2c4MutexHandle);

  return HAL_I2C_ERROR_NONE;
}

static void si5338Init(void)
{

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
