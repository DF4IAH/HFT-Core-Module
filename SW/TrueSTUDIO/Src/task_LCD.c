/*
 * task_LCD.c
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#include <string.h>
#include <math.h>

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "usb.h"
#include "bus_i2c.h"

#include "task_LCD.h"


extern osMutexId            i2c4MutexHandle;
extern I2C_HandleTypeDef    hi2c4;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];


static void lcdInit(void)
{
  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< LcdInit -\r\n");

  do {
    /* Signal Reset */
    HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, (GPIO_PIN_RESET));
    osDelay(1);
    HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, (GPIO_PIN_SET));
    osDelay(50);

#ifdef HISTORIC
    /* LCD NHD-C0220BiZ-FS(RGB)-FBW-3VM */

    /* Reset */
    i2c4TxBuffer[0] = 0x80;                                                                              // Co
    i2c4TxBuffer[1] = 0x38;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) i2c4TxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4LcdInit: ERROR LCD does not respond\r\n");
      break;
    }
    osDelay(10);

    i2c4TxBuffer[0] = 0x00;                                                                              // Co
    i2c4TxBuffer[1] = 0x39;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) i2c4TxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    osDelay(10);

    i2c4TxBuffer[0] = 0x80;                                                                              // Co
    i2c4TxBuffer[1] = 0x14;
    i2c4TxBuffer[2] = 0x78;
    i2c4TxBuffer[3] = 0x5E;
    i2c4TxBuffer[4] = 0x6D;
    i2c4TxBuffer[5] = 0x0C;
    i2c4TxBuffer[6] = 0x01;
    i2c4TxBuffer[7] = 0x06;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) i2c4TxBuffer, min(8U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }

    i2c4TxBuffer[ 0] = 0x40;                                                                              // Data
    i2c4TxBuffer[ 1] = '*';
    i2c4TxBuffer[ 2] = ' ';
    i2c4TxBuffer[ 3] = 'H';
    i2c4TxBuffer[ 4] = 'F';
    i2c4TxBuffer[ 5] = 'T';
    i2c4TxBuffer[ 6] = '-';
    i2c4TxBuffer[ 7] = 'C';
    i2c4TxBuffer[ 8] = 'o';
    i2c4TxBuffer[ 9] = 'r';
    i2c4TxBuffer[10] = 'e';
    i2c4TxBuffer[11] = '-';
    i2c4TxBuffer[12] = 'M';
    i2c4TxBuffer[13] = 'o';
    i2c4TxBuffer[14] = 'd';
    i2c4TxBuffer[15] = 'u';
    i2c4TxBuffer[16] = 'l';
    i2c4TxBuffer[17] = 'e';
    i2c4TxBuffer[18] = ' ';
    i2c4TxBuffer[19] = '*';
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) i2c4TxBuffer, min(20U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
#endif
  } while(0);

  usbLog("- i2cI2c4LcdInit >\r\n\r\n");

  osSemaphoreRelease(i2c4MutexHandle);
}


/* Task */

void lcdTaskInit(void)
{
  osDelay(500UL);
  lcdInit();
}

void lcdTaskLoop(void)
{
  const uint32_t  eachMs              = 250UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 500UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);
}
