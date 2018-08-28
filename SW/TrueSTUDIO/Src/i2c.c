/*
 * i2c.c
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "usb.h"

#include "i2c.h"


I2C_HandleTypeDef         I2cHandle;

uint8_t                   aTxBuffer[32]                       = { 0 };
uint8_t                   aRxBuffer[32]                       = { 0 };


static void i2c_Hygro_init(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  usbLog("< I2C4: HYGRO init -\r\n");
  do {
    /* SHT31-DIS hygro: stop any running jobs */
    aTxBuffer[0] = I2C_SLAVE_HYGRO_REG_BREAK_HI;
    aTxBuffer[1] = I2C_SLAVE_HYGRO_REG_BREAK_LO;
    if(HAL_I2C_Master_Transmit_DMA(&I2cHandle, (uint16_t) I2C_SLAVE_HYGRO_ADDR, (uint8_t*) aTxBuffer, min(2, TXBUFFERSIZE)) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      //Error_Handler();
      break;
    }
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY) {
    }
    osDelay(2);

    /* SHT31-DIS hygro: reset */
    aTxBuffer[0] = I2C_SLAVE_HYGRO_REG_RESET_HI;
    aTxBuffer[1] = I2C_SLAVE_HYGRO_REG_RESET_LO;
    if(HAL_I2C_Master_Transmit_DMA(&I2cHandle, (uint16_t) I2C_SLAVE_HYGRO_ADDR, (uint8_t*) aTxBuffer, min(2, TXBUFFERSIZE)) != HAL_OK) {
      break;
    }
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY) {
    }
    osDelay(2);

    /* SHT31-DIS hygro: return current status */
    aTxBuffer[0] = I2C_SLAVE_HYGRO_REG_STATUS_HI;
    aTxBuffer[1] = I2C_SLAVE_HYGRO_REG_STATUS_LO;
    if(HAL_I2C_Master_Transmit_DMA(&I2cHandle, (uint16_t) I2C_SLAVE_HYGRO_ADDR, (uint8_t*) aTxBuffer, min(2, TXBUFFERSIZE)) != HAL_OK) {
      break;
    }
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY) {
    }
    if(HAL_I2C_Master_Receive_DMA(&I2cHandle, (uint16_t) I2C_SLAVE_HYGRO_ADDR, (uint8_t*) aRxBuffer, min(2, RXBUFFERSIZE)) != HAL_OK) {
      break;
    }
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY) {
    }

    dbgLen = sprintf(dbgBuf, "  SHT31 state: 0x%04X\r\n", (((uint16_t)aRxBuffer[0] << 8U) | aRxBuffer[1]));
    usbLogLen(dbgBuf, dbgLen);
  } while(0);

  usbLog("- I2C4: HYGRO init>\r\n");
}

static void i2c_init_i2c4(void)
{
  I2cHandle.Instance              = I2C4;
  I2cHandle.Init.Timing           = 0x00D00E28UL;  // TODO: calculate value
  I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  I2cHandle.Init.OwnAddress1      = 0x70;
  I2cHandle.Init.OwnAddress2      = 0xFF;

  if(HAL_I2C_Init(&I2cHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle, I2C_ANALOGFILTER_ENABLE);

  i2c_Hygro_init();
}


void i2cTaskInit(void)  // TODO
{
  i2c_init_i2c4();
}

void i2cTaskLoop(void)  // TODO
{
  osDelay(100);
}
