/*
 * bus_i2c.c
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


extern osMutexId            i2c1MutexHandle;
extern osMutexId            i2c2MutexHandle;
extern osMutexId            i2c3MutexHandle;
extern osMutexId            i2c4MutexHandle;

extern I2C_HandleTypeDef    hi2c1;
extern I2C_HandleTypeDef    hi2c2;
extern I2C_HandleTypeDef    hi2c3;
extern I2C_HandleTypeDef    hi2c4;

uint8_t                     i2c4TxBuffer[I2C_TXBUFSIZE];
uint8_t                     i2c4RxBuffer[I2C_RXBUFSIZE];


#ifdef I2C_BUS_ADDR_SCAN
void i2cBusAddrScan(I2C_HandleTypeDef* dev, osMutexId mutexHandle) {
  /* DEBUG I2C4 Bus */
  char dbgBuf[64];
  int dbgLen;

  osSemaphoreWait(mutexHandle, osWaitForever);

  i2c4TxBuffer[0] = 0x00;
  for (uint8_t addr = 0x01U; addr <= 0x7FU; addr++) {
    if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U),
        (uint8_t*) i2c4TxBuffer, min(0U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME)
        != HAL_OK) {
      usbLog("ERROR: 1\r\n");
    }
    while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(dev) != HAL_I2C_ERROR_AF) {
      dbgLen = sprintf(dbgBuf, "GOOD:  Addr=0x%02X  got response\r\n", addr);
      usbLogLen(dbgBuf, dbgLen);
    }
    osDelay(25);
  }

  osSemaphoreRelease(mutexHandle);
}
#endif

//#define DEBUG_WRITE_MASK 1
uint32_t i2cSequenceWriteMask(I2C_HandleTypeDef* dev, osMutexId mutexHandle,
    uint8_t addr, uint16_t count, const Reg_Data_t dataAry[]) {
  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(mutexHandle, osWaitForever);

  for (uint16_t listIdx = 0U; listIdx < count; listIdx++) {
    if (dataAry[listIdx].Reg_Mask == 0xffU) {
      /* Write without read */
      i2c4TxBuffer[0] = dataAry[listIdx].Reg_Addr;
      i2c4TxBuffer[1] = dataAry[listIdx].Reg_Val;
      if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U),
          (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE),
          I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
        osDelay(1);
      }
      if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
        /* Return mutex */
        osSemaphoreRelease(mutexHandle);

        /* Chip not responding */
        usbLog("i2cSequenceWriteMask: ERROR chip does not respond\r\n");
        return HAL_I2C_ERROR_AF;
      }

#ifdef DEBUG_WRITE_MASK
      {
        int dbgLen = 0;
        char dbgBuf[128];

        dbgLen = sprintf(dbgBuf, "i2cSequenceWriteMask: ListIdx=%03d, I2C addr=0x%02X, val=0x%02X\r\n",
            listIdx,
            dataAry[listIdx].Reg_Addr,
            dataAry[listIdx].Reg_Val);
        usbLogLen(dbgBuf, dbgLen);
      }
#endif

    } else if (dataAry[listIdx].Reg_Mask) {
      /* Read current data */
      i2c4TxBuffer[0] = dataAry[listIdx].Reg_Addr;
      if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U),
          (uint8_t*) i2c4TxBuffer, min(1U, I2C_TXBUFSIZE), I2C_LAST_FRAME_NO_STOP)
          != HAL_OK) {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
        osDelay(1);
      }
      if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
        /* Return mutex */
        osSemaphoreRelease(mutexHandle);

        /* Chip not responding */
        usbLog("i2cSequenceWriteMask: ERROR chip does not respond\r\n");
        return HAL_I2C_ERROR_AF;
      }

      memset(i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
      if (HAL_I2C_Master_Sequential_Receive_IT(dev, (uint16_t) (addr << 1U),
          (uint8_t*) i2c4RxBuffer, min(1U, I2C_RXBUFSIZE), I2C_OTHER_FRAME)
          != HAL_OK) {
        Error_Handler();
      }
      while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
        osDelay(1);
      }
      uint8_t val1 = i2c4RxBuffer[0];
      uint8_t val2 = val1;

      /* Modify value */
      val2 &= ~(dataAry[listIdx].Reg_Mask);
      val2 |= dataAry[listIdx].Reg_Mask & dataAry[listIdx].Reg_Val;

      if (val2 != val1) {
        /* Write back value */
        i2c4TxBuffer[0] = dataAry[listIdx].Reg_Addr;
        i2c4TxBuffer[1] = val2;
        if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U),
            (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE),
            I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
          /* Error_Handler() function is called when error occurs. */
          Error_Handler();
        }
        while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
          osDelay(1);
        }
      }

#ifdef DEBUG_WRITE_MASK
      {
        int dbgLen = 0;
        char dbgBuf[128];

        dbgLen = sprintf(dbgBuf, "i2cSequenceWriteMask: ListIdx=%03d, I2C addr=0x%02X, val_before=0x%02X, val_after=0x%02X\r\n",
            listIdx,
            dataAry[listIdx].Reg_Addr,
            val1,
            val2);
        usbLogLen(dbgBuf, dbgLen);
      }
#endif
    }
  }

  /* Return mutex */
  osSemaphoreRelease(mutexHandle);

  return HAL_I2C_ERROR_NONE;
}

uint32_t i2cSequenceWriteLong(I2C_HandleTypeDef* dev, osMutexId mutexHandle,
    uint8_t addr, uint8_t i2cReg, uint16_t count,
    const uint8_t i2cWriteAryLong[]) {
  if (count && ((count - 1) >= I2C_TXBUFSIZE)) {
    return HAL_I2C_ERROR_SIZE;
  }

  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(mutexHandle, osWaitForever);

  i2c4TxBuffer[0] = i2cReg;

  for (uint8_t idx = 0; idx < count; idx++) {
    i2c4TxBuffer[idx + 1] = i2cWriteAryLong[idx];
  }

  if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U),
      (uint8_t*) i2c4TxBuffer, min((count + 1U), I2C_TXBUFSIZE),
      I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
    /* Error_Handler() function is called when error occurs. */
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1);
  }
  uint32_t i2cErr = HAL_I2C_GetError(dev);

  /* Return mutex */
  osSemaphoreRelease(mutexHandle);

  if (i2cErr == HAL_I2C_ERROR_AF) {
    /* Chip not responding */
    //usbLog("i2cSequenceWriteLong: ERROR chip does not respond\r\n");
    return HAL_I2C_ERROR_AF;
  }

  return HAL_I2C_ERROR_NONE;
}
