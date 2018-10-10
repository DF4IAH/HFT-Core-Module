/*
 * bus_i2c.c
 *
 *  Created on: 27.08.2018
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

#include "i2c.h"
#include "bus_i2c.h"


extern osSemaphoreId        i2c1_BSemHandle;
extern osSemaphoreId        i2c2_BSemHandle;
extern osSemaphoreId        i2c3_BSemHandle;
extern osSemaphoreId        i2c4_BSemHandle;

extern I2C_HandleTypeDef    hi2c1;
extern I2C_HandleTypeDef    hi2c2;
extern I2C_HandleTypeDef    hi2c3;
extern I2C_HandleTypeDef    hi2c4;

static uint8_t              s_i2cx_UseCtr[4]                  = { 0U };

volatile uint8_t            i2c4TxBuffer[I2C_TXBUFSIZE];
volatile uint8_t            i2c4RxBuffer[I2C_RXBUFSIZE];


void i2cBusAddrScan(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle) {
  /* DEBUG I2C4 Bus */
  char dbgBuf[64];
  int dbgLen;

  osSemaphoreWait(semaphoreHandle, osWaitForever);

  i2c4TxBuffer[0] = 0x00;
  for (uint8_t addr = 0x01U; addr <= 0x7FU; addr++) {
    if (HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(dev, ((uint16_t)addr << 1U), (uint8_t*) i2c4TxBuffer, 0U, I2C_FIRST_AND_LAST_FRAME)) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
      osDelay(1UL);
    }
    if (HAL_I2C_GetError(dev) != HAL_I2C_ERROR_AF) {
      dbgLen = sprintf(dbgBuf, "GOOD:  Addr=0x%02X  got response\r\n", addr);
      usbLogLen(dbgBuf, dbgLen);
    }
    osDelay(25UL);
  }

  osSemaphoreRelease(semaphoreHandle);
}

//#define DEBUG_WRITE_MASK 1
uint32_t i2cSequenceWriteMask(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle,
    uint8_t addr, uint16_t count, const Reg_Data_t dataAry[]) {
  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(semaphoreHandle, osWaitForever);

  for (uint16_t listIdx = 0U; listIdx < count; listIdx++) {
    if (dataAry[listIdx].Reg_Mask == 0xffU) {
      /* Write without read */
      i2c4TxBuffer[0] = dataAry[listIdx].Reg_Addr;
      i2c4TxBuffer[1] = dataAry[listIdx].Reg_Val;
      if (HAL_I2C_Master_Sequential_Transmit_IT(dev, ((uint16_t)addr << 1U),
          (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE),
          I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
        osDelay(1UL);
      }
      if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
        /* Return mutex */
        osSemaphoreRelease(semaphoreHandle);

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
        osDelay(1UL);
      }
      if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
        /* Return mutex */
        osSemaphoreRelease(semaphoreHandle);

        /* Chip not responding */
        usbLog("i2cSequenceWriteMask: ERROR chip does not respond\r\n");
        return HAL_I2C_ERROR_AF;
      }

      memset((uint8_t*) i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
      if (HAL_I2C_Master_Sequential_Receive_IT(dev, (uint16_t) (addr << 1U),
          (uint8_t*) i2c4RxBuffer, min(1U, I2C_RXBUFSIZE), I2C_OTHER_FRAME)
          != HAL_OK) {
        Error_Handler();
      }
      while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
        osDelay(1UL);
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
          osDelay(1UL);
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
  osSemaphoreRelease(semaphoreHandle);

  return HAL_I2C_ERROR_NONE;
}

uint32_t i2cSequenceWriteLong(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle,
    uint8_t addr, uint8_t i2cReg, uint16_t count,
    const uint8_t i2cWriteAryLong[]) {
  if (count && ((count - 1) >= I2C_TXBUFSIZE)) {
    return HAL_I2C_ERROR_SIZE;
  }

  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(semaphoreHandle, osWaitForever);

  i2c4TxBuffer[0] = i2cReg;

  for (uint8_t idx = 0; idx < count; idx++) {
    i2c4TxBuffer[idx + 1] = i2cWriteAryLong[idx];
  }

  if (HAL_OK != HAL_I2C_Master_Sequential_Transmit_IT(dev, ((uint16_t)addr << 1U), (uint8_t*) i2c4TxBuffer, min((1U + count), I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME)) {
    /* Error_Handler() function is called when error occurs. */
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1UL);
  }
  uint32_t i2cErr = HAL_I2C_GetError(dev);

  /* Return mutex */
  osSemaphoreRelease(semaphoreHandle);

  if (i2cErr == HAL_I2C_ERROR_AF) {
    /* Chip not responding */
    //usbLog("i2cSequenceWriteLong: ERROR chip does not respond\r\n");
    return HAL_I2C_ERROR_AF;
  }

  return HAL_I2C_ERROR_NONE;
}

uint32_t i2cSequenceRead(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle, uint8_t addr, uint8_t i2cRegLen, uint8_t i2cReg[], uint16_t readLen)
{
  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(semaphoreHandle, osWaitForever);

  for (uint8_t regIdx = 0; regIdx < i2cRegLen; regIdx++) {
    i2c4TxBuffer[regIdx] = i2cReg[regIdx];
  }
  if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) i2c4TxBuffer, min(i2cRegLen, I2C_TXBUFSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
    /* Error_Handler() function is called when error occurs. */
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1UL);
  }
  uint32_t i2cErr = HAL_I2C_GetError(dev);

  if (i2cErr == HAL_I2C_ERROR_AF) {
    /* Return mutex */
    osSemaphoreRelease(semaphoreHandle);

    /* Chip not responding */
    usbLog("i2cSequenceRead: ERROR chip does not respond\r\n");
    return HAL_I2C_ERROR_AF;
  }

  memset((uint8_t*) i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
  if (HAL_I2C_Master_Sequential_Receive_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) i2c4RxBuffer, min(readLen, I2C_RXBUFSIZE), I2C_LAST_FRAME) != HAL_OK) {
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1UL);
  }

  /* Return mutex */
  osSemaphoreRelease(semaphoreHandle);

  return HAL_I2C_ERROR_NONE;
}


static uint8_t i2cx_getDevIdx(I2C_HandleTypeDef* dev)
{
  if (&hi2c1 == dev) {
    return 0U;

  } else if (&hi2c2 == dev) {
    return 1U;

  } else if (&hi2c3 == dev) {
    return 2U;

  } else if (&hi2c4 == dev) {
    return 3U;
  }

  Error_Handler();
  return 0U;
}


void i2cx_Init(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle)
{
  const uint8_t devIdx = i2cx_getDevIdx(dev);

  osSemaphoreWait(semaphoreHandle, osWaitForever);

  if (!s_i2cx_UseCtr[devIdx]++) {
    switch (devIdx) {
    case 0:
      __HAL_RCC_GPIOG_CLK_ENABLE();                                                                   // I2C1: SCL, SDA
      MX_I2C1_Init();
      /* Do not turn off clock of GPIOx SCL */
      break;

    case 1:
      __HAL_RCC_GPIOF_CLK_ENABLE();                                                                   // I2C2: SCL, SDA
      MX_I2C2_Init();
      /* Do not turn off clock of GPIOx SCL */
      break;

    case 2:
      __HAL_RCC_GPIOG_CLK_ENABLE();                                                                   // I2C3: SCL, SDA
      MX_I2C3_Init();
      /* Do not turn off clock of GPIOx SCL */
      break;

    case 3:
      __HAL_RCC_GPIOD_CLK_ENABLE();                                                                   // I2C4: SDA
      __HAL_RCC_GPIOF_CLK_ENABLE();                                                                   // I2C4: SCL
      MX_I2C4_Init();
      __HAL_RCC_GPIOD_CLK_DISABLE();                                                                  // I2C4: SDA
      /* Do not turn off clock of GPIOx SCL */
      break;

    default: { }
    }
  }

  osSemaphoreRelease(semaphoreHandle);
}

void i2cx_DeInit(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle)
{
  const uint8_t devIdx = i2cx_getDevIdx(dev);

  osSemaphoreWait(semaphoreHandle, osWaitForever);

  if (!--s_i2cx_UseCtr[devIdx]) {
    HAL_I2C_MspDeInit(dev);

  } else if (s_i2cx_UseCtr[devIdx] == 255U) {
    /* Underflow */
    s_i2cx_UseCtr[devIdx] = 0U;
  }

  osSemaphoreRelease(semaphoreHandle);
}
