/*
 * bus_spi.c
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <bus_spi.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"
#include "stm32l4xx_hal_gpio.h"
#include "usb.h"


extern EventGroupHandle_t   extiEventGroupHandle;
extern osSemaphoreId        usbToHostBinarySemHandle;
extern osMutexId            spi1MutexHandle;
extern osMutexId            spi3MutexHandle;
extern EventGroupHandle_t   spiEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;


/* Buffer used for transmission */
#if 0
volatile uint8_t            spi1TxBuffer[SPI1_BUFFERSIZE]     = { 0 };
#endif
volatile uint8_t            spi3TxBuffer[SPI3_BUFFERSIZE]     = { 0 };

/* Buffer used for reception */
#if 0
volatile uint8_t            spi1RxBuffer[SPI1_BUFFERSIZE]     = { 0 };
#endif
volatile uint8_t            spi3RxBuffer[SPI3_BUFFERSIZE]     = { 0 };


SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;


/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t taskWoken = 0;

  if (&hspi3 == hspi) {
    uint8_t spi3BusInUse = 0U;

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_SX_SEL_GPIO_Port, MCU_OUT_SX_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_SX_SEL_GPIO_Port, MCU_OUT_SX_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_SX__BUS_DONE, &taskWoken);
    } else {
      spi3BusInUse = 1U;
    }

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_AX_SEL_GPIO_Port, MCU_OUT_AX_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_AX_SEL_GPIO_Port, MCU_OUT_AX_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_AX__BUS_DONE, &taskWoken);
    } else {
      spi3BusInUse = 1U;
    }

    if (!spi3BusInUse) {
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3__BUS_FREE, &taskWoken);
    }
  }
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t taskWoken = 0;

  if (&hspi3 == hspi) {
    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_SX_SEL_GPIO_Port, MCU_OUT_SX_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_SX_SEL_GPIO_Port, MCU_OUT_SX_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_SX__BUS_DONE, &taskWoken);
    }

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_AX_SEL_GPIO_Port, MCU_OUT_AX_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_AX_SEL_GPIO_Port, MCU_OUT_AX_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_AX__BUS_DONE, &taskWoken);
    }

    xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3__BUS_ERROR, &taskWoken);
  }
}


const uint16_t spiWait_EGW_MaxWaitTicks = 500;
uint8_t spiProcessSpiReturnWait(void)
{
  EventBits_t eb = xEventGroupWaitBits(spiEventGroupHandle, EG_SPI3_SX__BUS_DONE | EG_SPI3_AX__BUS_DONE | EG_SPI3__BUS_FREE | EG_SPI3__BUS_ERROR, 0, pdFALSE, spiWait_EGW_MaxWaitTicks);
  if (eb & (EG_SPI3_SX__BUS_DONE | EG_SPI3_AX__BUS_DONE | EG_SPI3__BUS_FREE)) {
    return HAL_OK;
  }

  if (eb & EG_SPI3__BUS_ERROR) {
    Error_Handler();
  }
  return HAL_ERROR;
}

uint8_t spiProcessSpi3MsgLocked(SPI3_CHIPS_t chip, uint8_t msgLen, uint8_t waitComplete)
{
  GPIO_TypeDef*     GPIOx     = (chip == SPI3_SX) ?  MCU_OUT_SX_SEL_GPIO_Port : ((chip == SPI3_AX) ?  MCU_OUT_AX_SEL_GPIO_Port  : 0);
  uint16_t          GPIO_Pin  = (chip == SPI3_SX) ?  MCU_OUT_SX_SEL_Pin       : ((chip == SPI3_AX) ?  MCU_OUT_AX_SEL_Pin        : 0);
  HAL_StatusTypeDef status    = HAL_OK;
  uint8_t           errCnt    = 0U;

  /* Sanity check */
  if (!GPIOx || !GPIO_Pin) {
    return HAL_ERROR;
  }

  /* Wait for free select line */
  do {
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
      break;
    }

    /* Wait some time for free bus */
    osDelay(5);
  } while (1);

  /* Activate low active NSS/SEL transaction */
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

  do {
    status = HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) spi3TxBuffer, (uint8_t *) spi3RxBuffer, msgLen);
    if (status == HAL_BUSY)
    {
      osThreadYield();

      if (++errCnt >= 100U) {
        /* Transfer error in transmission process */
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
        Error_Handler();
      }
    }
  } while (status == HAL_BUSY);

  if ((status == HAL_OK) && waitComplete) {
    /* Wait until the data is transfered */
    status = spiProcessSpiReturnWait();
  }

  return status;
}

uint8_t spiProcessSpi3MsgTemplateLocked(SPI3_CHIPS_t chip, uint16_t templateLen, const uint8_t* templateBuf, uint8_t waitComplete)
{
  /* Wait for SPI3 mutex */
  if (osOK != osMutexWait(spi3MutexHandle, 1000)) {
    return HAL_BUSY;
  }

  /* Copy from Template */
  memcpy((void*) spi3TxBuffer, (void*) templateBuf, templateLen);

  /* Execute SPI3 communication and leave with locked SPI3 mutex for read purpose */
  return spiProcessSpi3MsgLocked(chip, templateLen, waitComplete);
}

uint8_t spiProcessSpi3MsgTemplate(SPI3_CHIPS_t chip, uint16_t templateLen, const uint8_t* templateBuf)
{
  const uint8_t ret = spiProcessSpi3MsgTemplateLocked(chip, templateLen, templateBuf, 0U);

  /* Release SPI3 mutex */
  osMutexRelease(spi3MutexHandle);

  return ret;
}
