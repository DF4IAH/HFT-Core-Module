/*
 * bus_spi.c
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include "bus_spi.h"

#include <string.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"
#include "stm32l4xx_hal_gpio.h"

#include "spi.h"
#include "task_USB.h"



extern EventGroupHandle_t   extiEventGroupHandle;
extern osSemaphoreId        usbToHost_BSemHandle;
extern osSemaphoreId        spi1_BSemHandle;
extern osSemaphoreId        spi3_BSemHandle;
extern EventGroupHandle_t   spiEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;


static uint8_t              s_spix_UseCtr[2]                  = { 0 };

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

    /* Enable clocks of GPIOs */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

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

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_AUDIO_DAC_SEL_GPIO_Port, MCU_OUT_AUDIO_DAC_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_DAC_SEL_GPIO_Port, MCU_OUT_AUDIO_DAC_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_DAC__BUS_DONE, &taskWoken);
    } else {
      spi3BusInUse = 1U;
    }

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_AUDIO_ADC_SEL_GPIO_Port, MCU_OUT_AUDIO_ADC_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_ADC_SEL_GPIO_Port, MCU_OUT_AUDIO_ADC_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_ADC__BUS_DONE, &taskWoken);
    } else {
      spi3BusInUse = 1U;
    }

    #if 0
    /* Disable clocks again */
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    #endif

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

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_AUDIO_DAC_SEL_GPIO_Port, MCU_OUT_AUDIO_DAC_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_DAC_SEL_GPIO_Port, MCU_OUT_AUDIO_DAC_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_DAC__BUS_DONE, &taskWoken);
    }

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(MCU_OUT_AUDIO_ADC_SEL_GPIO_Port, MCU_OUT_AUDIO_ADC_SEL_Pin)) {
      /* Deactivate the NSS/SEL pin */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_ADC_SEL_GPIO_Port, MCU_OUT_AUDIO_ADC_SEL_Pin, GPIO_PIN_SET);
      xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3_ADC__BUS_DONE, &taskWoken);
    }

    xEventGroupSetBitsFromISR(spiEventGroupHandle, EG_SPI3__BUS_ERROR, &taskWoken);
  }
}


const uint16_t spiWait_EGW_MaxWaitTicks = 500;
uint8_t spiProcessSpiReturnWait(void)
{
  EventBits_t eb = xEventGroupWaitBits(spiEventGroupHandle,
      EG_SPI3_SX__BUS_DONE | EG_SPI3_AX__BUS_DONE | EG_SPI3_DAC__BUS_DONE | EG_SPI3_ADC__BUS_DONE |
      EG_SPI3__BUS_FREE | EG_SPI3__BUS_ERROR,
      0, pdFALSE, spiWait_EGW_MaxWaitTicks);
  if (eb & (EG_SPI3_SX__BUS_DONE | EG_SPI3_AX__BUS_DONE | EG_SPI3_DAC__BUS_DONE | EG_SPI3_ADC__BUS_DONE |
      EG_SPI3__BUS_FREE)) {
    return HAL_OK;
  }

  if (eb & EG_SPI3__BUS_ERROR) {
    Error_Handler();
  }
  return HAL_ERROR;
}

uint8_t spiProcessSpi3MsgLocked(SPI3_CHIPS_t chip, uint8_t msgLen, uint8_t waitComplete)
{
  HAL_StatusTypeDef status    = HAL_OK;
  uint8_t           errCnt    = 0U;

  GPIO_TypeDef*     GPIOx;
  uint16_t          GPIO_Pin;
  switch (chip) {
  case SPI3_SX:
    GPIOx     = MCU_OUT_SX_SEL_GPIO_Port;
    GPIO_Pin  = MCU_OUT_SX_SEL_Pin;

    /* Activate clock for this GPIO */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    break;

  case SPI3_AX:
    GPIOx     = MCU_OUT_AX_SEL_GPIO_Port;
    GPIO_Pin  = MCU_OUT_AX_SEL_Pin;

    /* Activate clock for this GPIO */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    break;

  case SPI3_DAC:
    GPIOx     = MCU_OUT_AUDIO_DAC_SEL_GPIO_Port;
    GPIO_Pin  = MCU_OUT_AUDIO_DAC_SEL_Pin;

    /* Activate clock for this GPIO */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    break;

  case SPI3_ADC:
    GPIOx     = MCU_OUT_AUDIO_ADC_SEL_GPIO_Port;
    GPIO_Pin  = MCU_OUT_AUDIO_ADC_SEL_Pin;

    /* Activate clock for this GPIO */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    break;

  default:
    GPIOx     = 0;
    GPIO_Pin  = 0;
  }

  /* Sanity check */
  if (!GPIOx) {
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

  #if 0
  /* Disable GPIO clocks again */
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  #endif

  return status;
}

uint8_t spiProcessSpi3MsgTemplateLocked(SPI3_CHIPS_t chip, uint16_t templateLen, const uint8_t* templateBuf, uint8_t waitComplete)
{
  /* Wait for SPI3 mutex */
  if (osOK != osSemaphoreWait(spi3_BSemHandle, 1000)) {
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
  osSemaphoreRelease(spi3_BSemHandle);

  return ret;
}


static uint8_t spix_getDevIdx(SPI_HandleTypeDef* dev)
{
  if (&hspi1 == dev) {
    return 0U;

  } else if (&hspi3 == dev) {
    return 1U;
  }

  Error_Handler();
  return 0U;
}


void spix_Init(SPI_HandleTypeDef* dev, osSemaphoreId semaphoreHandle)
{
  const uint8_t devIdx = spix_getDevIdx(dev);

  osSemaphoreWait(semaphoreHandle, osWaitForever);

  if (!s_spix_UseCtr[devIdx]++) {
    switch (devIdx) {
    case 0:
    //__HAL_RCC_GPIOx_CLK_ENABLE();                                                                   // SPI1: MCU_SPI1_SCK
      MX_SPI1_Init();
      break;

    case 1:
      __HAL_RCC_GPIOC_CLK_ENABLE();                                                                   // SPI3: MCU_SPI3_SCK, MCU_SPI3_MISO, MCU_SPI3_MOSI, MCU_OUT_AUDIO_DAC_SEL
      __HAL_RCC_GPIOD_CLK_ENABLE();                                                                   // SPI3: MCU_OUT_AUDIO_ADC_SEL
      __HAL_RCC_GPIOE_CLK_ENABLE();                                                                   // SPI3: MCU_OUT_AX_SEL, MCU_OUT_SX_SEL
      MX_SPI3_Init();
      break;

    default: { }
    }
  }

  osSemaphoreRelease(semaphoreHandle);
}

void spix_DeInit(SPI_HandleTypeDef* dev, osSemaphoreId semaphoreHandle)
{
  const uint8_t devIdx = spix_getDevIdx(dev);

  osSemaphoreWait(semaphoreHandle, osWaitForever);

  if (!--s_spix_UseCtr[devIdx]) {
    HAL_SPI_MspDeInit(dev);

  } else if (s_spix_UseCtr[devIdx] == 255U) {
    /* Underflow */
    s_spix_UseCtr[devIdx] = 0U;
  }

  osSemaphoreRelease(semaphoreHandle);
}
