/*
 * spi.c
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"
#include "stm32l4xx_hal_gpio.h"
#include "usb.h"


extern EventGroupHandle_t   extiEventGroupHandle;
extern osSemaphoreId        usbToHostBinarySemHandle;
extern EventGroupHandle_t   spiEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;


/* Buffer used for transmission */
volatile uint8_t            spi3TxBuffer[SPI1_BUFFERSIZE]     = { 0 };

/* Buffer used for reception */
volatile uint8_t            spi3RxBuffer[SPI1_BUFFERSIZE]     = { 0 };


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

uint8_t spiProcessSpi3Msg(SPI3_CHIPS_t chip, uint8_t msgLen)
{
  GPIO_TypeDef*     GPIOx     = (chip == SPI3_SX) ?  MCU_OUT_SX_SEL_GPIO_Port : ((chip == SPI3_AX) ?  MCU_OUT_AX_SEL_GPIO_Port  : 0);
  uint16_t          GPIO_Pin  = (chip == SPI3_SX) ?  MCU_OUT_SX_SEL_Pin       : ((chip == SPI3_AX) ?  MCU_OUT_AX_SEL_Pin        : 0);
  HAL_StatusTypeDef status    = HAL_OK;
  uint8_t           errCnt    = 0;

  /* Sanity check */
  if (!GPIOx || !GPIO_Pin) {
    return HAL_ERROR;
  }

  /* Wait for SPI3 mutex */
  // TODO: SPI3 mutex

  /* Activate low active NSS/SEL transaction */
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

  do {
    status = HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) spi3TxBuffer, (uint8_t *) spi3RxBuffer, msgLen);
    if (status == HAL_BUSY)
    {
      osDelay(1);

      if (++errCnt >= 100) {
        /* Transfer error in transmission process */
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
        Error_Handler();
      }
    }
  } while (status == HAL_BUSY);

  if (status == HAL_OK) {
    /* Wait until the data is transfered */
    status = spiProcessSpiReturnWait();
  }

  /* Release low active NSS/SEL transaction */
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

  /* Release SPI3 mutex */

  return status;
}


void spiSX127xReset(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* nRESET: Pull pin 6 down of the SX127x <--> CN9 pin 1 (PA3) of the mbed board */
  HAL_GPIO_WritePin(MCU_OUT_SX_nRESET_GPIO_Port, MCU_OUT_SX_nRESET_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = MCU_OUT_SX_nRESET_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(MCU_OUT_SX_nRESET_GPIO_Port, &GPIO_InitStruct);

  /* At least for 100 µs */
  osDelay(1);

  /* Release the nRESET line */
  HAL_GPIO_WritePin(MCU_OUT_SX_nRESET_GPIO_Port, MCU_OUT_SX_nRESET_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  HAL_GPIO_Init(MCU_OUT_SX_nRESET_GPIO_Port, &GPIO_InitStruct);

  /* Delay for 5 ms before accessing the chip */
  osDelay(5);
}

void spiSX127xFrequency_MHz(float mhz)
{
  /* Register value calc */
  float     fVal    = (mhz * 1e6 * (1UL << 19)) / 32e6;
  uint32_t  regVal  = (uint32_t) (fVal + 0.5f);

  /* Set frequency register */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x06;
  spi3TxBuffer[1] = (uint8_t) ((regVal >> 16) & 0xffUL);
  spi3TxBuffer[2] = (uint8_t) ((regVal >>  8) & 0xffUL);
  spi3TxBuffer[3] = (uint8_t) ((regVal >>  0) & 0xffUL);
  spiProcessSpi3Msg(SPI3_SX, 4);
}

#if 0
uint8_t spiSX1276Power_GetSetting(LoRaWANctx_t* ctx)
{
  int16_t pow_dBm = (int16_t)14 - ctx->LinkADR_TxPowerReduction_dB;
  uint8_t reg;

  if (pow_dBm >= +14) {
    reg = (0x5 << 4) | (0xe << 0);                                                              // --> -43 dBm @ RTL-stick      MaxPower +14dBm, TXpwr @ RFO pin

  } else if (pow_dBm >= +12) {
    reg = (0x5 << 4) | (0xc << 0);                                                              // --> -50 dBm @ RTL-stick

  } else if (pow_dBm >= +10) {
    reg = (0x0 << 4) | (0xe << 0);                                                              // --> -51 dBm @ RTL-stick

  } else if (pow_dBm >=  +8) {
    reg = (0x0 << 4) | (0xc << 0);                                                              // --> -53 dBm @ RTL-stick

  } else if (pow_dBm >=  +6) {
    reg = (0x0 << 4) | (0xa << 0);                                                              // --> -58 dBm @ RTL-stick

  } else if (pow_dBm >=  +4) {
    reg = (0x0 << 4) | (0x8 << 0);                                                              // --> -57 dBm @ RTL-stick

  } else if (pow_dBm >=  +2) {
    reg = (0x0 << 4) | (0x6 << 0);                                                              // --> -58 dBm @ RTL-stick

  } else if (pow_dBm >=   0) {
    reg = (0x0 << 4) | (0x4 << 0);                                                              // --> -56 dBm @ RTL-stick

  } else if (pow_dBm >=  -2) {
    reg = (0x0 << 4) | (0x2 << 0);                                                              // --> -47 dBm @ RTL-stick

  } else if (pow_dBm >=  -4) {
    reg = (0x0 << 4) | (0x1 << 0);                                                              // --> -47 dBm @ RTL-stick

  } else {
    reg = (0x0 << 4) | (0x0 << 0);                                                              // --> -47 dBm @ RTL-stick      Minimal power @ RFO pin
  }

  /* RFO pin used, not PA */
  return (0x0 << 7) || reg;
}
#endif

void spiSX127xDio_Mapping(DIO_TxRx_Mode_t mode)
{
#if 0
  /* DIO0..DIO5 settings */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x40;

  switch (mode) {
  case DIO_TxRx_Mode_TX:
    {
      spi3TxBuffer[1] = (0b01 << 6) | (0b10 << 4) | (0b00 << 2) | (0b01 << 0);  // DIO0: TX done, DIO1: CadDetected, DIO2: FhssChangeChannel, DIO3: ValidHeader
      spi3TxBuffer[2] = (0b01 << 6) | (0b00 << 4);                              // DIO4: PllLock, DIO5: ModeReady
    }
    break;

  case DIO_TxRx_Mode_RX:
  case DIO_TxRx_Mode_RX_Randomizer:
    {
      spi3TxBuffer[1] = (0b00 << 6) | (0b10 << 4) | (0b00 << 2) | (0b01 << 0);  // DIO0: RX done, DIO1: CadDetected, DIO2: FhssChangeChannel, DIO3: ValidHeader
      spi3TxBuffer[2] = (0b01 << 6) | (0b00 << 4);                              // DIO4: PllLock, DIO5: ModeReady
    }
    break;

  default:
    return;
  }

  /* Push settings */
  spiProcessSpi3Msg(3);
#endif
}

#if 0
uint8_t spiSX127xDR_to_SF(DataRates_t dr)
{
  if (DR0_SF12_125kHz_LoRa <= dr && dr <= DR5_SF7_125kHz_LoRa) {
    /* 125 kHz */
    return (12 - (uint8_t)dr);

  } else if (DR6_SF7_250kHz_LoRa) {
    /* 250 kHz */
    return 7;
  }

  /* No LoRa mode */
  return 0;
}
#endif


uint8_t spiSX127xMode_LoRa_GetBroadbandRSSI(void)
{
  /* Broadband RSSI */
  spi3TxBuffer[0] = SPI_RD_FLAG | 0x2c;
  spi3TxBuffer[1] = 0;
  spiProcessSpi3Msg(SPI3_SX, 2);

  return spi3RxBuffer[1];
}

void spiSX127xLoRa_setTxMsgLen(uint8_t payloadLen)
{
  /* Sanity correction */
  if (!payloadLen) {
    payloadLen = 1;
  }

  /* Message length to transmit */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x22;       // RegPayloadLength
  spi3TxBuffer[1] = payloadLen;
  spiProcessSpi3Msg(SPI3_SX, 2);
}

void spiSX127xLoRa_Fifo_Init(void)
{
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x0d;
  spi3TxBuffer[1] = 0x00;                     // 0x0D RegFifoAddrPtr
  spi3TxBuffer[2] = 0xC0;                     // 0x0E RegFifoTxBaseAddr
  spi3TxBuffer[3] = 0x00;                     // 0x0F RegFifoRxBaseAddr
  spiProcessSpi3Msg(SPI3_SX, 4);
}

void spiSX127xLoRa_Fifo_SetFifoPtrFromTxBase(void)
{
  spi3TxBuffer[0] = SPI_RD_FLAG | 0x0e;
  spiProcessSpi3Msg(SPI3_SX, 2);
  uint8_t fifoTxBaseAddr = spi3RxBuffer[1];   // RegFifoTxBaseAddr

  spi3TxBuffer[0] = SPI_WR_FLAG | 0x0d;
  spi3TxBuffer[1] = fifoTxBaseAddr;
  spiProcessSpi3Msg(SPI3_SX, 2);
}

void spiSX127xLoRa_Fifo_SetFifoPtrFromRxBase(void)
{
  spi3TxBuffer[0] = SPI_RD_FLAG | 0x0f;
  spiProcessSpi3Msg(SPI3_SX, 2);
  uint8_t fifoRxBaseAddr = spi3RxBuffer[1];   // RegFifoRxBaseAddr

  spi3TxBuffer[0] = SPI_WR_FLAG | 0x0d;
  spi3TxBuffer[1] = fifoRxBaseAddr;
  spiProcessSpi3Msg(SPI3_SX, 2);
}


uint8_t spiSX127xGetMode(void)
{
  spi3TxBuffer[0] = SPI_RD_FLAG | 0x01;

  /* Read current mode setting */
  spiProcessSpi3Msg(SPI3_SX, 2);

  return spi3RxBuffer[1];
}

void spiSX1276Mode(spiSX1276_Mode_t mode)
{
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x01;
  spi3TxBuffer[1] = mode;

  /* Switch RX/TX at PE4259 */
  switch (mode & TXRX_MODE_MASK) {
  case FSTX:
  case TX:
    /* Write TX mode */
    spiProcessSpi3Msg(SPI3_SX, 2);
    break;

  default:
    /* Write any other RX mode */
    spiProcessSpi3Msg(SPI3_SX, 2);
  }

  /* Delay after mode-change */
  osDelay(25);
}

void spiSX127xRegister_IRQ_clearAll(void)
{
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x12;     // LoRa: RegIrqFlags
  spi3TxBuffer[1] = 0xff;
  spiProcessSpi3Msg(SPI3_SX, 2);
}

void spiSX127xRegister_IRQ_enableBits(uint8_t enaBits)
{
  /* Use TxDone and RxDone - mask out all other IRQs */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x11;                                                         // RegIrqFlagsMask
  spi3TxBuffer[1] = enaBits;
  spiProcessSpi3Msg(SPI3_SX, 2);
}


//#define PPM_CALIBRATION
//#define POWER_CALIBRATION
#if 0
void spiSX1276_TxRx_Preps(LoRaWANctx_t* ctx, DIO_TxRx_Mode_t mode, LoRaWAN_TX_Message_t* msg)
{
#ifdef POWER_CALIBRATION
  ctx->SpreadingFactor              = SF7_DR5_VAL;
  ctx->LinkADR_TxPowerReduction_dB  = 0;                                                        // Range: 0 - 20 dB reduction
# ifdef PPM_CALIBRATION
  ctx->FrequencyMHz                 = 870.0;
  ctx->SpreadingFactor              = SF12_DR0_VAL;
# endif
#else
  /* Determine the frequency to use */
  if (!ctx->FrequencyMHz) {
    if (CurWin_RXTX1 == ctx->Current_RXTX_Window) {
      /* Set TX/RX1 frequency */
      ctx->FrequencyMHz                 = LoRaWAN_calc_Channel_to_MHz(
                                            ctx,
                                            ctx->Ch_Selected,
                                            ctx->Dir,
                                            0);

    } else if (CurWin_RXTX2 == ctx->Current_RXTX_Window) {
      /* Jump to RX2 frequency (default frequency) */
      ctx->FrequencyMHz                 = LoRaWAN_calc_Channel_to_MHz(
                                            ctx,
                                            0,                                                  // The default RX2 channel
                                            ctx->Dir,
                                            1);                                                 // Use default settings
    }
  }

  /* Determine the data rate to use */
  if (!ctx->SpreadingFactor) {
    if (CurWin_RXTX1 == ctx->Current_RXTX_Window) {
      /* TX/RX1 - UpLink and DownLink */
      ctx->SpreadingFactor  = spiSX127xDR_to_SF(ctx->Ch_DataRateTX_Selected[ctx->Ch_Selected - 1] + ctx->LinkADR_DataRate_RX1_DRofs);

    } else if (CurWin_RXTX2 == ctx->Current_RXTX_Window) {
      /* RX2 - DownLink */
      ctx->SpreadingFactor  = spiSX127xDR_to_SF(ctx->Ch_DataRateTX_Selected[ctx->Ch_Selected - 1]);
    }
  }
#endif

  const float     l_f               = ctx->FrequencyMHz;
  const uint8_t   l_SF              = (ctx->SpreadingFactor & 0x0f) << SFx_SHIFT;
  uint32_t        fmt_mhz;
  uint16_t        fmt_mhz_f1;

  mainCalc_Float2Int(l_f, &fmt_mhz, &fmt_mhz_f1);

  if (DIO_TxRx_Mode_IQ_Balancing == mode) {
    /* Switching to FSK/OOK via SLEEP mode */
    spiSX1276Mode(MODE_FSK_OOK | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | SLEEP);
    spiSX1276Mode(MODE_FSK_OOK | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | STANDBY);

    /* Set the frequency */
    spiSX127xFrequency_MHz(ctx->FrequencyMHz * (1 + 1e-6 * ctx->CrystalPpm));

#ifdef TRY
    spiSX1276Mode(MODE_FSK_OOK | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | RXCONTINUOUS);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, 50 / portTICK_PERIOD_MS);
    spiSX1276Mode(MODE_FSK_OOK | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | STANDBY);
#endif

    /* Start I/Q balancing */
    {
      uint8_t balState;
      uint8_t temp;

      /* Blue LED on */
      HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);

      /* Set bit to start */
      spi3TxBuffer[0] = SPI_WR_FLAG | 0x3b;
      spi3TxBuffer[1] = 0x42;                                                                   // RegImageCal
      spiProcessSpi3Msg(2);

      /* Wait until the balancing process has finished */
      uint32_t t0 = getRunTimeCounterValue();                                                   // Returns us since ARM init
      uint32_t t1;
      do {
        osDelay(1);

        /* Request balancing state */
        spi3TxBuffer[0] = SPI_RD_FLAG | 0x3b;
        spiProcessSpi3Msg(3);
        balState  = spi3RxBuffer[1];
        temp      = spi3RxBuffer[2];

        /* Test for completion */
        if (!(balState & 0x20)) {
          t1 = getRunTimeCounterValue();
          ctx->LastIqBalTemp        = temp - 212;                                               // Temperature device compensated
          ctx->LastIqBalTimeUs      = t0;                                                       // Returns us since ARM init
          ctx->LastIqBalDurationUs  = t1 - t0;
          break;
        }
      } while (1);
    }

    /* Blue LED off */
    HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);

    /* Return to LoRa mode */
    spiSX1276Mode(MODE_LoRa    | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | SLEEP);
    spiSX1276Mode(MODE_LoRa    | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | RXCONTINUOUS);

    return;
  }

  /* Skip for RX2 where only frequency and SpreadingFactor is changed */
  if (DIO_TxRx_Mode_RX2 != mode) {
    /* Switching to LoRa via SLEEP mode */
    spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | SLEEP);
    spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | STANDBY);
  }

  /* Debugging information */
  if (g_monMsk & MON_MASK__LORA) {
    uint8_t  buf[64];
    int bufLen;

    bufLen = sprintf((char*) buf, "LoRaWAN:\t f = %03lu.%03u MHz,\t SF = %02u\r\n", fmt_mhz, fmt_mhz_f1, ctx->SpreadingFactor);
    usbLogLen((char*) buf, bufLen);
  }

  /* Common presets for TX / RX */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x1d;
#ifdef PPM_CALIBRATION
  spi3TxBuffer[1] = BW_7kHz8  | CR_4_5 | IHM_OFF;                                               // ModemConfig1
#else
  spi3TxBuffer[1] = BW_125kHz | CR_4_5 | IHM_OFF;                                               // ModemConfig1
#endif
  spi3TxBuffer[2] = l_SF | TXCONT_OFF | RX_PAYLOAD_CRC_ON | (0b00 << 0);                        // ModemConfig2 with SymbTmeoutMsb = 0b00
  spiProcessSpi3Msg(3);

  spi3TxBuffer[0] = SPI_WR_FLAG | 0x26;
  spi3TxBuffer[1] = (l_SF >= SF11_DR1 ?  LOW_DR_OPTI_ON : LOW_DR_OPTI_OFF) | AGC_AUTO_ON;       // ModemConfig3
  spi3TxBuffer[2] = (uint8_t) (ctx->CrystalPpm *  0.95f);                                       // PPM Correction
  spiProcessSpi3Msg(3);

  /* Preamble length */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x20;
  spi3TxBuffer[1] = 0x00;                                                                       // PreambleMsb, PreambleLsb
  spi3TxBuffer[2] = 0x0a;                                                                       // +4.5 = 14.5 symbols
  spiProcessSpi3Msg(3);

  /* Reset the IRQ register */
  spiSX127xRegister_IRQ_clearAll();

  /* Skip for RX2 where only frequency and SpreadingFactor is changed */
  if (DIO_TxRx_Mode_RX2 != mode) {
    /* Frequency hopping disabled */
    spi3TxBuffer[0] = SPI_WR_FLAG | 0x24;
    spi3TxBuffer[1] = 0x00;
    spiProcessSpi3Msg(2);

    /* Sync word */
    spi3TxBuffer[0] = SPI_WR_FLAG | 0x39;
    spi3TxBuffer[1] = 0x34;
    spiProcessSpi3Msg(2);

    /* Interrupt DIO lines activation */
    spiSX127xDio_Mapping(mode);
  }

  switch (mode) {
  case DIO_TxRx_Mode_TX:
    {
      /* Set the frequency */
      spiSX127xFrequency_MHz(l_f * (1 + 1e-6 * ctx->CrystalPpm));

      spi3TxBuffer[0] = SPI_WR_FLAG | 0x09;
#ifdef PPM_CALIBRATION
      spi3TxBuffer[1] = (0x0 << 7) | (0x0 << 4) | (0x0 << 0);                                   // --> -43 dBm @ RTL-stick  Minimal power @ RFO pin
//    spi1TxBuffer[1] = (0x0 << 7) | (0x4 << 4) | (0xf << 0);                                   // --> -25 dBm @ RTL-stick
#else
      spi3TxBuffer[1] = spiSX1276Power_GetSetting(ctx);
#endif
      spi3TxBuffer[2] = PA_RAMP_50us;                                                           // PA ramp time 50us
      spi3TxBuffer[3] = (0x1 << 5) | (0xb << 0);                                                // OverCurrentProtection ON, normal: 100mA
      spiProcessSpi3Msg(4);

      /* I/Q inversion bits */
      spi3TxBuffer[0] = SPI_WR_FLAG | 0x33;
      spi3TxBuffer[1] = 0x27;                                                                   // This is the default value, no inversion
      spiProcessSpi3Msg(2);

      /* I/Q2 inversion bits */
      spi3TxBuffer[0] = SPI_WR_FLAG | 0x38;
      spi3TxBuffer[1] = 0x1d;                                                                   // This is the default value, no inversion
      spiProcessSpi3Msg(2);

      /* Set transmit message length */
      if (!msg) {
        Error_Handler();
      }
      spiSX127xLoRa_setTxMsgLen(msg->msg_encoded_Len);

      /* Prepare the transmitter circuits */
      spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | FSTX);
    }
    break;

  case DIO_TxRx_Mode_RX:
  case DIO_TxRx_Mode_RX2:
    {
      /* Set the frequency */
      spiSX127xFrequency_MHz(l_f * (1 + (1e-6 * ctx->CrystalPpm) - (1e-6 * ctx->GatewayPpm)));

      /* SymbtimeoutLsb */
      spi3TxBuffer[0] = SPI_WR_FLAG | 0x1f;
      spi3TxBuffer[1] = (l_SF >= SF10_DR2 ?  0x05 : 0x08);
      spiProcessSpi3Msg(2);

      /* DetectionThreshold */
      spi3TxBuffer[0] = SPI_WR_FLAG | 0x37;
      spi3TxBuffer[1] = (l_SF >= SF7_DR5 ?  0x0a : 0x0c);
      spiProcessSpi3Msg(2);

      /* Bugfix 2013-09 Rev.1 Section 2.3 - Receiver Spurious Reception of a LoRa Signal */
      {
        /* DetectionOptimize & BugFix (automatic disabled) */
        spi3TxBuffer[0] = SPI_WR_FLAG | 0x31;
        spi3TxBuffer[1] = 0x40 | (l_SF >= SF7_DR5 ?  0x03 : 0x05);                              // Instead of 0xc0 --> 0x40
        spiProcessSpi3Msg(2);

        spi3TxBuffer[0] = SPI_WR_FLAG | 0x2f;
        spi3TxBuffer[1] = 0x40;
        spi3TxBuffer[2] = 0x00;
        spiProcessSpi3Msg(3);
      }

      /* Skip for RX2 where only frequency and SpreadingFactor is changed */
      if (DIO_TxRx_Mode_RX2 != mode) {
        /* LNA to maximum */
        spi3TxBuffer[0] = SPI_WR_FLAG | 0x0c;
        spi3TxBuffer[1] = LnaGain_G1 | LnaBoost_Lf_XXX | LnaBoost_Hf_ON;
        spiProcessSpi3Msg(2);

        /* Max. payload length */
        spi3TxBuffer[0] = SPI_WR_FLAG | 0x23;
        spi3TxBuffer[1] = 0x40;
        spiProcessSpi3Msg(2);

        /* I/Q inversion bits */
        spi3TxBuffer[0] = SPI_WR_FLAG | 0x33;
        spi3TxBuffer[1] = (0x1 << 6) | 0x27;                                                    // Optimized for inverted IQ
        spiProcessSpi3Msg(2);

        /* I/Q2 inversion bits */
        spi3TxBuffer[0] = SPI_WR_FLAG | 0x38;
        spi3TxBuffer[1] = 0x19;                                                                 // Optimized for inverted IQ
        spiProcessSpi3Msg(2);
      }

      /* Prepare the receiver circuits */
      spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | FSRX);
    }
    break;

  case DIO_TxRx_Mode_RX_Randomizer:
    {
      /* LNA to maximum */
      spi3TxBuffer[0] = SPI_WR_FLAG | 0x0c;
      spi3TxBuffer[1] = LnaGain_G1 | LnaBoost_Lf_XXX | LnaBoost_Hf_ON;
      spiProcessSpi3Msg(2);

      /* Turn on receiver */
      spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | RXCONTINUOUS);
    }
    break;

  default:
    return;
  }
}
#endif

uint32_t spiSX127x_WaitUntil_TxDone(uint32_t stopTime)
{
  uint32_t              ts              = 0UL;
  volatile EventBits_t  eb              = { 0 };
  volatile uint8_t      irq             = 0U;

  /* Use TxDone and RxDone - mask out all other IRQs */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x11;       // RegIrqFlagsMask
  spi3TxBuffer[1] = (0x1 << RxTimeoutMask);   // Mask out: bit7 RxTimeout
  spiProcessSpi3Msg(SPI3_SX, 2);

  {
    /* Wait for EXTI / IRQ line(s) */
    TickType_t ticks = 1;
    uint32_t now = xTaskGetTickCount();
    if (stopTime > now) {
      ticks = (stopTime - now) / portTICK_PERIOD_MS;
      eb = xEventGroupWaitBits(extiEventGroupHandle,
          EXTI_SX__DIO0 | EXTI_SX__DIO1,
          EXTI_SX__DIO0 | EXTI_SX__DIO1,
          pdFALSE,
          ticks);

      spi3TxBuffer[0] = SPI_RD_FLAG | 0x12;     // RegIrqFlags
      spiProcessSpi3Msg(SPI3_SX, 2);
      irq = spi3RxBuffer[1];

      if ((eb & EXTI_SX__DIO0) || (irq & (1U << TxDoneMask))) {
        /* Remember point of time when TxDone was set */
        ts = xTaskGetTickCount();

        /* Reset all IRQ flags */
        spi3TxBuffer[0] = SPI_WR_FLAG | 0x12;   // RegIrqFlags
        spi3TxBuffer[1] = 0xff;
        spiProcessSpi3Msg(SPI3_SX, 2);
      }
    }
  }

  /* Time of TX completed */
  return ts;
}


//#define DEBUG_RX
//#define DEBUG_RX2
//#define DEBUG_RX_TIMING

#ifdef DEBUG_RX
static void spiSX127x_WaitUntil__RX_analyzer(LoRaWANctx_t* ctx, uint32_t now, uint32_t stopTime)
{
  TickType_t  xLastWakeTime = now;
  char        debugBuf[512] = { 0 };
  int         debugLen      = 0;

  do {
    /* Get the current IRQ flags */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x12;                                           // LoRa: RegIrqFlags
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t irq = spi3RxBuffer[1];

    /* Get the RF states */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x18;
    spiProcessSpi3Msg(SPI3_SX, 5);
#if 1
    uint8_t   modemStat   = spi3RxBuffer[1];                                        // LoRa: RegModemStat
    int8_t    packetSnr   = spi3RxBuffer[2];                                        // LoRa: RegPktSnrValue
    uint8_t   packetRssi  = spi3RxBuffer[3];                                        // LoRa: RegPktRssiValue
    uint16_t  rssi        = spi3RxBuffer[4];                                        // LoRa: RegRssiValue
#else
    if (modemStat & 0x0b) {
      packetRssi  = spi3RxBuffer[3];
    }
    if (modemStat & 0x40) {
      packetSnr   = spi3RxBuffer[2];
      rssi        = spi3RxBuffer[4];
    }
#endif
    ctx->LastRSSIDbm            = (int16_t) (packetSnr >= 0 ?  (-157 + 16.0/15.0 * packetRssi)  : (-157 +                                 (int16_t)rssi));
    ctx->LastPacketStrengthDBm  = (int16_t) (packetSnr >= 0 ?  (-157 +             rssi)        : (-157 + packetRssi + 0.25 * packetSnr                ));

    /* RX offset and PPM calculation */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x28;
    spiProcessSpi3Msg(SPI3_SX, 5);
    int32_t fei     = ((uint32_t)spi3RxBuffer[1] << 16) | ((uint32_t)spi3RxBuffer[2] << 8) | (spi3RxBuffer[3]);   // LoRa: RegFeiMsb, RegFeiMid, RegFeiLsb
    if (fei >= (1UL << 19)) {
      fei -= (1UL << 20);
    }

    float feiHz         = ((fei / 32e6) * (1UL << 24) * 125.) / 500.;
    float feiPpm        = feiHz / ctx->FrequencyMHz;

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x2c;
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t rssiWideband = spi3RxBuffer[1];                                         // LoRa: RegRssiWideband

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x14;
    spiProcessSpi3Msg(SPI3_SX, 5);
    uint16_t rxHeaderCnt   = ((uint16_t)spi3RxBuffer[1] << 8) | spi3RxBuffer[2];    // LoRa: RxHeaderCnt
    uint16_t rxValidPktCnt = ((uint16_t)spi3RxBuffer[3] << 8) | spi3RxBuffer[4];    // LoRa: RxValidPacketCnt

    /* Check FIFO pointer */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x10;
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t fifoRxCurAddr = spi3RxBuffer[1];                                        // LoRa: FifoRxCurrentAddr

    /* Check FIFO RX byte address */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x25;
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t fifoRxByteAddr = spi3RxBuffer[1];                                       // LoRa: FifoRxByteAddr

    debugLen += sprintf((char*) debugBuf + debugLen,
        "now=%10lu -->\t" \
        "timespan=%10lu -->\t" \
        "irq=0x%02x \t" \
        "modemStat=0x%02x \t" \
        "rssiWideband=%+-i dBm \tRSSI=%-4i dBm \tPacket Strength=%-4i dBm \tpacketSnr=%-4i dB \t\t" \
        "rxHdrCnt=%5u \trxValidPktCnt=%5u \t\t" \
        "fifoRxByteAddr=0x%02x \tfifoCurAddr=0x%02x \t\t" \
        "feiHz=%7ld \tfeiPpm=%+7ld\r\n",
        now,
        (now - ctx->TsEndOfTx),
        irq,
        modemStat,
        -157 + rssiWideband, ctx->LastRSSIDbm, ctx->LastPacketStrengthDBm, packetSnr,
        rxHeaderCnt, rxValidPktCnt,
        fifoRxByteAddr, fifoRxCurAddr,
        (int32_t)feiHz, (int32_t)feiPpm);

    /* Push debugging string to the USB DCD */
    if (debugLen) {
      osSemaphoreWait(usbToHostBinarySemHandle, 0);
      usbToHostWait((uint8_t*) debugBuf, debugLen);
      osSemaphoreRelease(usbToHostBinarySemHandle);

      debugLen = 0;
    }

    /* Next iteration */
    vTaskDelayUntil(&xLastWakeTime, 25 / portTICK_PERIOD_MS);
    now = xTaskGetTickCount();
  } while (stopTime > now);
}
#endif

#if 0
void spiSX127x_WaitUntil_RxDone(LoRaWANctx_t* ctx, LoRaWAN_RX_Message_t* msg, uint32_t stopTime1, uint32_t stopTime2)
{
  uint8_t               irq;
  uint32_t              now             = xTaskGetTickCount();
//uint8_t               modemStat       = 0;
  uint8_t               packetSnr       = 0;
  uint8_t               packetRssi      = 0;
  uint8_t               rssi            = 0;
  uint8_t               validHdr        = 0;

#ifdef DEBUG_RX
  static uint8_t        entryCntr       = 0;
#endif
#ifdef DEBUG_RX_TIMING
  uint32_t              rxWaitStartTs   = 0UL;
  uint32_t              rxCadDetTs      = 0UL;
  uint32_t              rxCadDoneTs     = 0UL;
  uint32_t              rxVldHdrTs      = 0UL;
  uint32_t              rxDoneTs        = 0UL;

  rxWaitStartTs = xTaskGetTickCount();
#endif

  /* Use TxDone and RxDone - mask out all other IRQs */
  spi3TxBuffer[0] = SPI_WR_FLAG | 0x11;                                                         // RegIrqFlagsMask
  spi3TxBuffer[1] = (0x1 << RxTimeoutMask);                                                     // Mask out: bit7 RxTimeout
  spiProcessSpi3Msg(SPI3_SX, 2);

#ifdef DEBUG_RX
  if (++entryCntr == 4) {
    spiSX127x_WaitUntil__RX_analyzer(ctx, now, stopTime2);

    while (1)  HAL_Delay(250);
  }
#endif

  do {
    /* Wait for EXTI / IRQ line(s) */
    TickType_t ticks = 1;

    if (!validHdr) {
      if (stopTime1 > now) {
        ticks = (stopTime1 - now) / portTICK_PERIOD_MS;
      } else {
        break;
      }

    } else {
      if (stopTime2 > now) {
        ticks = (stopTime2 - now) / portTICK_PERIOD_MS;
      } else {
        break;
      }
    }

    xEventGroupWaitBits(extiEventGroupHandle,
        (EXTI_SX__DIO0 | EXTI_SX__DIO1 | EXTI_SX__DIO3),
        (EXTI_SX__DIO0 | EXTI_SX__DIO1 | EXTI_SX__DIO3),
        0,
        ticks);
    now = xTaskGetTickCount();

    /* Get the current IRQ flags */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x12;                                                       // LoRa: RegIrqFlags
    spiProcessSpi3Msg(SPI3_SX, 2);
    irq = spi3RxBuffer[1];

    /* Reset all IRQ flags */
    spiSX127xRegister_IRQ_clearAll();

    #ifdef DEBUG_RX_TIMING
    if (!rxCadDetTs   && (irq & (0x1 << 0))) {                                                  // 0 CadDetectedMask
      rxCadDetTs = xTaskGetTickCount();
    }

    if (!rxCadDoneTs  && (irq & (0x1 << 2))) {                                                  // 2 CadDoneMask
      rxCadDoneTs = xTaskGetTickCount();
    }

    if (!rxVldHdrTs   && (irq & (0x1 << 4))) {                                                  // 4 ValidHeaderMask
      rxVldHdrTs = xTaskGetTickCount();
    }

    if (!rxDoneTs     && (irq & (0x1 << 6))) {                                                  // 6 RxDoneMask
      rxDoneTs = xTaskGetTickCount();
    }
#endif

    if (irq & (1U << RxTimeoutMask)) {
      // Not in use, yet
      //timeout = 1;

    } else if (irq & (1U << ValidHeaderMask)) {
      validHdr = 1;

    } else if (irq & (1U << RxDoneMask)) {
      /* Check for CRC */
      if (irq & (1U << PayloadCrcErrorMask)) {
        continue;
      }

      spi3TxBuffer[0] = SPI_RD_FLAG | 0x18;                                                     // LoRa: RegModemStat
      spiProcessSpi3Msg(SPI3_SX, 5);
//    modemStat   = spi1RxBuffer[1];
      packetSnr   = spi3RxBuffer[2];
      packetRssi  = spi3RxBuffer[3];
      rssi        = spi3RxBuffer[4];
      ctx->LastRSSIDbm           = (int16_t) (packetSnr >= 0 ?  (-157 + 16.0/15.0 * packetRssi) : (-157 + (int16_t)rssi));
//    ctx->LastPacketStrengthDbm = (int16_t) (packetSnr >= 0 ?  (-157 +                   rssi) : (-157 +    packetRssi + 0.25 * packetSnr));
      ctx->LastPacketSnrDb       = packetSnr;

      /* RX offset and PPM calculation */
      spi3TxBuffer[0] = SPI_RD_FLAG | 0x28;
      spiProcessSpi3Msg(SPI3_SX, 5);
      int32_t fei         = ((uint32_t)spi3RxBuffer[1] << 16) | ((uint32_t)spi3RxBuffer[2] << 8) | (spi3RxBuffer[3]);   // LoRa: RegFeiMsb, RegFeiMid, RegFeiLsb
      if (fei >= (1UL << 19)) {
        fei   -= (1UL << 20);
      }
      ctx->LastFeiHz  = ((fei / 32e6) * (1UL << 24) * 125.) / 500.;
      ctx->LastFeiPpm = ctx->LastFeiHz / ctx->FrequencyMHz;

#ifdef DEBUG_RX2
      spi3TxBuffer[0] = SPI_RD_FLAG | 0x14;    // ValidHeaderCnt, ValidPacketCnt
      spiProcessSpi3Msg(SPI3_SX, 5);
      uint16_t rxHeaderCnt   = ((uint16_t)spi3RxBuffer[1] << 8) | spi3RxBuffer[2];
      uint16_t rxValidPktCnt = ((uint16_t)spi3RxBuffer[3] << 8) | spi3RxBuffer[4];

      /* Check FIFO pointer */
      spi3TxBuffer[0] = SPI_RD_FLAG | 0x10;
      spiProcessSpi3Msg(SPI3_SX, 2);
      uint8_t fifoRxCurAddr = spi3RxBuffer[1];                                                  // LoRa: FifoRxCurrentAddr

      /* Check FIFO RX byte address */
      spi3TxBuffer[0] = SPI_RD_FLAG | 0x25;
      spiProcessSpi3Msg(SPI3_SX, 2);
      uint8_t fifoRxByteAddr = spi3RxBuffer[1];                                                 // LoRa: FifoRxByteAddr

      debugLen += sprintf((char*) debugBuf + debugLen,
            "timespan=%5lu ms -->\t" \
            "eb=0x%08lx | irq=0x%02x \t" \
            "modemStat=0x%02x \t" \
/*          "rssi=%-4i dBm \t" \ */
            "RSSI=%-4i dBm \t" \
/*          "Packet Strength=%-4i dBm \t" \ */
            "packetSnr=%-4i dB \t\t" \
            "rxHdrCnt=%5u \trxValidPktCnt=%5u \t\t" \
            "fifoRxByteAddr=0x%02x \tfifoCurAddr=0x%02x \t\t" \
            "feiHz=%7ld \tfeiPpm=%+7ld\r\n",
            (now - spiPreviousWakeTime),
            (uint32_t)eb, irq,
            modemStat,
//          (-157 + rssi),
            ctx->LastRSSIDbm,
//          ctx->LastPacketStrength_dBm,
            packetSnr,
            rxHeaderCnt, rxValidPktCnt,
            fifoRxByteAddr, fifoRxCurAddr,
            (int32_t)feiHz, (int32_t)feiPpm);
#endif

      spi3TxBuffer[0] = SPI_RD_FLAG | 0x13;                                                     // RegRxNbBytes
      spiProcessSpi3Msg(SPI3_SX, 2);
      uint8_t rxNbBytes = spi3RxBuffer[1];

      /* FIFO readout */
      if (rxNbBytes) {
        /* Positioning of the FIFO addr ptr */
        {
          spi3TxBuffer[0] = SPI_RD_FLAG | 0x10;                                                 // RegFifoRxCurrentAddr
          spiProcessSpi3Msg(SPI3_SX, 2);
          uint8_t fifoRxCurrentAddr = spi3RxBuffer[1];

          spi3TxBuffer[0] = SPI_WR_FLAG | 0x0d;                                                 // RegFifoAddrPtr
          spi3TxBuffer[1] = fifoRxCurrentAddr;
          spiProcessSpi3Msg(SPI3_SX, 2);

#ifdef DEBUG_RX2
        debugLen += sprintf((char*)debugBuf + debugLen, " rxNbBytes=%03u fifoRxCurrentAddr=%02x:", rxNbBytes, fifoRxCurrentAddr);
#endif
        }

        /* FIFO read out */
        if (rxNbBytes < sizeof(spi3RxBuffer)) {
          spi3TxBuffer[0] = SPI_RD_FLAG | 0x00;    // RegFifo
          spiProcessSpi3Msg(1 + rxNbBytes);

          /* Copy SPI receive buffer content to msg object */
          memcpy((void*)msg->msg_encoded_Buf, (const void*)spi3RxBuffer + 1, rxNbBytes);
          msg->msg_encoded_Len = rxNbBytes;

#ifdef DEBUG_RX2
          for (uint8_t idx = 0; idx < rxNbBytes; ++idx) {
            debugLen += sprintf((char*)debugBuf + debugLen, " %02x", spi3RxBuffer[1 + idx]);
          }
#endif
        } else {
          /* Buffer to small */
          Error_Handler();
        }
      }

#ifdef DEBUG_RX2
      /* Debugging */
      debugLen += sprintf((char*)debugBuf + debugLen, "\r\n");
      osSemaphoreWait(usbToHostBinarySemHandle, 0);
      usbToHostWait((uint8_t*) debugBuf, debugLen);
      osSemaphoreRelease(usbToHostBinarySemHandle);
#endif

#ifdef DEBUG_RX_TIMING
      {
        char usbDbgBuf[64];
        int  len;

        len = sprintf(usbDbgBuf,
            "(RxDone):\tb=%06lu,\t0=%06lu,\t2=%06lu,\t4=%06lu,\t6=%06lu\r\n",
            rxWaitStartTs, rxCadDetTs, rxCadDoneTs, rxVldHdrTs, rxDoneTs);
        usbLogLen(usbDbgBuf, len);
      }
#endif
      return;
    }
  } while (1);

#ifdef DEBUG_RX_TIMING
      {
        char usbDbgBuf[64];
        int  len;

        len = sprintf(usbDbgBuf,
            "(RxDone):\tb=%06lu,\t0=%06lu,\t2=%06lu,\t4=%06lu,\t6=%06lu\r\n",
            rxWaitStartTs, rxCadDetTs, rxCadDoneTs, rxVldHdrTs, rxDoneTs);
        usbLogLen(usbDbgBuf, len);
      }
#endif
}
#endif

#if 0
void spiSX127x_Process_RxDone(LoRaWANctx_t* ctx, LoRaWAN_RX_Message_t* msg)
{
  uint8_t irq;

  /* Get the current IRQ flags */
  spi3TxBuffer[0] = SPI_RD_FLAG | 0x12;                                                         // LoRa: RegIrqFlags
  spiProcessSpi3Msg(SPI3_SX, 2);
  irq = spi3RxBuffer[1];

  /* Reset all IRQ flags */
  spiSX127xRegister_IRQ_clearAll();

  if (irq & (1U << RxDoneMask)) {
    /* Check for CRC */
    if (irq & (1U << PayloadCrcErrorMask)) {
      return;
    }

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x13;                                                       // RegRxNbBytes
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t rxNbBytes = spi3RxBuffer[1];

    /* FIFO readout */
    if (rxNbBytes) {
      /* Positioning of the FIFO addr ptr */
      {
        spi3TxBuffer[0] = SPI_RD_FLAG | 0x10;                                                   // RegFifoRxCurrentAddr
        spiProcessSpi3Msg(SPI3_SX, 2);
        uint8_t fifoRxCurrentAddr = spi3RxBuffer[1];

        spi3TxBuffer[0] = SPI_WR_FLAG | 0x0d;                                                   // RegFifoAddrPtr
        spi3TxBuffer[1] = fifoRxCurrentAddr;
        spiProcessSpi3Msg(SPI3_SX, 2);
      }

      /* FIFO read out */
      if (rxNbBytes < sizeof(spi3RxBuffer)) {
        spi3TxBuffer[0] = SPI_RD_FLAG | 0x00;    // RegFifo
        spiProcessSpi3Msg(1 + rxNbBytes);

        /* Copy SPI receive buffer content to msg object */
        memcpy((void*)msg->msg_encoded_Buf, (const void*)spi3RxBuffer + 1, rxNbBytes);
        msg->msg_encoded_Len = rxNbBytes;

      } else {
        /* Buffer to small */
        Error_Handler();
      }
    }
  }
}
#endif


uint8_t spiDetectSX1276(void)
{
  /* Reset pulse for SX127x */
  spiSX127xReset();

  /* Turn to sleep mode if not already done */
  spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | SLEEP);

  /* Request RD-address 0x42 RegVersion */
  {
    uint8_t sxVersion = 0;

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x42;
    if (HAL_OK == spiProcessSpi3Msg(SPI3_SX, 2)) {
      sxVersion = spi3RxBuffer[1];
    }

    if (sxVersion != 0x12) {                                                                    // SX1276
      /* We can handle Version  0x12 (SX1276) only */
      return HAL_ERROR;
    }
  }

  /* SX1276 mbed shield found and ready for transmissions */
  return HAL_OK;
}

uint8_t spiDetectAx5243(void)
{
  /* Request RD-address 0x000 SILICONREV */
  {
    uint8_t axVersion = 0;

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x00;
    if (HAL_OK == spiProcessSpi3Msg(SPI3_AX, 2)) {
      axVersion = spi3RxBuffer[1];
    }

    if (axVersion != 0x51) {                                                                    // AX5243
      /* We can handle Version  0x51 (AX5243) only */
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}
