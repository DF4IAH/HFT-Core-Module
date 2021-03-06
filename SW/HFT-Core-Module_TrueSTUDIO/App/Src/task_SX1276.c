/*
 * task_SK1276.c
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <task_USB.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"
//#include "stm32l4xx_hal_gpio.h"

#include "main.h"
#include "bus_spi.h"
#include "task_Controller.h"

#include "task_SX1276.h"


extern osSemaphoreId        c2Sx1276_BSemHandle;
extern osSemaphoreId        spi3_BSemHandle;
extern osSemaphoreId        usbToHost_BSemHandle;

extern EventGroupHandle_t   extiEventGroupHandle;
extern EventGroupHandle_t   globalEventGroupHandle;
extern EventGroupHandle_t   spiEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;

extern SPI_HandleTypeDef    hspi3;
extern DMA_HandleTypeDef    hdma_spi3_rx;
extern DMA_HandleTypeDef    hdma_spi3_tx;

extern uint8_t              spi3TxBuffer[SPI3_BUFFERSIZE];
extern uint8_t              spi3RxBuffer[SPI3_BUFFERSIZE];


static uint8_t              s_sx1276_enable;
static uint32_t             s_sx1276StartTime;
static uint8_t              s_sx_version;


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

  /* At least for 100 us */
  osDelay(2);

  /* Release the nRESET line */
  HAL_GPIO_WritePin(MCU_OUT_SX_nRESET_GPIO_Port, MCU_OUT_SX_nRESET_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  HAL_GPIO_Init(MCU_OUT_SX_nRESET_GPIO_Port, &GPIO_InitStruct);

  /* Delay for 5 ms before accessing the chip */
  osDelay(5);
}

void spiSX127xFrequency_MHz(float mhz)
{
  /* Sanity checks */
  if  (                mhz < 136.f  ||
      ( 175.f < mhz && mhz < 410.f) ||
      (1020.f < mhz)) {
    /* Out of bands */
    return;
  }

  /* Register value calc */
  const float     fVal    = (mhz * 1e6 * (1UL << 19)) / 32e6;
  const uint32_t  regVal  = (uint32_t) (fVal + 0.5f);

  /* Set frequency register */
  {
    const uint8_t txMsg[4] = { SPI_WR_FLAG | 0x06U,                                                   // WR address 0x06:
        (uint8_t) ((regVal >> 16) & 0xffUL),
        (uint8_t) ((regVal >>  8) & 0xffUL),
        (uint8_t) ((regVal >>  0) & 0xffUL)
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }

  /* Set RF MUX for HF-band / LF-band */
  {
    if (600.f < mhz) {
      /* HF (band 1) */
      HAL_GPIO_WritePin(MCU_OUT_SX_HF_LF_CTRL_GPIO_Port, MCU_OUT_SX_HF_LF_CTRL_Pin, GPIO_PIN_SET);

    } else {
      /* LF band 2, band 3) */
      HAL_GPIO_WritePin(MCU_OUT_SX_HF_LF_CTRL_GPIO_Port, MCU_OUT_SX_HF_LF_CTRL_Pin, GPIO_PIN_RESET);
    }
  }
}

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

void spiSX127xDio_Mapping(DIO_TxRx_Mode_t mode)
{
#if 0
  /* Wait for SPI3 mutex */
  if (osOK != osMutexWait(spi3MutexHandle, 1000)) {
    return;
  }

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
  spiProcessSpi3MsgLocked(3);

  osMutexRelase(spiMutexHandle);
#endif
}

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


uint8_t spiSX127xMode_LoRa_GetBroadbandRSSI(void)
{
  uint8_t broadRSSI;

  /* Broadband RSSI */
  {
    const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x2cU,                                                   // RD address 0x2c: Broadband_RSSI
        0x00U
    };
    spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
    broadRSSI = spi3RxBuffer[1];
    osSemaphoreRelease(spi3_BSemHandle);
  }

  return broadRSSI;
}

void spiSX127xLoRa_setTxMsgLen(uint8_t payloadLen)
{
  /* Sanity correction */
  if (!payloadLen) {
    payloadLen = 1;
  }

  /* Message length to transmit */
  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x22U,                                                   // WR address 0x22: RegPayloadLength
        payloadLen
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }
}

void spiSX127xLoRa_Fifo_Init(void)
{
  {
    const uint8_t txMsg[4] = { SPI_WR_FLAG | 0x0dU,
        0x00U,                                                                                        // WR address 0x0d: RegFifoAddrPtr
        0xc0U,                                                                                        // WR address 0x0e: RegFifoTxBaseAddr
        0x00U                                                                                         // WR address 0x0f: RegFifoRxBaseAddr
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }
}

void spiSX127xLoRa_Fifo_SetFifoPtrFromTxBase(void)
{
  uint8_t fifoTxBaseAddr;

  {
    const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x0eU,                                                   // RD address 0x0e: RegFifoTxBaseAddr
        0x00U
    };
    spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
    fifoTxBaseAddr = spi3RxBuffer[1];
    osSemaphoreRelease(spi3_BSemHandle);
  }

  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x0dU,                                                   // WR address 0x0d: RegFifoAddrPtr
        fifoTxBaseAddr
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }
}

void spiSX127xLoRa_Fifo_SetFifoPtrFromRxBase(void)
{
  uint8_t fifoRxBaseAddr;

  {
    const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x0fU,                                                   // RD address 0x0e: RegFifoRxBaseAddr
        0x00U
    };
    spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
    fifoRxBaseAddr = spi3RxBuffer[1];
    osSemaphoreRelease(spi3_BSemHandle);
  }

  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x0dU,                                                   // WR address 0x0d: RegFifoAddrPtr
        fifoRxBaseAddr
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }
}


uint8_t spiSX127xGetMode(void)
{
  uint8_t mode;

  {
    const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x01U,                                                   // RD address 0x01: mode
        0x00U
    };
    spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
    mode = spi3RxBuffer[1];
    osSemaphoreRelease(spi3_BSemHandle);
  }

  return mode;
}

void spiSX1276Mode(spiSX1276_Mode_t mode)
{
  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x01U,                                                   // WR address 0x01: mode
        mode
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }

  /* Delay after mode-change */
  osDelay(25);
}

void spiSX127xRegister_IRQ_clearAll(void)
{
  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x12U,                                                   // WR address 0x12: LoRa: RegIrqFlags
        0xffU
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }
}

void spiSX127xRegister_IRQ_enableBits(uint8_t enaBits)
{
  /* Use TxDone and RxDone - mask out all other IRQs */
  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x11U,                                                   // WR address 0x12: LoRa: RegIrqFlagsMask
        enaBits
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }
}


//#define PPM_CALIBRATION
//#define POWER_CALIBRATION
void spiSX1276_TxRx_Preps(LoRaWANctx_t* ctx, DIO_TxRx_Mode_t mode, LoRaWAN_TX_Message_t* msg)
{
#ifdef POWER_CALIBRATION
  ctx->SpreadingFactor              = SF7_DR5_VAL;
  ctx->LinkADR_TxPowerReduction_dB  = 0;                                                              // Range: 0 - 20 dB reduction
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
                                            0,                                                        // The default RX2 channel
                                            ctx->Dir,
                                            1);                                                       // Use default settings
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
  int32_t         fmt_mhz;
  uint32_t        fmt_mhz_f1;

  mainCalcFloat2IntFrac(l_f, 1, &fmt_mhz, &fmt_mhz_f1);

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

      /* Set bit to start */
      {
        const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x3bU,                                               // RegImageCal
            0x42U
        };
        spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
      }

      /* Wait until the balancing process has finished */
      uint32_t t0 = getRunTimeCounterValue();                                                         // Returns us since ARM init
      uint32_t t1;
      do {
        osDelay(2);

        /* Request balancing state */
        {
          const uint8_t txMsg[3] = { SPI_RD_FLAG | 0x3bU,
              0x00U, 0x00U
          };

          spiProcessSpi3MsgTemplateLocked(SPI3_AX, sizeof(txMsg), txMsg, 1U);
          balState  = spi3RxBuffer[1];
          temp      = spi3RxBuffer[2];
          osSemaphoreRelease(spi3_BSemHandle);
        }

        /* Test for completion */
        if (!(balState & 0x20)) {
          t1 = getRunTimeCounterValue();
          ctx->LastIqBalTemp        = temp - 212;                                                     // Temperature device compensated
          ctx->LastIqBalTimeUs      = t0;                                                             // Returns us since ARM init
          ctx->LastIqBalDurationUs  = t1 - t0;
          break;
        }
      } while (1);
    }

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

    bufLen = sprintf((char*) buf, "LoRaWAN:\t f = %03ld.%03lu MHz,\t SF = %02u\r\n", fmt_mhz, fmt_mhz_f1, ctx->SpreadingFactor);
    usbLogLen((char*) buf, bufLen);
  }

  /* Common presets for TX / RX */
  {
    const uint8_t txMsg[3] = { SPI_WR_FLAG | 0x1dU,
#ifdef PPM_CALIBRATION
      BW_7kHz8  | CR_4_5 | IHM_OFF,                                                                   // ModemConfig1
#else
      BW_125kHz | CR_4_5 | IHM_OFF,                                                                   // ModemConfig1
#endif
      l_SF | TXCONT_OFF | RX_PAYLOAD_CRC_ON | (0b00 << 0)                                             // ModemConfig2 with SymbTmeoutMsb = 0b00
    };
    spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
  }
  {
    const uint8_t txMsg[3] = { SPI_WR_FLAG | 0x26U,
        (l_SF >= SF11_DR1 ?  LOW_DR_OPTI_ON : LOW_DR_OPTI_OFF) | AGC_AUTO_ON,                         // ModemConfig3
        (uint8_t) (ctx->CrystalPpm *  0.95f)                                                          // PPM Correction
    };
    spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
  }

  /* Preamble length */
  {
    const uint8_t txMsg[3] = { SPI_WR_FLAG | 0x20U,
        0x00U,                                                                                        // PreambleMsb, PreambleLsb
        0x0aU                                                                                         // +4.5 = 14.5 symbols
    };
    spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
  }

  /* Reset the IRQ register */
  spiSX127xRegister_IRQ_clearAll();

  /* Skip for RX2 where only frequency and SpreadingFactor is changed */
  if (DIO_TxRx_Mode_RX2 != mode) {
    /* Frequency hopping disabled */
    {
      const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x24U,
          0x00U
      };
      spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
    }

    /* Sync word */
    {
      const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x39U,
          0x34U
      };
      spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
    }

    /* Interrupt DIO lines activation */
    spiSX127xDio_Mapping(mode);
  }

  switch (mode) {
  case DIO_TxRx_Mode_TX:
    {
      /* Set the frequency */
      spiSX127xFrequency_MHz(l_f * (1 + 1e-6 * ctx->CrystalPpm));

      {
        const uint8_t txMsg[4] = { SPI_WR_FLAG | 0x09U,
        #ifdef PPM_CALIBRATION
            (0x0 << 7) | (0x0 << 4) | (0x0 << 0),                                                     // --> -43 dBm @ RTL-stick  Minimal power @ RFO pin
        //  (0x0 << 7) | (0x4 << 4) | (0xf << 0),                                                     // --> -25 dBm @ RTL-stick
        #else
            spiSX1276Power_GetSetting(ctx),
        #endif
            PA_RAMP_50us,                                                                             // PA ramp time 50us
            (0x1 << 5) | (0xb << 0)                                                                   // OverCurrentProtection ON, normal: 100mA
        };
        spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
      }

      /* I/Q inversion bits */
      {
        const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x33U,
            0x27U                                                                                     // This is the default value, no inversion
        };
        spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
      }

      /* I/Q2 inversion bits */
      {
        const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x38U,
            0x1dU                                                                                     // This is the default value, no inversion
        };
        spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
      }

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
      {
        const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x1fU,
            l_SF >= SF10_DR2 ?  0x05U : 0x08U
        };
        spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
      }

      /* DetectionThreshold */
      {
        const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x37U,
            l_SF >= SF7_DR5 ?  0x0aU : 0x0cU
        };
        spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
      }

      /* Bugfix 2013-09 Rev.1 Section 2.3 - Receiver Spurious Reception of a LoRa Signal */
      {
        /* DetectionOptimize & BugFix (automatic disabled) */
        {
          const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x31U,
              0x40U | (l_SF >= SF7_DR5 ?  0x03U : 0x05U)                                                 // Instead of 0xc0 --> 0x40
          };
          spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
        }

        {
          const uint8_t txMsg[3] = { SPI_WR_FLAG | 0x2fU,
              0x40U,
              0x00U
          };
          spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
        }
      }

      /* Skip for RX2 where only frequency and SpreadingFactor is changed */
      if (DIO_TxRx_Mode_RX2 != mode) {
        /* LNA to maximum */
        {
          const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x0cU,
              LnaGain_G1 | LnaBoost_Lf_XXX | LnaBoost_Hf_ON
          };
          spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
        }

        /* Max. payload length */
        {
          const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x23U,
              0x40U
          };
          spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
        }

        /* I/Q inversion bits */
        {
          const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x33U,
              (0x1U << 6) | 0x27U                                                                     // Optimized for inverted IQ
          };
          spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
        }

        /* I/Q2 inversion bits */
        {
          const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x38U,
              0x19U                                                                                   // Optimized for inverted IQ
          };
          spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
        }
      }

      /* Prepare the receiver circuits */
      spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | FSRX);
    }
    break;

  case DIO_TxRx_Mode_RX_Randomizer:
    {
      /* LNA to maximum */
      {
        const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x0cU,
            LnaGain_G1 | LnaBoost_Lf_XXX | LnaBoost_Hf_ON
        };
        spiProcessSpi3MsgTemplate(SPI3_AX, sizeof(txMsg), txMsg);
      }

      /* Turn on receiver */
      spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | RXCONTINUOUS);
    }
    break;

  default:
    return;
  }
}

uint32_t spiSX127x_WaitUntil_TxDone(uint32_t stopTime)
{
  uint32_t              ts              = 0UL;
  volatile EventBits_t  eb              = { 0 };
  volatile uint8_t      irq             = 0U;

  /* Use TxDone and RxDone - mask out all other IRQs */
  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x11U,                                                   // WR address 0x11: RegIrqFlagsMask
        0x1U << RxTimeoutMask                                                                         // Mask out: bit7 RxTimeout
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }

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

      /* Get current irq flags */
      {
        const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x12U,                                               // RD address 0x12: RegIrqFlags
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
        irq = spi3RxBuffer[1];
        osSemaphoreRelease(spi3_BSemHandle);
      }

      if ((eb & EXTI_SX__DIO0) || (irq & (1U << TxDoneMask))) {
        /* Remember point of time when TxDone was set */
        ts = xTaskGetTickCount();

        /* Reset all IRQ flags */
        {
          const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x12U,                                             // WR address 0x12: RegIrqFlags
              0xffU
          };
          spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
        }
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
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x12;                                                             // LoRa: RegIrqFlags
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t irq = spi3RxBuffer[1];

    /* Get the RF states */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x18;
    spiProcessSpi3Msg(SPI3_SX, 5);
#if 1
    uint8_t   modemStat   = spi3RxBuffer[1];                                                          // LoRa: RegModemStat
    int8_t    packetSnr   = spi3RxBuffer[2];                                                          // LoRa: RegPktSnrValue
    uint8_t   packetRssi  = spi3RxBuffer[3];                                                          // LoRa: RegPktRssiValue
    uint16_t  rssi        = spi3RxBuffer[4];                                                          // LoRa: RegRssiValue
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
    uint8_t rssiWideband = spi3RxBuffer[1];                                                           // LoRa: RegRssiWideband

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x14;
    spiProcessSpi3Msg(SPI3_SX, 5);
    uint16_t rxHeaderCnt   = ((uint16_t)spi3RxBuffer[1] << 8) | spi3RxBuffer[2];                      // LoRa: RxHeaderCnt
    uint16_t rxValidPktCnt = ((uint16_t)spi3RxBuffer[3] << 8) | spi3RxBuffer[4];                      // LoRa: RxValidPacketCnt

    /* Check FIFO pointer */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x10;
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t fifoRxCurAddr = spi3RxBuffer[1];                                                          // LoRa: FifoRxCurrentAddr

    /* Check FIFO RX byte address */
    spi3TxBuffer[0] = SPI_RD_FLAG | 0x25;
    spiProcessSpi3Msg(SPI3_SX, 2);
    uint8_t fifoRxByteAddr = spi3RxBuffer[1];                                                         // LoRa: FifoRxByteAddr

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
      osSemaphoreWait(usbToHost_BSemHandle, 0);
      usbToHostWait((uint8_t*) debugBuf, debugLen);
      osSemaphoreRelease(usbToHost_BSemHandle);

      debugLen = 0;
    }

    /* Next iteration */
    vTaskDelayUntil(&xLastWakeTime, 25 / portTICK_PERIOD_MS);
    now = xTaskGetTickCount();
  } while (stopTime > now);
}
#endif

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
  {
    const uint8_t txMsg[2] = { SPI_WR_FLAG | 0x11U,                                                   // WR address 0x11: RegIrqFlagsMask
        0x1U << RxTimeoutMask                                                                         // Mask out: bit7 RxTimeout
    };
    spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg), txMsg);
  }

#ifdef DEBUG_RX
  if (++entryCntr == 4) {
    spiSX127x_WaitUntil__RX_analyzer(ctx, now, stopTime2);

    while (1) {
      osDelay(250);
    }
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
        (EXTI_SX__DIO0 | EXTI_SX__DIO1),
        (EXTI_SX__DIO0 | EXTI_SX__DIO1),
        0,
        ticks);
    now = xTaskGetTickCount();

    /* Get the current IRQ flags */
    {
      const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x12U,                                                   // RD address 0x12: LoRa: RegIrqFlags
          0x00U
      };
      spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
      irq = spi3RxBuffer[1];
      osSemaphoreRelease(spi3_BSemHandle);
    }

    /* Reset all IRQ flags */
    spiSX127xRegister_IRQ_clearAll();

    #ifdef DEBUG_RX_TIMING
    if (!rxCadDetTs   && (irq & (0x1 << 0))) {                                                        // 0 CadDetectedMask
      rxCadDetTs = xTaskGetTickCount();
    }

    if (!rxCadDoneTs  && (irq & (0x1 << 2))) {                                                        // 2 CadDoneMask
      rxCadDoneTs = xTaskGetTickCount();
    }

    if (!rxVldHdrTs   && (irq & (0x1 << 4))) {                                                        // 4 ValidHeaderMask
      rxVldHdrTs = xTaskGetTickCount();
    }

    if (!rxDoneTs     && (irq & (0x1 << 6))) {                                                        // 6 RxDoneMask
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

      {
        const uint8_t txMsg[5] = { SPI_RD_FLAG | 0x18U,                                               // RD address 0x18: LoRa: RegModemStat
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
      //modemStat   = spi1RxBuffer[1];
        packetSnr   = spi3RxBuffer[2];
        packetRssi  = spi3RxBuffer[3];
        rssi        = spi3RxBuffer[4];

        osSemaphoreRelease(spi3_BSemHandle);
      }
      ctx->LastRSSIDbm           = (int16_t) (packetSnr >= 0 ?  (-157 + 16.0/15.0 * packetRssi) : (-157 + (int16_t)rssi));
//    ctx->LastPacketStrengthDbm = (int16_t) (packetSnr >= 0 ?  (-157 +                   rssi) : (-157 +    packetRssi + 0.25 * packetSnr));
      ctx->LastPacketSnrDb       = packetSnr;

      /* RX offset and PPM calculation */
      int32_t fei;
      {
        const uint8_t txMsg[5] = { SPI_RD_FLAG | 0x28U,                                               // RD address 0x28: LoRa: RegFeiMsb, RegFeiMid, RegFeiLsb
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
        fei = ((uint32_t)spi3RxBuffer[1] << 16) | ((uint32_t)spi3RxBuffer[2] << 8) | (spi3RxBuffer[3]);
        osSemaphoreRelease(spi3_BSemHandle);
      }
      if (fei >= (1UL << 19)) {
        fei   -= (1UL << 20);
      }
      ctx->LastFeiHz  = ((fei / 32e6) * (1UL << 24) * 125.) / 500.;
      ctx->LastFeiPpm = ctx->LastFeiHz / ctx->FrequencyMHz;

#ifdef DEBUG_RX2
      uint16_t rxHeaderCnt;
      uint16_t rxValidPktCnt;
      {
        const uint8_t txMsg[5] = { SPI_RD_FLAG | 0x14U,                                               // RD address 0x14: ValidHeaderCnt, ValidPacketCnt
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg);
        rxHeaderCnt   = ((uint16_t)spi3RxBuffer[1] << 8) | spi3RxBuffer[2];
        rxValidPktCnt = ((uint16_t)spi3RxBuffer[3] << 8) | spi3RxBuffer[4];

        osMutexRelease(spi3MutexHandle);
      }

      /* Check FIFO pointer */
      uint8_t fifoRxCurAddr;
      {
        const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x10U,                                               // RD address 0x10: LoRa: FifoRxCurrentAddr
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg);
        uint8_t fifoRxCurAddr = spi3RxBuffer[1];

        osMutexRelease(spi3MutexHandle);
      }

      /* Check FIFO RX byte address */
      uint8_t fifoRxByteAddr;
      {
        const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x25U,                                               // RD address 0x25: LoRa: FifoRxByteAddr
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg);
        fifoRxByteAddr = spi3RxBuffer[1];

        osMutexRelease(spi3MutexHandle);
      }

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

      uint8_t rxNbBytes;
      {
        const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x13U,                                               // RD address 0x13: LoRa: RegRxNbBytes
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
        rxNbBytes = spi3RxBuffer[1];
        osSemaphoreRelease(spi3_BSemHandle);
      }

      /* FIFO readout */
      if (rxNbBytes) {
        /* Positioning of the FIFO addr ptr */
        uint8_t fifoRxCurrentAddr;
        {
          const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x10U,                                             // RD address 0x10: RegFifoRxCurrentAddr
              0x00U
          };
          spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
          fifoRxCurrentAddr = spi3RxBuffer[1];
          osSemaphoreRelease(spi3_BSemHandle);

          const uint8_t txMsg2[2] = { SPI_WR_FLAG | 0x13U,                                            // WR address 0x13: RegFifoAddrPtr
              fifoRxCurrentAddr
          };
          spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg2), txMsg2);

#ifdef DEBUG_RX2
        debugLen += sprintf((char*)debugBuf + debugLen, " rxNbBytes=%03u fifoRxCurrentAddr=%02x:", rxNbBytes, fifoRxCurrentAddr);
#endif
        }

        /* FIFO read out */
        if (rxNbBytes < sizeof(spi3RxBuffer)) {
          {
            if (osOK != osSemaphoreWait(spi3_BSemHandle, 1000)) {
              return;
            }

            spi3TxBuffer[0] = SPI_RD_FLAG | 0x00U;                                                    // RegFifo
            spiProcessSpi3MsgLocked(SPI3_SX, (1 + rxNbBytes), 1U);

            /* Copy SPI receive buffer content to msg object */
            memcpy((void*)msg->msg_encoded_Buf, (const void*)spi3RxBuffer + 1, rxNbBytes);
            msg->msg_encoded_Len = rxNbBytes;

            osSemaphoreRelease(spi3_BSemHandle);
          }

#ifdef DEBUG_RX2
          for (uint8_t idx = 0; idx < rxNbBytes; ++idx) {
            debugLen += sprintf((char*)debugBuf + debugLen, " %02x", msg->msg_encoded_Buf[idx]);
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
      osSemaphoreWait(usbToHost_BSemHandle, 0);
      usbToHostWait((uint8_t*) debugBuf, debugLen);
      osSemaphoreRelease(usbToHost_BSemHandle);
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

void spiSX127x_Process_RxDone(LoRaWANctx_t* ctx, LoRaWAN_RX_Message_t* msg)
{
  uint8_t irq;

  /* Get the current IRQ flags */
  {
    const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x12U,                                                   // LoRa: RegIrqFlags
        0x00U
    };
    spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
    irq = spi3RxBuffer[1];
    osSemaphoreRelease(spi3_BSemHandle);
  }

  /* Reset all IRQ flags */
  spiSX127xRegister_IRQ_clearAll();

  if (irq & (1U << RxDoneMask)) {
    /* Check for CRC */
    if (irq & (1U << PayloadCrcErrorMask)) {
      return;
    }

    uint8_t rxNbBytes;
    {
      const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x13U,                                                 // RegRxNbBytes
          0x00U
      };
      spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
      rxNbBytes = spi3RxBuffer[1];
      osSemaphoreRelease(spi3_BSemHandle);
    }

    /* FIFO readout */
    if (rxNbBytes) {
      /* Positioning of the FIFO addr ptr */
      {
        const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x10U,                                               // RegFifoRxCurrentAddr
            0x00U
        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
        uint8_t fifoRxCurrentAddr = spi3RxBuffer[1];
        osSemaphoreRelease(spi3_BSemHandle);

        const uint8_t txMsg2[2] = { SPI_WR_FLAG | 0x0dU,                                              // RegFifoAddrPtr
            fifoRxCurrentAddr
        };
        spiProcessSpi3MsgTemplate(SPI3_SX, sizeof(txMsg2), txMsg2);
      }

      /* FIFO read out */
      if (rxNbBytes < sizeof(spi3RxBuffer)) {
        const uint8_t txMsg[8] = { SPI_RD_FLAG | 0x00U                                                // RegFifo

        };
        spiProcessSpi3MsgTemplateLocked(SPI3_SX, (1 + rxNbBytes), txMsg, 1U);
        rxNbBytes = spi3RxBuffer[1];

        /* Copy SPI receive buffer content to msg object */
        memcpy((void*)msg->msg_encoded_Buf, (const void*)spi3RxBuffer + 1, rxNbBytes);
        msg->msg_encoded_Len = rxNbBytes;

        osSemaphoreRelease(spi3_BSemHandle);

      } else {
        /* Buffer to small */
        Error_Handler();
      }
    }
  }
}


uint8_t spiDetectSX1276(void)
{
  /* Reset pulse for SX127x */
  spiSX127xReset();

  /* Turn to sleep mode if not already done */
  spiSX1276Mode(MODE_LoRa | ACCESS_SHARE_OFF | LOW_FREQ_MODE_OFF | SLEEP);

  /* Request RD-address 0x42 RegVersion */
  {
    const uint8_t txMsg[2] = { SPI_RD_FLAG | 0x42U,                                                   // RD address 0x42: RegVersion
        0x00U
    };
    spiProcessSpi3MsgTemplateLocked(SPI3_SX, sizeof(txMsg), txMsg, 1U);
    s_sx_version = spi3RxBuffer[1];
    osSemaphoreRelease(spi3_BSemHandle);
  }

  if (s_sx_version != 0x12) {                                                                         // SX1276
    /* We can handle Version  0x12 (SX1276) only */
    return HAL_ERROR;
  }

  /* SX1276 mbed shield found and ready for transmissions */
  return HAL_OK;
}


const char          PM_SPI_INIT_SX1276_01[]       = "< SX1276_Init -\r\n";
const char          PM_SPI_INIT_SX1276_02[]       = ". SX1276_Init:  SX1276 VHF/UHF LoRa and FSK transceiver\r\n";
const char          PM_SPI_INIT_SX1276_03[]       = ". SX1276_Init:  Silicon-Rev: 0x%02X\r\n";
const char          PM_SPI_INIT_SX1276_04[]       = ". SX1276_Init:  ERROR Device not found on board.\r\n";
const char          PM_SPI_INIT_SX1276_05[]       = "- SX1276_Init>\r\n\r\n";
static void sx1276Init(void)
{
  int  dbgLen;
  char dbgBuf[128];

  usbLog(PM_SPI_INIT_SX1276_01);
  usbLog(PM_SPI_INIT_SX1276_02);

  /* SPI3 init */
  spix_Init(&hspi3, spi3_BSemHandle);

  if (HAL_OK == spiDetectSX1276()) {
    dbgLen = snprintf(dbgBuf, sizeof(dbgBuf), PM_SPI_INIT_SX1276_03, s_sx_version);
    usbLogLen(dbgBuf, min(dbgLen, sizeof(dbgBuf)));

    loRaWANLoraTaskInit();

    s_sx1276_enable = 1U;

  } else {
    usbLog(PM_SPI_INIT_SX1276_04);
  }
  usbLog(PM_SPI_INIT_SX1276_05);
}

static void sx1276MsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                    msgIdx  = 0UL;
  const uint32_t              hdr     = msgAry[msgIdx++];
  const sx1276MsgSx1276Cmds_t cmd     = (sx1276MsgSx1276Cmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgSx1276__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_sx1276StartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      sx1276Init();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Radio_SX1276, 0U, MsgSx1276__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  default: { }
  }  // switch (cmd)
}


/* Tasks */

void sx1276TaskInit(void)
{
  s_sx1276_enable = 0U;
  s_sx_version    = 0U;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_sx1276StartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void sx1276TaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2Sx1276_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Radio_SX1276, 1UL);                  // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    sx1276MsgProcess(msgLen, msgAry);
  }
}
