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
#include "task_Controller.h"

#include "task_LCD.h"


extern osSemaphoreId        c2Lcd_BSemHandle;
extern osSemaphoreId        i2c4_BSemHandle;
extern I2C_HandleTypeDef    hi2c4;

extern EventGroupHandle_t   globalEventGroupHandle;


extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

static uint8_t              s_lcd_enable;
static uint32_t             s_lcdStartTime;


static void lcdClearDisplay(void)
{
  /* Clear Display */
  {
    const uint8_t cmdBuf[1] = { 0x01U };
    i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_LCD_ADDR, 0x00, sizeof(cmdBuf), cmdBuf);
    osDelay(3U);
  }
}

static uint8_t lcdTextWrite(uint8_t row, uint8_t col, uint8_t strLen, const uint8_t* strBuf)
{
  if (s_lcd_enable) {
    /* Sanity checks */
    if (row > 1 || col > 15) {
      return HAL_ERROR;
    }

    /* Write Text */
    for (uint8_t txtIdx = 0U; txtIdx < strLen; ++txtIdx) {
      const uint8_t cursorPos = 0x7fU & (row * 40U + col++);
      const uint8_t cmdBuf[1] = { 0x80U | cursorPos };

      for (uint8_t rptIdx = 0U; rptIdx < 3; rptIdx++) {
        /* Set Position */
        i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_LCD_ADDR, 0x00, sizeof(cmdBuf), cmdBuf);

        /* Set Character */
        i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_LCD_ADDR, 0x40, 1, strBuf + txtIdx);
      }
    }
    return HAL_OK;
  }
  return HAL_ERROR;
}

static void lcdInit(void)
{
  /* LCD MIDAS MCCOG21605B6W-FPTLWI */

  s_lcd_enable = 1U;

  usbLog("< LcdInit -\r\n");

  do {
    /* Hardware reset */
    {
      HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, (GPIO_PIN_RESET));
      osDelay(2U);

      HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, (GPIO_PIN_SET));
      osDelay(50U);
    }

    /* Function Set 0x38 (interface: 8bits; lines: 2; single height font; instruction table: 0) */
    {
      const uint8_t txMsg[1] = { 0x38U };

      uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_LCD_ADDR, 0x00U, 1U, txMsg);
      if (i2cErr == HAL_I2C_ERROR_AF) {
        s_lcd_enable = 0U;

        /* Chip not responding */
        usbLog(". LcdInit: ERROR display does not respond\r\n");
        break;
      }
    }

    /* Function set 0x39 (same above; instruction table: 1) */
    {
      const uint8_t txMsg[1] = { 0x39U };

      i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_LCD_ADDR, 0x00U, 1U, txMsg);
      osDelay(2U);
    }

    /* Settings */
    {
      const uint8_t cmds[6] = {
          0x14U,                                                                                      // Internal OSC frequency - 1/5 bias; Osc frequency: abt. 185 Hz
          0x74U,                                                                                      // Contrast: 4 of [0..63]
          0x54U,                                                                                      // Icon: off; Booster: on; Contrast: MSB of 4
          0x6fU,                                                                                      // Follower: on; amplification ratio: 7 of [0..7]
          0x0cU,                                                                                      // Display: on; Cursor: off; Cursor position: off
          0x01U                                                                                       // Clear display
      };
      const uint8_t cmdCnt = sizeof(cmds);

      for (uint8_t cmdIdx = 0U; cmdIdx < cmdCnt; ++cmdIdx) {
        i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_LCD_ADDR, 0x00U, 1U, cmds + cmdIdx);
        osDelay(2U);
      }
    }

    /* LCD-backlight default settings */
    //LcdBacklightInit();
  } while(0);

  usbLog("- LcdInit >\r\n\r\n");
}

static void lcdMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const LcdMsgLcdCmds_t   cmd     = (LcdMsgLcdCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgLcd__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_lcdStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      lcdInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Actor_LCD, 0U, MsgLcd__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgLcd__CallFunc01_ClearDisplay:
    {
      lcdClearDisplay();
    }
    break;

  case MsgLcd__CallFunc02_WriteString:
    {
      const uint8_t byteCnt = 0xffU & (hdr >> 8U);
      const uint8_t pos = 0xffU & (msgAry[msgIdx] >> 24U);
      const uint8_t row = 0x0fU & (pos >> 4U);
      const uint8_t col = 0x0fU &  pos       ;
      uint8_t strBuf[CONTROLLER_MSG_Q_LEN << 2] = { 0 };

      /* Copy string */
      uint8_t wordPos = 2;
      uint8_t strIdx = 0U;
      for (uint8_t pos = 1; pos < byteCnt; pos++) {
        strBuf[strIdx++] = (uint8_t) (0xffU & (msgAry[msgIdx] >> (wordPos << 3U)));

        if (wordPos) {
          --wordPos;

        } else {
          msgIdx++;
          wordPos = 3U;
        }
      }

      /* Write text onto display */
      lcdTextWrite(row, col, strIdx, strBuf);
    }
    break;

  default: { }
  }  // switch (cmd)
}


/* Task */

void lcdTaskInit(void)
{
  s_lcd_enable = 0U;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_lcdStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void lcdTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2Lcd_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Actor_LCD, 1UL);                     // Special case of callbacks need to limit blocking time
    osSemaphoreRelease(c2Lcd_BSemHandle);
    osDelay(3UL);
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    lcdMsgProcess(msgLen, msgAry);
  }
}
