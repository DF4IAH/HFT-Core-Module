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

static uint8_t              s_lcd_enable                      = 1U;


uint8_t lcdTextWrite(uint8_t row, uint8_t col, uint8_t strLen, const uint8_t* strBuf)
{
  if (s_lcd_enable) {
    /* Sanity checks */
    if (row > 1 || col > 15) {
      return HAL_ERROR;
    }

    const uint8_t cursorPos = 0x7fU & (row * 40U + col);

    /* Set Position */
    {
      const uint8_t cmdBuf[1] = { 0x80U | cursorPos };
      i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_LCD_ADDR, 0x00, sizeof(cmdBuf), cmdBuf);
      osDelay(2U);
    }

    /* Write Text */
    {
      for (uint8_t txtIdx = 0U; txtIdx < strLen; ++txtIdx) {
        i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_LCD_ADDR, 0x40, 1, strBuf + txtIdx);
        osDelay(2U);
      }
    }
    return HAL_OK;
  }
  return HAL_ERROR;
}

static void lcdInit(void)
{
  /* LCD MIDAS MCCOG21605B6W-FPTLWI */

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

      uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_LCD_ADDR, 0x00U, 1U, txMsg);
      if (i2cErr == HAL_I2C_ERROR_AF) {
        s_lcd_enable = 0U;

        /* Chip not responding */
        usbLog(". LcdInit: ERROR display does not respond\r\n");
        break;
      }
      osDelay(2U);
    }

    /* Function set 0x39 (same above; instruction table: 1) */
    {
      const uint8_t txMsg[1] = { 0x39U };

      i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_LCD_ADDR, 0x00U, 1U, txMsg);
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
        i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_LCD_ADDR, 0x00U, 1U, cmds + cmdIdx);
        osDelay(2U);
      }
    }

    /* Welcome Text / Logo */
    {
      const uint8_t strBuf[]  = "*HFT-CoreModule*";
      const uint8_t strLen    = sizeof(strBuf);

      for (uint8_t strIdx = 0U; strIdx < strLen; ++strIdx) {
        i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_LCD_ADDR, 0x40, 1U, strBuf + strIdx);
        osDelay(2U);
      }
    }
  } while(0);

  usbLog("- LcdInit >\r\n\r\n");
}


/* Task */

void lcdTaskInit(void)
{
  osDelay(200UL);

  if (s_lcd_enable) {
    lcdInit();
  }
}

void lcdTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 200UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);

  if (s_lcd_enable) {
    const int32_t p_100     = baroGetValue(BARO_GET_TYPE__QNH_100);
    const uint16_t p_100_i  = (uint16_t) (p_100 / 100UL);
    const uint16_t p_100_f  = (uint16_t) (p_100 % 100UL);

    const int16_t rh_100    = hygroGetValue(HYGRO_GET_TYPE__RH_100);
    const int16_t rh_100_i  = rh_100 / 100U;
    const int16_t rh_100_f  = (rh_100 % 100U) / 10U;

    uint8_t strBuf[17];
    const uint8_t len       = sprintf((char*)strBuf, "%04u.%02uhPa %02u.%01u%%", p_100_i, p_100_f, rh_100_i, rh_100_f);

    if (p_100) {
      lcdTextWrite(1U, 0U, len, strBuf);
    }
  }
}
