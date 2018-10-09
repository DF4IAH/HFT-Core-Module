/*
 * task_Si5338.c
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"
//#include "stm32l4xx_hal_gpio.h"

#include "usb.h"
#include "bus_i2c.h"
#include "task_Controller.h"

#include "task_Si5338_RegMap_MCU_8MHz.h"
#include "task_Si5338_RegMap_MCU_12MHz.h"
#include "task_Si5338_RegMap_TCXO.h"

#include "task_Si5338.h"


extern osSemaphoreId        c2Si5338_BSemHandle;
extern osSemaphoreId        i2c4_BSemHandle;
extern I2C_HandleTypeDef    hi2c4;

extern EventGroupHandle_t   globalEventGroupHandle;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

extern const Reg_Data_t     si5338_Reg_Store_MCU_8MHz[];
extern const Reg_Data_t     si5338_Reg_Store_MCU_12MHz[];
extern const Reg_Data_t     si5338_Reg_Store_TCXO[];


static uint8_t              s_si5338_enable;
static uint32_t             s_si5338StartTime;
static uint8_t              s_si5338_waitMask;
static I2C_SI5338_CLKIN_VARIANT_t s_si5338_variant;
static I2C_SI5338_CLKIN_VARIANT_t ls_si5338_variant;


static void si5338SettingStep1(void)
{
  /* Figure 9 of DS: reg230[4] = 1, reg241[7] = 1 */
  const Reg_Data_t rdAry[2] = {
      { 230, 0x10, 0x10 },
      { 241, 0x80, 0x80 }
  };

  uint32_t i2cErr = i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  if (i2cErr == HAL_I2C_ERROR_AF) {
    /* Chip not responding */
    usbLog(". si5338: ERROR chip does not respond\r\n");
  }
}

static void si5338SettingStep3(void)
{
  /* Wait for input getting stable */
  while (1) {
    uint8_t regQry[1] = { 218 };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
        /* Leave when ready */
        if (!(i2c4RxBuffer[0] & s_si5338_waitMask)) {
          break;
        }
        osDelay(1);
    }
  }

  /* Figure 9 of DS: reg49[7] = 0, reg246[1] = 1 */
  {
    const Reg_Data_t rdAry[2] = {
        {  49, 0x00, 0x80 },
        { 246, 0x02, 0x02 }
    };

    i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }

  osDelay(25);

  /* Figure 9 of DS: reg241[7] = 0, reg241 = 0x65 */
  {
    const Reg_Data_t rdAry[2] = {
        { 241, 0x00, 0x80 },
        { 241, 0x65, 0xFF }
    };

    i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }

  /* Wait for PLL getting stable */
  while (1) {
    uint8_t regQry[1] = { 218 };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
        /* Leave when ready */
        if (!(i2c4RxBuffer[0] & 0x11)) {
          break;
        }
        osDelay(1);
    }
  }

  /* FCAL values */
  {
    uint8_t val = 0U;

    {
      uint8_t regQry[1] = { 237 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = i2c4RxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[1] = {
            {  47, cVal, 0x03 }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, 1, rdAry);
      }
    }

    {
      uint8_t regQry[1] = { 236 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = i2c4RxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[1] = {
            {  46, cVal, 0xFF }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, 1, rdAry);
      }
    }

    {
      uint8_t regQry[1] = { 235 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = i2c4RxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[2] = {
            {  45, cVal, 0xFF },
            {  47, 0x14, 0xFC }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
      }
    }
  }

  /* Figure 9 of DS: reg49[7] = 1, reg230[4] = 0 */
  {
    const Reg_Data_t rdAry[2] = {
        {  49, 0x80, 0x80 },
        { 230, 0x00, 0x10 }
    };
    i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }
}

static void si5338Execute(void)
{
  /* Avoid to set same variant again */
  if (ls_si5338_variant == s_si5338_variant) {
    return;
  }

  /* Execute variant */
  switch (s_si5338_variant) {
  case I2C_SI5338_CLKIN_VARIANT__MCU_MCO_8MHZ:
    si5338SettingStep1();
    i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, SI5338_MCU_8MHZ_NUM_REGS_MAX, si5338_Reg_Store_MCU_8MHz);
    s_si5338_waitMask = 0x04U;
    si5338SettingStep3();
    break;

  case I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ:
    si5338SettingStep1();
    i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, SI5338_MCU_12MHZ_NUM_REGS_MAX, si5338_Reg_Store_MCU_12MHz);
    s_si5338_waitMask = 0x04U;
    si5338SettingStep3();
    break;

  case I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ:
    si5338SettingStep1();
    i2cSequenceWriteMask(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_SI5338_ADDR, SI5338_TCXO_NUM_REGS_MAX, si5338_Reg_Store_TCXO);
    s_si5338_waitMask = 0x08U;
    si5338SettingStep3();
    break;

  default:
    s_si5338_enable = 0U;
  }

  /* Update last variant */
  ls_si5338_variant = s_si5338_variant;
}

static void si5338Init(void)
{

  /* Switch on TCXO and high-current circuits */
  {
    mainPowerSwitchDo(POWERSWITCH__3V3_HICUR, 1U);
    osDelay(10);
  }

  /* Check if TCXO is enabled */
  {
    /* Enable clock of Port C */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Check if TCXO is enabled */
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(MCU_OUT_20MHZ_EN_GPIO_Port, MCU_OUT_20MHZ_EN_Pin)) {
      s_si5338_variant = I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ;
    }

    /* Disable clock of Port C, again */
    __HAL_RCC_GPIOC_CLK_DISABLE();
  }

  /* Si5338 is active */
  s_si5338_enable = 1U;

  /* Init after power-up */
  si5338Execute();
}


static void si5338MsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                    msgIdx  = 0UL;
  const uint32_t              hdr     = msgAry[msgIdx++];
  const Si5338MsgSi5338Cmds_t cmd     = (Si5338MsgSi5338Cmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgSi5338__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_si5338StartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      si5338Init();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Osc_Si5338, 0U, MsgSi5338__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgSi5338__SetVar01_Variant:
    {
      s_si5338_variant = (uint8_t) (0xffUL & (msgAry[msgIdx++] >> 24U));
    }
    break;

  case MsgSi5338__CallFunc01_Execute:
    {
      si5338Execute();
    }
    break;

  default: { }
  }  // switch (cmd)
}


/* Tasks */

void si5338TaskInit(void)
{
  s_si5338_enable   = 0U;
  s_si5338_waitMask = 0U;
  s_si5338_variant  = I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ;
  ls_si5338_variant = 0U;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_si5338StartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void si5338TaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2Si5338_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Osc_Si5338, 1UL);                    // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    si5338MsgProcess(msgLen, msgAry);
  }
}
