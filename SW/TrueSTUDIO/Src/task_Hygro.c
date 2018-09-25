/*
 * task_Hygro.c
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

#include "task_Hygro.h"


extern osMutexId            i2c4MutexHandle;
extern I2C_HandleTypeDef    hi2c4;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

static uint8_t              s_hygro_enable                    = 1U;
static uint8_t              s_hygroValid                      = 0U;
static uint16_t             s_hygroState                      = 0U;
static int16_t              s_hygro_T_100                     = 0;
static int16_t              s_hygro_RH_100                    = 0;
static uint16_t             s_hygro_S_T                       = 0U;
static int16_t              s_hygro_S_RH                      = 0;
static int16_t              s_hygro_T_cor_100                 = 0;
static int16_t              s_hygro_RH_cor_100                = 0;
static int16_t              s_hygro_DP_100                    = 0;



int16_t hygroGetValue(HYGRO_GET_TYPE_t type)
{
  if (s_hygroValid) {
    switch (type) {
    case HYGRO_GET_TYPE__RH_100:
      return s_hygro_RH_100;
      break;

    case HYGRO_GET_TYPE__T_100:
      return s_hygro_T_100;
      break;

    case HYGRO_GET_TYPE__DP_100:
      return s_hygro_DP_100;
      break;

    default:
      { }
    }
  }

  return 0;
}

static void hygroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< HygroInit -\r\n");

  do {
   /* SHT31-DIS hygro: stop any running jobs */
    i2c4TxBuffer[0] = I2C_SLAVE_HYGRO_REG_BREAK_HI;
    i2c4TxBuffer[1] = I2C_SLAVE_HYGRO_REG_BREAK_LO;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". HygroInit: ERROR chip does not respond\r\n");
      break;
    }
    osDelay(2);

    /* SHT31-DIS hygro: reset */
    i2c4TxBuffer[0] = I2C_SLAVE_HYGRO_REG_RESET_HI;
    i2c4TxBuffer[1] = I2C_SLAVE_HYGRO_REG_RESET_LO;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    osDelay(2);

    /* SHT31-DIS hygro: return current status */
    i2c4TxBuffer[0] = I2C_SLAVE_HYGRO_REG_STATUS_HI;
    i2c4TxBuffer[1] = I2C_SLAVE_HYGRO_REG_STATUS_LO;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) i2c4RxBuffer, min(2U, I2C_RXBUFSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_hygroState = ((uint16_t)i2c4RxBuffer[0] << 8U) | i2c4RxBuffer[1];

    dbgLen = sprintf(dbgBuf, ". HygroInit: SHT31 state: 0x%04X\r\n", s_hygroState);
    usbLogLen(dbgBuf, dbgLen);

    if (s_hygroState) {
      s_hygroValid = 1U;
    }
  } while(0);

  usbLog("- HygroInit >\r\n\r\n");

  osSemaphoreRelease(i2c4MutexHandle);
}

static void hygroFetch(void)
{
  /* Read current measurement data */
  {
    uint8_t regQry[2] = { I2C_SLAVE_HYGRO_REG_FETCH_DATA_HI, I2C_SLAVE_HYGRO_REG_FETCH_DATA_LO };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_HYGRO_ADDR, sizeof(regQry), regQry, 5);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_hygro_S_T  = ((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1];
      s_hygro_S_RH = ((uint16_t)i2c4RxBuffer[3] << 8) | i2c4RxBuffer[4];
    }
  }

  /* Start next measurement - available 15ms later */
  {
    uint8_t dataAry[1] = { I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_LO };
    i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_HYGRO_ADDR, I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_HI, sizeof(dataAry), dataAry);
  }
}

static void hygroCalc(void)
{
  // Calculations for the presentation layer
  static uint16_t   sf_i2cI2c4_hygro_S_T        = 0UL;
  static uint16_t   sf_i2cI2c4_hygro_S_RH       = 0UL;
  int16_t           l_i2cI2c4_hygro_T_100;
  int16_t           l_i2cI2c4_hygro_RH_100;
  int16_t           l_i2cI2c4_hygro_T_cor_100;
  int16_t           l_i2cI2c4_hygro_RH_cor_100;
  uint16_t          l_i2cI2c4_hygro_S_T;
  uint16_t          l_i2cI2c4_hygro_S_RH;
  uint8_t           hasChanged                  = 0U;

  /* Getting the global values */
  {
    l_i2cI2c4_hygro_T_100      = s_hygro_T_100;                                               // last value
    l_i2cI2c4_hygro_RH_100     = s_hygro_RH_100;                                              // last value
    l_i2cI2c4_hygro_S_T        = s_hygro_S_T;                                                 // fetch sensor value
    l_i2cI2c4_hygro_S_RH       = s_hygro_S_RH;                                                // fetch sensor value
    l_i2cI2c4_hygro_T_cor_100  = s_hygro_T_cor_100;                                           // offset correction value
    l_i2cI2c4_hygro_RH_cor_100 = s_hygro_RH_cor_100;                                          // offset correction value
  }

  /* Calculate and present Temp value when a different measurement has arrived */
  int16_t temp_100 = l_i2cI2c4_hygro_T_100;
  if (sf_i2cI2c4_hygro_S_T != l_i2cI2c4_hygro_S_T) {
    temp_100  = (int16_t) ((((int32_t)l_i2cI2c4_hygro_S_T  * 17500) / 0xFFFF) - 4500);
    temp_100 += l_i2cI2c4_hygro_T_cor_100;

    /* Setting the global value */
    s_hygro_T_100 = temp_100;

    hasChanged = 1U;
    sf_i2cI2c4_hygro_S_T = l_i2cI2c4_hygro_S_T;
  }

  /* Calculate and present Hygro value when a different measurement has arrived */
  int16_t rh_100 = l_i2cI2c4_hygro_RH_100;
  if (sf_i2cI2c4_hygro_S_RH != l_i2cI2c4_hygro_S_RH) {
    rh_100  = (int16_t)( ((int32_t)l_i2cI2c4_hygro_S_RH * 10000) / 0xFFFF);
    rh_100 += l_i2cI2c4_hygro_RH_cor_100;

    /* Setting the global value */
    s_hygro_RH_100 = rh_100;

    hasChanged = 1U;
    sf_i2cI2c4_hygro_S_RH = l_i2cI2c4_hygro_S_RH;
  }

  /* Calculate the dew point temperature */
  /* @see https://de.wikipedia.org/wiki/Taupunkt  formula (15) */
  if (hasChanged)
  {
    //const float K1  = 6.112f;
    const float K2    = 17.62f;
    const float K3    = 243.12f;
    const float K2_m_K3 = 4283.7744f; // = K2 * K3;
    float ln_phi    = log(rh_100 / 10000.f);
    float k2_m_theta  = K2 * (temp_100 / 100.f);
    float k3_p_theta  = K3 + (temp_100 / 100.f);
    float term_z    = k2_m_theta / k3_p_theta + ln_phi;
    float term_n    = K2_m_K3    / k3_p_theta - ln_phi;
    float tau_100   = 0.5f + ((100.f * K3) * term_z) / term_n;

    /* Setting the global value */
    s_hygro_DP_100 = (int16_t) tau_100;
  }
}

static void hygroDistributor(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "HYGRO:\t Temp=%+02d.%02uC, RH=%02u.%02u%%, Tau=%+02d.%02uC\r\n",
      (s_hygro_T_100  / 100), (s_hygro_T_100  % 100),
      (s_hygro_RH_100 / 100), (s_hygro_RH_100 % 100),
      (s_hygro_DP_100 / 100), (s_hygro_DP_100 % 100));
  usbLogLen(dbgBuf, dbgLen);
}


/* Task */

void hygroTaskInit(void)
{
  osDelay(550UL);
  hygroInit();
}

void hygroTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 550UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);

  if (s_hygro_enable) {
    hygroFetch();
    hygroCalc();

    //hygroDistributor();
  }
}
