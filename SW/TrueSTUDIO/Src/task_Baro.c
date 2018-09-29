/*
 * task_Baro.c
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

#include "task_Baro.h"


#define C_0DEGC_K                                             (273.15f)


extern osMutexId            i2c4MutexHandle;
extern I2C_HandleTypeDef    hi2c4;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];


/* Correction offset values */
static int16_t              s_qnh_height_m                    = 103;
static int16_t              s_baro_temp_cor_100               = 0;
static int16_t              s_baro_p_cor_100                  = 170;

static uint8_t              s_baro_enable                     = 1U;
static uint8_t              s_baroValid                       = 0U;
static uint16_t             s_baroVersion                     = 0U;
static uint16_t             s_baro_c[C_I2C_BARO_C_CNT]        = { 0U };
static uint32_t             s_baro_d1                         = 0UL;
static uint32_t             s_baro_d2                         = 0UL;

/* Out values */
static int32_t              s_baro_temp_100                   = 0L;
static int32_t              s_baro_p_100                      = 0L;
static int32_t              s_baro_qnh_p_h_100                = 0L;


int32_t baroGetValue(BARO_GET_TYPE_t type)
{
  if (s_baroValid) {
    switch (type) {
    case BARO_GET_TYPE__TEMP_100:
      return s_baro_temp_100;
      break;

    case BARO_GET_TYPE__P_100:
      return s_baro_p_100;
      break;

    case BARO_GET_TYPE__QNH_100:
      return s_baro_qnh_p_h_100;
      break;

    default:
      { }
    }
  }

  return 0L;
}

static void baroFetch(void)
{
  /* Request D1 */
  {
    uint8_t dataAry[0] = { };
    i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, I2C_SLAVE_BARO_REG_CONV_D1_4096, sizeof(dataAry), dataAry);
  }
  osDelay(20);

  /* Get D1 data */
  {
    uint8_t regQry[1] = { I2C_SLAVE_BARO_REG_ADC_READ };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 3);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_baro_d1 = ((uint32_t)i2c4RxBuffer[0] << 16) | ((uint32_t)i2c4RxBuffer[1] << 8) | i2c4RxBuffer[2];
    }
  }

  /* Request D2 */
  {
    uint8_t dataAry[0] = { };
    i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, I2C_SLAVE_BARO_REG_CONV_D2_4096, sizeof(dataAry), dataAry);
  }
  osDelay(20);

  /* Get D2 data */
  {
    uint8_t regQry[1] = { I2C_SLAVE_BARO_REG_ADC_READ };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 3);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_baro_d2 = ((uint32_t)i2c4RxBuffer[0] << 16) | ((uint32_t)i2c4RxBuffer[1] << 8) | i2c4RxBuffer[2];
    }
  }
}

static void baroCalc(void)
{
  static uint32_t sf_i2cI2c4Baro_d1  = 0UL;
  static uint32_t sf_i2cI2c4Baro_d2  = 0UL;
  static int32_t  sf_p       = 0L;
  uint32_t        l_i2cI2c4Baro_d1;
  uint32_t        l_i2cI2c4Baro_d2;
  int16_t         l_i2cI2c4Baro_temp_cor_100;
  int16_t         l_i2cI2c4Baro_p_cor_100;
  int32_t         l_p_h;

  /* Getting the global values */
  l_i2cI2c4Baro_d1            = s_baro_d1;
  l_i2cI2c4Baro_d2            = s_baro_d2;
  l_i2cI2c4Baro_temp_cor_100  = s_baro_temp_cor_100;
  l_i2cI2c4Baro_p_cor_100     = s_baro_p_cor_100;

  /* Calculate and present Baro and Temp values when a different measurement has arrived */
  if ((l_i2cI2c4Baro_d1 != sf_i2cI2c4Baro_d1) || (l_i2cI2c4Baro_d2 != sf_i2cI2c4Baro_d2)) {
    int32_t dT        = (int32_t)l_i2cI2c4Baro_d2 - ((int32_t)s_baro_c[5] << 8);
    int32_t temp_p20  = (int32_t)(((int64_t)dT * s_baro_c[6]) >> 23);
    int32_t l_t_100   = temp_p20 + 2000L;
    int64_t off       = ((int64_t)s_baro_c[2] << 17) + (((int64_t)s_baro_c[4] * dT) >> 6);
    int64_t sens      = ((int64_t)s_baro_c[1] << 16) + (((int64_t)s_baro_c[3] * dT) >> 7);

    /* Low temp and very low temp corrections */
    if (l_t_100 < 2000L) {
      int32_t t2          = (int32_t)(((int64_t)dT * (int64_t)dT) >> 31);
      int32_t temp_p20_2  = temp_p20 * temp_p20;
      int32_t off2        = (61 * temp_p20_2) >> 4;
      int32_t sens2       = temp_p20_2 << 1;

      if (l_t_100 < -1500L) {
        int32_t temp_m15    = l_t_100 + 1500L;
        int32_t temp_m15_2  = temp_m15 * temp_m15;
        off2               += 15 * temp_m15_2;
        sens2              +=  8 * temp_m15_2;
      }
      l_t_100 -= t2;
      off     -= off2;
      sens    -= sens2;
    }

    /* Temperature correction for this sensor */
    l_t_100 += l_i2cI2c4Baro_temp_cor_100;

    int32_t l_p_100 = (int32_t)((((l_i2cI2c4Baro_d1 * sens) >> 21) - off) >> 15);

    /* Pressure correction for this sensor */
    l_p_100 += l_i2cI2c4Baro_p_cor_100;

    /* Store data and calculate QNH within valid data range, only */
    if ((-3000 < l_t_100) && (l_t_100 < 8000) && (30000L < l_p_100) && (l_p_100 < 120000L)) {
      /* Setting the global values */
      s_baro_temp_100  = l_t_100;
      s_baro_p_100     = l_p_100;

      int32_t l_i2cI2c4Baro_temp_100  = l_t_100;

      /* A valid height value (3D navigation) seems to be access able */
      if (sf_p != l_p_100) {
        float a_m_h   = 0.0065f * s_qnh_height_m;
        float Th0     = C_0DEGC_K + (l_i2cI2c4Baro_temp_100 / 100.f);
        float term    = 1.f + (a_m_h / Th0);
        l_p_h         = l_p_100 * pow(term, 5.255f);

        /* Setting the global values */
        s_baro_qnh_p_h_100 = l_p_h;

        sf_p = l_p_100;
      }
    }
  }
}

static void baroDistributor(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "BARO:\t Temp=%+02ld.%02luC, QFE=%04lu.%02luhPa, QNH=%04lu.%02luhPa\r\n",
      (s_baro_temp_100    / 100), (s_baro_temp_100    % 100),
      (s_baro_p_100       / 100), (s_baro_p_100       % 100),
      (s_baro_qnh_p_h_100 / 100), (s_baro_qnh_p_h_100 % 100));
  usbLogLen(dbgBuf, dbgLen);
}


/* Tasks */

static void baroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  usbLog("< BaroInit -\r\n");

  /* MS560702BA03-50 Baro: RESET all internal data paths */
  {
    const uint8_t i2cWriteLongAry[0] = {
    };

    uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, I2C_SLAVE_BARO_REG_RESET, sizeof(i2cWriteLongAry), i2cWriteLongAry);
    if (i2cErr == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". BaroInit: ERROR Baro does not respond\r\n");
      return;
    }
    osDelay(2);
  }

  /* MS560702BA03-50 Baro: get version information */
  {
    uint8_t regQry[1] = { I2C_SLAVE_BARO_REG_VERSION };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_baroVersion = (((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1]) >> 4;

      dbgLen = sprintf(dbgBuf, ". BaroInit: MS560702BA03-50 version: %d\r\n", s_baroVersion);
      usbLogLen(dbgBuf, dbgLen);
    }
  }

  /* MS560702BA03-50 Baro: get correction data from the PROM */
  for (uint8_t adr = 1; adr < C_I2C_BARO_C_CNT; ++adr) {
    uint8_t regQry[1] = { (I2C_SLAVE_BARO_REG_PROM | (adr << 1)) };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_baroVersion = (((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1]) >> 4;
    }

    s_baro_c[adr] = ((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1];

    dbgLen = sprintf(dbgBuf, ". BaroInit: PROM adr=%d value=0x%04X\r\n", adr, s_baro_c[adr]);
    usbLogLen(dbgBuf, dbgLen);
  }

  if (s_baroVersion) {
    s_baroValid = 1U;
  }

  usbLog("- BaroInit >\r\n\r\n");
}


/* Task */

void baroTaskInit(void)
{
  osDelay(600UL);
  baroInit();
}

void baroTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 600UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);

  if (s_baro_enable) {
    #if 0
    {
      char dbgBuf[128];
      int  dbgLen;

      uint32_t now = osKernelSysTick();
      dbgLen = sprintf(dbgBuf, "BARO: prev=%010lu is=%010lu\r\n", sf_previousWakeTime, now);
      usbLogLen(dbgBuf, dbgLen);
    }
    #endif

    baroFetch();
    baroCalc();

    baroDistributor();
  }
}
