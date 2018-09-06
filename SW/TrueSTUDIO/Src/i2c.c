/*
 * i2c.c
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

#include "i2c.h"
#include "si5338_RegMap_TCXO.h"
#include "si5338_RegMap_MCU_8MHz.h"


#define C_0DEGC_K                                             (273.15f)


extern osThreadId           tcxo20MhzTaskHandle;

extern osMutexId            i2c1MutexHandle;
extern osMutexId            i2c2MutexHandle;
extern osMutexId            i2c3MutexHandle;
extern osMutexId            i2c4MutexHandle;

extern I2C_HandleTypeDef    hi2c1;
extern I2C_HandleTypeDef    hi2c2;
extern I2C_HandleTypeDef    hi2c3;
extern I2C_HandleTypeDef    hi2c4;

uint8_t                     aTxBuffer[32]                     = { 0U };
uint8_t                     aRxBuffer[32]                     = { 0U };

static uint8_t              s_i2cI2c4HygroValid               = 0U;
static uint16_t             s_i2cI2c4HygroState               = 0U;
static int16_t              s_i2cI2c4Hygro_T_100              = 0;
static int16_t              s_i2cI2c4Hygro_RH_100             = 0;
static uint16_t             s_i2cI2c4Hygro_S_T                = 0U;
static int16_t              s_i2cI2c4Hygro_S_RH               = 0;
static int16_t              s_i2cI2c4Hygro_T_cor_100          = 0;
static int16_t              s_i2cI2c4Hygro_RH_cor_100         = 0;
static int16_t              s_i2cI2c4Hygro_DP_100             = 0;

static uint8_t              s_i2cI2c4BaroValid                = 0U;
static uint16_t             s_i2cI2c4BaroVersion              = 0U;
static uint16_t             s_i2cI2c4Baro_c[C_I2C_BARO_C_CNT] = { 0U };
static uint32_t             s_i2cI2c4Baro_d1                  = 0UL;
static uint32_t             s_i2cI2c4Baro_d2                  = 0UL;
static int16_t              s_i2cI2c4Baro_temp_cor_100        = 0;
static int16_t              s_i2cI2c4Baro_p_cor_100           = 0;
static int32_t              s_i2cI2c4Baro_temp_100            = 0L;
static int32_t              s_i2cI2c4Baro_p_100               = 0L;
static int32_t              s_i2cI2c4Baro_qnh_p_h_100         = 0L;

static uint8_t              s_i2cI2c4GyroValid                = 0U;
static uint8_t              s_i2cI2c4Gyro1Version             = 0U;
static uint8_t              s_i2cI2c4Gyro2Version             = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaX                = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaY                = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaZ                = 0U;


static uint16_t             s_i2cI2c4Tcxo20MhzDacStatus       = 0U;


#ifdef I2C_BUS_ADDR_SCAN
void i2cBusAddrScan(I2C_HandleTypeDef* dev, osMutexId mutexHandle)
{
  /* DEBUG I2C4 Bus */
  char dbgBuf[64];
  int  dbgLen;

  osSemaphoreWait(mutexHandle, osWaitForever);

  aTxBuffer[0] = 0x00;
  for (uint8_t addr = 0x01U; addr <= 0x7FU; addr++) {
    if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aTxBuffer, min(0U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      usbLog("ERROR: 1\r\n");
    }
    while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(dev) != HAL_I2C_ERROR_AF) {
      dbgLen = sprintf(dbgBuf, "GOOD:  Addr=0x%02X  got response\r\n", addr);
      usbLogLen(dbgBuf, dbgLen);
    }
    osDelay(25);
  }

  osSemaphoreRelease(mutexHandle);
}
#endif

//#define DEBUG_WRITE_MASK 1
uint32_t i2cSequenceWriteMask(I2C_HandleTypeDef* dev, osMutexId mutexHandle, uint8_t addr, uint16_t count, const Reg_Data_t dataAry[])
{
  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(mutexHandle, osWaitForever);

  for (uint16_t listIdx = 0U; listIdx < count; listIdx++) {
    if (dataAry[listIdx].Reg_Mask == 0xffU) {
      /* Write without read */
      aTxBuffer[0] = dataAry[listIdx].Reg_Addr;
      aTxBuffer[1] = dataAry[listIdx].Reg_Val;
      if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
        osDelay(1);
      }
	  if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
        /* Return mutex */
        osSemaphoreRelease(mutexHandle);

        /* Chip not responding */
		usbLog("i2cSequenceWriteMask: ERROR chip does not respond\r\n");
		return HAL_I2C_ERROR_AF;
	  }

#ifdef DEBUG_WRITE_MASK
      {
        int   dbgLen = 0;
        char  dbgBuf[128];

        dbgLen = sprintf(dbgBuf, "i2cSequenceWriteMask: ListIdx=%03d, I2C addr=0x%02X, val=0x%02X\r\n",
            listIdx,
			dataAry[listIdx].Reg_Addr,
			dataAry[listIdx].Reg_Val);
        usbLogLen(dbgBuf, dbgLen);
      }
#endif

    } else if (dataAry[listIdx].Reg_Mask) {
      /* Read current data */
	  aTxBuffer[0] = dataAry[listIdx].Reg_Addr;
	  if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
        /* Error_Handler() function is called when error occurs. */
		Error_Handler();
	  }
	  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
		osDelay(1);
	  }
	  if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
        /* Return mutex */
        osSemaphoreRelease(mutexHandle);

        /* Chip not responding */
		usbLog("i2cSequenceWriteMask: ERROR chip does not respond\r\n");
		return HAL_I2C_ERROR_AF;
	  }

      memset(aRxBuffer, 0, sizeof(aRxBuffer));
	  if (HAL_I2C_Master_Sequential_Receive_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aRxBuffer, min(1U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
		Error_Handler();
	  }
	  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
		osDelay(1);
	  }
	  uint8_t val1 = aRxBuffer[0];
	  uint8_t val2 = val1;

	  /* Modify value */
	  val2 &= ~(dataAry[listIdx].Reg_Mask);
	  val2 |= dataAry[listIdx].Reg_Mask & dataAry[listIdx].Reg_Val;

	  if (val2 != val1) {
		/* Write back value */
		aTxBuffer[0] = dataAry[listIdx].Reg_Addr;
		aTxBuffer[1] = val2;
		if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
	      /* Error_Handler() function is called when error occurs. */
		  Error_Handler();
		}
		while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
		  osDelay(1);
        }
      }

#ifdef DEBUG_WRITE_MASK
      {
        int   dbgLen = 0;
        char  dbgBuf[128];

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
  osSemaphoreRelease(mutexHandle);

  return HAL_I2C_ERROR_NONE;
}

uint32_t i2cSequenceWriteLong(I2C_HandleTypeDef* dev, osMutexId mutexHandle, uint8_t addr, uint8_t i2cReg, uint16_t count, const uint8_t i2cWriteAryLong[])
{
  if (count && ((count - 1) >= TXBUFFERSIZE)) {
    return HAL_I2C_ERROR_SIZE;
  }

  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(mutexHandle, osWaitForever);

  aTxBuffer[0] = i2cReg;

  for (uint8_t idx = 0; idx < count; idx++) {
    aTxBuffer[idx + 1] = i2cWriteAryLong[idx];
  }

  if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aTxBuffer, min((count + 1U), TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
    /* Error_Handler() function is called when error occurs. */
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1);
  }
  uint32_t i2cErr = HAL_I2C_GetError(dev);

  /* Return mutex */
  osSemaphoreRelease(mutexHandle);

  if (i2cErr == HAL_I2C_ERROR_AF) {
    /* Chip not responding */
	usbLog("i2cSequenceWriteLong: ERROR chip does not respond\r\n");
	return HAL_I2C_ERROR_AF;
  }

  return HAL_I2C_ERROR_NONE;
}

uint32_t i2cSequenceRead(I2C_HandleTypeDef* dev, osMutexId mutexHandle, uint8_t addr, uint8_t i2cRegLen, uint8_t i2cReg[], uint16_t readLen)
{
  /* Wait for the I2Cx bus to be free */
  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  for (uint8_t regIdx = 0; regIdx < i2cRegLen; regIdx++) {
    aTxBuffer[regIdx] = i2cReg[regIdx];
  }
  if (HAL_I2C_Master_Sequential_Transmit_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aTxBuffer, min(i2cRegLen, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
    /* Error_Handler() function is called when error occurs. */
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1);
  }
  if (HAL_I2C_GetError(dev) == HAL_I2C_ERROR_AF) {
    /* Return mutex */
    osSemaphoreRelease(i2c4MutexHandle);

    /* Chip not responding */
    usbLog("i2cSequenceRead: ERROR chip does not respond\r\n");
    return HAL_I2C_ERROR_AF;
  }

  memset(aRxBuffer, 0, sizeof(aRxBuffer));
  if (HAL_I2C_Master_Sequential_Receive_IT(dev, (uint16_t) (addr << 1U), (uint8_t*) aRxBuffer, min(readLen, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
    Error_Handler();
  }
  while (HAL_I2C_GetState(dev) != HAL_I2C_STATE_READY) {
    osDelay(1);
  }

  /* Return mutex */
  osSemaphoreRelease(i2c4MutexHandle);

  return HAL_I2C_ERROR_NONE;
}


static void i2cI2c4HygroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< i2cI2c4HygroInit -\r\n");

  do {
   /* SHT31-DIS hygro: stop any running jobs */
    aTxBuffer[0] = I2C_SLAVE_HYGRO_REG_BREAK_HI;
    aTxBuffer[1] = I2C_SLAVE_HYGRO_REG_BREAK_LO;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4HygroInit: ERROR chip does not respond\r\n");
      break;
    }
    osDelay(2);

    /* SHT31-DIS hygro: reset */
    aTxBuffer[0] = I2C_SLAVE_HYGRO_REG_RESET_HI;
    aTxBuffer[1] = I2C_SLAVE_HYGRO_REG_RESET_LO;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    osDelay(2);

    /* SHT31-DIS hygro: return current status */
    aTxBuffer[0] = I2C_SLAVE_HYGRO_REG_STATUS_HI;
    aTxBuffer[1] = I2C_SLAVE_HYGRO_REG_STATUS_LO;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(aRxBuffer, 0, sizeof(aRxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_HYGRO_ADDR << 1U), (uint8_t*) aRxBuffer, min(2U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4HygroState = ((uint16_t)aRxBuffer[0] << 8U) | aRxBuffer[1];

    dbgLen = sprintf(dbgBuf, ". i2cI2c4HygroInit: SHT31 state: 0x%04X\r\n", s_i2cI2c4HygroState);
    usbLogLen(dbgBuf, dbgLen);

    if (s_i2cI2c4HygroState) {
      s_i2cI2c4HygroValid = 1U;
    }
  } while(0);

  usbLog("- i2cI2c4HygroInit >\r\n\r\n");

  osSemaphoreRelease(i2c4MutexHandle);
}

static void i2cI2c4HygroFetch(void)
{
  /* Read current measurement data */
  {
    uint8_t regQry[2] = { I2C_SLAVE_HYGRO_REG_FETCH_DATA_HI, I2C_SLAVE_HYGRO_REG_FETCH_DATA_LO };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_HYGRO_ADDR, sizeof(regQry), regQry, 5);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_i2cI2c4Hygro_S_T  = ((uint16_t)aRxBuffer[0] << 8) | aRxBuffer[1];
      s_i2cI2c4Hygro_S_RH = ((uint16_t)aRxBuffer[3] << 8) | aRxBuffer[4];
    }
  }

  /* Start next measurement - available 15ms later */
  {
    uint8_t dataAry[1] = { I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_LO };
    i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_HYGRO_ADDR, I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_HI, sizeof(dataAry), dataAry);
  }
}

static void i2cI2c4HygroCalc(void)
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
    l_i2cI2c4_hygro_T_100      = s_i2cI2c4Hygro_T_100;                                               // last value
    l_i2cI2c4_hygro_RH_100     = s_i2cI2c4Hygro_RH_100;                                              // last value
    l_i2cI2c4_hygro_S_T        = s_i2cI2c4Hygro_S_T;                                                 // fetch sensor value
    l_i2cI2c4_hygro_S_RH       = s_i2cI2c4Hygro_S_RH;                                                // fetch sensor value
    l_i2cI2c4_hygro_T_cor_100  = s_i2cI2c4Hygro_T_cor_100;                                           // offset correction value
    l_i2cI2c4_hygro_RH_cor_100 = s_i2cI2c4Hygro_RH_cor_100;                                          // offset correction value
  }

  /* Calculate and present Temp value when a different measurement has arrived */
  int16_t temp_100 = l_i2cI2c4_hygro_T_100;
  if (sf_i2cI2c4_hygro_S_T != l_i2cI2c4_hygro_S_T) {
    temp_100  = (int16_t) ((((int32_t)l_i2cI2c4_hygro_S_T  * 17500) / 0xFFFF) - 4500);
    temp_100 += l_i2cI2c4_hygro_T_cor_100;

    /* Setting the global value */
    s_i2cI2c4Hygro_T_100 = temp_100;

    hasChanged = 1U;
    sf_i2cI2c4_hygro_S_T = l_i2cI2c4_hygro_S_T;
  }

  /* Calculate and present Hygro value when a different measurement has arrived */
  int16_t rh_100 = l_i2cI2c4_hygro_RH_100;
  if (sf_i2cI2c4_hygro_S_RH != l_i2cI2c4_hygro_S_RH) {
    rh_100  = (int16_t)( ((int32_t)l_i2cI2c4_hygro_S_RH * 10000) / 0xFFFF);
    rh_100 += l_i2cI2c4_hygro_RH_cor_100;

    /* Setting the global value */
    s_i2cI2c4Hygro_RH_100 = rh_100;

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
    s_i2cI2c4Hygro_DP_100 = (int16_t) tau_100;
  }
}

static void i2cI2c4HygroDistributor(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "HYGRO:\t Temp=%+02d.%02uC, RH=%02u.%02u%%, Tau=%+02d.%02uC\r\n",
      (s_i2cI2c4Hygro_T_100  / 100), (s_i2cI2c4Hygro_T_100  % 100),
      (s_i2cI2c4Hygro_RH_100 / 100), (s_i2cI2c4Hygro_RH_100 % 100),
      (s_i2cI2c4Hygro_DP_100 / 100), (s_i2cI2c4Hygro_DP_100 % 100));
  usbLogLen(dbgBuf, dbgLen);
}


static void i2cI2c4BaroFetch(void)
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
      s_i2cI2c4Baro_d1 = ((uint32_t)aRxBuffer[0] << 16) | ((uint32_t)aRxBuffer[1] << 8) | aRxBuffer[2];
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
      s_i2cI2c4Baro_d2 = ((uint32_t)aRxBuffer[0] << 16) | ((uint32_t)aRxBuffer[1] << 8) | aRxBuffer[2];
    }
  }
}

static void i2cI2c4BaroCalc(void)
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
  l_i2cI2c4Baro_d1            = s_i2cI2c4Baro_d1;
  l_i2cI2c4Baro_d2            = s_i2cI2c4Baro_d2;
  l_i2cI2c4Baro_temp_cor_100  = s_i2cI2c4Baro_temp_cor_100;
  l_i2cI2c4Baro_p_cor_100     = s_i2cI2c4Baro_p_cor_100;

  /* Calculate and present Baro and Temp values when a different measurement has arrived */
  if ((l_i2cI2c4Baro_d1 != sf_i2cI2c4Baro_d1) || (l_i2cI2c4Baro_d2 != sf_i2cI2c4Baro_d2)) {
    int32_t dT        = (int32_t)l_i2cI2c4Baro_d2 - ((int32_t)s_i2cI2c4Baro_c[5] << 8);
    int32_t temp_p20  = (int32_t)(((int64_t)dT * s_i2cI2c4Baro_c[6]) >> 23);
    int32_t l_t_100   = temp_p20 + 2000L;
    int64_t off       = ((int64_t)s_i2cI2c4Baro_c[2] << 17) + (((int64_t)s_i2cI2c4Baro_c[4] * dT) >> 6);
    int64_t sens      = ((int64_t)s_i2cI2c4Baro_c[1] << 16) + (((int64_t)s_i2cI2c4Baro_c[3] * dT) >> 7);

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
      s_i2cI2c4Baro_temp_100  = l_t_100;
      s_i2cI2c4Baro_p_100     = l_p_100;

      int16_t l_qnh_height_m          = 103; /* s_qnh_height_m; */
      int32_t l_i2cI2c4Baro_temp_100  = l_t_100;

      /* A valid height value (3D navigation) seems to be access able */
      if (sf_p != l_p_100) {
        float a_m_h   = 0.0065f * l_qnh_height_m;
        float Th0     = C_0DEGC_K + (l_i2cI2c4Baro_temp_100 / 100.f);
        float term    = 1.f + (a_m_h / Th0);
        l_p_h         = l_p_100 * pow(term, 5.255f);

        /* Setting the global values */
        s_i2cI2c4Baro_qnh_p_h_100 = l_p_h;

        sf_p = l_p_100;
      }
    }
  }
}

static void i2cI2c4BaroDistributor(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "BARO:\t Temp=%+02ld.%02luC, QFE=%04lu.%02luhPa, QNH=%04lu.%02luhPa\r\n",
      (s_i2cI2c4Baro_temp_100    / 100), (s_i2cI2c4Baro_temp_100    % 100),
      (s_i2cI2c4Baro_p_100       / 100), (s_i2cI2c4Baro_p_100       % 100),
      (s_i2cI2c4Baro_qnh_p_h_100 / 100), (s_i2cI2c4Baro_qnh_p_h_100 % 100));
  usbLogLen(dbgBuf, dbgLen);
}


static void i2cI2c4BaroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  usbLog("< i2cI2c4BaroInit -\r\n");

  /* MS560702BA03-50 Baro: RESET all internal data paths */
  {
    const uint8_t i2cWriteLongAry[0] = {
    };

    uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, I2C_SLAVE_BARO_REG_RESET, sizeof(i2cWriteLongAry), i2cWriteLongAry);
    if (i2cErr == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4BaroInit: ERROR Baro does not respond\r\n");
      return;
    }
    osDelay(2);
  }

  /* MS560702BA03-50 Baro: get version information */
  {
    uint8_t regQry[1] = { I2C_SLAVE_BARO_REG_VERSION };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_i2cI2c4BaroVersion = (((uint16_t)aRxBuffer[0] << 8) | aRxBuffer[1]) >> 4;

      dbgLen = sprintf(dbgBuf, ". i2cI2c4BaroInit: MS560702BA03-50 version: %d\r\n", s_i2cI2c4BaroVersion);
      usbLogLen(dbgBuf, dbgLen);
    }
  }

  /* MS560702BA03-50 Baro: get correction data from the PROM */
  for (uint8_t adr = 1; adr < C_I2C_BARO_C_CNT; ++adr) {
    uint8_t regQry[1] = { (I2C_SLAVE_BARO_REG_PROM | (adr << 1)) };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_i2cI2c4BaroVersion = (((uint16_t)aRxBuffer[0] << 8) | aRxBuffer[1]) >> 4;
    }

    s_i2cI2c4Baro_c[adr] = ((uint16_t)aRxBuffer[0] << 8) | aRxBuffer[1];

    dbgLen = sprintf(dbgBuf, ". i2cI2c4BaroInit: PROM adr=%d value=0x%04X\r\n", adr, s_i2cI2c4Baro_c[adr]);
    usbLogLen(dbgBuf, dbgLen);
  }

  if (s_i2cI2c4BaroVersion) {
    s_i2cI2c4BaroValid = 1U;
  }

  usbLog("- i2cI2c4BaroInit >\r\n\r\n");
}

static void i2cI2c4GyroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  // TODO: refactor to use i2cSequenceWrite()

  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< i2cI2c4GyroInit -\r\n");

  do {
    /* MPU-9250 6 axis: RESET */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_PWR_MGMT_1;
    aTxBuffer[1] = I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__HRESET | I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__CLKSEL_VAL;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4GyroInit: ERROR 6D-Gyro chip does not respond\r\n");
      break;
    }
    osDelay(10);

    /* MPU-9250 6 axis: read Who Am I control value */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_WHOAMI;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(aRxBuffer, 0, sizeof(aRxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aRxBuffer, min(1U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro1Version = aRxBuffer[0];
    dbgLen = sprintf(dbgBuf, ". i2cI2c4GyroInit: 6D-Gyro version=%d\r\n", s_i2cI2c4Gyro1Version);
    usbLogLen(dbgBuf, dbgLen);

    /* MPU-9250 6 axis: I2C bypass on to access the Magnetometer chip */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_INT_PIN_CFG;
    aTxBuffer[1] = I2C_SLAVE_GYRO_DTA_1_INT_PIN_CFG__BYPASS_EN;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }

    /* Magnetometer: soft reset */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL2;
    aTxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL2__SRST;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4GyroInit: ERROR 3D-Mag chip does not respond\r\n");
      break;
    }
    osDelay(10);
    usbLog(". i2cI2c4GyroInit: state 04\r\n");

    /* Magnetometer: read Device ID */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_2_WIA;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(aRxBuffer, 0, sizeof(aRxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aRxBuffer, min(1U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro2Version = aRxBuffer[0];
    dbgLen = sprintf(dbgBuf, ". i2cI2c4GyroInit: 3D-Mag version=%d\r\n", s_i2cI2c4Gyro2Version);
    usbLogLen(dbgBuf, dbgLen);

    /* Magnetometer: 16 bit access and prepare for PROM access */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL1;
    aTxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_PROM_VAL;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 06\r\n");

    /* Magnetometer: read correction data for X, Y and Z */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_2_ASAX;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(aRxBuffer, 0, sizeof(aRxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aRxBuffer, min(3U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro2AsaX = aRxBuffer[0];
    s_i2cI2c4Gyro2AsaY = aRxBuffer[1];
    s_i2cI2c4Gyro2AsaZ = aRxBuffer[2];
    usbLog(". i2cI2c4GyroInit: state 07\r\n");
    dbgLen = sprintf(dbgBuf, ". i2cI2c4GyroInit: Mag asaX=%d, asaY=%d, asaZ=%d\r\n", s_i2cI2c4Gyro2AsaX, s_i2cI2c4Gyro2AsaY, s_i2cI2c4Gyro2AsaZ);
    usbLogLen(dbgBuf, dbgLen);

    /* Magnetometer: mode change via power-down mode */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL1;
    aTxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_POWER_DOWN;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    osDelay(10);
    usbLog(". i2cI2c4GyroInit: state 08\r\n");

    /* Magnetometer: mode change for 16bit and run all axis at 8 Hz */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL1;
    aTxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_RUN_8HZ_VAL;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 09\r\n");

#if 0
    /* MPU-9250 6 axis: GYRO set offset values */
    sc = i2cI2c4_gyro_gyro_offset_set();
    if (sc != STATUS_OK) {
      break;
    }

    /* MPU-9250 6 axis: ACCEL set offset values */
    sc = i2cI2c4_gyro_accel_offset_set();
    if (sc != STATUS_OK) {
      break;
    }
#endif

    /* MPU-9250 6 axis: FIFO frequency = 10 Hz */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_SMPLRT_DIV;
    aTxBuffer[1] = 99;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 10\r\n");

    /* MPU-9250 6 axis: GYRO Bandwidth = 5 Hz, Fs = 1 kHz */
    aTxBuffer[0] = I2C_SLAVE_GYRO_ADDR_1;
    aTxBuffer[1] = 6;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 11\r\n");

    /* MPU-9250 6 axis: ACCEL Bandwidth = 5 Hz, Fs = 1 kHz */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_ACCEL_CONFIG2;
    aTxBuffer[1] = 6;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 12\r\n");

    /* MPU-9250 6 axis: Wake On Motion interrupt = 0.1 g (1 LSB = 4 mg) */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_WOM_THR;
    aTxBuffer[1] = 25;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 13\r\n");

    /* MPU-9250 6 axis: RESET all internal data paths */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_USER_CTRL;
    aTxBuffer[1] = I2C_SLAVE_GYRO_DTA_1_USER_CTRL__SIG_COND_RST;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    osDelay(10);
    usbLog(". i2cI2c4GyroInit: state 14\r\n");

    if (s_i2cI2c4Gyro1Version && s_i2cI2c4Gyro2Version) {
      s_i2cI2c4GyroValid = 1U;
      usbLog(". i2cI2c4GyroInit: state 15\r\n");
    }
  } while(0);

  usbLog("- i2cI2c4GyroInit >\r\n\r\n");

  osSemaphoreRelease(i2c4MutexHandle);
}

static void i2cI2c4LcdInit(void)
{
  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< i2cI2c4LcdInit -\r\n");

  do {
    /* Signal Reset */
    HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, (GPIO_PIN_RESET));
    osDelay(1);
    HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, (GPIO_PIN_SET));
    osDelay(50);

#ifdef HISTORIC
    /* LCD NHD-C0220BiZ-FS(RGB)-FBW-3VM */

    /* Reset */
    aTxBuffer[0] = 0x80;                                                                              // Co
    aTxBuffer[1] = 0x38;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4LcdInit: ERROR LCD does not respond\r\n");
      break;
    }
    osDelay(10);

    aTxBuffer[0] = 0x00;                                                                              // Co
    aTxBuffer[1] = 0x39;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) aTxBuffer, min(2U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    osDelay(10);

    aTxBuffer[0] = 0x80;                                                                              // Co
    aTxBuffer[1] = 0x14;
    aTxBuffer[2] = 0x78;
    aTxBuffer[3] = 0x5E;
    aTxBuffer[4] = 0x6D;
    aTxBuffer[5] = 0x0C;
    aTxBuffer[6] = 0x01;
    aTxBuffer[7] = 0x06;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) aTxBuffer, min(8U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }

    aTxBuffer[ 0] = 0x40;                                                                              // Data
    aTxBuffer[ 1] = '*';
    aTxBuffer[ 2] = ' ';
    aTxBuffer[ 3] = 'H';
    aTxBuffer[ 4] = 'F';
    aTxBuffer[ 5] = 'T';
    aTxBuffer[ 6] = '-';
    aTxBuffer[ 7] = 'C';
    aTxBuffer[ 8] = 'o';
    aTxBuffer[ 9] = 'r';
    aTxBuffer[10] = 'e';
    aTxBuffer[11] = '-';
    aTxBuffer[12] = 'M';
    aTxBuffer[13] = 'o';
    aTxBuffer[14] = 'd';
    aTxBuffer[15] = 'u';
    aTxBuffer[16] = 'l';
    aTxBuffer[17] = 'e';
    aTxBuffer[18] = ' ';
    aTxBuffer[19] = '*';
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_LCD_ADDR << 1U), (uint8_t*) aTxBuffer, min(20U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
#endif
  } while(0);

  usbLog("- i2cI2c4LcdInit >\r\n\r\n");

  osSemaphoreRelease(i2c4MutexHandle);
}

void i2cI2c4Tcxo20MhzDacInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  usbLog("< i2cI2c4Tcxo20MhzDacInit -\r\n");

  {
    const uint8_t i2cWriteLongAry[2] = {
      0x00, 0x20                                                                                      // 0x0020: clear value Mid, nAUX disable, DAC enabled
    };

    uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_USER_CONFIG, sizeof(i2cWriteLongAry), i2cWriteLongAry);
    if (i2cErr == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4Tcxo20MhzDacInit: ERROR DAC does not respond\r\n");
      return;
    }
  }

  {
    const uint8_t i2cWriteLongAry[2] = {
      0x00, 0x00                                                                                      // DC
    };

    i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_SW_CLEAR, sizeof(i2cWriteLongAry), i2cWriteLongAry);
  }

  /* Read ID and status */
  {
    uint8_t regQry[1] = { I2C_SLAVE_20MHZ_DAC_REG_RD_STATUS };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_i2cI2c4Tcxo20MhzDacStatus = ((uint16_t)aRxBuffer[0] << 8U) | aRxBuffer[1];

      dbgLen = sprintf(dbgBuf, ". i2cI2c4Tcxo20MhzDacInit: Status=0x%04X\r\n", s_i2cI2c4Tcxo20MhzDacStatus);
      usbLogLen(dbgBuf, dbgLen);
    }
  }

  usbLog("- i2cI2c4Tcxo20MhzDacInit >\r\n\r\n");
}

void i2cI2c4Tcxo20MhzDacSet(uint16_t dac)
{
  const uint8_t dacHi = (uint8_t) (dac >> 8U);
  const uint8_t dacLo = (uint8_t) (dac & 0xFFU);
  const uint8_t i2cWriteLongAry[2] = {
    dacHi, dacLo
  };

  i2cSequenceWriteLong(&hi2c4, i2c4MutexHandle, I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR, I2C_SLAVE_20MHZ_DAC_REG_WR_CODELOAD, sizeof(i2cWriteLongAry), i2cWriteLongAry);
}


void i2cI2c4Si5338Init(I2C_SI5338_CLKIN_VARIANT_t variant)
{
  uint8_t waitMask = 0U;

  usbLog("< i2cI2c4Si5338Init -\r\n");

  /* Figure 9 of DS: reg230[4] = 1, reg241[7] = 1 */
  {
	  const Reg_Data_t rdAry[2] = {
			  { 230, 0x10, 0x10 },
			  { 241, 0x80, 0x80 }
	  };

	  uint32_t i2cErr = i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
	  if (i2cErr == HAL_I2C_ERROR_AF) {
		/* Chip not responding */
		usbLog(". i2cI2c4Si5338Init: ERROR chip does not respond\r\n");
	  }
  }

  switch (variant) {
  case I2C_SI5338_CLKIN_VARIANT__MCU_MCO_8MHZ:
    {
      /* Register map for MCU MCO 8 MHz */
      i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_MCU_8MHZ_NUM_REGS_MAX, si5338_Reg_Store_MCU_8MHz);

      waitMask = 0x04;
    }
  break;

  case I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ:
    {
      /* Turn on TCXO */
      PowerSwitchDo(POWERSWITCH__3V3_XO, 1);

      /* Init the TCXO DAC when starting the TCXO task */
      osThreadResume(tcxo20MhzTaskHandle);
      osDelay(500);

      /* Register map for TCXO source */
      i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, SI5338_TCXO_NUM_REGS_MAX, si5338_Reg_Store_TCXO);

      waitMask = 0x08;
    }
    break;

  default:
    return;
  }

  /* Wait for input getting stable */
  while (1) {
    uint8_t regQry[1] = { 218 };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
        /* Leave when ready */
        if (!(aRxBuffer[0] & waitMask)) {
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

	  i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }

  osDelay(25);

  /* Figure 9 of DS: reg241[7] = 0, reg241 = 0x65 */
  {
	  const Reg_Data_t rdAry[2] = {
			  { 241, 0x00, 0x80 },
			  { 241, 0x65, 0xFF }
	  };

	  i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }

  /* Wait for PLL getting stable */
  while (1) {
    uint8_t regQry[1] = { 218 };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
        /* Leave when ready */
        if (!(aRxBuffer[0] & 0x11)) {
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
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = aRxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[1] = {
            {  47, cVal, 0x03 }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 1, rdAry);
      }
    }

    {
      uint8_t regQry[1] = { 236 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = aRxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[1] = {
            {  46, cVal, 0xFF }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 1, rdAry);
      }
    }

    {
      uint8_t regQry[1] = { 235 };
      uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, sizeof(regQry), regQry, 1);
      if (i2cErr == HAL_I2C_ERROR_NONE) {
        val = aRxBuffer[0];
      }
      {
        const uint8_t cVal = val;
        const Reg_Data_t rdAry[2] = {
            {  45, cVal, 0xFF },
            {  47, 0x14, 0xFC }
        };
        i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
      }
    }
  }

  /* Figure 9 of DS: reg49[7] = 1, reg230[4] = 0 */
  {
	  const Reg_Data_t rdAry[2] = {
			  {  49, 0x80, 0x80 },
			  { 230, 0x00, 0x10 }
	  };
	  i2cSequenceWriteMask(&hi2c4, i2c4MutexHandle, I2C_SLAVE_SI5338_ADDR, 2, rdAry);
  }

  usbLog("- i2cI2c4Si5338Init >\r\n\r\n");
}


/* Tasks */

void i2cI2c4LcdTaskInit(void)
{
  osDelay(500UL);
  i2cI2c4LcdInit();
}

void i2cI2c4LcdTaskLoop(void)
{
  const uint32_t  eachMs              = 250UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 500UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);
}


void i2cI2c4HygroTaskInit(void)
{
  osDelay(550UL);
  i2cI2c4HygroInit();
}

void i2cI2c4HygroTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 550UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);

  i2cI2c4HygroFetch();
  i2cI2c4HygroCalc();
  i2cI2c4HygroDistributor();
}


void i2cI2c4BaroTaskInit(void)
{
  osDelay(600UL);
  i2cI2c4BaroInit();
}

void i2cI2c4BaroTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 600UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);
#if 0
  {
    char dbgBuf[128];
    int  dbgLen;

    uint32_t now = osKernelSysTick();
    dbgLen = sprintf(dbgBuf, "BARO: prev=%010lu is=%010lu\r\n", sf_previousWakeTime, now);
    usbLogLen(dbgBuf, dbgLen);
  }
#endif

  i2cI2c4BaroFetch();
  i2cI2c4BaroCalc();
  i2cI2c4BaroDistributor();
}


void i2cI2c4GyroTaskInit(void)
{
  osDelay(650UL);
  i2cI2c4GyroInit();
}

void i2cI2c4GyroTaskLoop(void)
{
  const uint32_t  eachMs              = 1000UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 650UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);
}
