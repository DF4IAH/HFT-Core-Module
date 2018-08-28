/*
 * i2c.c
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "usb.h"

#include "i2c.h"


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

static uint8_t              s_i2cI2c4BaroValid                = 0U;
static uint16_t             s_i2cI2c4BaroVersion              = 0U;
static uint16_t             s_i2cI2c4Baro_c[C_I2C_BARO_C_CNT] = { 0U };

static uint8_t              s_i2cI2c4GyroValid                = 0U;
static uint8_t              s_i2cI2c4Gyro1Version             = 0U;
static uint8_t              s_i2cI2c4Gyro2Version             = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaX                = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaY                = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaZ                = 0U;


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

static void i2cI2c4BaroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< i2cI2c4BaroInit -\r\n");

  do {
    /* MS560702BA03-50 Baro: RESET all internal data paths */
    aTxBuffer[0] = I2C_SLAVE_BARO_REG_RESET;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_BARO_ADDR << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) {
      /* Chip not responding */
      usbLog(". i2cI2c4BaroInit: ERROR chip does not respond\r\n");
      break;
    }
    osDelay(2);

    /* MS560702BA03-50 Baro: get version information */
    aTxBuffer[0] = I2C_SLAVE_BARO_REG_VERSION;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_BARO_ADDR << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_BARO_ADDR << 1U), (uint8_t*) aRxBuffer, min(2U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4BaroVersion = (((uint16_t)aRxBuffer[0] << 8) | aRxBuffer[1]) >> 4;
    dbgLen = sprintf(dbgBuf, ". i2cI2c4BaroInit: MS560702BA03-50 version: %d\r\n", s_i2cI2c4BaroVersion);
    usbLogLen(dbgBuf, dbgLen);

    /* MS560702BA03-50 Baro: get correction data from the PROM */
    for (uint8_t adr = 1; adr < C_I2C_BARO_C_CNT; ++adr) {
      aTxBuffer[0] = I2C_SLAVE_BARO_REG_PROM | (adr << 1);
      if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_BARO_ADDR << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
        Error_Handler();
      }
      while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
        osDelay(1);
      }
      if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_BARO_ADDR << 1U), (uint8_t*) aRxBuffer, min(2U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
        Error_Handler();
      }
      while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
        osDelay(1);
      }
      s_i2cI2c4Baro_c[adr] = ((uint16_t)aRxBuffer[0] << 8) | aRxBuffer[1];

      dbgLen = sprintf(dbgBuf, ". i2cI2c4BaroInit: PROM adr=%d value=0x%04X\r\n", adr, s_i2cI2c4Baro_c[adr]);
      usbLogLen(dbgBuf, dbgLen);
    }

    if (s_i2cI2c4BaroVersion) {
      s_i2cI2c4BaroValid = 1U;
    }
  } while(0);

  usbLog("- i2cI2c4BaroInit >\r\n\r\n");

  osSemaphoreRelease(i2c4MutexHandle);
}

static void i2cI2c4GyroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

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
    usbLog(". i2cI2c4GyroInit: state 01\r\n");

    /* MPU-9250 6 axis: read Who Am I control value */
    aTxBuffer[0] = I2C_SLAVE_GYRO_REG_1_WHOAMI;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aTxBuffer, min(1U, TXBUFFERSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) aRxBuffer, min(1U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro1Version = aRxBuffer[0];
    usbLog(". i2cI2c4GyroInit: state 02\r\n");

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
    usbLog(". i2cI2c4GyroInit: state 03\r\n");

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
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) aRxBuffer, min(1U, RXBUFFERSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro2Version = aRxBuffer[0];
    usbLog(". i2cI2c4GyroInit: state 05\r\n");

    dbgLen = sprintf(dbgBuf, ". i2cI2c4GyroInit: 6D-Gyro version=%d, 3D-Mag version=%d\r\n", s_i2cI2c4Gyro1Version, s_i2cI2c4Gyro2Version);
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
    sc = twi1_gyro_gyro_offset_set();
    if (sc != STATUS_OK) {
      break;
    }

    /* MPU-9250 6 axis: ACCEL set offset values */
    sc = twi1_gyro_accel_offset_set();
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
  int   dbgLen = 0;
  char  dbgBuf[128];

  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< i2cI2c4LcdInit -\r\n");

  do {

  } while(0);

  usbLog("- i2cI2c4LcdInit >\r\n\r\n");

  osSemaphoreRelease(i2c4MutexHandle);
}


/* Tasks */

void i2cI2c4HygroTaskInit(void)
{
  osDelay(200);
  i2cI2c4HygroInit();
}

void i2cI2c4HygroTaskLoop(void)
{
  osDelay(100);
}


void i2cI2c4BaroTaskInit(void)
{
  osDelay(300);
  i2cI2c4BaroInit();
}

void i2cI2c4BaroTaskLoop(void)
{
  osDelay(100);
}


void i2cI2c4GyroTaskInit(void)
{
  osDelay(400);
  i2cI2c4GyroInit();
}

void i2cI2c4GyroTaskLoop(void)
{
  osDelay(100);
}


void i2cI2c4LcdTaskInit(void)
{
  osDelay(100);
  i2cI2c4LcdInit();
}

void i2cI2c4LcdTaskLoop(void)
{
  osDelay(100);
}
