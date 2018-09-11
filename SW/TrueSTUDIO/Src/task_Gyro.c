/*
 * task_Gyro.c
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

#include "task_Gyro.h"


extern osMutexId            i2c4MutexHandle;
extern I2C_HandleTypeDef    hi2c4;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

static uint8_t              s_i2cI2c4GyroValid                = 0U;
static uint8_t              s_i2cI2c4Gyro1Version             = 0U;
static uint8_t              s_i2cI2c4Gyro2Version             = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaX                = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaY                = 0U;
static uint8_t              s_i2cI2c4Gyro2AsaZ                = 0U;


static void i2cI2c4GyroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  // TODO: refactor to use i2cSequenceWrite()

  osSemaphoreWait(i2c4MutexHandle, osWaitForever);

  usbLog("< i2cI2c4GyroInit -\r\n");

  do {
    /* MPU-9250 6 axis: RESET */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_1_PWR_MGMT_1;
    i2c4TxBuffer[1] = I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__HRESET | I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__CLKSEL_VAL;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
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
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_1_WHOAMI;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(1U, I2C_TXBUFSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4RxBuffer, min(1U, I2C_RXBUFSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro1Version = i2c4RxBuffer[0];
    dbgLen = sprintf(dbgBuf, ". i2cI2c4GyroInit: 6D-Gyro version=%d\r\n", s_i2cI2c4Gyro1Version);
    usbLogLen(dbgBuf, dbgLen);

    /* MPU-9250 6 axis: I2C bypass on to access the Magnetometer chip */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_1_INT_PIN_CFG;
    i2c4TxBuffer[1] = I2C_SLAVE_GYRO_DTA_1_INT_PIN_CFG__BYPASS_EN;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }

    /* Magnetometer: soft reset */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL2;
    i2c4TxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL2__SRST;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
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
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_2_WIA;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4TxBuffer, min(1U, I2C_TXBUFSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4RxBuffer, min(1U, I2C_RXBUFSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro2Version = i2c4RxBuffer[0];
    dbgLen = sprintf(dbgBuf, ". i2cI2c4GyroInit: 3D-Mag version=%d\r\n", s_i2cI2c4Gyro2Version);
    usbLogLen(dbgBuf, dbgLen);

    /* Magnetometer: 16 bit access and prepare for PROM access */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL1;
    i2c4TxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_PROM_VAL;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 06\r\n");

    /* Magnetometer: read correction data for X, Y and Z */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_2_ASAX;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4TxBuffer, min(1U, I2C_TXBUFSIZE), I2C_LAST_FRAME_NO_STOP) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    memset(i2c4RxBuffer, 0, sizeof(i2c4RxBuffer));
    if (HAL_I2C_Master_Sequential_Receive_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4RxBuffer, min(3U, I2C_RXBUFSIZE), I2C_OTHER_FRAME) != HAL_OK) {
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    s_i2cI2c4Gyro2AsaX = i2c4RxBuffer[0];
    s_i2cI2c4Gyro2AsaY = i2c4RxBuffer[1];
    s_i2cI2c4Gyro2AsaZ = i2c4RxBuffer[2];
    usbLog(". i2cI2c4GyroInit: state 07\r\n");
    dbgLen = sprintf(dbgBuf, ". i2cI2c4GyroInit: Mag asaX=%d, asaY=%d, asaZ=%d\r\n", s_i2cI2c4Gyro2AsaX, s_i2cI2c4Gyro2AsaY, s_i2cI2c4Gyro2AsaZ);
    usbLogLen(dbgBuf, dbgLen);

    /* Magnetometer: mode change via power-down mode */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL1;
    i2c4TxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_POWER_DOWN;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    osDelay(10);
    usbLog(". i2cI2c4GyroInit: state 08\r\n");

    /* Magnetometer: mode change for 16bit and run all axis at 8 Hz */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_2_CNTL1;
    i2c4TxBuffer[1] = I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_RUN_8HZ_VAL;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_2 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
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
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_1_SMPLRT_DIV;
    i2c4TxBuffer[1] = 99;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 10\r\n");

    /* MPU-9250 6 axis: GYRO Bandwidth = 5 Hz, Fs = 1 kHz */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_ADDR_1;
    i2c4TxBuffer[1] = 6;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 11\r\n");

    /* MPU-9250 6 axis: ACCEL Bandwidth = 5 Hz, Fs = 1 kHz */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_1_ACCEL_CONFIG2;
    i2c4TxBuffer[1] = 6;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 12\r\n");

    /* MPU-9250 6 axis: Wake On Motion interrupt = 0.1 g (1 LSB = 4 mg) */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_1_WOM_THR;
    i2c4TxBuffer[1] = 25;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
      osDelay(1);
    }
    usbLog(". i2cI2c4GyroInit: state 13\r\n");

    /* MPU-9250 6 axis: RESET all internal data paths */
    i2c4TxBuffer[0] = I2C_SLAVE_GYRO_REG_1_USER_CTRL;
    i2c4TxBuffer[1] = I2C_SLAVE_GYRO_DTA_1_USER_CTRL__SIG_COND_RST;
    if (HAL_I2C_Master_Sequential_Transmit_IT(&hi2c4, (uint16_t) (I2C_SLAVE_GYRO_ADDR_1 << 1U), (uint8_t*) i2c4TxBuffer, min(2U, I2C_TXBUFSIZE), I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
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


/* Task */

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
