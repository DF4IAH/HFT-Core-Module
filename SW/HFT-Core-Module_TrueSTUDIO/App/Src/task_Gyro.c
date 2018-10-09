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
#include "task_Controller.h"

#include "task_Gyro.h"


extern osTimerId            gyroTimerHandle;
extern osSemaphoreId        c2Gyro_BSemHandle;
extern osSemaphoreId        i2c4_BSemHandle;
extern I2C_HandleTypeDef    hi2c4;

extern EventGroupHandle_t   globalEventGroupHandle;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];

static uint8_t              s_gyro_enable;
static uint32_t             s_gyroStartTime;
static uint8_t              s_gyro1Version;
static uint8_t              s_gyro2Version;
static uint32_t             s_gyro2AsaX;
static uint32_t             s_gyro2AsaY;
static uint32_t             s_gyro2AsaZ;


int32_t gyroGetValue(BARO_GET_TYPE_t type)
{
  if (s_gyro_enable) {
    switch (type) {
//  case GYRO_GET_TYPE__xxx:
//    return s_baro_temp_100;
//    break;

    default:
      { }
    }
  }

  return 0L;
}

static void gyroFetch(void)
{
}

static void gyroCalc(void)
{
}

static void gyroDistributor(void)
{
}


static void gyroDoMeasure(void)
{
  gyroFetch();
  gyroCalc();
}


/* Timer functions */

static void gyroCyclicStart(uint32_t period_ms)
{
  osTimerStart(gyroTimerHandle, period_ms);
}

static void gyroCyclicStop(void)
{
  osTimerStop(gyroTimerHandle);
}

void gyroTimerCallback(void const *argument)
{
  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Sensor_Gyro, Destinations__Sensor_Gyro, 0U, MsgGyro__CallFunc02_CyclicTimerEvent);
  controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
}

static void gyroCyclicTimerEvent(void)
{
  gyroDoMeasure();
  gyroDistributor();
}


static void gyroInit(void)
{
  //int   dbgLen = 0;
  //char  dbgBuf[128];

  // TODO: refactor to use i2cSequenceWrite()

  usbLog("< GyroInit -\r\n");

#if 0
  do {
    /* I2C4 init */
    i2cx_Init(&hi2c4, i2c4_BSemHandle);

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
      usbLog(". GyroInit: ERROR 6D-Gyro chip does not respond\r\n");
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
    s_gyro1Version = i2c4RxBuffer[0];
    dbgLen = sprintf(dbgBuf, ". GyroInit: 6D-Gyro version=%d\r\n", s_gyro1Version);
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
      usbLog(". GyroInit: ERROR 3D-Mag chip does not respond\r\n");
      break;
    }
    osDelay(10);
    usbLog(". GyroInit: state 04\r\n");

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
    s_gyro2Version = i2c4RxBuffer[0];
    dbgLen = sprintf(dbgBuf, ". GyroInit: 3D-Mag version=%d\r\n", s_gyro2Version);
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
    usbLog(". GyroInit: state 06\r\n");

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
    s_gyro2AsaX = i2c4RxBuffer[0];
    s_gyro2AsaY = i2c4RxBuffer[1];
    s_gyro2AsaZ = i2c4RxBuffer[2];
    usbLog(". GyroInit: state 07\r\n");
    dbgLen = sprintf(dbgBuf, ". GyroInit: Mag asaX=%d, asaY=%d, asaZ=%d\r\n", s_gyro2AsaX, s_gyro2AsaY, s_gyro2AsaZ);
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
    usbLog(". GyroInit: state 08\r\n");

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
    usbLog(". GyroInit: state 09\r\n");

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
    usbLog(". GyroInit: state 10\r\n");

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
    usbLog(". GyroInit: state 11\r\n");

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
    usbLog(". GyroInit: state 12\r\n");

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
    usbLog(". GyroInit: state 13\r\n");

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
    usbLog(". GyroInit: state 14\r\n");

    if (s_gyro1Version && s_gyro2Version) {
      s_gyro_enable = 1U;
      usbLog(". GyroInit: state 15\r\n");
    }
  } while(0);
#endif

  usbLog("- GyroInit >\r\n\r\n");
}

static void gyroMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const gyroMsgGyroCmds_t cmd     = (gyroMsgGyroCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgGyro__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_gyroStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      gyroInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Sensor_Gyro, 0U, MsgGyro__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgGyro__CallFunc01_DoMeasure:
    {
      /* Get the values */
      gyroDoMeasure();

      /* Send them to the controller */
      {
        uint32_t cmdBack[4];

        cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Sensor_Gyro, sizeof(cmdBack) - 4U, MsgGyro__CallFunc01_DoMeasure);
        cmdBack[1] = s_gyro2AsaX;
        cmdBack[2] = s_gyro2AsaY;
        cmdBack[3] = s_gyro2AsaZ;

        controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
      }
    }
    break;

  case MsgGyro__CallFunc02_CyclicTimerEvent:
    {
      gyroCyclicTimerEvent();
    }
    break;

  case MsgGyro__CallFunc03_CyclicTimerStart:
    {
      /* Start cyclic measurements with that period in ms */
      gyroCyclicStart(msgAry[msgIdx++]);
    }
    break;

  case MsgGyro__CallFunc04_CyclicTimerStop:
    {
      /* Stop cyclic measurements */
      gyroCyclicStop();
    }
    break;


  default: { }
  }  // switch (cmd)
}


/* Task */

void gyroTaskInit(void)
{
  s_gyro_enable   = 0U;
  s_gyro1Version  = 0U;
  s_gyro2Version  = 0U;
  s_gyro2AsaX     = 0UL;
  s_gyro2AsaY     = 0UL;
  s_gyro2AsaZ     = 0UL;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_gyroStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void gyroTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2Gyro_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Sensor_Gyro, 1UL);                   // Special case of callbacks need to limit blocking time
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    gyroMsgProcess(msgLen, msgAry);
  }
}
