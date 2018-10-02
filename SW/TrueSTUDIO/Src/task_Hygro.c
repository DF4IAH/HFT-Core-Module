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
#include "task_Controller.h"

#include "task_Hygro.h"


extern osTimerId            hygroTimerHandle;
extern osSemaphoreId        c2Hygro_BSemHandle;
extern osSemaphoreId        i2c4_BSemHandle;
extern I2C_HandleTypeDef    hi2c4;

extern EventGroupHandle_t   globalEventGroupHandle;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];


/* Correction values */
static int16_t              s_hygro_T_cor_100;
static int16_t              s_hygro_RH_cor_100;

static uint8_t              s_hygro_enable;
static uint16_t             s_hygroState;
static uint32_t             s_hygroStartTime;
static uint16_t             s_hygro_S_T;
static int16_t              s_hygro_S_RH;

/* Out values */
static int16_t              s_hygro_T_100;
static int16_t              s_hygro_RH_100;
static int16_t              s_hygro_DP_100;


int16_t hygroGetValue(HYGRO_GET_TYPE_t type)
{
  if (s_hygro_enable) {
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


static void hygroFetch(void)
{
  /* Read current measurement data */
  {
    uint8_t regQry[2] = { I2C_SLAVE_HYGRO_REG_FETCH_DATA_HI, I2C_SLAVE_HYGRO_REG_FETCH_DATA_LO };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_HYGRO_ADDR, sizeof(regQry), regQry, 5);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_hygro_S_T  = ((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1];
      s_hygro_S_RH = ((uint16_t)i2c4RxBuffer[3] << 8) | i2c4RxBuffer[4];
    }
  }

  /* Start next measurement - available 15ms later */
  {
    uint8_t dataAry[1] = { I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_LO };
    i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_HYGRO_ADDR, I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_HI, sizeof(dataAry), dataAry);
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

#if 1
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
#endif


void hygroDoMeasure(void)
{
  hygroFetch();
  hygroCalc();
}


/* Timer functions */

static void hygroCyclicStart(uint32_t period_ms)
{
  osTimerStart(hygroTimerHandle, period_ms);
}

static void hygroCyclicStop(void)
{
  osTimerStop(hygroTimerHandle);
}

void hygroTimerCallback(void const *argument)
{
  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Sensor_Hygro, Destinations__Sensor_Hygro, 0U, MsgHygro__CallFunc02_CyclicTimerEvent);
  controllerMsgPushToOutQueue(msgLen, msgAry, osWaitForever);
}

static void hygroCyclicTimerEvent(void)
{
  hygroDoMeasure();
  hygroDistributor();
}


static void hygroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  osSemaphoreWait(i2c4_BSemHandle, osWaitForever);

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
      s_hygro_enable = 1U;
    }
  } while(0);

  usbLog("- HygroInit >\r\n\r\n");

  osSemaphoreRelease(i2c4_BSemHandle);
}


static void hygroMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                  msgIdx  = 0UL;
  const uint32_t            hdr     = msgAry[msgIdx++];
  const hygroMsgHygroCmds_t cmd     = (hygroMsgHygroCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgHygro__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_hygroStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      hygroInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Sensor_Hygro, 0U, MsgHygro__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgHygro__CallFunc01_DoMeasure:
    {
      /* Get the values */
      hygroDoMeasure();

      /* Send them to the controller */
      {
        uint32_t cmdBack[4];

        cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Sensor_Hygro, sizeof(cmdBack) - 4U, MsgHygro__CallFunc01_DoMeasure);
        cmdBack[1] = s_hygro_T_100;
        cmdBack[2] = s_hygro_RH_100;
        cmdBack[3] = s_hygro_DP_100;

        controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
      }
    }
    break;

  case MsgHygro__CallFunc02_CyclicTimerEvent:
    {
      hygroCyclicTimerEvent();
    }
    break;

  case MsgHygro__CallFunc03_CyclicTimerStart:
    {
      /* Start cyclic measurements with that period in ms */
      hygroCyclicStart(msgAry[msgIdx++]);
    }
    break;

  case MsgHygro__CallFunc04_CyclicTimerStop:
    {
      /* Stop cyclic measurements */
      hygroCyclicStop();
    }
    break;

  default: { }
  }  // switch (cmd)

#if 0
  if (s_hygro_enable) {
    hygroFetch();
    hygroCalc();

    hygroDistributor();
  }
#endif
}


/* Task */

void hygroTaskInit(void)
{
  /* Correction values */
  s_hygro_T_cor_100   = 0;
  s_hygro_RH_cor_100  = 700;

  s_hygro_enable      = 0U;
  s_hygroState        = 0U;
  s_hygro_S_T         = 0U;
  s_hygro_S_RH        = 0;

  /* Out values */
  s_hygro_T_100       = 0;
  s_hygro_RH_100      = 0;
  s_hygro_DP_100      = 0;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_hygroStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void hygroTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2Hygro_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Sensor_Hygro, 1UL);                  // Special case of callbacks need to limit blocking time
    osSemaphoreRelease(c2Hygro_BSemHandle);
    osDelay(3UL);
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    hygroMsgProcess(msgLen, msgAry);
  }
}
