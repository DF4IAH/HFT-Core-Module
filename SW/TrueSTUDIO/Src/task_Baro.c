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
#include "task_Controller.h"

#include "task_Baro.h"


#define C_0DEGC_K                                             (273.15f)


extern osTimerId            baroTimerHandle;
extern osSemaphoreId        c2Baro_BSemHandle;
extern osSemaphoreId        i2c4_BSemHandle;
extern I2C_HandleTypeDef    hi2c4;

extern EventGroupHandle_t   globalEventGroupHandle;

extern uint8_t              i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t              i2c4RxBuffer[I2C_RXBUFSIZE];


/* Correction offset values */
static int16_t              s_qnh_height_m;
static int16_t              s_baro_temp_cor_100;
static int16_t              s_baro_p_cor_100;

static uint8_t              s_baro_enable;
static uint32_t             s_baroStartTime;
static uint16_t             s_baroVersion;
static uint16_t             s_baro_c[C_I2C_BARO_C_CNT];
static uint32_t             s_baro_d1;
static uint32_t             s_baro_d2;

/* Out values */
static int32_t              s_baro_temp_100;
static int32_t              s_baro_p_100;
static int32_t              s_baro_qnh_p_h_100;


int32_t baroGetValue(BARO_GET_TYPE_t type)
{
  if (s_baro_enable) {
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
    i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_BARO_ADDR, I2C_SLAVE_BARO_REG_CONV_D1_4096, sizeof(dataAry), dataAry);
  }
  osDelay(20);

  /* Get D1 data */
  {
    uint8_t regQry[1] = { I2C_SLAVE_BARO_REG_ADC_READ };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 3);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_baro_d1 = ((uint32_t)i2c4RxBuffer[0] << 16) | ((uint32_t)i2c4RxBuffer[1] << 8) | i2c4RxBuffer[2];
    }
  }

  /* Request D2 */
  {
    uint8_t dataAry[0] = { };
    i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_BARO_ADDR, I2C_SLAVE_BARO_REG_CONV_D2_4096, sizeof(dataAry), dataAry);
  }
  osDelay(20);

  /* Get D2 data */
  {
    uint8_t regQry[1] = { I2C_SLAVE_BARO_REG_ADC_READ };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 3);
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

#if 1
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
#endif


void baroDoMeasure(void)
{
  baroFetch();
  baroCalc();
}


/* Timer functions */

static void baroCyclicTimerStart(uint32_t period_ms)
{
  osTimerStart(baroTimerHandle, period_ms);
}

static void baroCyclicTimerStop(void)
{
  osTimerStop(baroTimerHandle);
}

void baroTimerCallback(void const *argument)
{
  /* Context of RTOS Daemon Task */
  uint32_t msgAry[1];

  /* Write cyclic timer message to this destination */
  uint8_t msgLen    = 0U;
  msgAry[msgLen++]  = controllerCalcMsgHdr(Destinations__Sensor_Baro, Destinations__Sensor_Baro, 0U, MsgBaro__CallFunc02_CyclicTimerEvent);
  controllerMsgPushToOutQueue(msgLen, msgAry, 1UL);
}

static void baroCyclicTimerEvent(void)
{
  baroDoMeasure();
  baroDistributor();
}


static void baroInit(void)
{
  int   dbgLen = 0;
  char  dbgBuf[128];

  usbLog("< BaroInit -\r\n");

  /* MS560702BA03-50 Baro: RESET all internal data paths */
  {
    const uint8_t i2cWriteLongAry[0] = {
    };

    uint32_t i2cErr = i2cSequenceWriteLong(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_BARO_ADDR, I2C_SLAVE_BARO_REG_RESET, sizeof(i2cWriteLongAry), i2cWriteLongAry);
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
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_baroVersion = (((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1]) >> 4;

      dbgLen = sprintf(dbgBuf, ". BaroInit: MS560702BA03-50 version: %d\r\n", s_baroVersion);
      usbLogLen(dbgBuf, dbgLen);
    }
  }

  /* MS560702BA03-50 Baro: get correction data from the PROM */
  for (uint8_t adr = 1; adr < C_I2C_BARO_C_CNT; ++adr) {
    uint8_t regQry[1] = { (I2C_SLAVE_BARO_REG_PROM | (adr << 1)) };
    uint32_t i2cErr = i2cSequenceRead(&hi2c4, i2c4_BSemHandle, I2C_SLAVE_BARO_ADDR, sizeof(regQry), regQry, 2);
    if (i2cErr == HAL_I2C_ERROR_NONE) {
      s_baroVersion = (((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1]) >> 4;
    }

    s_baro_c[adr] = ((uint16_t)i2c4RxBuffer[0] << 8) | i2c4RxBuffer[1];

    dbgLen = sprintf(dbgBuf, ". BaroInit: PROM adr=%d value=0x%04X\r\n", adr, s_baro_c[adr]);
    usbLogLen(dbgBuf, dbgLen);
  }

  if (s_baroVersion) {
    s_baro_enable = 1U;
  }

  usbLog("- BaroInit >\r\n\r\n");
}


static void baroMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const baroMsgBaroCmds_t cmd     = (baroMsgBaroCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgBaro__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_baroStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      baroInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Sensor_Baro, 0U, MsgBaro__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  case MsgBaro__CallFunc01_DoMeasure:
    {
      /* Get the values */
      baroDoMeasure();

      /* Send them to the controller */
      {
        uint32_t cmdBack[4];

        cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Sensor_Baro, sizeof(cmdBack) - 4U, MsgBaro__CallFunc01_DoMeasure);
        cmdBack[1] = s_baro_temp_100;
        cmdBack[2] = s_baro_p_100;
        cmdBack[3] = s_baro_qnh_p_h_100;

        controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
      }
    }
    break;

  case MsgBaro__CallFunc02_CyclicTimerEvent:
    {
      baroCyclicTimerEvent();
    }
    break;

  case MsgBaro__CallFunc03_CyclicTimerStart:
    {
      /* Start cyclic measurements with that period in ms */
      baroCyclicTimerStart(msgAry[msgIdx++]);
    }
    break;

  case MsgBaro__CallFunc04_CyclicTimerStop:
    {
      /* Stop cyclic measurements */
      baroCyclicTimerStop();
    }
    break;

  default: { }
  }  // switch (cmd)

#if 0
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
#endif
}


/* Task */

void baroTaskInit(void)
{
  /* Correction offset values */
  s_qnh_height_m                    = 103;
  s_baro_temp_cor_100               = 0;
  s_baro_p_cor_100                  = 170;

  s_baro_enable                     = 0U;
  s_baroVersion                     = 0U;
  memset(s_baro_c, 0, C_I2C_BARO_C_CNT);
  s_baro_d1                         = 0UL;
  s_baro_d2                         = 0UL;

  /* Out values */
  s_baro_temp_100                   = 0L;
  s_baro_p_100                      = 0L;
  s_baro_qnh_p_h_100                = 0L;

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_baroStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);
}

void baroTaskLoop(void)
{
  uint32_t  msgLen                        = 0UL;
  uint32_t  msgAry[CONTROLLER_MSG_Q_LEN];

  /* Wait for door bell and hand-over controller out queue */
  {
    osSemaphoreWait(c2Baro_BSemHandle, osWaitForever);
    msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Sensor_Baro, 1UL);                   // Special case of callbacks need to limit blocking time
    osSemaphoreRelease(c2Baro_BSemHandle);
    osDelay(3UL);
  }

  /* Decode and execute the commands when a message exists
   * (in case of callbacks the loop catches its wakeup semaphore
   * before ctrlQout is released results to request on an empty queue) */
  if (msgLen) {
    baroMsgProcess(msgLen, msgAry);
  }
}
