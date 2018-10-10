/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
// Core, USB_DEVICE
#include "main.h"
#include "usb_device.h"

// App
#include "device_adc.h"
#include "task_Controller.h"
#include "task_USB.h"
#include "task_TCXO_20MHz.h"
#include "task_Si5338.h"
#include "task_LCD.h"
#include "task_Baro.h"
#include "task_Hygro.h"
#include "task_Gyro.h"
#include "task_Audio_ADC.h"
#include "task_Audio_DAC.h"
#include "task_AX5243.h"
#include "task_SX1276.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId usbToHostTaskHandle;
osThreadId usbFromHostTaskHandle;
osThreadId hygroTaskHandle;
osThreadId baroTaskHandle;
osThreadId gyroTaskHandle;
osThreadId lcdTaskHandle;
osThreadId tcxo20MhzTaskHandle;
osThreadId ax5243TaskHandle;
osThreadId sx1276TaskHandle;
osThreadId controllerTaskHandle;
osThreadId si5338TaskHandle;
osThreadId audioAdcTaskHandle;
osThreadId audioDacTaskHandle;
osMessageQId usbToHostQueueHandle;
osMessageQId usbFromHostQueueHandle;
osMessageQId controllerInQueueHandle;
osMessageQId controllerOutQueueHandle;
osMessageQId loraMacQueueHandle;
osTimerId baroTimerHandle;
osTimerId hygroTimerHandle;
osTimerId defaultTimerHandle;
osTimerId gyroTimerHandle;
osTimerId controllerTimerHandle;
osTimerId tcxoTimerHandle;
osTimerId audioAdcTimerHandle;
osTimerId audioDacTimerHandle;
osSemaphoreId c2Ax5243_BSemHandle;
osSemaphoreId c2Sx1276_BSemHandle;
osSemaphoreId c2Si5338_BSemHandle;
osSemaphoreId c2Tcxo_BSemHandle;
osSemaphoreId c2Baro_BSemHandle;
osSemaphoreId c2Gyro_BSemHandle;
osSemaphoreId c2Hygro_BSemHandle;
osSemaphoreId c2Lcd_BSemHandle;
osSemaphoreId c2Default_BSemHandle;
osSemaphoreId i2c1_BSemHandle;
osSemaphoreId i2c2_BSemHandle;
osSemaphoreId i2c3_BSemHandle;
osSemaphoreId i2c4_BSemHandle;
osSemaphoreId spi1_BSemHandle;
osSemaphoreId spi3_BSemHandle;
osSemaphoreId cQin_BSemHandle;
osSemaphoreId cQout_BSemHandle;
osSemaphoreId c2AudioAdc_BSemHandle;
osSemaphoreId c2AudioDac_BSemHandle;
osSemaphoreId c2usbToHost_BSemHandle;
osSemaphoreId c2usbFromHost_BSemHandle;
osSemaphoreId usb_BSemHandle;

/* USER CODE BEGIN Variables */
EventGroupHandle_t                    adcEventGroupHandle;
EventGroupHandle_t                    extiEventGroupHandle;
EventGroupHandle_t                    globalEventGroupHandle;
EventGroupHandle_t                    spiEventGroupHandle;
EventGroupHandle_t                    usbToHostEventGroupHandle;


extern ENABLE_MASK_t                  g_enableMsk;
extern MON_MASK_t                     g_monMsk;

static uint8_t                        s_rtos_DefaultTask_adc_enable;
static uint32_t                       s_rtos_DefaultTaskStartTime;


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartUsbToHostTask(void const * argument);
void StartUsbFromHostTask(void const * argument);
void StartHygroTask(void const * argument);
void StartBaroTask(void const * argument);
void StartGyroTask(void const * argument);
void StartLcdTask(void const * argument);
void StartTcxo20MhzTask(void const * argument);
void StartAx5243Task(void const * argument);
void StartSx1276Task(void const * argument);
void StartControllerTask(void const * argument);
void StartSi5338Task(void const * argument);
void StartAudioAdcTask(void const * argument);
void StartAudioDacTask(void const * argument);
void rtosBaroTimerCallback(void const * argument);
void rtosHygroTimerCallback(void const * argument);
void rtosDefaultTimerCallback(void const * argument);
void rtosGyroTimerCallback(void const * argument);
void rtosControllerTimerCallback(void const * argument);
void rtosTcxoTimerCallback(void const * argument);
void rtosAudioAdcTimerCallback(void const * argument);
void rtosAudioDacTimerCallback(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}


/* Local functions */

static void rtosDefaultInit(void)
{
  /* Power switch settings */
  mainPowerSwitchInit();
}

static void rtosDefaultMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                    msgIdx  = 0UL;
  const uint32_t              hdr     = msgAry[msgIdx++];
  const RtosMsgDefaultCmds_t  cmd     = (RtosMsgDefaultCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgDefault__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_rtos_DefaultTaskStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      rtosDefaultInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Main_Default, 0U, MsgDefault__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  /* ADC single conversion */
  case MsgDefault__CallFunc01_MCU_ADC:
    {
      int   dbgLen;
      char  dbgBuf[128];

      /* Do ADC conversion and logging of ADC data */
      if (s_rtos_DefaultTask_adc_enable) {
        adcStartConv(ADC_ADC1_TEMP);

        const uint32_t regMask = EG_ADC1__CONV_AVAIL_V_REFINT | EG_ADC1__CONV_AVAIL_V_SOLAR | EG_ADC1__CONV_AVAIL_V_BAT | EG_ADC1__CONV_AVAIL_TEMP;
        BaseType_t regBits = xEventGroupWaitBits(adcEventGroupHandle, regMask, regMask, pdTRUE, 100 / portTICK_PERIOD_MS);
        if ((regBits & regMask) == regMask) {
          /* All channels of ADC1 are complete */
          float     l_adc_v_vdda    = adcGetVal(ADC_ADC1_V_VDDA);
          float     l_adc_v_solar   = adcGetVal(ADC_ADC1_INT8_V_SOLAR);
          float     l_adc_v_bat     = adcGetVal(ADC_ADC1_V_BAT);
          float     l_adc_temp      = adcGetVal(ADC_ADC1_TEMP);
          int32_t   l_adc_temp_i    = 0L;
          uint32_t  l_adc_temp_f100 = 0UL;

          mainCalcFloat2IntFrac(l_adc_temp, 2, &l_adc_temp_i, &l_adc_temp_f100);

          dbgLen = sprintf(dbgBuf, "ADC: Vdda   = %4d mV, Vsolar = %4d mV, Vbat = %4d mV, temp = %+3ld.%02luC\r\n",
              (int16_t) (l_adc_v_vdda   + 0.5f),
              (int16_t) (l_adc_v_solar  + 0.5f),
              (int16_t) (l_adc_v_bat    + 0.5f),
              l_adc_temp_i, l_adc_temp_f100);
          usbLogLen(dbgBuf, dbgLen);
        }
      }
    }
    break;

  default: { }
  }  // switch (cmd)
}



/* USER CODE END 5 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of c2Ax5243_BSem */
  osSemaphoreDef(c2Ax5243_BSem);
  c2Ax5243_BSemHandle = osSemaphoreCreate(osSemaphore(c2Ax5243_BSem), 1);

  /* definition and creation of c2Sx1276_BSem */
  osSemaphoreDef(c2Sx1276_BSem);
  c2Sx1276_BSemHandle = osSemaphoreCreate(osSemaphore(c2Sx1276_BSem), 1);

  /* definition and creation of c2Si5338_BSem */
  osSemaphoreDef(c2Si5338_BSem);
  c2Si5338_BSemHandle = osSemaphoreCreate(osSemaphore(c2Si5338_BSem), 1);

  /* definition and creation of c2Tcxo_BSem */
  osSemaphoreDef(c2Tcxo_BSem);
  c2Tcxo_BSemHandle = osSemaphoreCreate(osSemaphore(c2Tcxo_BSem), 1);

  /* definition and creation of c2Baro_BSem */
  osSemaphoreDef(c2Baro_BSem);
  c2Baro_BSemHandle = osSemaphoreCreate(osSemaphore(c2Baro_BSem), 1);

  /* definition and creation of c2Gyro_BSem */
  osSemaphoreDef(c2Gyro_BSem);
  c2Gyro_BSemHandle = osSemaphoreCreate(osSemaphore(c2Gyro_BSem), 1);

  /* definition and creation of c2Hygro_BSem */
  osSemaphoreDef(c2Hygro_BSem);
  c2Hygro_BSemHandle = osSemaphoreCreate(osSemaphore(c2Hygro_BSem), 1);

  /* definition and creation of c2Lcd_BSem */
  osSemaphoreDef(c2Lcd_BSem);
  c2Lcd_BSemHandle = osSemaphoreCreate(osSemaphore(c2Lcd_BSem), 1);

  /* definition and creation of c2Default_BSem */
  osSemaphoreDef(c2Default_BSem);
  c2Default_BSemHandle = osSemaphoreCreate(osSemaphore(c2Default_BSem), 1);

  /* definition and creation of i2c1_BSem */
  osSemaphoreDef(i2c1_BSem);
  i2c1_BSemHandle = osSemaphoreCreate(osSemaphore(i2c1_BSem), 1);

  /* definition and creation of i2c2_BSem */
  osSemaphoreDef(i2c2_BSem);
  i2c2_BSemHandle = osSemaphoreCreate(osSemaphore(i2c2_BSem), 1);

  /* definition and creation of i2c3_BSem */
  osSemaphoreDef(i2c3_BSem);
  i2c3_BSemHandle = osSemaphoreCreate(osSemaphore(i2c3_BSem), 1);

  /* definition and creation of i2c4_BSem */
  osSemaphoreDef(i2c4_BSem);
  i2c4_BSemHandle = osSemaphoreCreate(osSemaphore(i2c4_BSem), 1);

  /* definition and creation of spi1_BSem */
  osSemaphoreDef(spi1_BSem);
  spi1_BSemHandle = osSemaphoreCreate(osSemaphore(spi1_BSem), 1);

  /* definition and creation of spi3_BSem */
  osSemaphoreDef(spi3_BSem);
  spi3_BSemHandle = osSemaphoreCreate(osSemaphore(spi3_BSem), 1);

  /* definition and creation of cQin_BSem */
  osSemaphoreDef(cQin_BSem);
  cQin_BSemHandle = osSemaphoreCreate(osSemaphore(cQin_BSem), 1);

  /* definition and creation of cQout_BSem */
  osSemaphoreDef(cQout_BSem);
  cQout_BSemHandle = osSemaphoreCreate(osSemaphore(cQout_BSem), 1);

  /* definition and creation of c2AudioAdc_BSem */
  osSemaphoreDef(c2AudioAdc_BSem);
  c2AudioAdc_BSemHandle = osSemaphoreCreate(osSemaphore(c2AudioAdc_BSem), 1);

  /* definition and creation of c2AudioDac_BSem */
  osSemaphoreDef(c2AudioDac_BSem);
  c2AudioDac_BSemHandle = osSemaphoreCreate(osSemaphore(c2AudioDac_BSem), 1);

  /* definition and creation of c2usbToHost_BSem */
  osSemaphoreDef(c2usbToHost_BSem);
  c2usbToHost_BSemHandle = osSemaphoreCreate(osSemaphore(c2usbToHost_BSem), 1);

  /* definition and creation of c2usbFromHost_BSem */
  osSemaphoreDef(c2usbFromHost_BSem);
  c2usbFromHost_BSemHandle = osSemaphoreCreate(osSemaphore(c2usbFromHost_BSem), 1);

  /* definition and creation of usb_BSem */
  osSemaphoreDef(usb_BSem);
  usb_BSemHandle = osSemaphoreCreate(osSemaphore(usb_BSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of baroTimer */
  osTimerDef(baroTimer, rtosBaroTimerCallback);
  baroTimerHandle = osTimerCreate(osTimer(baroTimer), osTimerPeriodic, NULL);

  /* definition and creation of hygroTimer */
  osTimerDef(hygroTimer, rtosHygroTimerCallback);
  hygroTimerHandle = osTimerCreate(osTimer(hygroTimer), osTimerPeriodic, NULL);

  /* definition and creation of defaultTimer */
  osTimerDef(defaultTimer, rtosDefaultTimerCallback);
  defaultTimerHandle = osTimerCreate(osTimer(defaultTimer), osTimerPeriodic, NULL);

  /* definition and creation of gyroTimer */
  osTimerDef(gyroTimer, rtosGyroTimerCallback);
  gyroTimerHandle = osTimerCreate(osTimer(gyroTimer), osTimerPeriodic, NULL);

  /* definition and creation of controllerTimer */
  osTimerDef(controllerTimer, rtosControllerTimerCallback);
  controllerTimerHandle = osTimerCreate(osTimer(controllerTimer), osTimerPeriodic, NULL);

  /* definition and creation of tcxoTimer */
  osTimerDef(tcxoTimer, rtosTcxoTimerCallback);
  tcxoTimerHandle = osTimerCreate(osTimer(tcxoTimer), osTimerPeriodic, NULL);

  /* definition and creation of audioAdcTimer */
  osTimerDef(audioAdcTimer, rtosAudioAdcTimerCallback);
  audioAdcTimerHandle = osTimerCreate(osTimer(audioAdcTimer), osTimerPeriodic, NULL);

  /* definition and creation of audioDacTimer */
  osTimerDef(audioDacTimer, rtosAudioDacTimerCallback);
  audioDacTimerHandle = osTimerCreate(osTimer(audioDacTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usbToHostTask */
  osThreadDef(usbToHostTask, StartUsbToHostTask, osPriorityNormal, 0, 128);
  usbToHostTaskHandle = osThreadCreate(osThread(usbToHostTask), NULL);

  /* definition and creation of usbFromHostTask */
  osThreadDef(usbFromHostTask, StartUsbFromHostTask, osPriorityNormal, 0, 128);
  usbFromHostTaskHandle = osThreadCreate(osThread(usbFromHostTask), NULL);

  /* definition and creation of hygroTask */
  osThreadDef(hygroTask, StartHygroTask, osPriorityNormal, 0, 256);
  hygroTaskHandle = osThreadCreate(osThread(hygroTask), NULL);

  /* definition and creation of baroTask */
  osThreadDef(baroTask, StartBaroTask, osPriorityNormal, 0, 256);
  baroTaskHandle = osThreadCreate(osThread(baroTask), NULL);

  /* definition and creation of gyroTask */
  osThreadDef(gyroTask, StartGyroTask, osPriorityNormal, 0, 256);
  gyroTaskHandle = osThreadCreate(osThread(gyroTask), NULL);

  /* definition and creation of lcdTask */
  osThreadDef(lcdTask, StartLcdTask, osPriorityNormal, 0, 256);
  lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);

  /* definition and creation of tcxo20MhzTask */
  osThreadDef(tcxo20MhzTask, StartTcxo20MhzTask, osPriorityNormal, 0, 256);
  tcxo20MhzTaskHandle = osThreadCreate(osThread(tcxo20MhzTask), NULL);

  /* definition and creation of ax5243Task */
  osThreadDef(ax5243Task, StartAx5243Task, osPriorityAboveNormal, 0, 512);
  ax5243TaskHandle = osThreadCreate(osThread(ax5243Task), NULL);

  /* definition and creation of sx1276Task */
  osThreadDef(sx1276Task, StartSx1276Task, osPriorityAboveNormal, 0, 512);
  sx1276TaskHandle = osThreadCreate(osThread(sx1276Task), NULL);

  /* definition and creation of controllerTask */
  osThreadDef(controllerTask, StartControllerTask, osPriorityLow, 0, 256);
  controllerTaskHandle = osThreadCreate(osThread(controllerTask), NULL);

  /* definition and creation of si5338Task */
  osThreadDef(si5338Task, StartSi5338Task, osPriorityNormal, 0, 256);
  si5338TaskHandle = osThreadCreate(osThread(si5338Task), NULL);

  /* definition and creation of audioAdcTask */
  osThreadDef(audioAdcTask, StartAudioAdcTask, osPriorityNormal, 0, 256);
  audioAdcTaskHandle = osThreadCreate(osThread(audioAdcTask), NULL);

  /* definition and creation of audioDacTask */
  osThreadDef(audioDacTask, StartAudioDacTask, osPriorityNormal, 0, 256);
  audioDacTaskHandle = osThreadCreate(osThread(audioDacTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of usbToHostQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(usbToHostQueue, 4096, uint8_t);
  usbToHostQueueHandle = osMessageCreate(osMessageQ(usbToHostQueue), NULL);

  /* definition and creation of usbFromHostQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(usbFromHostQueue, 32, uint8_t);
  usbFromHostQueueHandle = osMessageCreate(osMessageQ(usbFromHostQueue), NULL);

  /* definition and creation of controllerInQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(controllerInQueue, 8, uint32_t);
  controllerInQueueHandle = osMessageCreate(osMessageQ(controllerInQueue), NULL);

  /* definition and creation of controllerOutQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(controllerOutQueue, 32, uint32_t);
  controllerOutQueueHandle = osMessageCreate(osMessageQ(controllerOutQueue), NULL);

  /* definition and creation of loraMacQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(loraMacQueue, 32, uint8_t);
  loraMacQueueHandle = osMessageCreate(osMessageQ(loraMacQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* add event groups */
  adcEventGroupHandle = xEventGroupCreate();
  extiEventGroupHandle = xEventGroupCreate();
  globalEventGroupHandle = xEventGroupCreate();
  spiEventGroupHandle = xEventGroupCreate();
  usbToHostEventGroupHandle = xEventGroupCreate();

  /* add to registry */
  vQueueAddToRegistry(i2c1_BSemHandle,          "Resc I2C1 BSem");
  vQueueAddToRegistry(i2c2_BSemHandle,          "Resc I2C2 BSem");
  vQueueAddToRegistry(i2c3_BSemHandle,          "Resc I2C3 BSem");
  vQueueAddToRegistry(i2c4_BSemHandle,          "Resc I2C4 BSem");
  vQueueAddToRegistry(spi1_BSemHandle,          "Resc SPI1 BSem");
  vQueueAddToRegistry(spi3_BSemHandle,          "Resc SPI2 BSem");
  vQueueAddToRegistry(cQin_BSemHandle,          "Resc cQin BSem");
  vQueueAddToRegistry(cQout_BSemHandle,         "Resc cQout BSem");
  vQueueAddToRegistry(usb_BSemHandle,           "Resc USB BSem");

  vQueueAddToRegistry(c2AudioAdc_BSemHandle,    "Wake c2AudioAdc BSem");
  vQueueAddToRegistry(c2AudioDac_BSemHandle,    "Wake c2AudioDac BSem");
  vQueueAddToRegistry(c2Ax5243_BSemHandle,      "Wake c2Ax5243 BSem");
  vQueueAddToRegistry(c2Default_BSemHandle,     "Wake c2Default BSem");
  vQueueAddToRegistry(c2Baro_BSemHandle,        "Wake c2Baro BSem");
  vQueueAddToRegistry(c2Gyro_BSemHandle,        "Wake c2Gyro BSem");
  vQueueAddToRegistry(c2Hygro_BSemHandle,       "Wake c2Hygro BSem");
  vQueueAddToRegistry(c2Lcd_BSemHandle,         "Wake c2LCD BSem");
  vQueueAddToRegistry(c2Si5338_BSemHandle,      "Wake c2Si5338 BSem");
  vQueueAddToRegistry(c2Sx1276_BSemHandle,      "Wake c2Sx1276 BSem");
  vQueueAddToRegistry(c2Tcxo_BSemHandle,        "Wake c2TCXO BSem");
  vQueueAddToRegistry(c2usbFromHost_BSemHandle, "Wake c2usbFromHost BSem");
  vQueueAddToRegistry(c2usbToHost_BSemHandle,   "Wake c2usbToHost BSem");

  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  // Above function is voided. USB-DCD is activated when needed

  /* defaultTaskInit() section */
  {
    g_enableMsk                   = 0UL;                                                              // ENABLE_MASK__LORA_BARE;
    g_monMsk                      = 0UL;
    s_rtos_DefaultTask_adc_enable = 0U;
    s_rtos_DefaultTaskStartTime   = 0UL;
  }

  /* Wait until controller is up */
  xEventGroupWaitBits(globalEventGroupHandle,
      EG_GLOBAL__Controller_CTRL_IS_RUNNING,
      0UL,
      0, portMAX_DELAY);

  /* Store start time */
  s_rtos_DefaultTaskStartTime = osKernelSysTick();

  /* Give other tasks time to do the same */
  osDelay(10UL);

  do {
    uint32_t msgLen                       = 0UL;
    uint32_t msgAry[CONTROLLER_MSG_Q_LEN];

    /* Wait for door bell and hand-over controller out queue */
    {
      osSemaphoreWait(c2Default_BSemHandle, osWaitForever);
      msgLen = controllerMsgPullFromOutQueue(msgAry, Destinations__Main_Default, 1UL);                // Special case of callbacks need to limit blocking time
    }

    /* Decode and execute the commands when a message exists
     * (in case of callbacks the loop catches its wakeup semaphore
     * before ctrlQout is released results to request on an empty queue) */
    if (msgLen) {
      rtosDefaultMsgProcess(msgLen, msgAry);
    }
  } while (1);
  /* USER CODE END StartDefaultTask */
}

/* StartUsbToHostTask function */
void StartUsbToHostTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbToHostTask */
  usbUsbToHostTaskInit();

  /* Infinite loop */
  for (;;) {
    usbUsbToHostTaskLoop();
  }
  /* USER CODE END StartUsbToHostTask */
}

/* StartUsbFromHostTask function */
void StartUsbFromHostTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbFromHostTask */
  usbUsbFromHostTaskInit();

  /* Infinite loop */
  for (;;) {
    usbUsbFromHostTaskLoop();
  }
  /* USER CODE END StartUsbFromHostTask */
}

/* StartHygroTask function */
void StartHygroTask(void const * argument)
{
  /* USER CODE BEGIN StartHygroTask */
  hygroTaskInit();

  /* Infinite loop */
  for(;;)
  {
    hygroTaskLoop();
  }
  /* USER CODE END StartHygroTask */
}

/* StartBaroTask function */
void StartBaroTask(void const * argument)
{
  /* USER CODE BEGIN StartBaroTask */
  baroTaskInit();

  /* Infinite loop */
  for(;;)
  {
    baroTaskLoop();
  }
  /* USER CODE END StartBaroTask */
}

/* StartGyroTask function */
void StartGyroTask(void const * argument)
{
  /* USER CODE BEGIN StartGyroTask */
  gyroTaskInit();

  /* Infinite loop */
  for(;;)
  {
    gyroTaskLoop();
  }
  /* USER CODE END StartGyroTask */
}

/* StartLcdTask function */
void StartLcdTask(void const * argument)
{
  /* USER CODE BEGIN StartLcdTask */
  lcdTaskInit();

  /* Infinite loop */
  for(;;)
  {
    lcdTaskLoop();
  }
  /* USER CODE END StartLcdTask */
}

/* StartTcxo20MhzTask function */
void StartTcxo20MhzTask(void const * argument)
{
  /* USER CODE BEGIN StartTcxo20MhzTask */
  tcxo20MhzTaskInit();

  /* Infinite loop */
  for (;;) {
    tcxo20MhzTaskLoop();
  }
  /* USER CODE END StartTcxo20MhzTask */
}

/* StartAx5243Task function */
void StartAx5243Task(void const * argument)
{
  /* USER CODE BEGIN StartAx5243Task */
  ax5243TaskInit();

  /* Infinite loop */
  for(;;)
  {
    ax5243TaskLoop();
  }
  /* USER CODE END StartAx5243Task */
}

/* StartSx1276Task function */
void StartSx1276Task(void const * argument)
{
  /* USER CODE BEGIN StartSx1276Task */
  sx1276TaskInit();

  /* Infinite loop */
  for(;;)
  {
    sx1276TaskLoop();
  }
  /* USER CODE END StartSx1276Task */
}

/* StartControllerTask function */
void StartControllerTask(void const * argument)
{
  /* USER CODE BEGIN StartControllerTask */
  controllerTaskInit();

  /* Infinite loop */
  for(;;)
  {
    controllerTaskLoop();
  }
  /* USER CODE END StartControllerTask */
}

/* StartSi5338Task function */
void StartSi5338Task(void const * argument)
{
  /* USER CODE BEGIN StartSi5338Task */
  si5338TaskInit();

  /* Infinite loop */
  for(;;)
  {
    si5338TaskLoop();
  }
  /* USER CODE END StartSi5338Task */
}

/* StartAudioAdcTask function */
void StartAudioAdcTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioAdcTask */
  audioAdcTaskInit();

  /* Infinite loop */
  for(;;)
  {
    audioAdcTaskLoop();
  }
  /* USER CODE END StartAudioAdcTask */
}

/* StartAudioDacTask function */
void StartAudioDacTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioDacTask */
  audioDacTaskInit();

  /* Infinite loop */
  for(;;)
  {
    audioDacTaskLoop();
  }
  /* USER CODE END StartAudioDacTask */
}

/* rtosBaroTimerCallback function */
void rtosBaroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosBaroTimerCallback */
  baroTimerCallback(argument);
  /* USER CODE END rtosBaroTimerCallback */
}

/* rtosHygroTimerCallback function */
void rtosHygroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosHygroTimerCallback */
  hygroTimerCallback(argument);
  /* USER CODE END rtosHygroTimerCallback */
}

/* rtosDefaultTimerCallback function */
void rtosDefaultTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosDefaultTimerCallback */
  // TODO: code here

  /* USER CODE END rtosDefaultTimerCallback */
}

/* rtosGyroTimerCallback function */
void rtosGyroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosGyroTimerCallback */
  gyroTimerCallback(argument);
  /* USER CODE END rtosGyroTimerCallback */
}

/* rtosControllerTimerCallback function */
void rtosControllerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosControllerTimerCallback */
  controllerTimerCallback(argument);
  /* USER CODE END rtosControllerTimerCallback */
}

/* rtosTcxoTimerCallback function */
void rtosTcxoTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosTcxoTimerCallback */
  tcxoTimerCallback(argument);
  /* USER CODE END rtosTcxoTimerCallback */
}

/* rtosAudioAdcTimerCallback function */
void rtosAudioAdcTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosAudioAdcTimerCallback */
  audioAdcTimerCallback(argument);
  /* USER CODE END rtosAudioAdcTimerCallback */
}

/* rtosAudioDacTimerCallback function */
void rtosAudioDacTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosAudioDacTimerCallback */
  audioDacTimerCallback(argument);
  /* USER CODE END rtosAudioDacTimerCallback */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
