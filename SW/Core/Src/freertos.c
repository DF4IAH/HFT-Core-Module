/* USER CODE BEGIN Header */
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
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

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

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
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
  osMessageQDef(usbToHostQueue, 4096, uint8_t);
  usbToHostQueueHandle = osMessageCreate(osMessageQ(usbToHostQueue), NULL);

  /* definition and creation of usbFromHostQueue */
  osMessageQDef(usbFromHostQueue, 32, uint8_t);
  usbFromHostQueueHandle = osMessageCreate(osMessageQ(usbFromHostQueue), NULL);

  /* definition and creation of controllerInQueue */
  osMessageQDef(controllerInQueue, 8, uint32_t);
  controllerInQueueHandle = osMessageCreate(osMessageQ(controllerInQueue), NULL);

  /* definition and creation of controllerOutQueue */
  osMessageQDef(controllerOutQueue, 32, uint32_t);
  controllerOutQueueHandle = osMessageCreate(osMessageQ(controllerOutQueue), NULL);

  /* definition and creation of loraMacQueue */
  osMessageQDef(loraMacQueue, 32, uint8_t);
  loraMacQueueHandle = osMessageCreate(osMessageQ(loraMacQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUsbToHostTask */
/**
* @brief Function implementing the usbToHostTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbToHostTask */
void StartUsbToHostTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbToHostTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsbToHostTask */
}

/* USER CODE BEGIN Header_StartUsbFromHostTask */
/**
* @brief Function implementing the usbFromHostTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbFromHostTask */
void StartUsbFromHostTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbFromHostTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsbFromHostTask */
}

/* USER CODE BEGIN Header_StartHygroTask */
/**
* @brief Function implementing the hygroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHygroTask */
void StartHygroTask(void const * argument)
{
  /* USER CODE BEGIN StartHygroTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHygroTask */
}

/* USER CODE BEGIN Header_StartBaroTask */
/**
* @brief Function implementing the baroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBaroTask */
void StartBaroTask(void const * argument)
{
  /* USER CODE BEGIN StartBaroTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBaroTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
* @brief Function implementing the gyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void const * argument)
{
  /* USER CODE BEGIN StartGyroTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartLcdTask */
/**
* @brief Function implementing the lcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLcdTask */
void StartLcdTask(void const * argument)
{
  /* USER CODE BEGIN StartLcdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLcdTask */
}

/* USER CODE BEGIN Header_StartTcxo20MhzTask */
/**
* @brief Function implementing the tcxo20MhzTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTcxo20MhzTask */
void StartTcxo20MhzTask(void const * argument)
{
  /* USER CODE BEGIN StartTcxo20MhzTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTcxo20MhzTask */
}

/* USER CODE BEGIN Header_StartAx5243Task */
/**
* @brief Function implementing the ax5243Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAx5243Task */
void StartAx5243Task(void const * argument)
{
  /* USER CODE BEGIN StartAx5243Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAx5243Task */
}

/* USER CODE BEGIN Header_StartSx1276Task */
/**
* @brief Function implementing the sx1276Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSx1276Task */
void StartSx1276Task(void const * argument)
{
  /* USER CODE BEGIN StartSx1276Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSx1276Task */
}

/* USER CODE BEGIN Header_StartControllerTask */
/**
* @brief Function implementing the controllerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControllerTask */
void StartControllerTask(void const * argument)
{
  /* USER CODE BEGIN StartControllerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartControllerTask */
}

/* USER CODE BEGIN Header_StartSi5338Task */
/**
* @brief Function implementing the si5338Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSi5338Task */
void StartSi5338Task(void const * argument)
{
  /* USER CODE BEGIN StartSi5338Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSi5338Task */
}

/* USER CODE BEGIN Header_StartAudioAdcTask */
/**
* @brief Function implementing the audioAdcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioAdcTask */
void StartAudioAdcTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioAdcTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAudioAdcTask */
}

/* USER CODE BEGIN Header_StartAudioDacTask */
/**
* @brief Function implementing the audioDacTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioDacTask */
void StartAudioDacTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioDacTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAudioDacTask */
}

/* rtosBaroTimerCallback function */
void rtosBaroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosBaroTimerCallback */
  
  /* USER CODE END rtosBaroTimerCallback */
}

/* rtosHygroTimerCallback function */
void rtosHygroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosHygroTimerCallback */
  
  /* USER CODE END rtosHygroTimerCallback */
}

/* rtosDefaultTimerCallback function */
void rtosDefaultTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosDefaultTimerCallback */
  
  /* USER CODE END rtosDefaultTimerCallback */
}

/* rtosGyroTimerCallback function */
void rtosGyroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosGyroTimerCallback */
  
  /* USER CODE END rtosGyroTimerCallback */
}

/* rtosControllerTimerCallback function */
void rtosControllerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosControllerTimerCallback */
  
  /* USER CODE END rtosControllerTimerCallback */
}

/* rtosTcxoTimerCallback function */
void rtosTcxoTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosTcxoTimerCallback */
  
  /* USER CODE END rtosTcxoTimerCallback */
}

/* rtosAudioAdcTimerCallback function */
void rtosAudioAdcTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosAudioAdcTimerCallback */
  
  /* USER CODE END rtosAudioAdcTimerCallback */
}

/* rtosAudioDacTimerCallback function */
void rtosAudioDacTimerCallback(void const * argument)
{
  /* USER CODE BEGIN rtosAudioDacTimerCallback */
  
  /* USER CODE END rtosAudioDacTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
