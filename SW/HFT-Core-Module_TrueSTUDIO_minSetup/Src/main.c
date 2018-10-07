
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
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
void mainBaroTimerCallback(void const * argument);
void mainHygroTimerCallback(void const * argument);
void mainDefaultTimerCallback(void const * argument);
void mainGyroTimerCallback(void const * argument);
void mainControllerTimerCallback(void const * argument);
void mainTcxoTimerCallback(void const * argument);
void mainAudioAdcTimerCallback(void const * argument);
void mainAudioDacTimerCallback(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of baroTimer */
  osTimerDef(baroTimer, mainBaroTimerCallback);
  baroTimerHandle = osTimerCreate(osTimer(baroTimer), osTimerPeriodic, NULL);

  /* definition and creation of hygroTimer */
  osTimerDef(hygroTimer, mainHygroTimerCallback);
  hygroTimerHandle = osTimerCreate(osTimer(hygroTimer), osTimerPeriodic, NULL);

  /* definition and creation of defaultTimer */
  osTimerDef(defaultTimer, mainDefaultTimerCallback);
  defaultTimerHandle = osTimerCreate(osTimer(defaultTimer), osTimerPeriodic, NULL);

  /* definition and creation of gyroTimer */
  osTimerDef(gyroTimer, mainGyroTimerCallback);
  gyroTimerHandle = osTimerCreate(osTimer(gyroTimer), osTimerPeriodic, NULL);

  /* definition and creation of controllerTimer */
  osTimerDef(controllerTimer, mainControllerTimerCallback);
  controllerTimerHandle = osTimerCreate(osTimer(controllerTimer), osTimerPeriodic, NULL);

  /* definition and creation of tcxoTimer */
  osTimerDef(tcxoTimer, mainTcxoTimerCallback);
  tcxoTimerHandle = osTimerCreate(osTimer(tcxoTimer), osTimerPeriodic, NULL);

  /* definition and creation of audioAdcTimer */
  osTimerDef(audioAdcTimer, mainAudioAdcTimerCallback);
  audioAdcTimerHandle = osTimerCreate(osTimer(audioAdcTimer), osTimerPeriodic, NULL);

  /* definition and creation of audioDacTimer */
  osTimerDef(audioDacTimer, mainAudioDacTimerCallback);
  audioDacTimerHandle = osTimerCreate(osTimer(audioDacTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usbToHostTask */
  osThreadDef(usbToHostTask, StartUsbToHostTask, osPriorityHigh, 0, 128);
  usbToHostTaskHandle = osThreadCreate(osThread(usbToHostTask), NULL);

  /* definition and creation of usbFromHostTask */
  osThreadDef(usbFromHostTask, StartUsbFromHostTask, osPriorityHigh, 0, 128);
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
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 31;
  hrtc.Init.SynchPrediv = 1023;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 3 */

  /* USER CODE END RTC_Init 3 */

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 4 */

  /* USER CODE END RTC_Init 4 */

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 5 */

  /* USER CODE END RTC_Init 5 */

    /**Enable the Alarm B 
    */
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_B;

    /**Enable the WakeUp 
    */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 6 */

  /* USER CODE END RTC_Init 6 */

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PF0   ------> I2C2_SDA
     PF1   ------> I2C2_SCL
     PF3   ------> ADC3_IN6
     PF6   ------> S_TIM5_CH1
     PF7   ------> S_TIM5_CH2
     PF8   ------> S_TIM5_CH3
     PF9   ------> S_TIM5_CH4
     PA0   ------> USART2_CTS
     PA2   ------> USART2_TX
     PA3   ------> ADCx_IN8
     PA6   ------> USART3_CTS
     PA7   ------> S_TIM3_CH2
     PC4   ------> USART3_TX
     PC5   ------> USART3_RX
     PB0   ------> S_TIM3_CH3
     PB1   ------> USART3_RTS
     PF14   ------> I2C4_SCL
     PE8   ------> SAI1_SCK_B
     PE9   ------> SAI1_FS_B
     PE10   ------> SAI1_MCLK_B
     PE13   ------> SPI1_SCK
     PE14   ------> SPI1_MISO
     PE15   ------> SPI1_MOSI
     PB13   ------> SAI2_SCK_A
     PD11   ------> SAI2_SD_A
     PD12   ------> SAI2_FS_A
     PD13   ------> I2C4_SDA
     PG7   ------> I2C3_SCL
     PG8   ------> I2C3_SDA
     PC6   ------> S_TIM3_CH1
     PA8   ------> RCC_MCO
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> SPI3_SCK
     PC11   ------> SPI3_MISO
     PC12   ------> SPI3_MOSI
     PD4   ------> USART2_RTS
     PD6   ------> USART2_RX
     PG9   ------> USART1_TX
     PG10   ------> USART1_RX
     PG11   ------> USART1_CTS
     PG12   ------> USART1_RTS
     PG13   ------> I2C1_SDA
     PG14   ------> I2C1_SCL
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE7 PE11 PE12 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_I2C2_SDA_Pin MCU_I2C2_SCL_Pin */
  GPIO_InitStruct.Pin = MCU_I2C2_SDA_Pin|MCU_I2C2_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF10 PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC3_IN6_20MHZ_PULL_Pin */
  GPIO_InitStruct.Pin = ADC3_IN6_20MHZ_PULL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC3_IN6_20MHZ_PULL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_IN_TIMCAP_N_Pin MCU_IN_TIMCAP_W_Pin MCU_IN_TIMCAP_S_Pin MCU_IN_TIMCAP_VSOL_Pin */
  GPIO_InitStruct.Pin = MCU_IN_TIMCAP_N_Pin|MCU_IN_TIMCAP_W_Pin|MCU_IN_TIMCAP_S_Pin|MCU_IN_TIMCAP_VSOL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UART2_CTS_IN_Pin */
  GPIO_InitStruct.Pin = UART2_CTS_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(UART2_CTS_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART2_TX_OUT_Pin */
  GPIO_InitStruct.Pin = UART2_TX_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(UART2_TX_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_VSOL_ADC1_Pin */
  GPIO_InitStruct.Pin = MCU_VSOL_ADC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UART3_CTS_IN_Pin */
  GPIO_InitStruct.Pin = UART3_CTS_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(UART3_CTS_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_OUT_PWM_LED_G_Pin */
  GPIO_InitStruct.Pin = MCU_OUT_PWM_LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(MCU_OUT_PWM_LED_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UART3_TX_OUT_Pin UART3_RX_IN_Pin */
  GPIO_InitStruct.Pin = UART3_TX_OUT_Pin|UART3_RX_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_OUT_PWM_LED_B_Pin */
  GPIO_InitStruct.Pin = MCU_OUT_PWM_LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(MCU_OUT_PWM_LED_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART3_RTS_OUT_Pin */
  GPIO_InitStruct.Pin = UART3_RTS_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(UART3_RTS_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_I2C4_SCL_Pin */
  GPIO_InitStruct.Pin = MCU_I2C4_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(MCU_I2C4_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3 
                           PG4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S1B_SCK_OUT_Pin I2S1B_FS_OUT_Pin I2S1B_MCLK_OUT_Pin */
  GPIO_InitStruct.Pin = I2S1B_SCK_OUT_Pin|I2S1B_FS_OUT_Pin|I2S1B_MCLK_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_SPI1_SCK_Pin MCU_SPI1_MISO_Pin MCU_SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = MCU_SPI1_SCK_Pin|MCU_SPI1_MISO_Pin|MCU_SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB14 PB3 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_3 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S2A_SCK_IN_Pin */
  GPIO_InitStruct.Pin = I2S2A_SCK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(I2S2A_SCK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD14 
                           PD15 PD0 PD1 PD2 
                           PD3 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S2A_SD_IN_Pin I2S2A_FS_IN_Pin */
  GPIO_InitStruct.Pin = I2S2A_SD_IN_Pin|I2S2A_FS_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_I2C4_SDA_Pin */
  GPIO_InitStruct.Pin = MCU_I2C4_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(MCU_I2C4_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_I2C3_SCL_Pin MCU_I2C3_SDA_Pin */
  GPIO_InitStruct.Pin = MCU_I2C3_SCL_Pin|MCU_I2C3_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_OUT_PWM_LED_R_Pin */
  GPIO_InitStruct.Pin = MCU_OUT_PWM_LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(MCU_OUT_PWM_LED_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_MCO_Pin */
  GPIO_InitStruct.Pin = MCU_MCO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(MCU_MCO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA_USB_N_Pin PA_USB_P_Pin */
  GPIO_InitStruct.Pin = PA_USB_N_Pin|PA_USB_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_SPI3_SCK_Pin MCU_SPI3_MISO_Pin MCU_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = MCU_SPI3_SCK_Pin|MCU_SPI3_MISO_Pin|MCU_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UART2_RTS_OUT_Pin */
  GPIO_InitStruct.Pin = UART2_RTS_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(UART2_RTS_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART2_RX_IN_Pin */
  GPIO_InitStruct.Pin = UART2_RX_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(UART2_RX_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UART1_TX_OUT_Pin UART1_RX_IN_Pin */
  GPIO_InitStruct.Pin = UART1_TX_OUT_Pin|UART1_RX_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : UART1_CTS_IN_Pin UART1_RTS_OUT_Pin */
  GPIO_InitStruct.Pin = UART1_CTS_IN_Pin|UART1_RTS_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_I2C1_SDA_Pin MCU_I2C1_SCL_Pin */
  GPIO_InitStruct.Pin = MCU_I2C1_SDA_Pin|MCU_I2C1_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* StartUsbToHostTask function */
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

/* StartUsbFromHostTask function */
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

/* StartHygroTask function */
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

/* StartBaroTask function */
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

/* StartGyroTask function */
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

/* StartLcdTask function */
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

/* StartTcxo20MhzTask function */
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

/* StartAx5243Task function */
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

/* StartSx1276Task function */
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

/* StartControllerTask function */
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

/* StartSi5338Task function */
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

/* StartAudioAdcTask function */
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

/* StartAudioDacTask function */
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

/* mainBaroTimerCallback function */
void mainBaroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainBaroTimerCallback */
  
  /* USER CODE END mainBaroTimerCallback */
}

/* mainHygroTimerCallback function */
void mainHygroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainHygroTimerCallback */
  
  /* USER CODE END mainHygroTimerCallback */
}

/* mainDefaultTimerCallback function */
void mainDefaultTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainDefaultTimerCallback */
  
  /* USER CODE END mainDefaultTimerCallback */
}

/* mainGyroTimerCallback function */
void mainGyroTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainGyroTimerCallback */
  
  /* USER CODE END mainGyroTimerCallback */
}

/* mainControllerTimerCallback function */
void mainControllerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainControllerTimerCallback */
  
  /* USER CODE END mainControllerTimerCallback */
}

/* mainTcxoTimerCallback function */
void mainTcxoTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainTcxoTimerCallback */
  
  /* USER CODE END mainTcxoTimerCallback */
}

/* mainAudioAdcTimerCallback function */
void mainAudioAdcTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainAudioAdcTimerCallback */
  
  /* USER CODE END mainAudioAdcTimerCallback */
}

/* mainAudioDacTimerCallback function */
void mainAudioDacTimerCallback(void const * argument)
{
  /* USER CODE BEGIN mainAudioDacTimerCallback */
  
  /* USER CODE END mainAudioDacTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
