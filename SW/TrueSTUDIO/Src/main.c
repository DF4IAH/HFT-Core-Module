
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
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"

#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include "device_adc.h"
#include "bus_i2c.h"
#include "bus_spi.h"
#include "task_Controller.h"
#include "task_TCXO_20MHz.h"
#include "task_Si5338.h"
#include "task_AX5243.h"
#include "task_SX1276.h"
#include "usb.h"


#define  PERIOD_VALUE       (uint32_t)(16000UL - 1)                                             /* Period Value = 1ms */
#define  PULSE_RED_VALUE    (uint32_t)(PERIOD_VALUE * 75 / 100)                                 /* Capture Compare 1 Value  */
#define  PULSE_GREEN_VALUE  (uint32_t)(PERIOD_VALUE * 25 / 100)                                 /* Capture Compare 2 Value  */
#define  PULSE_BLUE_VALUE   (uint32_t)(PERIOD_VALUE * 50 / 100)                                 /* Capture Compare 3 Value  */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

CRC_HandleTypeDef hcrc;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA2;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai2_a;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

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
osMessageQId usbToHostQueueHandle;
osMessageQId usbFromHostQueueHandle;
osMessageQId controllerInQueueHandle;
osMessageQId controllerOutQueueHandle;
osMutexId i2c1MutexHandle;
osMutexId i2c2MutexHandle;
osMutexId i2c3MutexHandle;
osMutexId i2c4MutexHandle;
osMutexId spi1MutexHandle;
osMutexId spi3MutexHandle;
osSemaphoreId c2Ax5243_BSemHandle;
osSemaphoreId c2Sx1276_BSemHandle;
osSemaphoreId c2Si5338_BSemHandle;
osSemaphoreId c2Tcxo_BSemHandle;
osSemaphoreId c2Baro_BSemHandle;
osSemaphoreId c2Gyro_BSemHandle;
osSemaphoreId c2Hygro_BSemHandle;
osSemaphoreId c2Lcd_BSemHandle;
osSemaphoreId c2Default_BSemHandle;

/* USER CODE BEGIN PV */
extern uint32_t                       uwTick;

/* Private variables ---------------------------------------------------------*/
EventGroupHandle_t                    extiEventGroupHandle;
EventGroupHandle_t                    usbToHostEventGroupHandle;
EventGroupHandle_t                    adcEventGroupHandle;
EventGroupHandle_t                    spiEventGroupHandle;

osSemaphoreId                         usbToHostBinarySemHandle;

/* Timer handler declaration */
TIM_HandleTypeDef                     TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef                    sConfig;




/* Counter Prescaler value */
uint32_t                              uhPrescalerValue        = 0;

volatile uint32_t                     g_rtc_ssr_last          = 0UL;

static uint64_t                       s_timerLast_us          = 0ULL;
static uint64_t                       s_timerStart_us         = 0ULL;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void mainCalcFloat2IntFrac(float val, uint8_t fracCnt, int32_t* outInt, uint32_t* outFrac)
{
  const uint8_t isNeg = val >= 0 ?  0U : 1U;

  if (!outInt || !outFrac) {
    return;
  }

  *outInt = (int32_t) val;
  val -= *outInt;

  if (isNeg) {
    val = -val;
  }
  val *= pow(10, fracCnt);
  *outFrac = (uint32_t) (val + 0.5f);
}

void PowerSwitchDo(POWERSWITCH_ENUM_t sw, uint8_t enable)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __asm volatile( "NOP" );

  switch (sw) {
  case POWERSWITCH__USB_SW:
    /* Port: PC2 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (enable ?  GPIO_PIN_SET : GPIO_PIN_RESET));
    break;

  case POWERSWITCH__3V3_HICUR:
    /* Port: PC1 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (enable ?  GPIO_PIN_SET : GPIO_PIN_RESET));
    break;

  case POWERSWITCH__3V3_XO:
    /* Port: PC3 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (enable ?  GPIO_PIN_SET : GPIO_PIN_RESET));
    break;

  case POWERSWITCH__1V2_DCDC:
    /* V1.0: no hardware support */
    return;

  case POWERSWITCH__1V2_SW:
    /* SMPS handling */
    if (enable)
    {
      /* Port: PC0 */
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

      osDelay(10);

      /* Scale1: 1.2V up to 80MHz */
      /* Scale2: 1.0V up to 24MHz */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

    } else {
      /* Scale1: 1.2V up to 80MHz */
      /* Scale2: 1.0V up to 24MHz */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

      osDelay(100);

      /* Port: PC0 */
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    break;

  case POWERSWITCH__BAT_SW:
    if (enable) {
      HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_5);

    } else {
      HAL_PWREx_DisableBatteryCharging();
    }
    break;

  case POWERSWITCH__BAT_HICUR:
    if (enable) {
      HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_1_5);

    } else {
      HAL_PWREx_EnableBatteryCharging(PWR_BATTERY_CHARGING_RESISTOR_5);
    }
    break;

  default:
    { }
  }

  __HAL_RCC_GPIOC_CLK_DISABLE();
}

void PowerSwitchInit(void)
{
  /* Connect Vusb with +5V0 */
  PowerSwitchDo(POWERSWITCH__USB_SW, 1);

  /* Disable high current system */
  PowerSwitchDo(POWERSWITCH__3V3_HICUR, 0);

  /* Disable TCXO - enabled by i2cI2c4Si5338Init() on request */
  PowerSwitchDo(POWERSWITCH__3V3_XO, 0);

  /* Enable SMPS */
  {
//  PowerSwitchDo(POWERSWITCH__1V2_DCDC, 1);
//  for (uint16_t i = 10000; i; i--) ;
    PowerSwitchDo(POWERSWITCH__1V2_SW, 1);

    /*
     * SMPS DC/DC converter enabled but disconnected:
     *  --> Quiescent   current: 3.0 mA
     *  --> Application current: 3.0 mA + 13.5  mA = 16.5  mA
     *  --> Power estimation   : 3.0 mA +  2.97 mA =  5.97 mA
     *
     * SMPS DC/DC converter enabled and connected:
     *  --> Quiescent   current: 3.0 mA
     *  --> Application current: 3.0 mA + 11.0  mA = 14.0  mA
     *  --> Power estimation   : 3.0 mA +  1.33 mA =  4.33 mA
     */
  }

  /* Vbat charger of MCU enabled with 1.5 kOhm */
  PowerSwitchDo(POWERSWITCH__BAT_HICUR, 1);
}

void LcdBacklightInit(void)
{
  /* PWM initial code */
  TimHandle.Instance = TIM3;

  TimHandle.Init.Prescaler         = uhPrescalerValue;
  TimHandle.Init.Period            = PERIOD_VALUE;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* PWM: LCD-backlight Red */
  sConfig.Pulse = PULSE_RED_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* PWM: LCD-backlight Green */
  sConfig.Pulse = PULSE_GREEN_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* PWM: LCD-backlight Blue */
  sConfig.Pulse = PULSE_BLUE_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Start channel 1 - Red */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 - Green */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 - Blue */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
}

void SystemResetbyARMcore(void)
{
  /* Set SW reset bit */
  SCB->AIRCR = 0x05FA0000UL | SCB_AIRCR_SYSRESETREQ_Msk;
}

/* Used by the run-time stats */
void configureTimerForRunTimeStats(void)
{
  getRunTimeCounterValue();

  /* Interrupt disabled block */
  {
    __disable_irq();

    s_timerStart_us = s_timerLast_us;

    __enable_irq();
  }
}

/* Used by the run-time stats */
unsigned long getRunTimeCounterValue(void)
{
  uint64_t l_timerStart_us = 0ULL;
  uint64_t l_timer_us = HAL_GetTick() & 0x003fffffUL;                                                   // avoid overflows

  /* Add microseconds */
  l_timer_us *= 1000ULL;
  l_timer_us += TIM2->CNT % 1000UL;                                                                     // TIM2 counts microseconds

  /* Interrupt disabled block */
  {
    __disable_irq();

    s_timerLast_us  = l_timer_us;
    l_timerStart_us = s_timerStart_us;

    __enable_irq();
  }

  uint64_t l_timerDiff64 = (l_timer_us >= l_timerStart_us) ?  (l_timer_us - l_timerStart_us) : l_timer_us;
  uint32_t l_timerDiff32 = (uint32_t) (l_timerDiff64 & 0xffffffffULL);
  return l_timerDiff32;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* Check if ARM core is already in reset state */
  if (!(RCC->CSR & 0xff000000UL)) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __asm volatile( "NOP" );

    /* Disable SMPS */
    HAL_GPIO_WritePin(MCU_OUT_VDD12_EN_GPIO_Port, MCU_OUT_VDD12_EN_Pin, GPIO_PIN_RESET);
    //POWERSWITCH__1V2_DCDC, RESET;

    /* Turn off battery charger of Vbat */
    HAL_PWREx_DisableBatteryCharging();

    /* HICUR off */
    HAL_GPIO_WritePin(MCU_OUT_HICUR_EN_GPIO_Port, MCU_OUT_HICUR_EN_Pin, GPIO_PIN_RESET);

    /* 20MHz oscillator off */
    HAL_GPIO_WritePin(MCU_OUT_20MHZ_EN_GPIO_Port, MCU_OUT_20MHZ_EN_Pin, GPIO_PIN_RESET);

    /* VUSB off */
    HAL_GPIO_WritePin(MCU_OUT_VUSB_EN_GPIO_Port, MCU_OUT_VUSB_EN_Pin, GPIO_PIN_RESET);

    /* LCD reset */
    HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, GPIO_PIN_RESET);

    /* ARM software reset to be done */
    SystemResetbyARMcore();
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();  // 23.0mA --> 23.0mA

#if 0
  /* Give PMIC devices 3 seconds time to stabilize before demand of power ramps up */
  for (uint32_t delayCntr = 1000000UL; delayCntr; delayCntr--) { }
#endif

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* HSI16 trim */
  __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(0x3f);                                                        // 0x40 centered

  /* MSI trim */
  //__HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(0x00);                                                      // Signed
  HAL_RCCEx_EnableMSIPLLMode();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM5_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_DFSDM1_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

#if 0
  /* Disable clocks again to save power */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();
  __HAL_RCC_GPIOG_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

  __HAL_RCC_CRC_CLK_DISABLE();

  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_I2C2_CLK_DISABLE();
  __HAL_RCC_I2C3_CLK_DISABLE();

  __HAL_RCC_RNG_CLK_DISABLE();

  __HAL_RCC_SAI1_CLK_DISABLE();
  __HAL_RCC_SAI2_CLK_DISABLE();

  __HAL_RCC_SPI1_CLK_DISABLE();
  __HAL_RCC_SPI3_CLK_DISABLE();

  __HAL_RCC_USART1_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_USART3_CLK_DISABLE();

  __HAL_RCC_DFSDM1_CLK_DISABLE();
#endif

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of i2c1Mutex */
  osMutexDef(i2c1Mutex);
  i2c1MutexHandle = osMutexCreate(osMutex(i2c1Mutex));

  /* definition and creation of i2c2Mutex */
  osMutexDef(i2c2Mutex);
  i2c2MutexHandle = osMutexCreate(osMutex(i2c2Mutex));

  /* definition and creation of i2c3Mutex */
  osMutexDef(i2c3Mutex);
  i2c3MutexHandle = osMutexCreate(osMutex(i2c3Mutex));

  /* definition and creation of i2c4Mutex */
  osMutexDef(i2c4Mutex);
  i2c4MutexHandle = osMutexCreate(osMutex(i2c4Mutex));

  /* definition and creation of spi1Mutex */
  osMutexDef(spi1Mutex);
  spi1MutexHandle = osMutexCreate(osMutex(spi1Mutex));

  /* definition and creation of spi3Mutex */
  osMutexDef(spi3Mutex);
  spi3MutexHandle = osMutexCreate(osMutex(spi3Mutex));

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(usbToHostBinarySem);
  usbToHostBinarySemHandle = osSemaphoreCreate(osSemaphore(usbToHostBinarySem), 1);

  /* add event groups */
  extiEventGroupHandle = xEventGroupCreate();
  usbToHostEventGroupHandle = xEventGroupCreate();
  adcEventGroupHandle = xEventGroupCreate();
  spiEventGroupHandle = xEventGroupCreate();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usbToHostTask */
  osThreadDef(usbToHostTask, StartUsbToHostTask, osPriorityHigh, 0, 128);
  usbToHostTaskHandle = osThreadCreate(osThread(usbToHostTask), NULL);

  /* definition and creation of usbFromHostTask */
  osThreadDef(usbFromHostTask, StartUsbFromHostTask, osPriorityHigh, 0, 128);
  usbFromHostTaskHandle = osThreadCreate(osThread(usbFromHostTask), NULL);

  /* definition and creation of hygroTask */
  osThreadDef(hygroTask, StartHygroTask, osPriorityBelowNormal, 0, 256);
  hygroTaskHandle = osThreadCreate(osThread(hygroTask), NULL);

  /* definition and creation of baroTask */
  osThreadDef(baroTask, StartBaroTask, osPriorityBelowNormal, 0, 256);
  baroTaskHandle = osThreadCreate(osThread(baroTask), NULL);

  /* definition and creation of gyroTask */
  osThreadDef(gyroTask, StartGyroTask, osPriorityBelowNormal, 0, 256);
  gyroTaskHandle = osThreadCreate(osThread(gyroTask), NULL);

  /* definition and creation of lcdTask */
  osThreadDef(lcdTask, StartLcdTask, osPriorityNormal, 0, 256);
  lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);

  /* definition and creation of tcxo20MhzTask */
  osThreadDef(tcxo20MhzTask, StartTcxo20MhzTask, osPriorityBelowNormal, 0, 256);
  tcxo20MhzTaskHandle = osThreadCreate(osThread(tcxo20MhzTask), NULL);

  /* definition and creation of ax5243Task */
  osThreadDef(ax5243Task, StartAx5243Task, osPriorityBelowNormal, 0, 256);
  ax5243TaskHandle = osThreadCreate(osThread(ax5243Task), NULL);

  /* definition and creation of sx1276Task */
  osThreadDef(sx1276Task, StartSx1276Task, osPriorityBelowNormal, 0, 256);
  sx1276TaskHandle = osThreadCreate(osThread(sx1276Task), NULL);

  /* definition and creation of controllerTask */
  osThreadDef(controllerTask, StartControllerTask, osPriorityAboveNormal, 0, 256);
  controllerTaskHandle = osThreadCreate(osThread(controllerTask), NULL);

  /* definition and creation of si5338Task */
  osThreadDef(si5338Task, StartSi5338Task, osPriorityBelowNormal, 0, 256);
  si5338TaskHandle = osThreadCreate(osThread(si5338Task), NULL);

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
  osMessageQDef(controllerInQueue, 32, uint32_t);
  controllerInQueueHandle = osMessageCreate(osMessageQ(controllerInQueue), NULL);

  /* definition and creation of controllerOutQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(controllerOutQueue, 8, uint32_t);
  controllerOutQueueHandle = osMessageCreate(osMessageQ(controllerOutQueue), NULL);

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
  RCC_CRSInitTypeDef RCC_CRSInitStruct;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_SYSCLK;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_2);

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

    /**Enable the SYSCFG APB clock 
    */
  __HAL_RCC_CRS_CLK_ENABLE();

    /**Configures CRS 
    */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.NbrOfDiscConversion = 1;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = ENABLE;
  hadc3.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DFSDM1 init function */
static void MX_DFSDM1_Init(void)
{

  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = DISABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = DISABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 1;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = DISABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = DISABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 1;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 2;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0x00;
  hdfsdm1_channel0.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0x00;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00500822;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00500822;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00200C28;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C4 init function */
static void MX_I2C4_Init(void)
{

  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00500822;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 3 */

  /* USER CODE END RTC_Init 3 */

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 18;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 4 */

  /* USER CODE END RTC_Init 4 */

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 5 */

  /* USER CODE END RTC_Init 5 */

    /**Enable the Alarm B 
    */
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 6 */

  /* USER CODE END RTC_Init 6 */

    /**Enable the WakeUp 
    */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 7 */

  /* USER CODE END RTC_Init 7 */

}

/* SAI1 init function */
static void MX_SAI1_Init(void)
{

  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SAI2 init function */
static void MX_SAI2_Init(void)
{

  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1599999999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIMEx_RemapConfig(&htim16, TIM_TIM16_TI1_LSE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC;

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIMEx_RemapConfig(&htim17, TIM_TIM17_TI1_MSI) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA3   ------> ADCx_IN8
     PA8   ------> RCC_MCO
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MCU_OUT_VDD12_EN_Pin|MCU_OUT_HICUR_EN_Pin|MCU_OUT_VUSB_EN_Pin|MCU_OUT_20MHZ_EN_Pin 
                          |MCU_OUT_AUDIO_DAC_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_OUT_LCD_nRST_GPIO_Port, MCU_OUT_LCD_nRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MCU_OUT_AX_SEL_Pin|MCU_OUT_SX_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MCU_OUT_SX_nRESET_Pin|MCU_OUT_AUDIO_ADC_nRESET_Pin|MCU_OUT_AUDIO_ADC_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MCU_INOUT_PW03_Pin MCU_INOUT_PW02_Pin MCU_INOUT_PW01_Pin MCU_IN_AX_IRQ_Pin */
  GPIO_InitStruct.Pin = MCU_INOUT_PW03_Pin|MCU_INOUT_PW02_Pin|MCU_INOUT_PW01_Pin|MCU_IN_AX_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_INOUT_PW00_Pin */
  GPIO_InitStruct.Pin = MCU_INOUT_PW00_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MCU_INOUT_PW00_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_EVENTOUT_PF10_Pin */
  GPIO_InitStruct.Pin = MCU_EVENTOUT_PF10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(MCU_EVENTOUT_PF10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_OUT_VDD12_EN_Pin MCU_OUT_HICUR_EN_Pin MCU_OUT_VUSB_EN_Pin MCU_OUT_20MHZ_EN_Pin 
                           MCU_OUT_AUDIO_DAC_SEL_Pin */
  GPIO_InitStruct.Pin = MCU_OUT_VDD12_EN_Pin|MCU_OUT_HICUR_EN_Pin|MCU_OUT_VUSB_EN_Pin|MCU_OUT_20MHZ_EN_Pin 
                          |MCU_OUT_AUDIO_DAC_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_VSOL_ADC1_Pin */
  GPIO_InitStruct.Pin = MCU_VSOL_ADC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCU_VSOL_ADC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_OUT_LCD_nRST_Pin */
  GPIO_InitStruct.Pin = MCU_OUT_LCD_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_OUT_LCD_nRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_IN_AX_GPIO1_Pin */
  GPIO_InitStruct.Pin = MCU_IN_AX_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MCU_IN_AX_GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_OUT_AX_SEL_Pin MCU_OUT_SX_SEL_Pin */
  GPIO_InitStruct.Pin = MCU_OUT_AX_SEL_Pin|MCU_OUT_SX_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_IN_RE_I_Pin MCU_IN_RE_Q_Pin MCU_IN_RE_PB_Pin */
  GPIO_InitStruct.Pin = MCU_IN_RE_I_Pin|MCU_IN_RE_Q_Pin|MCU_IN_RE_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_OUT_SX_nRESET_Pin MCU_OUT_AUDIO_ADC_nRESET_Pin MCU_OUT_AUDIO_ADC_SEL_Pin */
  GPIO_InitStruct.Pin = MCU_OUT_SX_nRESET_Pin|MCU_OUT_AUDIO_ADC_nRESET_Pin|MCU_OUT_AUDIO_ADC_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_INOUT_PS00_Pin MCU_INOUT_PS01_Pin MCU_INOUT_PS02_Pin MCU_INOUT_PS03_Pin 
                           MCU_IN_AUDIO_ADC_MDAT1_Pin MCU_IN_AUDIO_ADC_MDAT0_Pin */
  GPIO_InitStruct.Pin = MCU_INOUT_PS00_Pin|MCU_INOUT_PS01_Pin|MCU_INOUT_PS02_Pin|MCU_INOUT_PS03_Pin 
                          |MCU_IN_AUDIO_ADC_MDAT1_Pin|MCU_IN_AUDIO_ADC_MDAT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_IN_EXTI2_SX_DIO0_TXRXDONE_Pin MCU_IN_EXTI3_SX_DIO1_RXTO_Pin */
  GPIO_InitStruct.Pin = MCU_IN_EXTI2_SX_DIO0_TXRXDONE_Pin|MCU_IN_EXTI3_SX_DIO1_RXTO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_EVENTOUT_PG4_Pin */
  GPIO_InitStruct.Pin = MCU_EVENTOUT_PG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(MCU_EVENTOUT_PG4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_IN_AUDIO_DAC_nRDY_Pin */
  GPIO_InitStruct.Pin = MCU_IN_AUDIO_DAC_nRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCU_IN_AUDIO_DAC_nRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_MCO_Pin */
  GPIO_InitStruct.Pin = MCU_MCO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(MCU_MCO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_IN_AUDIO_ADC_nDR_Pin */
  GPIO_InitStruct.Pin = MCU_IN_AUDIO_ADC_nDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCU_IN_AUDIO_ADC_nDR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_IN_EXTI7_INTR_SI5338_Pin */
  GPIO_InitStruct.Pin = MCU_IN_EXTI7_INTR_SI5338_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MCU_IN_EXTI7_INTR_SI5338_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_INOUT_PN00_Pin MCU_INOUT_PN01_Pin MCU_INOUT_PN02_Pin MCU_INOUT_PN03_Pin */
  GPIO_InitStruct.Pin = MCU_INOUT_PN00_Pin|MCU_INOUT_PN01_Pin|MCU_INOUT_PN02_Pin|MCU_INOUT_PN03_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_EVENTOUT_PE0_Pin */
  GPIO_InitStruct.Pin = MCU_EVENTOUT_PE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(MCU_EVENTOUT_PE0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t taskWoken = 0L;

  switch (GPIO_Pin) {
  case GPIO_PIN_2:
    /* Check for PG2 pin */
    if (GPIOG->IDR & GPIO_IDR_ID2) {
      xEventGroupSetBitsFromISR(extiEventGroupHandle, EXTI_SX__DIO0, &taskWoken);
      //xEventGroupSetBitsFromISR(loraEventGroupHandle, /*Lora_EGW__EXTI_DIO0*/ 0x00001000UL, &taskWoken);
    }
    break;

  case GPIO_PIN_3:
    /* Check for PG3 pin */
    if (GPIOG->IDR & GPIO_IDR_ID3) {
      xEventGroupSetBitsFromISR(extiEventGroupHandle, EXTI_SX__DIO1, &taskWoken);
    }
    break;

  default:
    { }
  }
}

#if 1
void  vApplicationIdleHook(void)
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
  /* TODO:
   * 1) Reduce 80 MHz  to  2 MHz
   * 2)  go to LPRun  (SMPS 2 High (-->  MR range 1) --> MR range 2 --> LPR
   * 3)  Go to LPSleep
   *
   * WAKEUP
   * 1)  In LPRun go to 80 MHz (LPR --> MR range 2 (--> MR range 1) --> SMPS 2 High)
   * 2)  Increase 2 MHz to 80 MHz
   */

  /* Enter sleep mode */
  __asm volatile( "WFI" );

  /* Increase clock frequency to 80 MHz */
  // TODO: TBD
}
#endif

#if 0
void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
#if 0
  HAL_SuspendTick();
#endif
  g_rtc_ssr_last = RTC->SSR;
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
#if 0
  volatile uint32_t l_rtc_ssr_now = RTC->SSR;
  volatile uint32_t l_rtc_sub1024 = (l_rtc_ssr_now >= g_rtc_ssr_last) ?  (l_rtc_ssr_now - g_rtc_ssr_last) : (1024UL - (g_rtc_ssr_last - l_rtc_ssr_now));
  volatile uint32_t l_millis = (l_rtc_sub1024 * 1000UL) / 1024UL;

  if (l_millis <= *ulExpectedIdleTime) {
    uwTick += l_millis;
  }
  HAL_ResumeTick();
#endif
}
#endif

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */

  /* Power switch settings */
  PowerSwitchInit();        // 32.0mA --> 28.0mA

  /* Si5338 clock generator */
#if 0
  /* Switch on Si5338 clock PLL */
  PowerSwitchDo(POWERSWITCH__3V3_HICUR, 1);
  osDelay(10);
# if 0
  i2cI2c4Si5338Init(I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ);
# else
  i2cI2c4Si5338Init(I2C_SI5338_CLKIN_VARIANT__MCU_MCO_8MHZ);
# endif
#endif

  /* LCD-backlight default settings */
  LcdBacklightInit();       // 28.0mA --> 30.0mA  !!!!

#ifdef I2C4_BUS_ADDR_SCAN
  i2cI2c4AddrScan();
#endif

#define SPI3_AX_CHECK
#ifdef SPI3_AX_CHECK
  if (HAL_OK == spiDetectAx5243()) {
    usbLog("AX5243 detected.\r\n");
  } else {
    usbLog("AX5243 does not respond!\r\n");
  }
#endif

  osDelay(850UL);

  /* Infinite loop */
  for(;;)
  {
    const uint32_t  eachMs              = 1000UL;
    static uint32_t sf_previousWakeTime = 0UL;
    int             dbgLen;
    char            dbgBuf[256];

    if (!sf_previousWakeTime) {
      sf_previousWakeTime  = osKernelSysTick();
    }

    /* Repeat each time period ADC conversion */
    osDelayUntil(&sf_previousWakeTime, eachMs);
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

  /* USER CODE END 5 */ 
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
  /* Infinite loop */
  sx1276TaskInit();

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
  /* Infinite loop */
  controllerTaskInit();

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
  /* Infinite loop */
  si5338TaskInit();

  for(;;)
  {
    si5338TaskLoop();
  }
  /* USER CODE END StartSi5338Task */
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
  int  dbgLen;
  char dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "***ERROR: ERROR-HANDLER  Wrong parameters value: file %s on line %d\r\n", file, line);
  usbLogLen(dbgBuf, dbgLen);

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
  int  dbgLen;
  char dbgBuf[128];

  dbgLen = sprintf(dbgBuf, "***ERROR: ASSERT-FAILED  Wrong parameters value: file %s on line %ld\r\n", file, line);
  usbLogLen(dbgBuf, dbgLen);
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
