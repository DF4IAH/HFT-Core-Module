
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
#include "adc.h"
#include "crc.h"
#include "dfsdm.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "sai.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

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
#include "task_Audio_ADC.h"
#include "task_Audio_DAC.h"
#include "task_AX5243.h"
#include "task_SX1276.h"
#include "usb.h"


#define  PERIOD_VALUE       (uint32_t)(16000UL - 1)                                             /* Period Value = 1ms */
#define  PULSE_RED_VALUE    (uint32_t)(PERIOD_VALUE * 75 / 100)                                 /* Capture Compare 1 Value  */
#define  PULSE_GREEN_VALUE  (uint32_t)(PERIOD_VALUE * 25 / 100)                                 /* Capture Compare 2 Value  */
#define  PULSE_BLUE_VALUE   (uint32_t)(PERIOD_VALUE * 50 / 100)                                 /* Capture Compare 3 Value  */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint32_t                       uwTick;

extern uint8_t                        i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t                        i2c4RxBuffer[I2C_RXBUFSIZE];

extern uint8_t                        spi3TxBuffer[SPI3_BUFFERSIZE];
extern uint8_t                        spi3RxBuffer[SPI3_BUFFERSIZE];

/* Private variables ---------------------------------------------------------*/
EventGroupHandle_t                    adcEventGroupHandle;
EventGroupHandle_t                    extiEventGroupHandle;
EventGroupHandle_t                    globalEventGroupHandle;
EventGroupHandle_t                    spiEventGroupHandle;
EventGroupHandle_t                    usbToHostEventGroupHandle;

osSemaphoreId                         usbToHostBinarySemHandle;

/* Timer handler declaration */
TIM_HandleTypeDef                     TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef                    sConfig;

ENABLE_MASK_t                         g_enableMsk;
MON_MASK_t                            g_monMsk;

static uint8_t                        s_adc_enable;
static uint32_t                       s_mainStartTime;

/* Counter Prescaler value */
uint32_t                              uhPrescalerValue        = 0;

volatile uint32_t                     g_rtc_ssr_last          = 0UL;

static uint64_t                       s_timerLast_us          = 0ULL;
static uint64_t                       s_timerStart_us         = 0ULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t crcCalc(const uint32_t* ptr, uint32_t len)
{
  return HAL_CRC_Calculate(&hcrc, (uint32_t*) ptr, len);
}


uint8_t sel_u8_from_u32(uint32_t in_u32, uint8_t sel)
{
  return 0xff & (in_u32 >> (sel << 3));
}

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
    HAL_GPIO_WritePin(MCU_OUT_VUSB_EN_GPIO_Port, MCU_OUT_VUSB_EN_Pin,
        (enable ?  GPIO_PIN_SET : GPIO_PIN_RESET));
    break;

  case POWERSWITCH__3V3_HICUR:
    /* Port: PC1 */
    HAL_GPIO_WritePin(MCU_OUT_HICUR_EN_GPIO_Port, MCU_OUT_HICUR_EN_Pin,
        (enable ?  GPIO_PIN_SET : GPIO_PIN_RESET));
    break;

  case POWERSWITCH__3V3_XO:
    /* Port: PC3 */
    HAL_GPIO_WritePin(MCU_OUT_20MHZ_EN_GPIO_Port, MCU_OUT_20MHZ_EN_Pin,
        (enable ?  GPIO_PIN_SET : GPIO_PIN_RESET));
    break;

  case POWERSWITCH__1V2_DCDC:
    /* V1.0: no hardware support */
    return;

  case POWERSWITCH__1V2_SW:
    /* SMPS handling */
    if (enable)
    {
      /* Port: PC0 */
      HAL_GPIO_WritePin(MCU_OUT_VDD12_EN_GPIO_Port, MCU_OUT_VDD12_EN_Pin,
          GPIO_PIN_SET);

      osDelay(10UL);

      /* Scale1: 1.2V up to 80MHz */
      /* Scale2: 1.0V up to 24MHz */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

    } else {
      /* Scale1: 1.2V up to 80MHz */
      /* Scale2: 1.0V up to 24MHz */
      HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

      osDelay(100UL);

      /* Port: PC0 */
      HAL_GPIO_WritePin(MCU_OUT_VDD12_EN_GPIO_Port, MCU_OUT_VDD12_EN_Pin,
          GPIO_PIN_RESET);
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
  PowerSwitchDo(POWERSWITCH__USB_SW,    1U);

  /* Disable high current system */
  PowerSwitchDo(POWERSWITCH__3V3_HICUR, 0U);

  /* Disable TCXO - enabled by i2cI2c4Si5338Init() on request */
  PowerSwitchDo(POWERSWITCH__3V3_XO,    0U);

  /* Enable SMPS */
  {
//  PowerSwitchDo(POWERSWITCH__1V2_DCDC,
//      1U);
//  for (uint16_t i = 10000; i; i--) ;
    PowerSwitchDo(POWERSWITCH__1V2_SW,  1U);

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
  PowerSwitchDo(POWERSWITCH__BAT_HICUR, 1U);
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
  __HAL_RCC_CLEAR_RESET_FLAGS();

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
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  #define GPIO_AUDIO_DAC_SEL_OFF
  #ifdef GPIO_AUDIO_DAC_SEL_OFF
  gpio_AudioAdc_TurnOffSel();
  #endif

  //#define GPIO_ABCDEFGH_CLK_DISABLE
  #ifdef GPIO_ABCDEFGH_CLK_DISABLE
  /* Disable clocks again to save power */
  gpio_ABCDEFGH_ClkDisable();
  #endif


  //#define I2C_BUS4_SCAN
  #ifdef I2C_BUS4_SCAN
  i2cBusAddrScan(&hi2c4, i2c4MutexHandle);
  #endif

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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

static void mainDefaultInit(void)
{
  /* Activate USB communication */
  HFTcore_USB_DEVICE_Init();

  /* Power switch settings */
  PowerSwitchInit();
}


static void mainMsgProcess(uint32_t msgLen, const uint32_t* msgAry)
{
  uint32_t                msgIdx  = 0UL;
  const uint32_t          hdr     = msgAry[msgIdx++];
  const MainMsgMainCmds_t cmd     = (MainMsgMainCmds_t) (0xffUL & hdr);

  switch (cmd) {
  case MsgMain__InitDo:
    {
      /* Start at defined point of time */
      const uint32_t delayMs = msgAry[msgIdx++];
      if (delayMs) {
        uint32_t  previousWakeTime = s_mainStartTime;
        osDelayUntil(&previousWakeTime, delayMs);
      }

      /* Init module */
      mainDefaultInit();

      /* Return Init confirmation */
      uint32_t cmdBack[1];
      cmdBack[0] = controllerCalcMsgHdr(Destinations__Controller, Destinations__Main_Default, 0U, MsgMain__InitDone);
      controllerMsgPushToInQueue(sizeof(cmdBack) / sizeof(int32_t), cmdBack, osWaitForever);
    }
    break;

  /* ADC single conversion */
  case MsgMain__CallFunc01_MCU_ADC:
    {
      int   dbgLen;
      char  dbgBuf[128];

      /* Do ADC conversion and logging of ADC data */
      if (s_adc_enable) {
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

/* USER CODE END 4 */

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
