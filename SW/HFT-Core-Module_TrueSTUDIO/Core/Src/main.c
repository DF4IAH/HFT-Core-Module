
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
#include <task_USB.h>
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


#define  PERIOD_VALUE       (uint32_t)(16000UL - 1)                                             /* Period Value = 1ms */
#define  PULSE_RED_VALUE    (uint32_t)(PERIOD_VALUE * 75 / 100)                                 /* Capture Compare 1 Value  */
#define  PULSE_GREEN_VALUE  (uint32_t)(PERIOD_VALUE * 25 / 100)                                 /* Capture Compare 2 Value  */
#define  PULSE_BLUE_VALUE   (uint32_t)(PERIOD_VALUE * 50 / 100)                                 /* Capture Compare 3 Value  */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if !defined(__MCO1_CLK_ENABLE)
# define __MCO1_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
# define __MCO1_CLK_DISABLE()         __HAL_RCC_GPIOA_CLK_DISABLE()
# define MCO1_GPIO_PORT               GPIOA
# define MCO1_PIN                     GPIO_PIN_8
#endif

/* Private variables ---------------------------------------------------------*/
extern EventGroupHandle_t             adcEventGroupHandle;
extern EventGroupHandle_t             extiEventGroupHandle;
extern EventGroupHandle_t             globalEventGroupHandle;
extern EventGroupHandle_t             spiEventGroupHandle;
extern EventGroupHandle_t             usbToHostEventGroupHandle;

extern uint32_t                       uwTick;

extern uint8_t                        i2c4TxBuffer[I2C_TXBUFSIZE];
extern uint8_t                        i2c4RxBuffer[I2C_RXBUFSIZE];

extern uint8_t                        spi3TxBuffer[SPI3_BUFFERSIZE];
extern uint8_t                        spi3RxBuffer[SPI3_BUFFERSIZE];

/* Private variables ---------------------------------------------------------*/
ENABLE_MASK_t                         g_enableMsk;
MON_MASK_t                            g_monMsk;
SYSCLK_CONFIG_t                       g_main_SYSCLK_CONFIG    = SYSCLK_CONFIG_04MHz_MSI;

/* Typical values for 24 MHz MSI configuration */
uint32_t                              g_main_HSE_VALUE        =   20000000UL;
uint32_t                              g_main_HSE_START_MS     =        100UL;
uint32_t                              g_main_MSI_VALUE        =    4000000UL;
uint32_t                              g_main_HSI_VALUE        =   16000000UL;
uint32_t                              g_main_HSI48_VALUE      =   48000000UL;
uint32_t                              g_main_LSI_VALUE        =      32000UL;
uint32_t                              g_main_LSE_VALUE        =      32768UL;
uint32_t                              g_main_LSE_START_MS     =       5000UL;
uint32_t                              g_main_i2c1_timing      = 0x00000000UL;
uint32_t                              g_main_i2c2_timing      = 0x00000000UL;
uint32_t                              g_main_i2c3_timing      = 0x00000000UL;
uint32_t                              g_main_i2c4_timing      = 0x00000000UL;

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

void mainPowerSwitchDo(POWERSWITCH_ENUM_t sw, uint8_t enable)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __asm volatile( "NOP" );

  switch (sw) {
  case POWERSWITCH__USB_SW:
    /* Port: PC2 */
    HAL_GPIO_WritePin(MCU_OUT_VUSB_EN_GPIO_Port, MCU_OUT_VUSB_EN_Pin, (enable ?  GPIO_PIN_SET : GPIO_PIN_RESET));
    break;

  case POWERSWITCH__3V3_HICUR:
    /* Port: PC1 */
    if (enable) {
      /* Connect chip select with SPI bus signal */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_ADC_SEL_GPIO_Port, MCU_OUT_AUDIO_ADC_SEL_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_DAC_SEL_GPIO_Port, MCU_OUT_AUDIO_DAC_SEL_Pin, GPIO_PIN_SET);

      /* Power switch: 3V3_HICUR */
      HAL_GPIO_WritePin(MCU_OUT_HICUR_EN_GPIO_Port, MCU_OUT_HICUR_EN_Pin, GPIO_PIN_SET);

      /* Disable RESET */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_ADC_nRESET_GPIO_Port, MCU_OUT_AUDIO_ADC_nRESET_Pin, GPIO_PIN_SET);

    } else {
      /* Enable RESET */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_ADC_nRESET_GPIO_Port, MCU_OUT_AUDIO_ADC_nRESET_Pin, GPIO_PIN_RESET);

      /* Power switch: 3V3_HICUR */
      HAL_GPIO_WritePin(MCU_OUT_HICUR_EN_GPIO_Port, MCU_OUT_HICUR_EN_Pin, GPIO_PIN_RESET);

      /* No energy through select pins during power off */
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_ADC_SEL_GPIO_Port, MCU_OUT_AUDIO_ADC_SEL_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MCU_OUT_AUDIO_DAC_SEL_GPIO_Port, MCU_OUT_AUDIO_DAC_SEL_Pin, GPIO_PIN_RESET);
    }
    break;

  case POWERSWITCH__3V3_XO:
    /* Port: PC3 */
    if (enable) {
      HAL_GPIO_WritePin(MCU_OUT_20MHZ_EN_GPIO_Port, MCU_OUT_20MHZ_EN_Pin, GPIO_PIN_SET);

    } else {
      HAL_GPIO_WritePin(MCU_OUT_20MHZ_EN_GPIO_Port, MCU_OUT_20MHZ_EN_Pin, GPIO_PIN_RESET);
    }
    break;

  case POWERSWITCH__1V2_DCDC:
    /* V1.0: no hardware support */
    return;

  case POWERSWITCH__1V2_SW:
    /* SMPS handling */
    if (enable)
    {
      /* Port: PC0 */
      HAL_GPIO_WritePin(MCU_OUT_VDD12_EN_GPIO_Port, MCU_OUT_VDD12_EN_Pin, GPIO_PIN_SET);

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
      HAL_GPIO_WritePin(MCU_OUT_VDD12_EN_GPIO_Port, MCU_OUT_VDD12_EN_Pin, GPIO_PIN_RESET);
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

void mainPowerSwitchInit(void)
{
  /* Connect Vusb with +5V0 */
  mainPowerSwitchDo(POWERSWITCH__USB_SW,    1U);

  /* Disable high current system */
  mainPowerSwitchDo(POWERSWITCH__3V3_HICUR, 0U);

  /* Disable TCXO - enabled by i2cI2c4Si5338Init() on request */
  mainPowerSwitchDo(POWERSWITCH__3V3_XO,    0U);

  /* Enable SMPS */
  {
//  PowerSwitchDo(POWERSWITCH__1V2_DCDC,
//      1U);
//  for (uint16_t i = 10000; i; i--) ;
    mainPowerSwitchDo(POWERSWITCH__1V2_SW,  1U);

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
  mainPowerSwitchDo(POWERSWITCH__BAT_HICUR, 1U);
}

void LcdBacklightInit(void)
{
  TIM_HandleTypeDef   TimHandle;
  TIM_OC_InitTypeDef  sConfig;

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

  // During RESET: 26.0mA @ 3.3V

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

  // Here: 25.5mA @ 3.3V

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Here: 26.0mA @ 3.3V

  #ifdef STD_INIT
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  #else
  HFT_SystemClock_Config(SYSCLK_CONFIG_16MHz_MSI);
  #endif

  // Here: 28.5mA @ 3.3V

  /* HSI16 trim */
  __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(0x3f);                                                        // 0x40 centered

  /* MSI trim */
  //__HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(0x00);                                                      // Signed
  HAL_RCCEx_EnableMSIPLLMode();

  #ifdef STD_INIT
  // Here: 28.5mA @ 3.3V

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

  /* Copy of above for documentation */
  //MX_GPIO_Init();                                                                                   // Here: 27.5mA @ 3.3V    -0.5mA
  //MX_DMA_Init();                                                                                    // Here: 28.0mA @ 3.3V    +0.5mA
  //MX_ADC1_Init();                                                                                   // Here: 28.0mA @ 3.3V
  //MX_ADC3_Init();                                                                                   // Here: 28.0mA @ 3.3V
  //MX_CRC_Init();                                                                                    // Here: 28.0mA @ 3.3V
  //MX_DFSDM1_Init();                                                                                 // Here: 28.0mA @ 3.3V
  //MX_I2C1_Init();                                                                                   // Here: 28.5mA @ 3.3V    +0.5mA
  //MX_I2C2_Init();                                                                                   // Here: 29.0mA @ 3.3V    +0.5mA
  //MX_I2C3_Init();                                                                                   // Here: 29.0mA @ 3.3V
  //MX_I2C4_Init();                                                                                   // Here: 29.0mA @ 3.3V
  //MX_RNG_Init();                                                                                    // Here: 29.5mA @ 3.3V    +0.5mA
  //MX_RTC_Init();                                                                                    // Here: 29.5mA @ 3.3V
  //MX_SAI1_Init();                                                                                   // Here: 30.0mA @ 3.3V    +0.5mA
  //MX_SAI2_Init();                                                                                   // Here: 30.0mA @ 3.3V
  //MX_SPI1_Init();                                                                                   // Here: 31.0mA @ 3.3V    +1.0mA
  //MX_SPI3_Init();                                                                                   // Here: 31.0mA @ 3.3V
  //MX_TIM16_Init();                                                                                  // Here: 31.0mA @ 3.3V
  //MX_TIM17_Init();                                                                                  // Here: 31.0mA @ 3.3V
  //MX_TIM3_Init();                                                                                   // Here: 31.0mA @ 3.3V
  //MX_TIM5_Init();                                                                                   // Here: 31.0mA @ 3.3V
  //MX_USART1_UART_Init();                                                                            // Here: 31.5mA @ 3.3V    +0.5mA
  //MX_USART2_UART_Init();                                                                            // Here: 32.0mA @ 3.3V    +0.5mA
  //MX_USART3_UART_Init();                                                                            // Here: 32.0mA @ 3.3V

  //#define GPIO_AUDIO_DAC_SEL_OFF
  #ifdef GPIO_AUDIO_DAC_SEL_OFF
  gpio_AudioAdc_TurnOffSel();
  #endif

  #else

  /* Low energy set-up */
                                                                                                      // Here: 28.5mA @ 3.3V
  MX_GPIO_Init();                                                                                     // Here: 28.0mA @ 3.3V
  MX_DMA_Init();                                                                                      // Here: 28.0mA @ 3.3V
  MX_RTC_Init();                                                                                      // Here: 28.0mA @ 3.3V

  #define GPIO_ABCDEFGH_CLK_DISABLE
  #ifdef GPIO_ABCDEFGH_CLK_DISABLE
  /* Disable clocks again to save power */
  gpio_ABCDEFGH_ClkDisable();                                                                         // Here: 27.0mA @ 3.3V
  #endif

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV20;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 4;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV16;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_8);

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

void HFT_SystemClock_Config(SYSCLK_CONFIG_t sel)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_CRSInitTypeDef RCC_CRSInitStruct;

  /* Set global variable */
  g_main_SYSCLK_CONFIG = sel;

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);


  switch (sel) {
  case SYSCLK_CONFIG_01MHz_MSI_HSI:
    {
      /* Set global variables */
      {
        g_main_MSI_VALUE        =    1000000UL;
        g_main_HSI_VALUE        =   16000000UL;
      //g_main_HSI48_VALUE      =   48000000UL;
        g_main_LSE_VALUE        =      32768UL;
        g_main_LSE_START_MS     =       5000UL;

        g_main_i2c1_timing      = 0x00300711UL;
        g_main_i2c2_timing      = 0x00300711UL;
        g_main_i2c3_timing      = 0x00300711UL;
        g_main_i2c4_timing      = 0x00300711UL;
      }

      /**Initializes the CPU, AHB and APB busses clocks
      */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                                  |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
      RCC_OscInitStruct.HSIState = RCC_HSI_ON;
      RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
      RCC_OscInitStruct.HSICalibrationValue = 64;
      RCC_OscInitStruct.MSIState = RCC_MSI_ON;
      RCC_OscInitStruct.MSICalibrationValue = 0;
      RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
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

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
      PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
      PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
      PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
      PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_HSI;
      PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
      PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
      PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
      PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
      PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
      PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
      PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
      PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
      PeriphClkInit.PLLSAI1.PLLSAI1M = 4;
      PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
      PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV16;
      PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
        _Error_Handler(__FILE__, __LINE__);
      }

      HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

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
    }
    break;

  case SYSCLK_CONFIG_04MHz_MSI:
    {
      /* Set global variables */
      {
        g_main_MSI_VALUE        =    4000000UL;
      //g_main_HSI48_VALUE      =   48000000UL;
        g_main_LSE_VALUE        =      32768UL;
        g_main_LSE_START_MS     =       5000UL;

        g_main_i2c1_timing      = 0x00000000UL;
        g_main_i2c2_timing      = 0x00000000UL;
        g_main_i2c3_timing      = 0x00000000UL;
        g_main_i2c4_timing      = 0x00000000UL;
      }

      /**Initializes the CPU, AHB and APB busses clocks
      */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                                  |RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
      RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
      RCC_OscInitStruct.MSIState = RCC_MSI_ON;
      RCC_OscInitStruct.MSICalibrationValue = 0;
      RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
      PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
      PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
      PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV4;
      PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
      PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
      PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
        _Error_Handler(__FILE__, __LINE__);
      }

      HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

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
    }
    break;

  case SYSCLK_CONFIG_08MHz_MSI:
    {
      /* Set global variables */
      {
        g_main_MSI_VALUE        =    8000000UL;
      //g_main_HSI48_VALUE      =   48000000UL;
        g_main_LSE_VALUE        =      32768UL;
        g_main_LSE_START_MS     =       5000UL;

        g_main_i2c1_timing      = 0x00100006UL;
        g_main_i2c2_timing      = 0x00100006UL;
        g_main_i2c3_timing      = 0x00100006UL;
        g_main_i2c4_timing      = 0x00100006UL;
      }

     /**Initializes the CPU, AHB and APB busses clocks
     */
     RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                                 |RCC_OSCILLATORTYPE_MSI;
     RCC_OscInitStruct.LSEState = RCC_LSE_ON;
     RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
     RCC_OscInitStruct.MSIState = RCC_MSI_ON;
     RCC_OscInitStruct.MSICalibrationValue = 0;
     RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
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

     if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
     PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
     PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
     PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV16;
     PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
     PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
     PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
     {
       _Error_Handler(__FILE__, __LINE__);
     }

     HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

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
    }
    break;

  case SYSCLK_CONFIG_16MHz_MSI:
    {
      g_main_MSI_VALUE        =   16000000UL;
      g_main_HSI48_VALUE      =   48000000UL;
      g_main_LSE_VALUE        =      32768UL;
      g_main_LSE_START_MS     =       5000UL;

      g_main_i2c1_timing      = 0x00300711UL;
      g_main_i2c2_timing      = 0x00300711UL;
      g_main_i2c3_timing      = 0x00300711UL;
      g_main_i2c4_timing      = 0x00300711UL;

      /**Initializes the CPU, AHB and APB busses clocks
      */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                                  |RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
      RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
      RCC_OscInitStruct.MSIState = RCC_MSI_ON;
      RCC_OscInitStruct.MSICalibrationValue = 0;
      RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
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

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
      PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
      PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
      PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV16;
      PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
        _Error_Handler(__FILE__, __LINE__);
      }

      HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

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
    }
    break;

  default:
  case SYSCLK_CONFIG_24MHz_MSI:
    {
      /* Set global variables */
      {
        g_main_MSI_VALUE        =   24000000UL;
        g_main_HSI48_VALUE      =   48000000UL;
        g_main_LSE_VALUE        =      32768UL;
        g_main_LSE_START_MS     =       5000UL;

        g_main_i2c1_timing      = 0x00500822UL;
        g_main_i2c2_timing      = 0x00500822UL;
        g_main_i2c3_timing      = 0x00500822UL;
        g_main_i2c4_timing      = 0x00500822UL;
      }

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
    }
    break;

  case SYSCLK_CONFIG_80MHz_HSI_PLL:
    {
      /* Set global variables */
      {
        g_main_HSI_VALUE        =   16000000UL;
        g_main_HSI48_VALUE      =   48000000UL;
        g_main_LSE_VALUE        =      32768UL;
        g_main_LSE_START_MS     =       5000UL;

        g_main_i2c1_timing      = 0x00300711UL;
        g_main_i2c2_timing      = 0x00300711UL;
        g_main_i2c3_timing      = 0x00300711UL;
        g_main_i2c4_timing      = 0x00300711UL;
      }

      /**Configure LSE Drive Capability
      */
      HAL_PWR_EnableBkUpAccess();

      __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

      /**Initializes the CPU, AHB and APB busses clocks
      */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                                  |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
      RCC_OscInitStruct.HSIState = RCC_HSI_ON;
      RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
      RCC_OscInitStruct.HSICalibrationValue = 64;
      RCC_OscInitStruct.MSIState = RCC_MSI_ON;
      RCC_OscInitStruct.MSICalibrationValue = 0;
      RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
      RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
      RCC_OscInitStruct.PLL.PLLM = 4;
      RCC_OscInitStruct.PLL.PLLN = 40;
      RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV20;
      RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
      RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
        _Error_Handler(__FILE__, __LINE__);
      }

      /**Initializes the CPU, AHB and APB busses clocks
      */
      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
      PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
      PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
      PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
      PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
      PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
      PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
      PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_HSI;
      PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
      PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
      PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
      PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
      PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
      PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
      PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
      PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
      PeriphClkInit.PLLSAI1.PLLSAI1M = 4;
      PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
      PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV16;
      PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
        _Error_Handler(__FILE__, __LINE__);
      }

      HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_8);

      /**Configure the main internal regulator output voltage
      */
      if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
    }
    break;

  case SYSCLK_CONFIG_80MHz_HSE20_PLL:
    {
      /* Set global variables */
      {
        g_main_HSE_VALUE        =   20000000UL;
        g_main_HSE_START_MS     =        100UL;
        g_main_HSI48_VALUE      =   48000000UL;
        g_main_LSE_VALUE        =      32768UL;
        g_main_LSE_START_MS     =       5000UL;

        g_main_i2c1_timing      = 0x0080103EUL;
        g_main_i2c2_timing      = 0x0080103EUL;
        g_main_i2c3_timing      = 0x0080103EUL;
        g_main_i2c4_timing      = 0x0080103EUL;
      }

      /**Initializes the CPU, AHB and APB busses clocks
      */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE
                                  |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
      RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
      RCC_OscInitStruct.MSIState = RCC_MSI_ON;
      RCC_OscInitStruct.MSICalibrationValue = 0;
      RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
      RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
      RCC_OscInitStruct.PLL.PLLM = 4;
      RCC_OscInitStruct.PLL.PLLN = 32;
      RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV20;
      RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
      RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
        _Error_Handler(__FILE__, __LINE__);
      }

      /**Initializes the CPU, AHB and APB busses clocks
      */
      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
      PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
      PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
      PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
      PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
      PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
      PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
      PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
      PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
      PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
      PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
      PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
      PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
      PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
      PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
      PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
      PeriphClkInit.PLLSAI1.PLLSAI1M = 4;
      PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
      PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV16;
      PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
      PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
        _Error_Handler(__FILE__, __LINE__);
      }

      HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_8);

      /**Configure the main internal regulator output voltage
      */
      if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
    }
    break;
  }

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

void HFT_RCC_MCO_Disable(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* MCO Clock Enable */
  __MCO1_CLK_ENABLE();

  /* Mask MCOSEL[] and MCOPRE[] bits then set MCO1 clock source and prescaler */
  MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCOSEL), (0b000UL << RCC_CFGR_MCOSEL_Pos));                         // Disable MCO clock

  /* Configure the MCO1 pin as analog input with pulldown */
  GPIO_InitStruct.Pin = MCO1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  //GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

  /* MCO Clock Disable */
  __MCO1_CLK_DISABLE();
}


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
