/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <sys/_stdint.h>

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MCU_INOUT_PW03_Pin GPIO_PIN_2
#define MCU_INOUT_PW03_GPIO_Port GPIOE
#define I2S1B_SD_OUT_Pin GPIO_PIN_3
#define I2S1B_SD_OUT_GPIO_Port GPIOE
#define MCU_INOUT_PW02_Pin GPIO_PIN_4
#define MCU_INOUT_PW02_GPIO_Port GPIOE
#define MCU_INOUT_PW01_Pin GPIO_PIN_5
#define MCU_INOUT_PW01_GPIO_Port GPIOE
#define MCU_IN_WAKEUP_Pin GPIO_PIN_13
#define MCU_IN_WAKEUP_GPIO_Port GPIOC
#define MCU_OSC32_IN_Pin GPIO_PIN_14
#define MCU_OSC32_IN_GPIO_Port GPIOC
#define MCU_OSC32_OUT_Pin GPIO_PIN_15
#define MCU_OSC32_OUT_GPIO_Port GPIOC
#define MCU_I2C2_SDA_Pin GPIO_PIN_0
#define MCU_I2C2_SDA_GPIO_Port GPIOF
#define MCU_I2C2_SCL_Pin GPIO_PIN_1
#define MCU_I2C2_SCL_GPIO_Port GPIOF
#define MCU_INOUT_PW00_Pin GPIO_PIN_2
#define MCU_INOUT_PW00_GPIO_Port GPIOF
#define ADC3_IN6_20MHZ_PULL_Pin GPIO_PIN_3
#define ADC3_IN6_20MHZ_PULL_GPIO_Port GPIOF
#define MCU_IN_TIMCAP_N_Pin GPIO_PIN_6
#define MCU_IN_TIMCAP_N_GPIO_Port GPIOF
#define MCU_IN_TIMCAP_W_Pin GPIO_PIN_7
#define MCU_IN_TIMCAP_W_GPIO_Port GPIOF
#define MCU_IN_TIMCAP_S_Pin GPIO_PIN_8
#define MCU_IN_TIMCAP_S_GPIO_Port GPIOF
#define MCU_IN_TIMCAP_VSOL_Pin GPIO_PIN_9
#define MCU_IN_TIMCAP_VSOL_GPIO_Port GPIOF
#define MCU_EVENTOUT_PF10_Pin GPIO_PIN_10
#define MCU_EVENTOUT_PF10_GPIO_Port GPIOF
#define MCU_HSE_Pin GPIO_PIN_0
#define MCU_HSE_GPIO_Port GPIOH
#define MCU_OUT_VDD12_EN_Pin GPIO_PIN_0
#define MCU_OUT_VDD12_EN_GPIO_Port GPIOC
#define MCU_OUT_HICUR_EN_Pin GPIO_PIN_1
#define MCU_OUT_HICUR_EN_GPIO_Port GPIOC
#define MCU_OUT_VUSB_EN_Pin GPIO_PIN_2
#define MCU_OUT_VUSB_EN_GPIO_Port GPIOC
#define MCU_OUT_20MHZ_EN_Pin GPIO_PIN_3
#define MCU_OUT_20MHZ_EN_GPIO_Port GPIOC
#define UART2_CTS_IN_Pin GPIO_PIN_0
#define UART2_CTS_IN_GPIO_Port GPIOA
#define UART2_TX_OUT_Pin GPIO_PIN_2
#define UART2_TX_OUT_GPIO_Port GPIOA
#define MCU_VSOL_ADC1_Pin GPIO_PIN_3
#define MCU_VSOL_ADC1_GPIO_Port GPIOA
#define UART3_CTS_IN_Pin GPIO_PIN_6
#define UART3_CTS_IN_GPIO_Port GPIOA
#define MCU_OUT_PWM_LED_G_Pin GPIO_PIN_7
#define MCU_OUT_PWM_LED_G_GPIO_Port GPIOA
#define UART3_TX_OUT_Pin GPIO_PIN_4
#define UART3_TX_OUT_GPIO_Port GPIOC
#define UART3_RX_IN_Pin GPIO_PIN_5
#define UART3_RX_IN_GPIO_Port GPIOC
#define MCU_OUT_PWM_LED_B_Pin GPIO_PIN_0
#define MCU_OUT_PWM_LED_B_GPIO_Port GPIOB
#define UART3_RTS_OUT_Pin GPIO_PIN_1
#define UART3_RTS_OUT_GPIO_Port GPIOB
#define MCU_OUT_LCD_nRST_Pin GPIO_PIN_12
#define MCU_OUT_LCD_nRST_GPIO_Port GPIOF
#define MCU_I2C4_SCL_Pin GPIO_PIN_14
#define MCU_I2C4_SCL_GPIO_Port GPIOF
#define MCU_OUT_SX_HF_LF_CTRL_Pin GPIO_PIN_0
#define MCU_OUT_SX_HF_LF_CTRL_GPIO_Port GPIOG
#define MCU_IN_AX_GPIO1_Pin GPIO_PIN_1
#define MCU_IN_AX_GPIO1_GPIO_Port GPIOG
#define MCU_IN_AX_IRQ_Pin GPIO_PIN_7
#define MCU_IN_AX_IRQ_GPIO_Port GPIOE
#define I2S1B_SCK_OUT_Pin GPIO_PIN_8
#define I2S1B_SCK_OUT_GPIO_Port GPIOE
#define I2S1B_FS_OUT_Pin GPIO_PIN_9
#define I2S1B_FS_OUT_GPIO_Port GPIOE
#define I2S1B_MCLK_OUT_Pin GPIO_PIN_10
#define I2S1B_MCLK_OUT_GPIO_Port GPIOE
#define MCU_OUT_AX_SEL_Pin GPIO_PIN_11
#define MCU_OUT_AX_SEL_GPIO_Port GPIOE
#define MCU_OUT_SX_SEL_Pin GPIO_PIN_12
#define MCU_OUT_SX_SEL_GPIO_Port GPIOE
#define MCU_SPI1_SCK_Pin GPIO_PIN_13
#define MCU_SPI1_SCK_GPIO_Port GPIOE
#define MCU_SPI1_MISO_Pin GPIO_PIN_14
#define MCU_SPI1_MISO_GPIO_Port GPIOE
#define MCU_SPI1_MOSI_Pin GPIO_PIN_15
#define MCU_SPI1_MOSI_GPIO_Port GPIOE
#define MCU_IN_RE_I_Pin GPIO_PIN_10
#define MCU_IN_RE_I_GPIO_Port GPIOB
#define MCU_IN_RE_Q_Pin GPIO_PIN_12
#define MCU_IN_RE_Q_GPIO_Port GPIOB
#define I2S2A_SCK_IN_Pin GPIO_PIN_13
#define I2S2A_SCK_IN_GPIO_Port GPIOB
#define MCU_IN_RE_PB_Pin GPIO_PIN_14
#define MCU_IN_RE_PB_GPIO_Port GPIOB
#define MCU_OUT_SX_nRESET_Pin GPIO_PIN_8
#define MCU_OUT_SX_nRESET_GPIO_Port GPIOD
#define MCU_INOUT_PS00_Pin GPIO_PIN_9
#define MCU_INOUT_PS00_GPIO_Port GPIOD
#define MCU_INOUT_PS01_Pin GPIO_PIN_10
#define MCU_INOUT_PS01_GPIO_Port GPIOD
#define I2S2A_SD_IN_Pin GPIO_PIN_11
#define I2S2A_SD_IN_GPIO_Port GPIOD
#define I2S2A_FS_IN_Pin GPIO_PIN_12
#define I2S2A_FS_IN_GPIO_Port GPIOD
#define MCU_I2C4_SDA_Pin GPIO_PIN_13
#define MCU_I2C4_SDA_GPIO_Port GPIOD
#define MCU_INOUT_PS02_Pin GPIO_PIN_14
#define MCU_INOUT_PS02_GPIO_Port GPIOD
#define MCU_INOUT_PS03_Pin GPIO_PIN_15
#define MCU_INOUT_PS03_GPIO_Port GPIOD
#define MCU_IN_EXTI2_SX_DIO0_TXRXDONE_Pin GPIO_PIN_2
#define MCU_IN_EXTI2_SX_DIO0_TXRXDONE_GPIO_Port GPIOG
#define MCU_IN_EXTI2_SX_DIO0_TXRXDONE_EXTI_IRQn EXTI2_IRQn
#define MCU_IN_EXTI3_SX_DIO1_RXTO_Pin GPIO_PIN_3
#define MCU_IN_EXTI3_SX_DIO1_RXTO_GPIO_Port GPIOG
#define MCU_IN_EXTI3_SX_DIO1_RXTO_EXTI_IRQn EXTI3_IRQn
#define MCU_EVENTOUT_PG4_Pin GPIO_PIN_4
#define MCU_EVENTOUT_PG4_GPIO_Port GPIOG
#define MCU_I2C3_SCL_Pin GPIO_PIN_7
#define MCU_I2C3_SCL_GPIO_Port GPIOG
#define MCU_I2C3_SDA_Pin GPIO_PIN_8
#define MCU_I2C3_SDA_GPIO_Port GPIOG
#define MCU_OUT_PWM_LED_R_Pin GPIO_PIN_6
#define MCU_OUT_PWM_LED_R_GPIO_Port GPIOC
#define MCU_IN_AUDIO_DAC_nRDY_Pin GPIO_PIN_8
#define MCU_IN_AUDIO_DAC_nRDY_GPIO_Port GPIOC
#define MCU_OUT_AUDIO_DAC_SEL_Pin GPIO_PIN_9
#define MCU_OUT_AUDIO_DAC_SEL_GPIO_Port GPIOC
#define MCU_MCO_Pin GPIO_PIN_8
#define MCU_MCO_GPIO_Port GPIOA
#define MCU_IN_VBUS_Pin GPIO_PIN_9
#define MCU_IN_VBUS_GPIO_Port GPIOA
#define PA_USB_N_Pin GPIO_PIN_11
#define PA_USB_N_GPIO_Port GPIOA
#define PA_USB_P_Pin GPIO_PIN_12
#define PA_USB_P_GPIO_Port GPIOA
#define MCU_INOUT_SWDIO_Pin GPIO_PIN_13
#define MCU_INOUT_SWDIO_GPIO_Port GPIOA
#define MCU_IN_SWCLK_Pin GPIO_PIN_14
#define MCU_IN_SWCLK_GPIO_Port GPIOA
#define MCU_SPI3_SCK_Pin GPIO_PIN_10
#define MCU_SPI3_SCK_GPIO_Port GPIOC
#define MCU_SPI3_MISO_Pin GPIO_PIN_11
#define MCU_SPI3_MISO_GPIO_Port GPIOC
#define MCU_SPI3_MOSI_Pin GPIO_PIN_12
#define MCU_SPI3_MOSI_GPIO_Port GPIOC
#define MCU_IN_AUDIO_ADC_MDAT1_Pin GPIO_PIN_0
#define MCU_IN_AUDIO_ADC_MDAT1_GPIO_Port GPIOD
#define MCU_IN_AUDIO_ADC_MDAT0_Pin GPIO_PIN_1
#define MCU_IN_AUDIO_ADC_MDAT0_GPIO_Port GPIOD
#define MCU_IN_AUDIO_ADC_nDR_Pin GPIO_PIN_2
#define MCU_IN_AUDIO_ADC_nDR_GPIO_Port GPIOD
#define MCU_OUT_AUDIO_ADC_nRESET_Pin GPIO_PIN_3
#define MCU_OUT_AUDIO_ADC_nRESET_GPIO_Port GPIOD
#define UART2_RTS_OUT_Pin GPIO_PIN_4
#define UART2_RTS_OUT_GPIO_Port GPIOD
#define MCU_OUT_AUDIO_ADC_SEL_Pin GPIO_PIN_5
#define MCU_OUT_AUDIO_ADC_SEL_GPIO_Port GPIOD
#define UART2_RX_IN_Pin GPIO_PIN_6
#define UART2_RX_IN_GPIO_Port GPIOD
#define MCU_IN_EXTI7_INTR_SI5338_Pin GPIO_PIN_7
#define MCU_IN_EXTI7_INTR_SI5338_GPIO_Port GPIOD
#define UART1_TX_OUT_Pin GPIO_PIN_9
#define UART1_TX_OUT_GPIO_Port GPIOG
#define UART1_RX_IN_Pin GPIO_PIN_10
#define UART1_RX_IN_GPIO_Port GPIOG
#define UART1_CTS_IN_Pin GPIO_PIN_11
#define UART1_CTS_IN_GPIO_Port GPIOG
#define UART1_RTS_OUT_Pin GPIO_PIN_12
#define UART1_RTS_OUT_GPIO_Port GPIOG
#define MCU_I2C1_SDA_Pin GPIO_PIN_13
#define MCU_I2C1_SDA_GPIO_Port GPIOG
#define MCU_I2C1_SCL_Pin GPIO_PIN_14
#define MCU_I2C1_SCL_GPIO_Port GPIOG
#define MCU_INOUT_PN00_Pin GPIO_PIN_3
#define MCU_INOUT_PN00_GPIO_Port GPIOB
#define MCU_INOUT_PN01_Pin GPIO_PIN_5
#define MCU_INOUT_PN01_GPIO_Port GPIOB
#define MCU_INOUT_PN02_Pin GPIO_PIN_6
#define MCU_INOUT_PN02_GPIO_Port GPIOB
#define MCU_INOUT_PN03_Pin GPIO_PIN_7
#define MCU_INOUT_PN03_GPIO_Port GPIOB
#define MCU_EVENTOUT_PE0_Pin GPIO_PIN_0
#define MCU_EVENTOUT_PE0_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE                                           (g_main_HSE_VALUE)                      // Value of the External oscillator in Hz
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT                                 (g_main_HSE_START_MS)                   // Time out for HSE start up, in ms
#endif /* HSE_STARTUP_TIMEOUT */

#if !defined  (MSI_VALUE)
  #define MSI_VALUE                                           (g_main_MSI_VALUE)                      // Value of the Internal oscillator in Hz
#endif /* MSI_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE                                           (g_main_HSI_VALUE)                      // Value of the Internal oscillator in Hz
#endif /* HSI_VALUE */

#if !defined  (HSI48_VALUE)
  #define HSI48_VALUE                                         (g_main_HSI48_VALUE)                    // Value of the Internal High Speed oscillator for USB FS/SDMMC/RNG in Hz. The real value my vary depending on manufacturing process variations.
#endif /* HSI48_VALUE */

#if !defined  (LSI_VALUE)
  #define LSI_VALUE                                           (g_main_LSI_VALUE)                      // LSI Typical Value in Hz
#endif /* LSI_VALUE */

#if !defined  (LSE_VALUE)
  #define LSE_VALUE                                           (g_main_LSE_VALUE)                      // Value of the External oscillator in Hz
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT                                 (g_main_LSE_START_MS)                 // Time out for LSE start up, in ms
#endif /* LSE_STARTUP_TIMEOUT */


#ifndef PI
# define PI                                                   3.14159265358979f
#endif

#ifndef min
# define min(a,b)                                             (a) < (b) ?  (a) : (b)
#endif

#ifndef max
# define max(a,b)                                             (a) > (b) ?  (a) : (b)
#endif


#define HFTCOREMODULE_VERSION                                 20181016UL


typedef enum POWERSWITCH_ENUM {

  POWERSWITCH__USB_SW                                         = 1,
  POWERSWITCH__3V3_HICUR,
  POWERSWITCH__3V3_XO,
  POWERSWITCH__1V2_DCDC,                                                                              // V1.0: not implemented
  POWERSWITCH__1V2_SW,                                                                                // SMTP switch
  POWERSWITCH__BAT_SW,
  POWERSWITCH__BAT_HICUR,

} POWERSWITCH_ENUM_t;


typedef enum ENABLE_MASK {

  ENABLE_MASK__I2C_HYGRO                                      = 0x0001UL,
  ENABLE_MASK__I2C_BARO                                       = 0x0002UL,
  ENABLE_MASK__I2C_GYRO                                       = 0x0004UL,
  ENABLE_MASK__I2C_LCD                                        = 0x0008UL,
  ENABLE_MASK__LORA_BARE                                      = 0x0010UL,
  ENABLE_MASK__LORAWAN_DEVICE                                 = 0x0020UL,

} ENABLE_MASK_t;


typedef enum MON_MASK {

  MON_MASK__I2C_HYGRO                                         = 0x0001UL,
  MON_MASK__I2C_BARO                                          = 0x0002UL,
  MON_MASK__I2C_GYRO                                          = 0x0004UL,
  MON_MASK__LORA                                              = 0x0008UL,

} MON_MASK_t;


typedef enum SYSCLK_CONFIG_ENUM {

  SYSCLK_CONFIG_01MHz_MSI_HSI                                 =  1000,
  SYSCLK_CONFIG_04MHz_MSI                                     =  4000,
  SYSCLK_CONFIG_08MHz_MSI                                     =  8000,
  SYSCLK_CONFIG_16MHz_MSI                                     = 16000,
  SYSCLK_CONFIG_24MHz_MSI                                     = 24000,
  SYSCLK_CONFIG_80MHz_MSI16_PLL                               = 80000,
  SYSCLK_CONFIG_80MHz_HSE10_PLL                               = 80001,

} SYSCLK_CONFIG_t;



uint32_t crcCalc(const uint32_t* ptr, uint32_t len);

uint8_t sel_u8_from_u32(uint32_t in_u32, uint8_t sel);
void mainCalcFloat2IntFrac(float val, uint8_t fracCnt, int32_t* outInt, uint32_t* outFrac);
void mainPowerSwitchDo(POWERSWITCH_ENUM_t sw, uint8_t enable);
void SystemResetbyARMcore(void);

void mainPowerSwitchInit(void);
void HFT_SystemClock_Config(SYSCLK_CONFIG_t sel);
void HFT_RCC_MCO_Disable(void);
void HFT_TIM2_AdjustClock(uint8_t multiply);

/*
 * Power analysis:
 *
 * Startup main() with 23.0mA @ Vsol=3.3V
 *
 * 23.0mA --> 23.5mA:  __HAL_RCC_PLL_ENABLE()                                                                      File:stm32l4xx_hal_rcc.c  Line:831
 * 23.5mA --> 24.5mA:  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_ClkInitStruct->SYSCLKSource);                        File:stm32l4xx_hal_rcc.c  Line:1049     PLL selected as system clock
 * 24.5mA --> 25.0mA:  MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE), (RCC_MCOSource | RCC_MCODiv ));  File:stm32l4xx_hal_rcc.c  Line:1191     Drive 16 MHz @ MCO
 *
 * 26.0mA --> 26.5mA:  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);                                                     File:stm32l4xx_hal_msp.c  Line:1152     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM
 * 26.5mA --> 27.0mA:
 * 27.0mA --> 28.0mA:  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                                                     File:stm32l4xx_hal_msp.c  Line:694      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
 */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
