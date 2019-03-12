/* USER CODE BEGIN Header */
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_I2S1B_SD_OUT_Pin GPIO_PIN_3
#define MCU_I2S1B_SD_OUT_GPIO_Port GPIOE
#define MCU_BQ_EN_OUT_Pin GPIO_PIN_4
#define MCU_BQ_EN_OUT_GPIO_Port GPIOE
#define MCU_TPS_EN_OUT_Pin GPIO_PIN_5
#define MCU_TPS_EN_OUT_GPIO_Port GPIOE
#define MCU_PWM_LCD_BL_Pin GPIO_PIN_6
#define MCU_PWM_LCD_BL_GPIO_Port GPIOE
#define MCU_WAKEUP_IN_Pin GPIO_PIN_13
#define MCU_WAKEUP_IN_GPIO_Port GPIOC
#define MCU_OSC32_IN_Pin GPIO_PIN_14
#define MCU_OSC32_IN_GPIO_Port GPIOC
#define MCU_OSC32_OUT_Pin GPIO_PIN_15
#define MCU_OSC32_OUT_GPIO_Port GPIOC
#define MCU_I2C2_SDA_Pin GPIO_PIN_0
#define MCU_I2C2_SDA_GPIO_Port GPIOF
#define MCU_I2C2_SCL_Pin GPIO_PIN_1
#define MCU_I2C2_SCL_GPIO_Port GPIOF
#define MCU_EXTI3_PEXPA_Pin GPIO_PIN_3
#define MCU_EXTI3_PEXPA_GPIO_Port GPIOF
#define MCU_EXTI4_PEXPB_Pin GPIO_PIN_4
#define MCU_EXTI4_PEXPB_GPIO_Port GPIOF
#define MCU_VBUS_SOFT_SW_EN_OUT_Pin GPIO_PIN_5
#define MCU_VBUS_SOFT_SW_EN_OUT_GPIO_Port GPIOF
#define MCU_TIMCAP_PN_Pin GPIO_PIN_6
#define MCU_TIMCAP_PN_GPIO_Port GPIOF
#define MCU_TIMCAP_VSOL_Pin GPIO_PIN_7
#define MCU_TIMCAP_VSOL_GPIO_Port GPIOF
#define MCU_TIMCAP_PS_Pin GPIO_PIN_8
#define MCU_TIMCAP_PS_GPIO_Port GPIOF
#define MCU_TIMCAP_1PPS_Pin GPIO_PIN_9
#define MCU_TIMCAP_1PPS_GPIO_Port GPIOF
#define MCU_HSE_Pin GPIO_PIN_0
#define MCU_HSE_GPIO_Port GPIOH
#define MCU_ADC1_IN5_VSOL_Pin GPIO_PIN_0
#define MCU_ADC1_IN5_VSOL_GPIO_Port GPIOA
#define MCU_ADC1_IN6_XO_PULL_Pin GPIO_PIN_1
#define MCU_ADC1_IN6_XO_PULL_GPIO_Port GPIOA
#define MCU_LPUART_TX_OUT_Pin GPIO_PIN_2
#define MCU_LPUART_TX_OUT_GPIO_Port GPIOA
#define MCU_ADC1_IN8_LIPO_CUR_Pin GPIO_PIN_3
#define MCU_ADC1_IN8_LIPO_CUR_GPIO_Port GPIOA
#define MCU_LIPO_CEN_OUT_Pin GPIO_PIN_4
#define MCU_LIPO_CEN_OUT_GPIO_Port GPIOA
#define MCU_EXTI5_LIPO_Pin GPIO_PIN_5
#define MCU_EXTI5_LIPO_GPIO_Port GPIOA
#define MCU_LPUART_CTS_IN_Pin GPIO_PIN_6
#define MCU_LPUART_CTS_IN_GPIO_Port GPIOA
#define MCU_PWM_LED_G_Pin GPIO_PIN_7
#define MCU_PWM_LED_G_GPIO_Port GPIOA
#define MCU_BNO085_PS0_WAKE_OUT_Pin GPIO_PIN_5
#define MCU_BNO085_PS0_WAKE_OUT_GPIO_Port GPIOC
#define MCU_PWM_LED_B_Pin GPIO_PIN_0
#define MCU_PWM_LED_B_GPIO_Port GPIOB
#define MCU_LPUART_RTS_OUT_Pin GPIO_PIN_1
#define MCU_LPUART_RTS_OUT_GPIO_Port GPIOB
#define MCU_SW_PWR_OUT_Pin GPIO_PIN_2
#define MCU_SW_PWR_OUT_GPIO_Port GPIOB
#define MCU_BNO085_SEL_OUT_Pin GPIO_PIN_11
#define MCU_BNO085_SEL_OUT_GPIO_Port GPIOF
#define MCU_EXTI12_SX_DIO2_TXRXSW_Pin GPIO_PIN_12
#define MCU_EXTI12_SX_DIO2_TXRXSW_GPIO_Port GPIOF
#define MCU_EXTI13_BNO085_Pin GPIO_PIN_13
#define MCU_EXTI13_BNO085_GPIO_Port GPIOF
#define MCU_AX_SEL_OUT_Pin GPIO_PIN_14
#define MCU_AX_SEL_OUT_GPIO_Port GPIOF
#define MCU_SX_SEL_OUT_Pin GPIO_PIN_15
#define MCU_SX_SEL_OUT_GPIO_Port GPIOF
#define MCU_EXTI0_SX_DIO1_TXRXDONE_Pin GPIO_PIN_0
#define MCU_EXTI0_SX_DIO1_TXRXDONE_GPIO_Port GPIOG
#define MCU_EXTI1_SX_DIO3_TIMEOUT_Pin GPIO_PIN_1
#define MCU_EXTI1_SX_DIO3_TIMEOUT_GPIO_Port GPIOG
#define MCU_EXTI7_AX_Pin GPIO_PIN_7
#define MCU_EXTI7_AX_GPIO_Port GPIOE
#define MCU_I2S1B_SCK_OUT_Pin GPIO_PIN_8
#define MCU_I2S1B_SCK_OUT_GPIO_Port GPIOE
#define MCU_I2S1B_FS_OUT_Pin GPIO_PIN_9
#define MCU_I2S1B_FS_OUT_GPIO_Port GPIOE
#define MCU_I2S1B_MCLK_OUT_Pin GPIO_PIN_10
#define MCU_I2S1B_MCLK_OUT_GPIO_Port GPIOE
#define QUADSPI_BK12_NCS_Pin GPIO_PIN_11
#define QUADSPI_BK12_NCS_GPIO_Port GPIOE
#define MCU_I2S2A_SCK_IN_Pin GPIO_PIN_13
#define MCU_I2S2A_SCK_IN_GPIO_Port GPIOB
#define MCU_SPI2_MOSI_Pin GPIO_PIN_15
#define MCU_SPI2_MOSI_GPIO_Port GPIOB
#define MCU_SX_BUSY_IN_Pin GPIO_PIN_8
#define MCU_SX_BUSY_IN_GPIO_Port GPIOD
#define MCU_PS00_INOUT_Pin GPIO_PIN_9
#define MCU_PS00_INOUT_GPIO_Port GPIOD
#define MCU_PS01_INOUT_Pin GPIO_PIN_10
#define MCU_PS01_INOUT_GPIO_Port GPIOD
#define MCU_I2S2A_SD_IN_Pin GPIO_PIN_11
#define MCU_I2S2A_SD_IN_GPIO_Port GPIOD
#define MCU_I2S2A_FS_IN_Pin GPIO_PIN_12
#define MCU_I2S2A_FS_IN_GPIO_Port GPIOD
#define MCU_PS02_INOUT_Pin GPIO_PIN_14
#define MCU_PS02_INOUT_GPIO_Port GPIOD
#define MCU_PS03_INOUT_Pin GPIO_PIN_15
#define MCU_PS03_INOUT_GPIO_Port GPIOD
#define MCU_EINK_CS_OUT_Pin GPIO_PIN_3
#define MCU_EINK_CS_OUT_GPIO_Port GPIOG
#define MCU_EVENTOUT_Pin GPIO_PIN_4
#define MCU_EVENTOUT_GPIO_Port GPIOG
#define MCU_EINK_DC_OUT_Pin GPIO_PIN_5
#define MCU_EINK_DC_OUT_GPIO_Port GPIOG
#define MCU_LPUART_RI_IN_Pin GPIO_PIN_6
#define MCU_LPUART_RI_IN_GPIO_Port GPIOG
#define MCU_LPUART_DTR_OUT_Pin GPIO_PIN_7
#define MCU_LPUART_DTR_OUT_GPIO_Port GPIOG
#define MCU_LPUART_RX_IN_Pin GPIO_PIN_8
#define MCU_LPUART_RX_IN_GPIO_Port GPIOG
#define MCU_PWM_LED_R_Pin GPIO_PIN_6
#define MCU_PWM_LED_R_GPIO_Port GPIOC
#define MCU_LPUART_DCD_IN_Pin GPIO_PIN_7
#define MCU_LPUART_DCD_IN_GPIO_Port GPIOC
#define MCU_EINK_BUSY_IN_Pin GPIO_PIN_8
#define MCU_EINK_BUSY_IN_GPIO_Port GPIOC
#define MCU_EXTI9_SI5338_Pin GPIO_PIN_9
#define MCU_EXTI9_SI5338_GPIO_Port GPIOC
#define MCU_MCO_Pin GPIO_PIN_8
#define MCU_MCO_GPIO_Port GPIOA
#define MCU_USB_OTG_FS_VBUS_Pin GPIO_PIN_9
#define MCU_USB_OTG_FS_VBUS_GPIO_Port GPIOA
#define MCU_USB_OTG_FS_ID_Pin GPIO_PIN_10
#define MCU_USB_OTG_FS_ID_GPIO_Port GPIOA
#define MCU_USB_OTG_FS_DM_Pin GPIO_PIN_11
#define MCU_USB_OTG_FS_DM_GPIO_Port GPIOA
#define MCU_USB_OTG_FS_DP_Pin GPIO_PIN_12
#define MCU_USB_OTG_FS_DP_GPIO_Port GPIOA
#define MCU_SWDIO_Pin GPIO_PIN_13
#define MCU_SWDIO_GPIO_Port GPIOA
#define MCU_SWCLK_Pin GPIO_PIN_14
#define MCU_SWCLK_GPIO_Port GPIOA
#define MCU_SPI3_SCK_Pin GPIO_PIN_10
#define MCU_SPI3_SCK_GPIO_Port GPIOC
#define MCU_SPI3_MISO_Pin GPIO_PIN_11
#define MCU_SPI3_MISO_GPIO_Port GPIOC
#define MCU_SPI3_MOSI_Pin GPIO_PIN_12
#define MCU_SPI3_MOSI_GPIO_Port GPIOC
#define MCU_SPI2_SCK_Pin GPIO_PIN_1
#define MCU_SPI2_SCK_GPIO_Port GPIOD
#define MCU_EXTI2_AUDIO_ADC_Pin GPIO_PIN_2
#define MCU_EXTI2_AUDIO_ADC_GPIO_Port GPIOD
#define MCU_SPI2_MISO_Pin GPIO_PIN_3
#define MCU_SPI2_MISO_GPIO_Port GPIOD
#define MCU_UART1_TX_OUT_Pin GPIO_PIN_9
#define MCU_UART1_TX_OUT_GPIO_Port GPIOG
#define MCU_UART1_RX_IN_Pin GPIO_PIN_10
#define MCU_UART1_RX_IN_GPIO_Port GPIOG
#define MCU_UART1_CTS_IN_Pin GPIO_PIN_11
#define MCU_UART1_CTS_IN_GPIO_Port GPIOG
#define MCU_UART1_RTS_OUT_Pin GPIO_PIN_12
#define MCU_UART1_RTS_OUT_GPIO_Port GPIOG
#define MCU_I2C1_SDA_Pin GPIO_PIN_13
#define MCU_I2C1_SDA_GPIO_Port GPIOG
#define MCU_I2C1_SCL_Pin GPIO_PIN_14
#define MCU_I2C1_SCL_GPIO_Port GPIOG
#define MCU_PN00_INOUT_Pin GPIO_PIN_3
#define MCU_PN00_INOUT_GPIO_Port GPIOB
#define MCU_PN01_INOUT_Pin GPIO_PIN_5
#define MCU_PN01_INOUT_GPIO_Port GPIOB
#define MCU_PN02_INOUT_Pin GPIO_PIN_6
#define MCU_PN02_INOUT_GPIO_Port GPIOB
#define MCU_PN03_INOUT_Pin GPIO_PIN_7
#define MCU_PN03_INOUT_GPIO_Port GPIOB
#define MCU_BOOT0_IN_Pin GPIO_PIN_3
#define MCU_BOOT0_IN_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
