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

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

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
#define MCU_HSE_Pin GPIO_PIN_0
#define MCU_HSE_GPIO_Port GPIOH
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
#define MCU_I2C4_SCL_Pin GPIO_PIN_14
#define MCU_I2C4_SCL_GPIO_Port GPIOF
#define I2S1B_SCK_OUT_Pin GPIO_PIN_8
#define I2S1B_SCK_OUT_GPIO_Port GPIOE
#define I2S1B_FS_OUT_Pin GPIO_PIN_9
#define I2S1B_FS_OUT_GPIO_Port GPIOE
#define I2S1B_MCLK_OUT_Pin GPIO_PIN_10
#define I2S1B_MCLK_OUT_GPIO_Port GPIOE
#define MCU_SPI1_SCK_Pin GPIO_PIN_13
#define MCU_SPI1_SCK_GPIO_Port GPIOE
#define MCU_SPI1_MISO_Pin GPIO_PIN_14
#define MCU_SPI1_MISO_GPIO_Port GPIOE
#define MCU_SPI1_MOSI_Pin GPIO_PIN_15
#define MCU_SPI1_MOSI_GPIO_Port GPIOE
#define I2S2A_SCK_IN_Pin GPIO_PIN_13
#define I2S2A_SCK_IN_GPIO_Port GPIOB
#define I2S2A_SD_IN_Pin GPIO_PIN_11
#define I2S2A_SD_IN_GPIO_Port GPIOD
#define I2S2A_FS_IN_Pin GPIO_PIN_12
#define I2S2A_FS_IN_GPIO_Port GPIOD
#define MCU_I2C4_SDA_Pin GPIO_PIN_13
#define MCU_I2C4_SDA_GPIO_Port GPIOD
#define MCU_I2C3_SCL_Pin GPIO_PIN_7
#define MCU_I2C3_SCL_GPIO_Port GPIOG
#define MCU_I2C3_SDA_Pin GPIO_PIN_8
#define MCU_I2C3_SDA_GPIO_Port GPIOG
#define MCU_OUT_PWM_LED_R_Pin GPIO_PIN_6
#define MCU_OUT_PWM_LED_R_GPIO_Port GPIOC
#define MCU_MCO_Pin GPIO_PIN_8
#define MCU_MCO_GPIO_Port GPIOA
#define MCU_IN_VBUS_Pin GPIO_PIN_9
#define MCU_IN_VBUS_GPIO_Port GPIOA
#define PA_USB_N_Pin GPIO_PIN_11
#define PA_USB_N_GPIO_Port GPIOA
#define PA_USB_P_Pin GPIO_PIN_12
#define PA_USB_P_GPIO_Port GPIOA
#define MCU_IN_SWCLK_Pin GPIO_PIN_14
#define MCU_IN_SWCLK_GPIO_Port GPIOA
#define MCU_SPI3_SCK_Pin GPIO_PIN_10
#define MCU_SPI3_SCK_GPIO_Port GPIOC
#define MCU_SPI3_MISO_Pin GPIO_PIN_11
#define MCU_SPI3_MISO_GPIO_Port GPIOC
#define MCU_SPI3_MOSI_Pin GPIO_PIN_12
#define MCU_SPI3_MOSI_GPIO_Port GPIOC
#define UART2_RTS_OUT_Pin GPIO_PIN_4
#define UART2_RTS_OUT_GPIO_Port GPIOD
#define UART2_RX_IN_Pin GPIO_PIN_6
#define UART2_RX_IN_GPIO_Port GPIOD
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

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

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
