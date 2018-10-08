/*
 * task_Si5338.h
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#ifndef BUS_I2C_H_
#define BUS_I2C_H_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "task_Baro.h"
#include "task_Gyro.h"
#include "task_Hygro.h"
#include "task_LCD.h"
#include "task_TCXO_20MHz.h"
#include "task_Si5338.h"


/* Module */

#define I2C_TXBUFSIZE                                         32U
#define I2C_RXBUFSIZE                                         32U


void i2cBusAddrScan(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle);
uint32_t i2cSequenceWriteMask(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle, uint8_t addr, uint16_t count, const Reg_Data_t dataAry[]);
uint32_t i2cSequenceWriteLong(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle, uint8_t addr, uint8_t i2cReg, uint16_t count, const uint8_t i2cWriteAryLong[]);
uint32_t i2cSequenceRead(I2C_HandleTypeDef* dev, osSemaphoreId semaphoreHandle, uint8_t addr, uint8_t i2cRegLen, uint8_t i2cReg[], uint16_t readlen);

#endif /* BUS_I2C_H_ */
