#ifndef TASK_SI5338_H_
#define TASK_SI5338_H_

#include <sys/_stdint.h>

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"


typedef struct Reg_Data{
   uint8_t Reg_Addr;
   uint8_t Reg_Val;
   uint8_t Reg_Mask;
} Reg_Data_t;


typedef enum I2C_SI5338_CLKIN_VARIANT_ENUM {

  I2C_SI5338_CLKIN_VARIANT__MCU_MCO_8MHZ                    = 0,
  I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ,

} I2C_SI5338_CLKIN_VARIANT_t;


uint32_t i2cSequenceRead(I2C_HandleTypeDef* dev, osMutexId mutexHandle, uint8_t addr, uint8_t i2cRegLen, uint8_t i2cReg[], uint16_t readLen);

#endif /* TASK_SI5338_H_ */
