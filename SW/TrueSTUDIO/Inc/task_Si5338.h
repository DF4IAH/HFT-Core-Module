#ifndef TASK_SI5338_H_
#define TASK_SI5338_H_

#include <sys/_stdint.h>

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_it.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"


/* I2C devices */

#define I2C_SLAVE_SI5338_ADDR                                 0x70U


typedef struct Reg_Data{
   uint8_t Reg_Addr;
   uint8_t Reg_Val;
   uint8_t Reg_Mask;
} Reg_Data_t;


typedef enum I2C_SI5338_CLKIN_VARIANT_ENUM {

  I2C_SI5338_CLKIN_VARIANT__MCU_MCO_8MHZ                    = 0,
  I2C_SI5338_CLKIN_VARIANT__MCU_MCO_12MHZ,
  I2C_SI5338_CLKIN_VARIANT__TCXO_20MHZ,

} I2C_SI5338_CLKIN_VARIANT_t;


void si5338TaskInit(void);
void si5338TaskLoop(void);

#endif /* TASK_SI5338_H_ */
