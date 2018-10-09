/*
 * task_Baro.h
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#ifndef TASK_BARO_H_
#define TASK_BARO_H_


/* I2C devices */

#define I2C_SLAVE_BARO_ADDR                                   0x76U
#define I2C_SLAVE_BARO_REG_RESET                              0x1EU
#define I2C_SLAVE_BARO_REG_VERSION                            0xAEU
#define I2C_SLAVE_BARO_REG_PROM                               0xA0U
#define I2C_SLAVE_BARO_REG_CONV_D1_256                        0x40U
#define I2C_SLAVE_BARO_REG_CONV_D1_512                        0x42U
#define I2C_SLAVE_BARO_REG_CONV_D1_1024                       0x44U
#define I2C_SLAVE_BARO_REG_CONV_D1_2048                       0x46U
#define I2C_SLAVE_BARO_REG_CONV_D1_4096                       0x48U
#define I2C_SLAVE_BARO_REG_CONV_D2_256                        0x50U
#define I2C_SLAVE_BARO_REG_CONV_D2_512                        0x52U
#define I2C_SLAVE_BARO_REG_CONV_D2_1024                       0x54U
#define I2C_SLAVE_BARO_REG_CONV_D2_2048                       0x56U
#define I2C_SLAVE_BARO_REG_CONV_D2_4096                       0x58U
#define I2C_SLAVE_BARO_REG_ADC_READ                           0x00U

#define C_I2C_BARO_C_CNT                                      8


typedef enum BARO_GET_TYPE_ENUM {

  BARO_GET_TYPE__TEMP_100                                     = 1,
  BARO_GET_TYPE__P_100,
  BARO_GET_TYPE__QNH_100

} BARO_GET_TYPE_t;


typedef enum baroMsgBaroCmds_ENUM {

  MsgBaro__InitDo                                             = 0x01U,
  MsgBaro__InitDone,

  MsgBaro__SetVar01_x                                         = 0x41U,

  MsgBaro__GetVar01_y                                         = 0x81U,

  MsgBaro__CallFunc01_DoMeasure                               = 0xc1U,
  MsgBaro__CallFunc02_CyclicTimerEvent,
  MsgBaro__CallFunc03_CyclicTimerStart,
  MsgBaro__CallFunc04_CyclicTimerStop,

} baroMsgBaroCmds_t;



int32_t baroGetValue(BARO_GET_TYPE_t type);

void baroTimerCallback(void const *argument);


/* Task */

void baroTaskInit(void);
void baroTaskLoop(void);

#endif /* TASK_BARO_H_ */
