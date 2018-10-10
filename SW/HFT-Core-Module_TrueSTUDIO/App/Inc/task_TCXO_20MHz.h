/*
 * task_TCXO_20MHz.h
 *
 *  Created on: 30.08.2018
 *      Author: DF4IAH
 */

#ifndef TASK_TCXO_20MHZ_H_
#define TASK_TCXO_20MHZ_H_

#define TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VALUE                 36200U
#define TCXO_DAC_16BIT_3V3_PULL_DEFAULT_VOLTAGE_UV            1655250UL


#define I2C_SLAVE_20MHZ_DAC_SINGLE_ADDR                       0x1CU
#define I2C_SLAVE_20MHZ_DAC_BROADCAST_ADDR                    0x2AU

#define I2C_SLAVE_20MHZ_DAC_REG_WR_DC                         0x00U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_CODELOAD                   0x01U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_CODE                       0x02U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_LOAD                       0x03U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_CODELOADm                  0x05U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_CODEm                      0x06U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_USER_CONFIG                0x08U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_SW_RESET                   0x09U
#define I2C_SLAVE_20MHZ_DAC_REG_WR_SW_CLEAR                   0x0AU

#define I2C_SLAVE_20MHZ_DAC_REG_RD_STATUS                     0x00U
#define I2C_SLAVE_20MHZ_DAC_REG_RD_CODELOAD_RB                0x01U
#define I2C_SLAVE_20MHZ_DAC_REG_RD_CODE_RB                    0x02U
#define I2C_SLAVE_20MHZ_DAC_REG_RD_LOAD_RB                    0x03U
#define I2C_SLAVE_20MHZ_DAC_REG_RD_CODELOADm_RB               0x05U
#define I2C_SLAVE_20MHZ_DAC_REG_RD_CODEm_RB                   0x06U
#define I2C_SLAVE_20MHZ_DAC_REG_RD_CONFIG_RB                  0x08U


typedef enum Tcxo20MHzMsgTcxoCmds_ENUM {

  MsgTcxo__InitDo                                             = 0x01U,
  MsgTcxo__InitDone,

  Msgtcxo__DeInitDo                                           = 0x05U,

  MsgTcxo__SetVar01_Voltage                                   = 0x41U,

  MsgTcxo__GetVar01_Voltage                                   = 0x81U,

  MsgTcxo__CallFunc01_CyclicTimerEvent                        = 0xc1U,
  MsgTcxo__CallFunc02_CyclicTimerStart,
  MsgTcxo__CallFunc03_CyclicTimerStop,
  MsgTcxo__CallFunc04_SetDAC,

} Tcxo20MHzMsgTcxoCmds_t;



void tcxoTimerCallback(void const *argument);


/* Task */

void tcxo20MhzTaskInit(void);
void tcxo20MhzTaskLoop(void);

#endif /* TASK_TCXO_20MHZ_H_ */
