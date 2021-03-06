/*
 * task_Hygro.h
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#ifndef TASK_HYGRO_H_
#define TASK_HYGRO_H_


/* I2C devices */

#define I2C_SLAVE_HYGRO_ADDR                                  0x44U

#define I2C_SLAVE_HYGRO_REG_RESET_HI                          0x30U
#define I2C_SLAVE_HYGRO_REG_RESET_LO                          0xA2U

#define I2C_SLAVE_HYGRO_REG_STATUS_HI                         0xF3U
#define I2C_SLAVE_HYGRO_REG_STATUS_LO                         0x2DU

#define I2C_SLAVE_HYGRO_REG_BREAK_HI                          0x30U
#define I2C_SLAVE_HYGRO_REG_BREAK_LO                          0x93U

#define I2C_SLAVE_HYGRO_REG_PERIODIC_0MPS5_HIPREC_HI          0x20U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_0MPS5_HIPREC_LO          0x32U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_0MPS5_MEDPREC_HI         0x20U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_0MPS5_MEDPREC_LO         0x24U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_0MPS5_LOPREC_HI          0x20U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_0MPS5_LOPREC_LO          0x2FU

#define I2C_SLAVE_HYGRO_REG_PERIODIC_1MPS_HIPREC_HI           0x21U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_1MPS_HIPREC_LO           0x30U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_1MPS_MEDPREC_HI          0x21U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_1MPS_MEDPREC_LO          0x26U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_1MPS_LOPREC_HI           0x21U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_1MPS_LOPREC_LO           0x2DU

#define I2C_SLAVE_HYGRO_REG_PERIODIC_2MPS_HIPREC_HI           0x22U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_2MPS_HIPREC_LO           0x36U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_2MPS_MEDPREC_HI          0x22U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_2MPS_MEDPREC_LO          0x20U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_2MPS_LOPREC_HI           0x22U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_2MPS_LOPREC_LO           0x2BU

#define I2C_SLAVE_HYGRO_REG_PERIODIC_4MPS_HIPREC_HI           0x23U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_4MPS_HIPREC_LO           0x34U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_4MPS_MEDPREC_HI          0x23U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_4MPS_MEDPREC_LO          0x22U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_4MPS_LOPREC_HI           0x23U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_4MPS_LOPREC_LO           0x29U

#define I2C_SLAVE_HYGRO_REG_PERIODIC_10MPS_HIPREC_HI          0x27U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_10MPS_HIPREC_LO          0x37U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_10MPS_MEDPREC_HI         0x27U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_10MPS_MEDPREC_LO         0x21U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_10MPS_LOPREC_HI          0x27U
#define I2C_SLAVE_HYGRO_REG_PERIODIC_10MPS_LOPREC_LO          0x2AU

#define I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_HI    0x24U
#define I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_NOCLKSTRETCH_LO    0x00U
#define I2C_SLAVE_HYGRO_REG_ONESHOT_MIDPREC_NOCLKSTRETCH_HI   0x24U
#define I2C_SLAVE_HYGRO_REG_ONESHOT_MIDPREC_NOCLKSTRETCH_LO   0x0BU
#define I2C_SLAVE_HYGRO_REG_ONESHOT_LOPREC_NOCLKSTRETCH_HI    0x24U
#define I2C_SLAVE_HYGRO_REG_ONESHOT_LOPREC_NOCLKSTRETCH_LO    0x16U

#define I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_CLKSTRETCH_HI      0x2CU
#define I2C_SLAVE_HYGRO_REG_ONESHOT_HIPREC_CLKSTRETCH_LO      0x06U
#define I2C_SLAVE_HYGRO_REG_ONESHOT_MIDPREC_CLKSTRETCH_HI     0x2CU
#define I2C_SLAVE_HYGRO_REG_ONESHOT_MIDPREC_CLKSTRETCH_LO     0x0DU
#define I2C_SLAVE_HYGRO_REG_ONESHOT_LOPREC_CLKSTRETCH_HI      0x2CU
#define I2C_SLAVE_HYGRO_REG_ONESHOT_LOPREC_CLKSTRETCH_LO      0x10U

#define I2C_SLAVE_HYGRO_REG_FETCH_DATA_HI                     0xE0U
#define I2C_SLAVE_HYGRO_REG_FETCH_DATA_LO                     0x00U


typedef enum HYGRO_GET_TYPE_ENUM {

  HYGRO_GET_TYPE__RH_100                                      = 1,
  HYGRO_GET_TYPE__T_100,
  HYGRO_GET_TYPE__DP_100

} HYGRO_GET_TYPE_t;


typedef enum hygroMsgHygroCmds_ENUM {

  MsgHygro__InitDo                                            = 0x01U,
  MsgHygro__InitDone,

  MsgHygro__DeInitDo                                          = 0x05U,

//MsgHygro__SetVar01_x                                        = 0x41U,

//MsgHygro__GetVar01_y                                        = 0x81U,

  MsgHygro__CallFunc01_CyclicTimerEvent                       = 0xc1U,
  MsgHygro__CallFunc02_CyclicTimerStart,
  MsgHygro__CallFunc03_CyclicTimerStop,
  MsgHygro__CallFunc04_DoMeasure,

} hygroMsgHygroCmds_t;



int16_t hygroGetValue(HYGRO_GET_TYPE_t type);

void hygroTimerCallback(void const *argument);


/* Task */

void hygroTaskInit(void);
void hygroTaskLoop(void);

#endif /* TASK_HYGRO_H_ */
