/*
 * task_Gyro.h
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#ifndef TASK_GYRO_H_
#define TASK_GYRO_H_


/* I2C devices */

#define I2C_SLAVE_GYRO_ADDR_1                                 0x68U
#define I2C_SLAVE_GYRO_REG_1_GYRO_SELF_TEST_X_GYRO            0x00U
#define I2C_SLAVE_GYRO_REG_1_GYRO_SELF_TEST_Y_GYRO            0x01U
#define I2C_SLAVE_GYRO_REG_1_GYRO_SELF_TEST_Z_GYRO            0x02U
#define I2C_SLAVE_GYRO_REG_1_GYRO_SELF_TEST_X_ACCEL           0x0DU
#define I2C_SLAVE_GYRO_REG_1_GYRO_SELF_TEST_Y_ACCEL           0x0EU
#define I2C_SLAVE_GYRO_REG_1_GYRO_SELF_TEST_Z_ACCEL           0x0FU
#define I2C_SLAVE_GYRO_REG_1_GYRO_XG_OFFSET_H                 0x13U
#define I2C_SLAVE_GYRO_REG_1_GYRO_XG_OFFSET_L                 0x14U
#define I2C_SLAVE_GYRO_REG_1_GYRO_YG_OFFSET_H                 0x15U
#define I2C_SLAVE_GYRO_REG_1_GYRO_YG_OFFSET_L                 0x16U
#define I2C_SLAVE_GYRO_REG_1_GYRO_ZG_OFFSET_H                 0x17U
#define I2C_SLAVE_GYRO_REG_1_GYRO_ZG_OFFSET_L                 0x18U
#define I2C_SLAVE_GYRO_REG_1_SMPLRT_DIV                       0x19U
#define I2C_SLAVE_GYRO_REG_1_CONFIG                           0x1AU

#define I2C_SLAVE_GYRO_REG_1_GYRO_CONFIG                      0x1BU
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__FS_SEL_0250         0x00U
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__FS_SEL_0500         0x08U
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__FS_SEL_1000         0x10U
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__FS_SEL_2000         0x18U
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__0250DPS             250U
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__0500DPS             500U
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__1000DPS             1000U
#define I2C_SLAVE_GYRO_DTA_1_GYRO_CONFIG__2000DPS             2000U

#define I2C_SLAVE_GYRO_REG_1_ACCEL_CONFIG                     0x1CU
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__FS_SEL_L_02        0x00U
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__FS_SEL_L_04        0x08U
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__FS_SEL_L_08        0x10U
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__FS_SEL_L_16        0x18U
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__02G                2U
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__04G                4U
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__08G                8U
#define I2C_SLAVE_GYRO_DTA_1_ACCEL_CONFIG__16G                16U

#define I2C_SLAVE_GYRO_REG_1_ACCEL_CONFIG2                    0x1DU
#define I2C_SLAVE_GYRO_REG_1_LP_ACCEL_ODR                     0x1EU
#define I2C_SLAVE_GYRO_REG_1_WOM_THR                          0x1FU
#define I2C_SLAVE_GYRO_REG_1_FIFO_EN                          0x23U
#define I2C_SLAVE_GYRO_REG_1_I2C_MST_CTRL                     0x24U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV0_ADDR                    0x25U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV0_REG                     0x26U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV0_CTRL                    0x27U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV1_ADDR                    0x28U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV1_REG                     0x29U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV1_CTRL                    0x2AU
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV2_ADDR                    0x2BU
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV2_REG                     0x2CU
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV2_CTRL                    0x2DU
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV3_ADDR                    0x2EU
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV3_REG                     0x2FU
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV3_CTRL                    0x30U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV4_ADDR                    0x31U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV4_REG                     0x32U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV4_DO                      0x33U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV4_CTRL                    0x34U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV4_DI                      0x35U
#define I2C_SLAVE_GYRO_REG_1_I2C_MST_STATUS                   0x36U

#define I2C_SLAVE_GYRO_REG_1_INT_PIN_CFG                      0x37U
#define I2C_SLAVE_GYRO_DTA_1_INT_PIN_CFG__BYPASS_EN           0x02U

#define I2C_SLAVE_GYRO_REG_1_INT_ENABLE                       0x38U
#define I2C_SLAVE_GYRO_REG_1_INT_STATUS                       0x3AU
#define I2C_SLAVE_GYRO_REG_1_ACCEL_XOUT_H                     0x3BU
#define I2C_SLAVE_GYRO_REG_1_ACCEL_XOUT_L                     0x3CU
#define I2C_SLAVE_GYRO_REG_1_ACCEL_YOUT_H                     0x3DU
#define I2C_SLAVE_GYRO_REG_1_ACCEL_YOUT_L                     0x3EU
#define I2C_SLAVE_GYRO_REG_1_ACCEL_ZOUT_H                     0x3FU
#define I2C_SLAVE_GYRO_REG_1_ACCEL_ZOUT_L                     0x40U
#define I2C_SLAVE_GYRO_REG_1_TEMP_OUT_H                       0x41U
#define I2C_SLAVE_GYRO_REG_1_TEMP_OUT_L                       0x42U
#define I2C_SLAVE_GYRO_REG_1_GYRO_XOUT_H                      0x43U
#define I2C_SLAVE_GYRO_REG_1_GYRO_XOUT_L                      0x44U
#define I2C_SLAVE_GYRO_REG_1_GYRO_YOUT_H                      0x45U
#define I2C_SLAVE_GYRO_REG_1_GYRO_YOUT_L                      0x46U
#define I2C_SLAVE_GYRO_REG_1_GYRO_ZOUT_H                      0x47U
#define I2C_SLAVE_GYRO_REG_1_GYRO_ZOUT_L                      0x48U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_00                 0x49U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_01                 0x4AU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_02                 0x4BU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_03                 0x4CU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_04                 0x4DU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_05                 0x4EU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_06                 0x4FU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_07                 0x50U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_08                 0x51U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_09                 0x52U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_10                 0x53U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_11                 0x54U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_12                 0x55U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_13                 0x56U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_14                 0x57U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_15                 0x58U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_16                 0x59U
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_17                 0x5AU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_18                 0x5BU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_19                 0x5CU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_20                 0x5DU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_21                 0x5EU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_22                 0x5FU
#define I2C_SLAVE_GYRO_REG_1_EXT_SENS_DATA_23                 0x60U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV0_DO                      0x63U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV1_DO                      0x64U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV2_DO                      0x65U
#define I2C_SLAVE_GYRO_REG_1_I2C_SLV3_DO                      0x66U
#define I2C_SLAVE_GYRO_REG_1_I2C_MST_DELAY_CTRL               0x67U

#define I2C_SLAVE_GYRO_REG_1_SIGNAL_PATH_RESET                0x68U
#define I2C_SLAVE_GYRO_DTA_1_SIGNAL_PATH_RESET__TEMP_RST      0x01U
#define I2C_SLAVE_GYRO_DTA_1_SIGNAL_PATH_RESET__ACCEL_RST     0x02U
#define I2C_SLAVE_GYRO_DTA_1_SIGNAL_PATH_RESET__GYRO_RST      0x04U

#define I2C_SLAVE_GYRO_REG_1_MOT_DETECT_CTRL                  0x69U

#define I2C_SLAVE_GYRO_REG_1_USER_CTRL                        0x6AU
#define I2C_SLAVE_GYRO_DTA_1_USER_CTRL__SIG_COND_RST          0x01U
#define I2C_SLAVE_GYRO_DTA_1_USER_CTRL__I2C_MST_RST           0x02U
#define I2C_SLAVE_GYRO_DTA_1_USER_CTRL__FIFO_RST              0x04U
#define I2C_SLAVE_GYRO_DTA_1_USER_CTRL__I2C_IF_DIS            0x10U
#define I2C_SLAVE_GYRO_DTA_1_USER_CTRL__2C_MST_EN             0x20U
#define I2C_SLAVE_GYRO_DTA_1_USER_CTRL__FIFO_EN               0x40U

#define I2C_SLAVE_GYRO_REG_1_PWR_MGMT_1                       0x6BU
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__CLKSEL_VAL           0x01U                                   // Clock: auto select
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__PD_PTAT              0x08U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__GYRO_STANDBY         0x10U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__CYCLE                0x20U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__SLEEP                0x40U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_1__HRESET               0x80U

#define I2C_SLAVE_GYRO_REG_1_PWR_MGMT_2                       0x6CU
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_2__DISABLE_ZG           0x01U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_2__DISABLE_YG           0x02U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_2__DISABLE_XG           0x04U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_2__DISABLE_ZA           0x08U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_2__DISABLE_YA           0x10U
#define I2C_SLAVE_GYRO_DTA_1_PWR_MGMT_2__DISABLE_XA           0x20U

#define I2C_SLAVE_GYRO_REG_1_FIFO_COUNTH                      0x72U
#define I2C_SLAVE_GYRO_REG_1_FIFO_COUNTL                      0x73U
#define I2C_SLAVE_GYRO_REG_1_FIFO_R_W                         0x74U
#define I2C_SLAVE_GYRO_REG_1_WHOAMI                           0x75U
#define I2C_SLAVE_GYRO_REG_1_XA_OFFSET_H                      0x77U
#define I2C_SLAVE_GYRO_REG_1_XA_OFFSET_L                      0x78U
#define I2C_SLAVE_GYRO_REG_1_YA_OFFSET_H                      0x7AU
#define I2C_SLAVE_GYRO_REG_1_YA_OFFSET_L                      0x7BU
#define I2C_SLAVE_GYRO_REG_1_ZA_OFFSET_H                      0x7DU
#define I2C_SLAVE_GYRO_REG_1_ZA_OFFSET_L                      0x7EU

#define I2C_SLAVE_GYRO_ADDR_2                                 0x0CU
#define I2C_SLAVE_GYRO_REG_2_WIA                              0x00U
#define I2C_SLAVE_GYRO_REG_2_INFO                             0x01U

#define I2C_SLAVE_GYRO_REG_2_ST1                              0x02U
#define I2C_SLAVE_GYRO_DTA_2_ST1__DRDY                        0x01U
#define I2C_SLAVE_GYRO_DTA_2_ST1__DOR                         0x02U

#define I2C_SLAVE_GYRO_REG_2_HX_L                             0x03U
#define I2C_SLAVE_GYRO_REG_2_HX_H                             0x04U
#define I2C_SLAVE_GYRO_REG_2_HY_L                             0x05U
#define I2C_SLAVE_GYRO_REG_2_HY_H                             0x06U
#define I2C_SLAVE_GYRO_REG_2_HZ_L                             0x07U
#define I2C_SLAVE_GYRO_REG_2_HZ_Y                             0x08U

#define I2C_SLAVE_GYRO_REG_2_ST2                              0x09U
#define I2C_SLAVE_GYRO_DTA_2_ST2__HOFL                        0x08U
#define I2C_SLAVE_GYRO_DTA_2_ST2__BITM                        0x10U

#define I2C_SLAVE_GYRO_REG_2_CNTL1                            0x0AU
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE0                     0x01U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE1                     0x02U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE2                     0x04U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE3                     0x08U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__BIT                       0x10U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_POWER_DOWN       0x10U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_SINGLE_VAL       0x11U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_RUN_8HZ_VAL      0x12U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_16B_RUN_100HZ_VAL    0x16U
#define I2C_SLAVE_GYRO_DTA_2_CNTL1__MODE_PROM_VAL             0x1FU

#define I2C_SLAVE_GYRO_REG_2_CNTL2                            0x0BU
#define I2C_SLAVE_GYRO_DTA_2_CNTL2__SRST                      0x01U

#define I2C_SLAVE_GYRO_REG_2_ASTC                             0x0CU

#define I2C_SLAVE_GYRO_REG_2_I2CDIS                           0x0FU
#define I2C_SLAVE_GYRO_REG_2_I2CDIS__DISABLE_VAL              0b00011011U

#define I2C_SLAVE_GYRO_REG_2_ASAX                             0x10U
#define I2C_SLAVE_GYRO_REG_2_ASAY                             0x11U
#define I2C_SLAVE_GYRO_REG_2_ASAZ                             0x12U


typedef enum gyroMsgGyroCmds_ENUM {

  MsgGyro__InitDo                                             = 0x01U,
  MsgGyro__InitDone,

  MsgGyro__DeInitDo                                           = 0x05U,

//MsgGyro__SetVar01_x                                         = 0x41U,

//MsgGyro__GetVar01_y                                         = 0x81U,

  MsgGyro__CallFunc01_CyclicTimerEvent                        = 0xc1U,
  MsgGyro__CallFunc02_CyclicTimerStart,
  MsgGyro__CallFunc03_CyclicTimerStop,
  MsgGyro__CallFunc04_DoMeasure

} gyroMsgGyroCmds_t;



int32_t gyroGetValue(BARO_GET_TYPE_t type);

void gyroTimerCallback(void const *argument);


/* Task */

void gyroTaskInit(void);
void gyroTaskLoop(void);

#endif /* TASK_GYRO_H_ */
