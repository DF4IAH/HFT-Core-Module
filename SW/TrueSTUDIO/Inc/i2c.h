/*
 * i2c.h
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#ifndef I2C_H_
#define I2C_H_


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


#define I2C_SLAVE_SI5338_ADDR                                 0x70U


#ifdef HISTORIC
#define I2C_SLAVE_LCD_ADDR                                    0x3EU

#define I2C_SLAVE_LCD_REG_CLEAR_DISPLAY                       0x01U
#define I2C_SLAVE_LCD_REG_RETURN_HOME                         0x02U
#define I2C_SLAVE_LCD_REG_ENTRY_MODE_SET                      0x04U
#define I2C_SLAVE_LCD_REG_DISPLAY_ON_OFF                      0x08U
#define I2C_SLAVE_LCD_REG_FUNCTION_SET                        0x20U
#define I2C_SLAVE_LCD_REG_SET_DDRAM_ADDR                      0x80U

// Instruction table 0
#define I2C_SLAVE_LCD_IT0_REG_CURSOR_OR_DISPLAY_SHIFT         0x10U
#define I2C_SLAVE_LCD_IT0_REG_SET_CGRAM                       0x40U

// Instruction table 1
#define I2C_SLAVE_LCD_IT1_REG_BIAS_SET                        0x10U
#define I2C_SLAVE_LCD_IT1_REG_SET_ICON_ADDR                   0x40U
#define I2C_SLAVE_LCD_IT1_REG_ICON_CONTRAST                   0x50U
#define I2C_SLAVE_LCD_IT1_REG_FOLLOWER_CONTROL                0x60U
#define I2C_SLAVE_LCD_IT1_REG_CONTRAST_SET                    0x70U

// Instruction table 2
#define I2C_SLAVE_LCD_IT2_REG_DOUBLE_HEIGHT_POS               0x10U
#define I2C_SLAVE_LCD_IT2_REG_RESERVED                        0x40U
#endif


/* Module */

#define TXBUFFERSIZE                                          32U
#define RXBUFFERSIZE                                          32U


#define I2C4_BUS_ADDR_SCAN                                    1


void i2cI2c4AddrScan(void);
void i2cI2c4Tcxo20MhzDacInit(void);
void i2cI2c4Tcxo20MhzDacSet(uint16_t dac);
void i2cI2c4Si5338Init(void);


void i2cI2c4HygroTaskInit(void);
void i2cI2c4HygroTaskLoop(void);

void i2cI2c4BaroTaskInit(void);
void i2cI2c4BaroTaskLoop(void);

void i2cI2c4GyroTaskInit(void);
void i2cI2c4GyroTaskLoop(void);

void i2cI2c4LcdTaskInit(void);
void i2cI2c4LcdTaskLoop(void);

#endif /* I2C_H_ */
