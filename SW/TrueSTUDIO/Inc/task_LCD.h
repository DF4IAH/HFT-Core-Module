/*
 * task_LCD.h
 *
 *  Created on: 27.08.2018
 *      Author: DF4IAH
 */

#ifndef TASK_LCD_H_
#define TASK_LCD_H_


/* I2C devices */

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


/* Task */

void lcdTaskInit(void);
void lcdTaskLoop(void);

#endif /* TASK_LCD_H_ */
