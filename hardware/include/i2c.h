/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:43:01
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-16 21:05:45
*/
#ifndef __I2C_HW__H__
#define __I2C_HW__H__

#include "typedef.h"
#include "gpio.h"
/**
 * STM32F0 has 2 I2C module is I2C1 and I2C2.
 * I2C pin canbe config as folow:
 *         I2C1 CLK:    PB6 AF1         PB8 AF1
 *         I2C1 SDA:    PB7 AF1         PB9 AF1
 *
 *         I2C2 CLK:    PB10 AF1        PF6 AF
 *         I2C2 SDA:    PB11 AF1        PF7 AF
 */



#define I2C_MODULE_1 0x01
#define I2C_MODULE_2 0x02

/* Configure default  of I2C1 is PB8, PB9 - Fixed, not change*/
#define I2C1_SCL_PIN    GPIO_PIN(GPIO_PB, 8)
#define I2C1_SDA_PIN    GPIO_PIN(GPIO_PB, 9)
#define I2C1_FUNCTION   AF1

/* Configure default  of I2C2 is PB10, PB11 - Fixed, not change */
#define I2C2_SCL_PIN    GPIO_PIN(GPIO_PB, 10)
#define I2C2_SDA_PIN    GPIO_PIN(GPIO_PB, 11)
#define I2C2_FUNCTION   AF1


/* Init module I2C (canbe I2C_MODULE_1 or I2C_MODULE_2)*/
void i2c_init(uint8_t i2c_module);

/* Read 1byte from reg_addr of device*/
uint8_t i2c_read_byte (uint8_t device_addr, uint8_t reg_addr);

/* Read length byte from reg_addr of device to buffer*/
uint8_t i2c_read_data (uint8_t device_addr, uint8_t reg_addr, uint8_t * rx_buffer, uint8_t length);

/* Write 1byte from reg_addr of device*/
uint8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);

/* Write length byte from buffer of device to start from reg_addr*/
uint8_t i2c_write_data(uint8_t device_addr, uint8_t reg_addr, uint8_t * tx_buffer, uint8_t length);


#endif // __I2C__H__
