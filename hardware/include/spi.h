/*
* @Author: Kienltb
* @Date:   2016-12-26 15:01:44
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-26 15:35:13
*/

#ifndef __SPI_H__
#define __SPI_H__

#include "gpio.h"
#include "typedef.h"

/**
 * STM32F0 has 2 I2C module is SPI1 and SPI2.
 * SPI pin canbe config as folow:
 * SPI1 CLK:    PA5 AF0         PB3 AF0
 * SPI1 MISO:   PA6 AF0         PB4 AF0
 * SPI1 MOSI:   PA7 AF0         PB5 AF0
 *
 * SPI2 CLK:    PB13 AF0
 * SPI2 MISO:   PB14 AF0
 * SPI2 MOSI:   PB15 AF0
 */

#define SPI_MODULE_1 0x01
#define SPI_MODULE_2 0x02

/* Configure default  of SPI1*/
#define SPI1_CLK_PIN     GPIO_PIN(GPIO_PB, 3)
#define SPI1_MISO_PIN    GPIO_PIN(GPIO_PB, 4)
#define SPI1_MOSI_PIN    GPIO_PIN(GPIO_PB, 5)

/* Configure default  of SPI2 */
#define SPI2_CLK_PIN     GPIO_PIN(GPIO_PB, 13)
#define SPI2_MISO_PIN    GPIO_PIN(GPIO_PB, 14)
#define SPI2_MOSI_PIN    GPIO_PIN(GPIO_PB, 15)

#define SPI_FUNCTION     AF0

/* Init SPI module (SPI_MODULE1, SPI_MODULE2) */
void spi_init(uint8_t spi_module);

/* write 1 byte */
void spi_write_byte(uint8_t data);

/* write length byte to rx_buffer*/
void spi_write_data(uint8_t* rx_buffer, uint8_t length);

#endif //__SPI_H__
