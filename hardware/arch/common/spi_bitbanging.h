/*
* @Author: Kienltb
* @Date:   2016-12-28 16:31:15
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-28 18:16:10
*/

/**
 * Implement spi by bitbanging techinal so you can use any GPIO for comnunication.
 */

#ifndef __SPI_BITBANGING_H__
#define __SPI_BITBANGING_H__

/* Init spi bitbanging and gpio function */
void spi_init_bb(uint8_t clk_pin, uint8_t mosi_pin, uint8_t miso_pin);

/* write 1 byte and return 1byte received*/
uint8_t spi_tranfer_byte_bb(uint8_t data);

/* write length byte from rx_buffer and return last byte received*/
uint8_t spi_tranfer_data_bb(uint8_t *rx_buffer, uint8_t length);

#endif //__SPI_BITBANGING_H__
