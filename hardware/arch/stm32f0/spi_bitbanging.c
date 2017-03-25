/*
* @Author: Kienltb
* @Date:   2016-12-28 16:31:26
* @Last Modified by:   Kienltb
* @Last Modified time: 2017-01-11 16:37:10
*/

#include "spi_bitbanging.h"
#include "gpio.h"
#include "system.h" /* For delay_ms */

static uint8_t clk_pin;
static uint8_t mosi_pin;
static uint8_t miso_pin;

/* Init spi bitbanging and gpio function */
void spi_init_bb(uint8_t clk, uint8_t mosi, uint8_t miso)
{
        clk_pin = clk;
        gpio_init(clk_pin, GPIO_OUT);

        mosi_pin = mosi;
        gpio_init(mosi_pin, GPIO_OUT);

        miso_pin = miso;
        gpio_init(miso_pin, GPIO_IN);

        /*clk init as high*/
        gpio_set(clk_pin);
}

/* write 1 byte and return 1byte received*/
uint8_t spi_tranfer_byte_bb(uint8_t data)
{
        uint8_t i;
        for (i = 0; i < 8; ++i) {
                if ((data & BIT7) == BIT7)
                        gpio_set(mosi_pin);
                else
                        gpio_clear(mosi_pin);
                delay_ms(5);
                data = (data << 1);
                gpio_set(clk_pin);

                delay_ms(5);
                data |= gpio_read(miso_pin);
                gpio_clear(clk_pin);
        }
        return data;
}

/* write length byte from rx_buffer and return last byte received*/
uint8_t spi_tranfer_data_bb(uint8_t *rx_buffer, uint8_t length)
{
        uint8_t i;
        uint8_t buff;
        for(i = 0; i < length; ++i)
                buff = spi_tranfer_byte_bb(rx_buffer[i]);
        return buff;
}
