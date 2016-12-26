/*
* @Author: Kienltb
* @Date:   2016-12-26 15:01:58
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-26 15:35:07
*/

#include "spi.h"
#include "stm32f0xx.h"

enum SPI_PRESCALER_OPTION {
        PRESCALE_2 = 0,
        PRESCALE_4 = 1,
        PRESCALE_8 = 2,
        PRESCALE_16 = 3,
        PRESCALE_32 = 4,
        PRESCALE_64 = 5,
        PRESCALE_128 = 6,
        PRESCALE_256 = 7
};

/* Default prescaler is 8*/
#define SPI_PRESCALER PRESCALE_8

/* SPI module*/
static SPI_TypeDef *spi = NULL;

/* Init SPI module (SPI_MODULE1, SPI_MODULE2) */
void spi_init(uint8_t spi_module)
{
        if (spi_module == SPI_MODULE_1) {
                spi = SPI1;

                /* Enable clock then reset module */
                RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
                RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
                RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

                /* Setup GPIO*/
                gpio_init_function(SPI1_CLK_PIN,  SPI_FUNCTION, PUSH_PULL, NO_PULL);
                gpio_init_function(SPI1_MISO_PIN, SPI_FUNCTION, PUSH_PULL, NO_PULL);
                gpio_init_function(SPI1_MOSI_PIN, SPI_FUNCTION, PUSH_PULL, NO_PULL);

        } else if ( spi_module == SPI_MODULE_2) {
                spi = SPI2;

                /* Enable clock then reset module */
                RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
                RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
                RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

                /* Setup GPIO*/
                gpio_init_function(SPI2_CLK_PIN,  SPI_FUNCTION, PUSH_PULL, NO_PULL);
                gpio_init_function(SPI2_MISO_PIN, SPI_FUNCTION, PUSH_PULL, NO_PULL);
                gpio_init_function(SPI2_MOSI_PIN, SPI_FUNCTION, PUSH_PULL, NO_PULL);
        }

        /* Setup SPI param*/
        spi->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;
        spi->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (SPI_PRESCALER << 3) | SPI_CR1_SPE;

}

/* write 1 byte */
void spi_write_byte(uint8_t data)
{
        uint32_t spi_dr = (uint32_t) spi;
        spi_dr += 0x0C;

        while ((spi->SR & SPI_SR_TXE) == 0);    /* wait for empty TX buffer*/
        *(volatile uint8_t*) spi_dr = data;     /* send one byte of data*/
        while ((spi->SR & SPI_SR_RXNE) == 0);   /* wait for RX buffer contents*/
}

/* write length byte to rx_buffer*/
void spi_write_data(uint8_t* rx_buffer, uint8_t length)
{
        uint32_t spi_dr = (uint32_t) spi;
        spi_dr += 0x0C;

        int i;
        for (i = 0; i < length; ++i) {
                while ((spi->SR & SPI_SR_TXE) == 0);            /* wait for empty TX buffer*/
                *(volatile uint8_t*) spi_dr = rx_buffer[i];     /* send one byte of data*/
                while ((spi->SR & SPI_SR_RXNE) == 0);           /* wait for RX buffer contents*/
        }
}
