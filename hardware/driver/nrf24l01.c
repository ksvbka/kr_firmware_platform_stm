/*
* @Author: Kienltb
* @Date:   2016-12-29 16:50:54
* @Last Modified by:   Kienltb
* @Last Modified time: 2017-01-11 16:50:33
*/

#include "nrf24l01.h"
#include "gpio.h"
#include "spi_bitbanging.h"

extern void delay_ms(uint32_t time_ms);

static uint8_t ce_pin; /* Chip enable mode use in RX and TX mode */
static uint8_t cs_pin; /* Chip select pin of spi */

uint8_t TxBuf[32] = {
        'A', 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 29, 30, 31, 32
};

uint8_t  TX_ADDRESS[TX_ADR_WIDTH] = {0x68, 0x31, 0x08, 0x10, 0x01}; //dia chi truyen nhan TX giong RX
uint8_t  RX_ADDRESS[RX_ADR_WIDTH] = {0x68, 0x31, 0x08, 0x10, 0x01}; //
/* Helper function */

/* Write value to address reg, return value of STATUS reg */
static uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value);

/* Read length byte the value of registers start by reg to rx buffer return value of STATUS reg*/
static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t* rx_buff, uint8_t length);

/* Write length byte the value store in tx_buff to register start by reg, return value of STATUS reg*/
static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t* tx_buff, uint8_t length);

/* Init nrf24l01 which default configure and gpio pin ce, cs, miso, mosi */
void nrf24l01_init(uint8_t ce, uint8_t cs, uint8_t clk, uint8_t mosi, uint8_t miso)
{
        /* Init gpio and spi bitbagging*/
        ce_pin = ce;
        gpio_init(ce, GPIO_OUT_PU);
        spi_init_bb(clk, mosi, miso);

        /* Configure NRF24L01 */
        delay_ms(100);
        gpio_clear(ce_pin);     /* chip enable*/
        gpio_set(cs_pin);       /* disable spi*/

        nrf24l01_write_buf(WRITE_RE + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
        nrf24l01_write_buf(WRITE_RE + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);

        nrf24l01_write_reg(WRITE_RE + EN_AA, 0x01);             /* EN P0, 2-->P1*/
        nrf24l01_write_reg(WRITE_RE + EN_RXADDR, 0x01);         /* Enable data P0*/
        nrf24l01_write_reg(WRITE_RE + RF_CH, 0);                /* Chanel 0 RF = 2400 + RF_CH* (1or 2 M)*/
        nrf24l01_write_reg(WRITE_RE + RX_PW_P0, RX_PLOAD_WIDTH);/* Data width is 32byte*/
        nrf24l01_write_reg(WRITE_RE + RF_SETUP, 0x07);          /* 1M, 0dbm*/
        nrf24l01_write_reg(WRITE_RE + CONFIG, 0x0e);            /* Enable CRC, 2 byte CRC, Send*/
}

/* Set NRF24L01 as RX mode*/
void nrf24l01_set_rx_mode(void)
{
        gpio_clear(ce_pin);
        nrf24l01_write_reg(WRITE_RE + CONFIG, 0x0f);         // Enable CRC, 2 byte CRC, recive
        gpio_set(ce_pin);
        delay_ms(100);
}

/* Set NRF24L01 as TX mode*/
void nrf24l01_set_tx_mode(void)
{
        gpio_clear(ce_pin);
        nrf24l01_write_reg(WRITE_RE + CONFIG, 0x0e);         // Enable CRC, 2 byte CRC, Send
        gpio_set(ce_pin);
        delay_ms(100);
}

/* Check FIFO buff, if have package, read to rx_buff and return 1*/
uint8_t nrf24l01_rx_package(uint8_t* rx_buf)
{
        uint8_t have_data = 0;
        uint8_t status;

        //if(RX_DR)            // Data in RX FIFO
        status = spi_tranfer_byte_bb(STATUS); // Read Status
        if ((status & 0x40) != 0) { // Data in RX FIFO
                gpio_clear(ce_pin);     //SPI
                nrf24l01_read_buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH); // read receive payload from RX_FIFO buffer
                have_data = 1;
        }
        nrf24l01_write_reg(WRITE_RE + STATUS, status);
        return have_data;
}

/* Transfer data on tx_buff */
void nrf24l01_tx_package(uint8_t* tx_buf)
{
        gpio_clear(ce_pin);
        nrf24l01_write_buf(WRITE_RE + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
        nrf24l01_write_buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
        nrf24l01_write_reg(WRITE_RE + CONFIG, 0x0e);
        gpio_set(ce_pin);
}


/* Write value to address reg, return value of STATUS reg */
uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
        uint8_t status;
        gpio_clear(cs_pin);                     /* Start spi on nrf24l01 */
        status = spi_tranfer_byte_bb(reg);      /* Slect register*/
        spi_tranfer_byte_bb(value);             /* Write value to register*/
        gpio_set(cs_pin);                       /* Finish SPI tranfer */

        return status;
}

/* Read length byte the value of registers start by reg to rx buffer return value of STATUS reg*/
uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t* rx_buff, uint8_t length)
{
        uint8_t status;
        gpio_clear(cs_pin);                      /* Start spi on nrf24l01 */
        status = spi_tranfer_byte_bb(reg);      /* Slect register*/

        /* Read value to buffer */
        int i;
        for (i = 0; i < length; ++i)
                rx_buff[i] = spi_tranfer_byte_bb(0);

        gpio_set(cs_pin);                        /* Finish SPI tranfer */

        return status;
}

/* Write length byte the value store in tx_buff to register start by reg, return value of STATUS reg*/
uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t* tx_buff, uint8_t length)
{
        uint8_t status;
        gpio_clear(cs_pin);                     /* Start spi on nrf24l01 */
        status = spi_tranfer_byte_bb(reg);      /* Slect register*/

        spi_tranfer_data_bb(tx_buff, length);   /* Tranfer data on tx_buff*/

        gpio_set(cs_pin);                        /* Finish SPI tranfer */

        return status;
}
