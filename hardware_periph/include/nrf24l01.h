/*
* @Author: Kienltb
* @Date:   2016-12-29 16:50:37
* @Last Modified by:   Kienltb
* @Last Modified time: 2017-01-11 18:02:30
*/

/**
 * Implement driver for module RF NRL24L01 use SPI bitbanging.
 */

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

/* Default configrure */
#define TX_ADR_WIDTH    5      /* 5 uints TX address width*/
#define RX_ADR_WIDTH    5      /* 5 uints RX address width*/
#define TX_PLOAD_WIDTH  32     /* 32 uints TX payload*/
#define RX_PLOAD_WIDTH  32     /* 32 uints TX payload*/

/* SPI Instruction*/
#define READ_RE         0x00    /* Read register*/
#define WRITE_RE        0x20    /* Write register*/
#define RD_RX_PLOAD     0x61    /* Read RX_playload use in rx mode */
#define WR_TX_PLOAD     0xA0    /* Write TX_playload use in tx mode */
#define FLUSH_TX        0xE1    /* Flush TX FIFO, used in TX mode */
#define FLUSH_RX        0xE2    /* Flush RX FIFO, used in RX mode */
#define REUSE_TX_PL     0xE3    /* Reuse last send payload, used for PTX device*/
#define NOP             0xFF    /* No Opreation, used to read STATUS register*/

/* MRF24L01 register */
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define OBSERVE_TX      0x08
#define CD              0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17

/* Init nrf24l01 which default configure and gpio pin ce, cs, clk, miso, mosi */
void nrf24l01_init(uint8_t ce, uint8_t cs, uint8_t clk, uint8_t mosi, uint8_t miso);

/* Set NRF24L01 as RX mode*/
void nrf24l01_set_rx_mode(void);

/* Set NRF24L01 as TX mode*/
void nrf24l01_set_tx_mode(void);

/* Check FIFO buff, if have package, read to rx_buff and return 1*/
uint8_t nrf24l01_rx_package(uint8_t* rx_buf);

/* Transfer data on tx_buff */
void nrf24l01_tx_package(uint8_t* tx_buf);

#endif //__NRL24L01_H__
