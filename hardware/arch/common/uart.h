/*
* @Author: Trung Kien
* @Date:   2016-12-11 20:12:21
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-19 20:53:56
*/
#ifndef __UART_H__
#define __UART_H__

#include "typedef.h"
#include "stdarg.h"

/**
 *  USART1:
 *      TX_PIN GPIOA_9  or GPIOB_6
 *      RX_PIN GPIOA_10 or GPIOB_7
 *  USART2:
 *      TX_PIN GPIO_Pin_2
 *      RX_PIN GPIO_Pin_3
 *
 *  Note: Only use module USART1 on stm32f030
 */

/* Define module you want to use, UART1 or UART2*/
#define USE_UART1_MODULE        1
// #define UART_UART2_MODULE    1

#ifdef USE_UART1_MODULE
        #define USARTx                  (USART1)
        #define RCC_APB2Periph_USARTx   (RCC_APB2Periph_USART1)
        #define USARTx_IRQn             (USART1_IRQn)
#else /*Use UART2 module*/
        #define USARTx                  (USART2)
        #define RCC_APB2Periph_USARTx   (RCC_APB2Periph_USART2)
        #define USARTx_IRQn             (USART2_IRQn)
#endif

/* Define USART1 pin*/
/* Option 1: GPIOB_6 GPIOB_7*/
#define TX_PIN          GPIO_PIN(GPIO_PB, 6)
#define RX_PIN          GPIO_PIN(GPIO_PB, 7)
#define UART_FUNCTION   AF0

/* Option 2: GPIOA_9 GPIOA_10*/
// #define TX_PIN          GPIO_PIN(GPIO_PA, 9)
// #define RX_PIN          GPIO_PIN(GPIO_PA, 10)
// #define UART_FUNCTION   AF1

#define UART_ENABLE_INT      1
#define UART_DISABLE_INT     0

typedef enum {
        UART_BAUDRATE_9600 = 0,
        UART_BAUDRATE_19200,
        UART_BAUDRATE_38400,
        UART_BAUDRATE_56000,
        UART_BAUDRATE_115200,
        UART_BAUDRATE_128000,
        UART_BAUDRATE_256000
} baudrate_t;

void uart_init(baudrate_t baudrate, bool enable_interrupt);

void uart_putc(char c);

void uart_write(const char *data_buffer);

void uart_printf(char s[], ...);

void uart_irq_register_callback(callback fnCallback);

#endif// __UART_H__
