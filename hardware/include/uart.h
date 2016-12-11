#ifndef __UART_H__
#define __UART_H__

#include "typedef.h"

/**
 *  USART1:
 *      TX_PIN GPIO_Pin_9
 *      RX_PIN GPIO_Pin_10
 *  USART2:
 *      TX_PIN GPIO_Pin_2
 *      RX_PIN GPIO_Pin_3
 *
 *  Note: Only use module USART1 on stm32f030
 */

/* Define module you want to use, UART1 or UART2*/
#define USE_UART1_MODULE 1
// #define UART_UART2_MODULE 1

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

void uart_irq_register_callback(callback fnCallback);

#endif// __UART_H__
