/*
* @Author: Trung Kien
* @Date:   2016-12-11 20:13:34
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-11 23:31:34
*/
#include "uart.h"
#include "stm32f0xx.h"

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

#define TX_PIN (GPIO_Pin_9)
#define RX_PIN (GPIO_Pin_10)

#ifdef USE_UART1_MODULE
        #define TX_SOURCE               (GPIO_PinSource9)
        #define RX_SOURCE               (GPIO_PinSource10)
        #define USARTx                  (USART1)
        #define RCC_APB2Periph_USARTx   (RCC_APB2Periph_USART1)
#define USARTx_IRQn             (USART1_IRQn)

// #else /*Use UART2 module*/
// #define TX_SOURCE               (GPIO_PinSource2)
// #define RX_SOURCE               (GPIO_PinSource3)
// #define USARTx                  (USART2)
// #define RCC_APB2Periph_USARTx   (RCC_APB2Periph_USART2)
// #define USARTx_IRQn             (USART2_IRQn)
#endif


/* Global callback function variable*/
static callback g_uart_callback =  NULL;

/* Helper function */
uint32_t convert_baudrate(baudrate_t baudrate); /* Convert baudrate_t to uint32_t*/

void uart_init(baudrate_t baudrate, bool enable_interrupt)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTx, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        GPIO_PinAFConfig(GPIOA, TX_SOURCE, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, RX_SOURCE, GPIO_AF_1);

        /* Configure USART Tx and Rx pin as alternate function  */
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = TX_PIN | RX_PIN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Configure USART param*/
        USART_InitTypeDef USART_InitStructure;
        USART_InitStructure.USART_BaudRate = convert_baudrate(baudrate);
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

        USART_Init(USARTx, &USART_InitStructure);
        USART_Cmd(USARTx, ENABLE);

        /* Config Interrupt */
        if (enable_interrupt) {
                USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
                NVIC_EnableIRQ(USARTx_IRQn);
        }
}

void uart_putc(char c)
{
        // Wait for Transmit data register empty
        while ((USARTx->ISR & USART_ISR_TXE) == 0) {}

        // Transmit data by writing to TDR, clears TXE flag
        USARTx->TDR = c;
}
void uart_write(const char *data_buffer)
{
        while (*data_buffer != '\0') {
                if (*data_buffer == '\n') {
                        uart_putc('\r');
                }
                uart_putc(*data_buffer++);
        }
}


void uart_irq_register_callback(callback fnCallback)
{
        g_uart_callback = fnCallback;
}

/* TODO: Need to check*/
void uart_irq_handler(void)
{
        if (USART_GetITStatus(USARTx, USART_IT_RXNE)) {
                char buffer = USARTx->RDR;
                g_uart_callback(&buffer);
        }
}

/* Implement helper function*/

uint32_t convert_baudrate(baudrate_t baudrate)
{
        switch (baudrate) {
        case UART_BAUDRATE_9600:
                return 9600;
        case UART_BAUDRATE_38400:
                return 38400;
        case UART_BAUDRATE_56000:
                return 56000;
        case UART_BAUDRATE_115200:
                return 115200;
        case UART_BAUDRATE_128000:
                return 128000;
        case UART_BAUDRATE_256000:
                return 256000;
        default:
                return 115200;
        }
}
