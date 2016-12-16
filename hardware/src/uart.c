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
 *      TX_PIN GPIOA_9  or GPIOB_6
 *      RX_PIN GPIOA_10 or GPIOB_7
 *  USART2:
 *      TX_PIN GPIO_Pin_2
 *      RX_PIN GPIO_Pin_3
 *
 *  Note: stm32f030 only have USART1 module
 */

#ifdef USE_UART1_MODULE
        #define USARTx                  (USART1)
        #define RCC_APB2Periph_USARTx   (RCC_APB2Periph_USART1)
        #define USARTx_IRQn             (USART1_IRQn)
// #else /*Use UART2 module*/
//      #define USARTx                  (USART2)
//      #define RCC_APB2Periph_USARTx   (RCC_APB2Periph_USART2)
//      #define USARTx_IRQn             (USART2_IRQn)
#endif

#ifdef USE_UART_PIN_A9_A10
        #define TX_PIN                  (GPIO_Pin_9)
        #define RX_PIN                  (GPIO_Pin_10)
        #define TX_SOURCE               (GPIO_PinSource9)
        #define RX_SOURCE               (GPIO_PinSource10)
        #define GPIO_PORT               (GPIOA)
        #define RCC_AHBPeriph_GPIOx     (RCC_AHBPeriph_GPIOA)
        #define GPIO_UART               (GPIO_AF_1)
#endif

#ifdef USE_UART_PIN_B6_B7
        #define TX_PIN                  (GPIO_Pin_6)
        #define RX_PIN                  (GPIO_Pin_7)
        #define TX_SOURCE               (GPIO_PinSource6)
        #define RX_SOURCE               (GPIO_PinSource7)
        #define GPIO_PORT               (GPIOB)
        #define RCC_AHBPeriph_GPIOx     (RCC_AHBPeriph_GPIOB)
        #define GPIO_UART               (GPIO_AF_0)
#endif




/* Global callback function variable*/
static callback g_uart_callback =  NULL;

/* Helper function */
uint32_t convert_baudrate(baudrate_t baudrate); /* Convert baudrate_t to uint32_t*/

void uart_init(baudrate_t baudrate, bool enable_interrupt)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTx, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOx, ENABLE);

        GPIO_PinAFConfig(GPIO_PORT, TX_SOURCE, GPIO_UART);
        GPIO_PinAFConfig(GPIO_PORT, RX_SOURCE, GPIO_UART);

        /* Configure USART Tx and Rx pin as alternate function  */
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = TX_PIN | RX_PIN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; /* alternate mode*/
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

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

/*  Function handles uart irq, will be called in USART1_IRQHandler() implement in stm32f0xx_it.c*/
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
