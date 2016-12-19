/*
* @Author: Trung Kien
* @Date:   2016-12-11 20:13:34
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-11 23:31:34
*/

#include "stm32f0xx.h"
#include "uart.h"
#include "gpio.h"

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

/* Define USART1 pin*/
/* Option 1: GPIOB_6 GPIOB_7*/
#define TX_PIN          GPIO_PIN(GPIO_PB, 6)
#define RX_PIN          GPIO_PIN(GPIO_PB, 7)
#define UART_FUNCTION   AF0

/* Option 2: GPIOA_9 GPIOA_10*/
// #define TX_PIN          GPIO_PIN(GPIO_PA, 9)
// #define RX_PIN          GPIO_PIN(GPIO_PA, 10)
// #define UART_FUNCTION   AF1

/* Global callback function variable*/
static callback g_uart_callback =  NULL;

/* Helper function */
uint32_t convert_baudrate(baudrate_t baudrate); /* Convert baudrate_t to uint32_t*/

void uart_init(baudrate_t baudrate, bool enable_interrupt)
{
        /* Turn on clock for UARTx module*/
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTx, ENABLE);

        /* Define uart pin*/
        gpio_init_function(TX_PIN, UART_FUNCTION, PUSH_PULL, PULL_UP);
        gpio_init_function(RX_PIN, UART_FUNCTION, PUSH_PULL, PULL_UP);

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
/* Helper function*/
char* int_to_string(int i);

char* float_to_string(double d);

void uart_printf(char s[], ...)
{
        va_list arglist;
        va_start(arglist, s);

        // parse the format string
        uint16_t i = 0;
        while (s[i] != 0) {
                if (s[i] != '%') { // normal char
                        if (s[i] == '\n')
                                uart_putc('\r');
                        uart_putc(s[i]);
                        i++;
                        continue;
                }

                i++; // skip past the % to the format specifier
                switch (s[i]) {
                case '%':
                        uart_putc(s[i]);
                        i++;
                        break;
                case 'd': // int16
                        uart_write(int_to_string(va_arg(arglist, int)));
                        i++;
                        break;
                case 'f':
                        uart_write(float_to_string(va_arg(arglist, double)));
                        i++;
                        break;
                case 's':
                        uart_write(va_arg(arglist, char*));
                        i++;
                        break;
                default:
                        return; // invalid format specifier
                }
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


char* int_to_string(int i)
{
        static char buf[10];
        char* pbuf = buf;

        char sign = '+';
        short len = 0;

        if (i < 0) {
                sign = '-';
                i = -i;
        }
        do {
                *pbuf++ = (i % 10) + '0';
                len++;
                i /= 10;
        } while (i != 0);

        if (sign == '-') {
                *pbuf++ = '-';
                len++;
        }

        for (i = 0; i < len / 2; i++) {
                buf[len] = buf[i];
                buf[i] = buf[len - 1 - i];
                buf[len - 1 - i] = buf[len];
        }
        buf[len] = 0;
        return buf;
}

char* float_to_string(double d)
{
        int wholePart = (int) d;
        int precision = 3; /*Default 3 ditgit*/

       /* Deposit the whole part of the number.*/
        char* buffer = int_to_string(wholePart);

        /* Now work on the faction if we need one.*/
        if (precision > 0) {

                /* We do, so locate the end of the string and insert a decimal point.*/
                char *endOfString = buffer;
                while (*endOfString != '\0') endOfString++;
                *endOfString++ = '.';

                /* Now work on the fraction, be sure to turn any negative values positive.*/
                if (d < 0) {
                        d *= -1;
                        wholePart *= -1;
                }
                double fraction = d - wholePart;
                while (precision > 0) {
                        /* Multipleby ten and pull out the digit.*/
                        fraction *= 10;
                        wholePart = (long) fraction;
                        *endOfString++ = '0' + wholePart;

                        /* Update the fraction and move on to the next digit.*/
                        fraction -= wholePart;
                        precision--;
                }
                /* Terminate the string.*/
                *endOfString = '\0';
        }

        return buffer;
}
