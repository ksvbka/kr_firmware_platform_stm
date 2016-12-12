/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:28:56
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-11 23:31:03
*/

#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f0xx_gpio.h"
#include "typedef.h"

/**
 *  Use uint8_t to encode gpio.
 *  Bit 0-4 use for gpio pin number (0 - 15)
 *  Bit 5-7 use for GPIO_TypeDef (A,B,C,D,F)
 *
 *  To define a gpio use Macro as folow
 *  uint8_t led = GPIO_PIN(GPIO_PA, 15) //RCC_AHBPeriph_GPIOA and GPIO_Pin_15
 */

/* Define port of GPIO - RCC_AHBPeriph_GPIOx (x = A,B,C,D,F)*/
#define GPIO_PA 1
#define GPIO_PB 2
#define GPIO_PC 3
#define GPIO_PD 4
#define GPIO_PF 5

/* Define option use register pull up/down as internal (TRUE) or external (FALSE)*/
#define USE_INTERNAL_RES        FALSE

/* Define Port user for irq, default is GPIO_PA => PAx (x = 0:15) can config as gpio irq*/
#define GPIO_Px_IRQ     GPIO_PA

/* Define option for enable module GPIOx (x = A,B,C,D,F)*/
#define GPIO_PA_ENABLE (GPIO_PA << 0)
#define GPIO_PB_ENABLE (GPIO_PB << 1)
#define GPIO_PC_ENABLE (GPIO_PC << 2)
#define GPIO_PD_ENABLE (GPIO_PD << 3)
#define GPIO_PF_ENABLE (GPIO_PF << 4)

#define PORT_OFFSET 5 /*bit*/

#define GPIO_MASK(pin) ((uint16_t)(1 << (pin)))

#define GPIO_PIN(port, pin)  ((uint8_t)( (port << PORT_OFFSET) | pin ))
#define GET_PORT(gpio)  ((uint8_t)(gpio >> PORT_OFFSET))
#define GET_PIN(gpio)  (GPIO_MASK(gpio & 0x1F))

/* GPIO mode */
#define  GPIO_IN        0   /* Input*/
#define  GPIO_IN_PD     1   /* Input with pull down register*/
#define  GPIO_IN_PU     2   /* Input with pull up register*/
#define  GPIO_OUT       3   /* Output*/
#define  GPIO_OUT_PD    4   /* Output with pull down register*/
#define  GPIO_OUT_PU    5   /* Output with pull up register*/

/* Edge interrupt */
#define  GPIO_FALLING           0
#define  GPIO_RISING            1
#define  GPIO_RISING_FALLING    2

void gpio_module_init(uint8_t port_enable); /*eg: PORT_A + PORT_C*/

bool gpio_init(uint8_t pin, uint8_t mode);

bool gpio_read(uint8_t pin);

uint16_t gpio_read_port(uint8_t port);

void gpio_set(uint8_t pin);

void gpio_clear(uint8_t pin);

void gpio_toggle(uint8_t pin);

void gpio_write(uint8_t pin, bool value);

void gpio_write_port(uint8_t port, uint16_t port_value);

bool gpio_init_irq(uint8_t pin, uint8_t edge);

void gpio_irq_register_callback(callback fn_callback);

void gpio_irq_handler(void);

#endif //__GPIO_H__
