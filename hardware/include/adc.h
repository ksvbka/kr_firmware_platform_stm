/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:33:44
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-12 00:40:24
*/
#ifndef __ADC_H__
#define __ADC_H__

#include "stm32f0xx.h"
#include "typedef.h"

/**
 * Possible GPIO usage:
 *
 * PA0          ADC_IN0
 * PA1          ADC_IN1
 * PA2          ADC_IN2
 * PA3          ADC_IN3
 * PA4          ADC_IN4
 * PA5          ADC_IN5
 * PA6          ADC_IN6
 * PA7          ADC_IN7
 *
 * PB0          ADC_IN8
 * PB1          ADC_IN9
 *
 * PC0          ADC_IN10
 * PC1          ADC_IN11
 * PC2          ADC_IN12
 * PC3          ADC_IN13
 * PC4          ADC_IN14
 * PC5          ADC_IN15
 */

#define  ADC_CHA0           ((uint32_t)(1 << 0))
#define  ADC_CHA1           ((uint32_t)(1 << 1))
#define  ADC_CHA2           ((uint32_t)(1 << 2))
#define  ADC_CHA3           ((uint32_t)(1 << 3))
#define  ADC_CHA4           ((uint32_t)(1 << 4))
#define  ADC_CHA5           ((uint32_t)(1 << 5)
#define  ADC_CHA6           ((uint32_t)(1 << 6))
#define  ADC_CHA7           ((uint32_t)(1 << 7))
#define  ADC_CHA8           ((uint32_t)(1 << 8))
#define  ADC_CHA9           ((uint32_t)(1 << 9))
#define  ADC_CHA10          ((uint32_t)(1 << 10))
#define  ADC_CHA11          ((uint32_t)(1 << 11))
#define  ADC_CHA12          ((uint32_t)(1 << 12))
#define  ADC_CHA13          ((uint32_t)(1 << 13))
#define  ADC_CHA14          ((uint32_t)(1 << 14))
#define  ADC_CHA15          ((uint32_t)(1 << 15))
#define  ADC_CHA16          ((uint32_t)(1 << 16))

#define ADC_SINGLE_CHANNEL              1
#define ADC_SEQUENCE_CHANNEL            2
#define ADC_REPEAT_SINGLE_CHANNEL       3
#define ADC_REPEAT_SEQUENCE_CHANNEL     4

#define ENABLE_INTERRUPT_ADC            1
#define DISABLE_INTERRUPT_ADC           0

void adc_init(uint8_t channels, uint8_t mode, bool enable_interrupt_adc);

uint16_t adc_read_single_channel(void);

void adc_read_multiple_channel(uint8_t chanels, uint16_t buffer_addr);

void adc_stop(void);

void adc_irq_register_callback(callback fn_callback);

#endif// __ADC_H__
