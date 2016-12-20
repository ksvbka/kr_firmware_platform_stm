/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:36:28
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-12 00:39:26
*/

#include "stm32f0xx.h"
#include "adc.h"
#include "gpio.h"

static callback g_adc_irq_callback = NULL;
static uint8_t  g_adc_mode = 0;

/* Helper function*/

/* Setup gpio pin for channel adc*/
static void adc_init_pin(uint8_t channels);

void adc_init(uint8_t channels, uint8_t mode, bool enable_interrupt_adc)
{
        /* Enable ADC clock*/
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

        ADC1->CR &= ~ADC_CR_ADEN;                /* ensure ADC is off*/
        ADC1->CR |= ADC_CR_ADCAL;                /* begin calibration*/
        while ((ADC1->CR & ADC_CR_ADCAL) != 0);  /* wait until calibration is done*/

        /* Select channel*/
        ADC1->CHSELR = channels;

        /* Setup gpio pin as analog function*/
        adc_init_pin(channels);

        /*ADC configure */
        ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_WAIT; /* continuous mode, wait until samples are read before starting next conversion*/
        ADC1->CR |= ADC_CR_ADEN;                        /* enable ADC*/
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);       /* wait until ADC is ready*/

        if (enable_interrupt_adc) {
                ADC1->IER = ADC_IER_EOCIE;
                NVIC_EnableIRQ(ADC1_COMP_IRQn);
        }

        /*  start ADC conversions*/
        ADC1->CR |= ADC_CR_ADSTART;
}

/* Returns the last ADC1 conversion result data for ADC channel */
uint16_t adc_read_single_channel(void)
{
        return (uint16_t) ADC1->DR;
}

void adc_read_multiple_channel(uint8_t chanels, uint16_t buffer_addr)
{
        /* TODO: FIX ME!!!*/
        return;
}

void adc_stop(void)
{
        ADC1->CR |= (uint32_t)ADC_CR_ADSTP;
}

void adc_irq_register_callback(callback fn_callback)
{
        g_adc_irq_callback = fn_callback;
}

/*  Function handles adc irq, will be called in ADC1_COMP_IRQHandler() implement in stm32f0xx_it.c*/
void adc_irq_handler(void)
{
        uint16_t buffer = ADC1->DR;
        g_adc_irq_callback(&buffer);

}

void gpio_init_analog(uint8_t pin)
{
        /* Enable Clock source */
        uint8_t port = GET_PORT(pin);
        gpio_module_init(1 << port);

        GPIO_InitTypeDef    GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GET_PIN(pin);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void adc_init_pin(uint8_t channels)
{
        uint8_t port = 0;
        uint8_t pin  = 0;
        int i;
        /* Setup gpio pin */
        for (i = 0; i <= 16; ++i) {
                if (channels & (1 << i)) {
                        if (i < 8) {
                                port = GPIO_PA;
                                pin = i;
                        } else if ( i >= 8 && i < 10) {
                                port = GPIO_PB;
                                pin = i - 8;
                        } else {
                                port = GPIO_PC;
                                pin = i - 10;
                        }
                        gpio_init_analog(GPIO_PIN(port, pin));
                }
        }
}
