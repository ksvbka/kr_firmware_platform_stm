/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:28:00
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-11 23:30:38
*/

#include "gpio.h"

/* Helper function*/
static GPIO_TypeDef* get_port(uint8_t port);      /* get GPIO_TypeDef* from port define */
static GPIO_TypeDef* get_gpio_port(uint8_t gpio); /* get GPIO_TypeDef* from gpio define */
static uint8_t get_pin_source(uint8_t gpio);      /* get pin_soure for setup interrupt*/
static uint8_t get_exti_port_source(uint8_t gpio); /*Get port source for setup interrupt*/


void gpio_module_init(uint8_t port_enable)
{
        if (port_enable & GPIO_PA_ENABLE)
                RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
        if (port_enable & GPIO_PB_ENABLE)
                RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
        if (port_enable & GPIO_PC_ENABLE)
                RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        if (port_enable & GPIO_PD_ENABLE)
                RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
        if (port_enable & GPIO_PF_ENABLE)
                RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
}

bool gpio_init(uint8_t pin, uint8_t mode)
{
        GPIO_InitTypeDef  GPIO_InitStructure;

        GPIO_InitStructure.GPIO_Pin   = GET_PIN(pin);       /* Setup pin*/
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   /* Default 50MHz*/
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   /* Default NOPULL*/
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

        /*Config direction*/
        if (mode <= GPIO_IN_PU)// Input mode
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        else// Output mode
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

        /*Config pull up - pull down */
        if (mode == GPIO_IN_PU || mode == GPIO_OUT_PU)     /*  Pull up*/
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

        if (mode == GPIO_IN_PD || mode == GPIO_OUT_PD)     /*  pull down*/
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

        GPIO_TypeDef* GPIOx = get_gpio_port(pin);
        GPIO_Init(GPIOx, &GPIO_InitStructure);

        return 1;
}

bool gpio_read(uint8_t pin)
{
        GPIO_TypeDef* GPIOx = get_gpio_port(pin);
        return GPIO_ReadInputDataBit(GPIOx, GET_PIN(pin));
}


uint16_t gpio_read_port(uint8_t port)
{
        GPIO_TypeDef* GPIOx = get_port(port);
        return GPIO_ReadInputData(GPIOx);
}

void gpio_set(uint8_t pin)
{
        GPIO_TypeDef* GPIOx = get_gpio_port(pin);
        GPIO_SetBits(GPIOx, GET_PIN(pin));
}

void gpio_clear(uint8_t pin)
{
        GPIO_TypeDef* GPIOx = get_gpio_port(pin);
        GPIO_ResetBits(GPIOx, GET_PIN(pin));
}

void gpio_toggle(uint8_t pin)
{
        GPIO_TypeDef* GPIOx = get_gpio_port(pin);
        GPIOx->ODR ^= GET_PIN(pin);
}

void gpio_write(uint8_t pin, bool value)
{
        (value == TRUE) ? gpio_set(pin) : gpio_clear(pin);
}

void gpio_write_port(uint8_t port, uint16_t port_value)
{
        GPIO_TypeDef* GPIOx = get_port(port);
        return GPIO_Write(GPIOx, port_value);
}


bool gpio_init_irq(uint8_t pin, uint8_t edge)
{
        /* Config gpio as input*/
        if (USE_INTERNAL_RES == TRUE)
                /* Use internal pullup*/
                gpio_init(pin, GPIO_IN_PU);
        else
                /* Use external pullup*/
                gpio_init(pin, GPIO_IN);

        /* Enable SYSCFG clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Connect EXTIx Line to pin */
        uint8_t port_source = get_exti_port_source(pin);
        uint8_t pin_soure   = get_pin_source(pin);
        SYSCFG_EXTILineConfig(port_source, pin_soure);

        /* Configure EXTIx line (x = 0: 15)*/
        EXTI_InitTypeDef   EXTI_InitStructure;

        // EXTI_InitStructure.EXTI_Line = EXTI_Line0;
        EXTI_InitStructure.EXTI_Line = (uint32_t)GET_PIN(pin);
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;

        if (edge == GPIO_FALLING)
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        if (edge == GPIO_RISING)
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        if (edge == GPIO_RISING_FALLING)
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;

        EXTI_Init(&EXTI_InitStructure);

        /* Enable and set EXTIx Interrupt */
        NVIC_InitTypeDef   NVIC_InitStructure;
        if ((pin & 0x1F) <= 1)
                NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
        else if ((pin & 0x1F) <= 3)
                NVIC_InitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
        else
                NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        return 1;
}

static callback g_gpio_irq_callback;
void gpio_irq_register_callback(callback fn_callback)
{
        g_gpio_irq_callback = fn_callback;
}

void gpio_irq_exti0_1_handler(void)
{
        /* Determine which line have interrupt*/
        uint8_t line_irq;
        if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
                line_irq = 0;
                /* Clear the EXTI line 0 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line0);
        } else if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
                line_irq = 1;
                /* Clear the EXTI line 1 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line1);
        }
        uint8_t gpio_irq = GPIO_PIN(GPIO_Px_IRQ, line_irq);
        g_gpio_irq_callback(&gpio_irq);

}

void gpio_irq_exti2_3_handler(void)
{
        /* Determine which line have interrupt*/
        uint8_t line_irq;
        if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
                line_irq = 2;
                /* Clear the EXTI line 2 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line2);
        } else {
                line_irq = 3;
                /* Clear the EXTI line 3 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line3);
        }
        uint8_t gpio_irq = GPIO_PIN(GPIO_Px_IRQ, line_irq);
        g_gpio_irq_callback(&gpio_irq);

}

void gpio_irq_exti4_15_handler(void)
{
        /* Determine which line have interrupt*/
        uint8_t line_irq;

        if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
                line_irq = 4;
                /* Clear the EXTI line 4 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line4);
        } else if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
                line_irq = 5;
                /* Clear the EXTI line 5 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line5);
        } else if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
                line_irq = 6;
                /* Clear the EXTI line 6 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line6);
        } else if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
                line_irq = 7;
                /* Clear the EXTI line 7 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line7);
        } else if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
                line_irq = 8;
                /* Clear the EXTI line 8 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line8);
        } else if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
                line_irq = 9;
                /* Clear the EXTI line 9 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line9);
        } else if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
                line_irq = 10;
                /* Clear the EXTI line 10 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line10);
        } else if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
                line_irq = 11;
                /* Clear the EXTI line 11 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line11);
        } else if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
                line_irq = 12;
                /* Clear the EXTI line 12 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line12);
        } else if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
                line_irq = 13;
                /* Clear the EXTI line 13 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line13);
        } else if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
                line_irq = 14;
                /* Clear the EXTI line 14 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line14);
        } else if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
                line_irq = 15;
                /* Clear the EXTI line 15 pending bit */
                EXTI_ClearITPendingBit(EXTI_Line15);
        }
        uint8_t gpio_irq = GPIO_PIN(GPIO_Px_IRQ, line_irq);
        g_gpio_irq_callback(&gpio_irq);
}

/* Implement Helper function */

GPIO_TypeDef* get_port(uint8_t port)
{
        GPIO_TypeDef* GPIOx = NULL;

        switch (port) {
        case GPIO_PA:
                GPIOx =  (GPIO_TypeDef *) GPIOA_BASE;
                break;
        case GPIO_PB:
                GPIOx =  (GPIO_TypeDef *) GPIOB_BASE;
                break;
        case GPIO_PC:
                GPIOx =  (GPIO_TypeDef *) GPIOC_BASE;
                break;
        case GPIO_PD:
                GPIOx =  (GPIO_TypeDef *) GPIOD_BASE;
                break;
        case GPIO_PF:
                GPIOx =  (GPIO_TypeDef *) GPIOF_BASE;
                break;
        }
        return GPIOx;
}

GPIO_TypeDef* get_gpio_port(uint8_t gpio)
{
        uint8_t port = GET_PORT(gpio);
        return get_port(port);
}

/**
 * #define EXTI_PinSource0            ((uint8_t)0x00)
 * #define EXTI_PinSource1            ((uint8_t)0x01)
 * #define EXTI_PinSource2            ((uint8_t)0x02)
 * #define EXTI_PinSource3            ((uint8_t)0x03)
 * #define EXTI_PinSource4            ((uint8_t)0x04)
 * #define EXTI_PinSource5            ((uint8_t)0x05)
 * #define EXTI_PinSource6            ((uint8_t)0x06)
 * #define EXTI_PinSource7            ((uint8_t)0x07)
 * #define EXTI_PinSource8            ((uint8_t)0x08)
 * #define EXTI_PinSource9            ((uint8_t)0x09)
 * #define EXTI_PinSource10           ((uint8_t)0x0A)
 * #define EXTI_PinSource11           ((uint8_t)0x0B)
 * #define EXTI_PinSource12           ((uint8_t)0x0C)
 * #define EXTI_PinSource13           ((uint8_t)0x0D)
 * #define EXTI_PinSource14           ((uint8_t)0x0E)
 * #define EXTI_PinSource15           ((uint8_t)0x0F)
 */

static uint8_t get_pin_source(uint8_t gpio)
{
        return ((uint8_t)(gpio & 0x1F));
}

/**
 * Define port_source in stm32f0xx_syscfg.h
 * #define EXTI_PortSourceGPIOA       ((uint8_t)0x00)
 * #define EXTI_PortSourceGPIOB       ((uint8_t)0x01)
 * #define EXTI_PortSourceGPIOC       ((uint8_t)0x02)
 * #define EXTI_PortSourceGPIOD       ((uint8_t)0x03)
 * #define EXTI_PortSourceGPIOF       ((uint8_t)0x05)
 *
 */

static uint8_t get_exti_port_source(uint8_t gpio)
{
        uint8_t EXTI_PortSourceGPIOx;
        switch (GET_PORT(gpio)) {
        case GPIO_PA:
                EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOA;
                break;
        case GPIO_PB:
                EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOB;
                break;
        case GPIO_PC:
                EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOC;
                break;
        case GPIO_PD:
                EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOD;
                break;
        case GPIO_PF:
                EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOF;
                break;
        }
        return EXTI_PortSourceGPIOx;
}
