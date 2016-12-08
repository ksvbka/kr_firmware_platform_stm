#include "gpio.h"

static GPIO_TypeDef* get_gpio_port(uint8_t gpio)
{
    GPIO_TypeDef* GPIOx = NULL;
    switch (GET_PORT(gpio)) {
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

    GPIO_InitStructure.GPIO_Pin   = GET_PIN(pin); /* Setup pin*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /* Default 50MHz*/
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  /* Default NOPULL*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    /*Config direction*/
    if (mode <= GPIO_IN_PU)// Input mode
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    else// Output mode
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

    /*Config pull up - pull down */
    /*  Pull up*/
    if (mode == GPIO_IN_PU || mode == GPIO_OUT_PU)
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    /*  pull down*/
    if (mode == GPIO_IN_PD || mode == GPIO_OUT_PD)
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
    bool state = GPIO_ReadOutputDataBit(GPIOx, GET_PIN(pin));
    if (state)
        gpio_clear(pin);
    else
        gpio_set(pin);
}

void gpio_write(uint8_t pin, bool value)
{
    (value == TRUE) ? gpio_set(pin) : gpio_clear(pin);
}
