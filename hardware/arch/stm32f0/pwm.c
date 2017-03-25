/*
* @Author: Trung Kien
* @Date:   2016-12-15 20:58:33
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-27 10:38:00
*/

#include "pwm.h"
#include "gpio.h"

static uint16_t timer_period = 0;

static uint8_t channel1_pin = GPIO_PIN(GPIO_PA, 8);
static uint8_t channel2_pin = GPIO_PIN(GPIO_PA, 9);
static uint8_t channel3_pin = GPIO_PIN(GPIO_PA, 10);
static uint8_t channel4_pin = GPIO_PIN(GPIO_PA, 11);


/* Init pwm module, channels canbe init as multi channel.
*  Eg:
*       pwm_init(PWM_CHANNEL1 + PWM_CHANNEL_2, 10000);
*/
void pwm_init(uint8_t channel, uint16_t frequency)
{
        timer_period = (SystemCoreClock / frequency) - 1;

        /* TIM1 clock enable */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

        /* Time Base configuration */
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

        TIM_TimeBaseStructure.TIM_Prescaler = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = timer_period;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

        TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

        /* Channel 1, 2, 3 and 4 Configuration in PWM mode */
        TIM_OCInitTypeDef  TIM_OCInitStructure;
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_Pulse = 0; /* Default is 0*/

        /* Configure output chanel*/
        if (channel & PWM_CHANNEL_1) {
                TIM_OC1Init(TIM1, &TIM_OCInitStructure);
                TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

                /* Configure GPIOA8 as TIM1 output*/
                gpio_init_function(channel1_pin, AF2, PUSH_PULL, PULL_UP);
        }
        if (channel & PWM_CHANNEL_2) {
                TIM_OC2Init(TIM1, &TIM_OCInitStructure);
                TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

                /* Configure GPIOA9 as TIM1 output*/
                gpio_init_function(channel2_pin, AF2, PUSH_PULL, PULL_UP);
        }
        if (channel & PWM_CHANNEL_3) {
                TIM_OC3Init(TIM1, &TIM_OCInitStructure);
                TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

                /* Configure GPIOA10 as TIM1 output*/
                gpio_init_function(channel3_pin, AF2, PUSH_PULL, PULL_UP);
        }
        if (channel & PWM_CHANNEL_4) {
                TIM_OC4Init(TIM1, &TIM_OCInitStructure);
                TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

                /* Configure GPIOA11 as TIM1 output*/
                gpio_init_function(channel4_pin, AF2, PUSH_PULL, PULL_UP);
        }

        /* TIM1 counter enable */
        TIM_Cmd(TIM1, ENABLE);

        /* TIM1 Main Output Enable */
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/* Set duty cycle in [0 : 100]*/
void pwm_set_duty(uint8_t channel, uint16_t duty_cycle)
{
        if (duty_cycle > 100)
                duty_cycle = 100;

        uint16_t pulse = (uint16_t) (((uint32_t) duty_cycle * (timer_period - 1)) / 100);
        if (channel & PWM_CHANNEL_1)
                TIM1->CCR1 = pulse;
        if (channel & PWM_CHANNEL_2)
                TIM1->CCR2 = pulse;
        if (channel & PWM_CHANNEL_3)
                TIM1->CCR3 = pulse;
        if (channel & PWM_CHANNEL_4)
                TIM1->CCR4 = pulse;
}
