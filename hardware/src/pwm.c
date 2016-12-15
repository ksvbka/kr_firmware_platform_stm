/*
* @Author: Trung Kien
* @Date:   2016-12-15 20:58:33
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-15 21:40:58
*/

#include "pwm.h"

static uint16_t timer_period = 0;

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
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_Pulse = 0; /* Default is 0*/

        /* Configure GPIO as output */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        /* Configure output chanel*/
        if (channel & PWM_CHANNEL_1) {
                TIM_OC1Init(TIM1, &TIM_OCInitStructure);

                /* Configure GPIOA8 as TIM1 output*/
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
                GPIO_Init(GPIOA, &GPIO_InitStructure);
                GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
        }
        if (channel & PWM_CHANNEL_2) {
                TIM_OC2Init(TIM1, &TIM_OCInitStructure);

                /* Configure GPIOA8 as TIM1 output*/
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
                GPIO_Init(GPIOA, &GPIO_InitStructure);
                GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
        }
        if (channel & PWM_CHANNEL_3) {
                TIM_OC3Init(TIM1, &TIM_OCInitStructure);

                /* Configure GPIOA8 as TIM1 output*/
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
                GPIO_Init(GPIOA, &GPIO_InitStructure);
                GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
        }
        if (channel & PWM_CHANNEL_4) {
                TIM_OC4Init(TIM1, &TIM_OCInitStructure);

                /* Configure GPIOA8 as TIM1 output*/
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
                GPIO_Init(GPIOA, &GPIO_InitStructure);
                GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
        }

        /* TIM1 counter enable */
        TIM_Cmd(TIM1, ENABLE);

        /* TIM1 Main Output Enable */
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
void pwm_set_duty(uint8_t channel, uint16_t duty_cycle)
{
        if (duty_cycle > 1000)
                duty_cycle = 1000;
        if (channel & PWM_CHANNEL_1)
                TIM1->CCR1 = (uint16_t)(((uint32_t)duty_cycle * (timer_period) - 1) / 1000);
        if (channel & PWM_CHANNEL_2)
                TIM1->CCR2 = (uint16_t)(((uint32_t)duty_cycle * (timer_period) - 1) / 1000);
        if (channel & PWM_CHANNEL_3)
                TIM1->CCR3 = (uint16_t)(((uint32_t)duty_cycle * (timer_period) - 1) / 1000);
        if (channel & PWM_CHANNEL_4)
                TIM1->CCR4 = (uint16_t)(((uint32_t)duty_cycle * (timer_period) - 1) / 1000);
}
