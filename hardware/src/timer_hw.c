/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:33:44
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-20 22:12:26
*/

#include "stm32f0xx.h"
#include "timer_hw.h"

static callback g_callback_timer = NULL;

void timer_hw_init(void)
{
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

}

void timer_hw_start(uint32_t time , callback task)
{
        g_callback_timer = task;

        /* Time base configuration */
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

        TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock) / 1000000) - 1; //frequency=1000000
        TIM_TimeBaseStructure.TIM_Period = time - 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
        TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
        TIM_Cmd(TIM3, ENABLE);
}

void timer_hw_stop(void* param)
{
        TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
        TIM_Cmd(TIM3, DISABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
        g_callback_timer = NULL;
}

void timer_hw_irq_handler(void)
{
        if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
                g_callback_timer(NULL);
                TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        }
}

