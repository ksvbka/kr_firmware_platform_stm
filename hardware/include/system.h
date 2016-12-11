/*
* @Author: Trung Kien
* @Date:   2016-12-11 20:11:27
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-11 23:31:10
*/
#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#define CLOCK_SOURCE_HSI        0  /* HSI (~8MHz) used to clock the PLL, and the PLL is used as system clock source*/
#define CLOCK_SOURCE_HSE        1  /* HSE (8MHz) used to clock the PLL, and the PLL is used as system clock source*/
#define CLOCK_SOURCE_HSE_BYPASS 2  /* HSE bypassed with an external clock (8MHz, coming from ST-Link) used to clock*/
                                   /* the PLL, and the PLL is used as system clock source*/
void system_init(uint8_t clock_source);

void update_tick_count();

uint32_t get_tick_count();

uint32_t get_second_count();

void delay_ms(uint32_t time_ms);

#endif //__SYSTEM_H__
