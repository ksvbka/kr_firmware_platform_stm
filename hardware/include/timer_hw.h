/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:33:44
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-23 11:03:51
*/

/**
 * Timer hardware module, use for critical task which high resolution timer (us)
 *
 * Implement base TIM3
 */

#ifndef __TIMER_HW_H__
#define __TIMER_HW_H__

#include "typedef.h"

void timer_hw_init(void);

void timer_hw_start(uint32_t time , callback task, void* param);

void timer_hw_stop(void* param);

void timer_hw_irq_handler(void);

#endif //__TIMER_HW_H__
