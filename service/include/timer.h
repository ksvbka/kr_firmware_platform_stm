/*
* @Author: Trung Kien
* @Date:   2016-12-11 17:41:41
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-11 23:31:55
*/
#ifndef __TIMER_H__
#define __TIMER_H__

#include "system.h"
#include "typedef.h"

typedef enum {
    MILI_TIMER_ONE_TIME,
    MILI_TIMER_REPEAT,
    SECOND_TIMER_ONE_TIME,
    SECOND_TIMER_REPEAT
} timer_type_t;

bool timer_create(timer_type_t type, uint16_t timeout, callback timer_callback, void* param);

void timer_delete(void* timer_callback);

bool timer_is_running(void* timer_callback);

void handle_timer_events(void);

#endif //__TIMER_H__
