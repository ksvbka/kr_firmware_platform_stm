#include "timer.h"

#define MAX_TIME_COUNT 10

/* Timer structure: */
typedef struct {
    timer_type_t    type;          /* second timer or milisecond timer, repeat or not*/
    uint16_t        periodic;      /* timeout value*/
    uint16_t        stop_point;    /* tick number when expries*/
    callback        timer_callback;/* Callback function when timer expries*/
    void*           param;         /* Param for callback*/
} timer_t;

/* Global internal timer list */
static timer_t timer_list[MAX_TIME_COUNT];

bool timer_create(timer_type_t type, uint16_t timeout, callback fn_callback, void* param) {
    /* Find a free timer */
    uint8_t i;
    for (i = 0; i < MAX_TIME_COUNT; ++i) {
        if (timer_list[i].timer_callback == NULL)
            break;
    }

    /* Full timer, cannot add more timer*/
    if ( i == MAX_TIME_COUNT ) return FALSE;

    /* Regist new timer */
    timer_list[i].type = type;
    timer_list[i].periodic = timeout;
    timer_list[i].timer_callback = fn_callback;
    timer_list[i].param = param;

    if (type >= SECOND_TIMER_ONE_TIME) /*Second timer*/
        timer_list[i].stop_point = timeout + get_second_count();
    else /*Milisecond timer*/
        timer_list[i].stop_point = timeout + get_tick_count();

    return TRUE;
}

void timer_delete(void* pTimer_callback) { /*use void* to use as param of callback in timer_create*/
    uint8_t i;
    callback timer_callback = (callback)pTimer_callback;
    for (i = 0; i < MAX_TIME_COUNT; ++i)
        if (timer_list[i].timer_callback == timer_callback) {
            timer_list[i].timer_callback = NULL;
            return;
        }
}

bool timer_is_running(void* pTimer_callback) {
    uint8_t i;
    callback timer_callback = (callback)pTimer_callback;
    for (i = 0; i < MAX_TIME_COUNT; ++i ) {
        if (timer_list[i].timer_callback == timer_callback)
            return TRUE;
    }
    return FALSE;
}

/**
    TODO:
    - Reimplement for more elegant
 */
void handle_timer_events() {
    bool passed;
    static uint16_t last_tick = 0;
    uint16_t ticks = get_tick_count();

    uint8_t i;
    for (i = 0; i < MAX_TIME_COUNT; ++i) {
        passed = 0;
        if ( timer_list[i].timer_callback != NULL) {
            if (timer_list[i].type >= 2) { /* second timer */
                passed = (timer_list[i].stop_point == get_second_count());
            } else { /*milisecond timer*/
                if (last_tick < ticks)
                    passed = (last_tick < timer_list[i].stop_point) &&
                             (timer_list[i].stop_point <= ticks);
                else if (last_tick > ticks)
                    passed = (last_tick < timer_list[i].stop_point) ||
                             (timer_list[i].stop_point <= ticks);
                // last_tick == ticks => passed = false
            }
        }

        if (passed) {
            /* Invoke the callback */
            timer_list[i].timer_callback(timer_list[i].param);

            /* Update stop point for rereapt timer */
            if (timer_list[i].type == MILI_TIMER_REPEAT)
                timer_list[i].stop_point = timer_list[i].periodic + get_tick_count();
            else if (timer_list[i].type == SECOND_TIMER_REPEAT)
                timer_list[i].stop_point = timer_list[i].periodic + get_second_count();
            else /* remove callback if one time timer*/
                timer_list[i].timer_callback = NULL;
        }
    }
    last_tick = ticks;
}

