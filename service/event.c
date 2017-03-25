/*
* @Author: Trung Kien
* @Date:   2016-11-29 14:48:53
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-11 23:40:34
*/
#include "event.h"

/*Global event_queue */
event_queue_t g_event_queue = {
        .head = 0,
        .tail = 0,
        .count = 0
};

bool poll_event(event_t* to_evnet)
{
        /*Check empty queue*/
        if (g_event_queue.count == 0)
                return 0;

        /*Get event*/
        *to_evnet = g_event_queue.node[g_event_queue.head];

        /*Update queue*/
        g_event_queue.head++;

        if (g_event_queue.head == MAX_EVENT)
                g_event_queue.head = 0;

        g_event_queue.count--;

        return 1;
}

bool bind_event(event_t* from_event)
{
        /*Check full queue*/
        if (g_event_queue.count == MAX_EVENT - 1)
                return 0;

        /*Update queue*/
        (g_event_queue.tail)++;
        if (g_event_queue.tail == MAX_EVENT)
                g_event_queue.tail = 0;

        /*Push event*/
        g_event_queue.node[g_event_queue.tail] = *from_event;
        g_event_queue.count++;

        return 1;
}
