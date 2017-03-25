/*
* @Author: Trung Kien
* @Date:   2016-11-29 14:48:54
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-12 00:41:52
*/
#ifndef __EVENT_H__
#define __EVENT_H__

/**
 *  EVENT HANDLER SEVICE
 *  Outhor: KienLtb
 *    Implement event handler sevice to process event (uart, button...);
 *
 *  USING:
 *   create an event and bind to event_queue;
 *   In superloop poll evnet from queue and process.
 *   eg:
 *   event_t cmd_event = {CMD_RECEIVED, GO};
 *   bind_event(&cmd_event);
 *   ...
 *   in main function:
 *   while(!done){
 *       ...
 *       while(poll_evnet(&event)!= 0)
 *       {
 *           switch(event.type){
 *           case CMD...
 *           }
 *       }
 *   }
 */
#include "typedef.h"

#define MAX_EVENT (10)

typedef struct {
        uint8_t type;     /* Type of event*/
        uint8_t event_id; /* Bitmask of active event id*/
} event_t;

/*Queue to store event*/
typedef struct {
        int8_t head, tail, count;
        event_t node[MAX_EVENT];
} event_queue_t;

/*Global variable event_queue*/
extern event_queue_t g_event_queue;

bool poll_event(event_t* to_evnet);

bool bind_event(event_t* from_event);

#endif
