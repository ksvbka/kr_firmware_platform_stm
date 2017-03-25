// /*
// * @Author: Kienltb
// * @Date:   2016-12-28 11:44:11
// * @Last Modified by:   Kienltb
// * @Last Modified time: 2016-12-28 16:29:01
// */

// #ifndef __BUTTON_H__
// #define __BUTTON_H__

// /**
//  *  Implement button service, support some basic evnet like click, double click
//  *  hold.
//  */

// #include "typedef.h"

// enum BUTTON_EVETNT {
//         CLICK = 0,
//         DOUBLE_CLICK,
//         LONG_PRESS_START,
//         LONG_PRESS_STOP,
//         LONG_PRESS_ON_DURING,
// };

// typedef struct button {
//         uint8_t pin;
//         uint16_t click_ticks;
//         uint16_t press_ticks;

//         callback on_click;
//         callback on_double_click;
//         callback on_long_press_start;
//         callback on_long_press_stop;
//         callback on_long_press_during;

//         uint8_t state;
//         uint8_t start_time;

// } button_t;

// /* Init button at gpio pin */
// void button_init(button_t* bt,uint8_t pin);

// /* Register callback handler, callback = NULL for delete*/
// void button_register_callback(button_t* bt, uint8_t event, callback callback_handler);

// #endif //__BUTTON_H__
