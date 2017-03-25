// /*
// * @Author: Kienltb
// * @Date:   2016-12-28 11:44:22
// * @Last Modified by:   Kienltb
// * @Last Modified time: 2016-12-28 16:28:58
// */

// #include "button.h"
// #include "gpio.h"
// #include "system.h" /* for get_tick_count()*/

// enum BUTTON_STATE {
//     IDLE,
//     FIRST_CLICK,
//     FIRST_RELEASE,
//     SECOND_CLICK,
//     SECOND_RELEASE,
//     PRESS,
// };

// /* Handler state machine of button state*/
// void button_state_machine(void* button);

// /* Init button at gpio pin */
// void button_init(button_t* bt, uint8_t pin)
// {
//         bt.pin = pin;
//         bt->on_click = NULL;
//         bt->on_double_click = NULL;
//         bt->on_long_press_start = NULL;
//         bt->on_long_press_stop = NULL;
//         bt->on_long_press_during = NULL;

//         /* Init gpio pin as interrupt pin */
//         gpio_init_irq(pin, GPIO_RISING);

//         /* Register interrupt handler */
//         gpio_irq_register_callback(button_state_machine);
// }

// /* Register callback handler, callback = NULL for delete*/
// void button_register_callback(button_t* bt, uint8_t event, callback callback_handler)
// {
//         switch (event) {
//         case CLICK :
//                 bt->on_click = callback_handler;
//                 break;
//         case DOUBLE_CLICK:
//                 bt->on_double_click = callback_handler;
//                 break;
//         case LONG_PRESS_START:
//                 bt->on_long_press_start = callback_handler;
//                 break;
//         case LONG_PRESS_STOP:
//                 bt->on_long_press_stop = callback_handler;
//                 break;
//         case LONG_PRESS_ON_DURING:
//                 bt->on_long_press_during = callback_handler;
//                 break;
//         }
// }

// void button_state_machine(void* button)
// {

// }
