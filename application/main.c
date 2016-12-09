#include "stm32f0_discovery.h"

#include "system.h"
#include "timer.h"
#include "typedef.h"
#include "gpio.h"

uint8_t led_3 = GPIO_PIN(GPIO_PC, 9);
uint8_t led_4 = GPIO_PIN(GPIO_PC, 8);

void pulse_led(void* param) {
    gpio_toggle(led_3);
    gpio_toggle(led_4);
}

int main(void) {

    system_init(CLOCK_SOURCE_HSI);

    gpio_module_init(GPIO_PC_ENABLE);
    gpio_init(led_3, GPIO_OUT);
    gpio_init(led_4, GPIO_OUT);

    /*Set init state of LED*/
    gpio_set(led_4);
    gpio_clear(led_3);

    timer_create(MILI_TIMER_REPEAT, 500, pulse_led, NULL);
    while (1) {
        handle_timer_events();
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
