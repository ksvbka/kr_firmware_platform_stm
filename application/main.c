#include "stm32f0_discovery.h"

#include "system.h"
#include "timer.h"
#include "typedef.h"
#include "gpio.h"
#include "uart.h"

uint8_t led_3 = GPIO_PIN(GPIO_PC, 9);
uint8_t led_4 = GPIO_PIN(GPIO_PC, 8);
uint8_t button = GPIO_PIN(GPIO_PA, 0);

void pulse_led_3(void* param) {
    gpio_toggle(led_3);
}

void pulse_led_4(void* param) {
    gpio_toggle(led_4);
}

void print_data(void* param) {
    char buff = *(char*)(param);
    uart_write("\n get data: ");
    uart_putc(buff);
}



int main(void) {

    system_init(CLOCK_SOURCE_HSI);

    gpio_module_init(GPIO_PC_ENABLE);
    gpio_init(led_3, GPIO_OUT);
    gpio_init(led_4, GPIO_OUT);

    /*Set init state of LED*/
    gpio_clear(led_4);
    gpio_clear(led_3);

    gpio_init_irq(button, GPIO_RISING);
    gpio_irq_register_callback(pulse_led_4);

    uart_init(UART_BAUDRATE_115200, UART_ENABLE_INT);
    uart_irq_register_callback(print_data);

    uart_write("\n Test uart_irq");
    timer_create(MILI_TIMER_REPEAT, 500, pulse_led_3, NULL);
    while (1) {
        handle_timer_events();
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
