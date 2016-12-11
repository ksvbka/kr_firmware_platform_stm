/*
* @Author: Trung Kien
* @Date:   2016-11-30 21:32:06
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-12 00:06:44
*/

#ifndef __PLATFORM_TEST_CASE__H__
#define __PLATFORM_TEST_CASE__H__

/* Hardware*/
#include "system.h"
#include "gpio.h"
#include "uart.h"
// #include "i2c_hw.h"
// #include "adc.h"
// #include "pwm.h"
// #include "lcd_5110.h"
// #include "flash.h"
// #include "comparator.h"
// #include "mpu6050.h"

/*Service*/
#include "timer.h"
// #include "event.h"

void platform_test_case(void);

/* Test case for hardware*/
void system_test(void);
void uart_test(void);
void get_confirm(void* param);
void gpio_test(void);
// void pwm_test(void);
// void pwm_test_dimming_led(void);
// void adc_test(void); // Meansure value of PWM output
// void lcd_5110(void);
/* Test case for Service*/
void timer_test(void);
// void event_cb(void* param);
// void handle_event_queue(void);
// void event_test(void);

/* Test i2c and external sensor*/
// void mpu6050_test(void);
// void sampling_process(void* param);

#endif //__PLATFORM_TEST_CASE__H__


