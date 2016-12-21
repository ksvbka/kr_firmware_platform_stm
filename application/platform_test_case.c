/*
* @Author: Trung Kien
* @Date:   2016-11-30 22:30:36
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-20 22:10:58
*/

#include "platform_test_case.h"

#define OK 'y'

static volatile bool g_test_ok = FALSE;

void platform_test_case(void)
{
        system_test();
        uart_test();
        // gpio_test();

        timer_hw_test();
        /*Service testing*/
        // timer_test();
        // event_test();
        // i2c_test();
        // mpu6050_test();
        // pwm_test();
        // adc_test();
        /* Supper loop*/
        while (1) {
                handle_timer_events();
                handle_event_queue();
        }
        // pwm_test_dimming_led();

        // lcd_5110();
}


/* Set clock and watchdog timer*/
void system_test(void)
{
        system_init(CLOCK_SOURCE_HSI);
}

/* Test uart first for loging data*/
void uart_test(void)
{
        g_test_ok = FALSE;
        uart_init(UART_BAUDRATE_115200, UART_ENABLE_INT);
        uart_printf("\n-------------------------------------");
        uart_printf("\n|      TEST FIRMWARE FLATFORM       |");
        uart_printf("\n|      Author : KienLTb             |");
        uart_printf("\n|      Version: 1.0                 |");
        uart_printf("\n-------------------------------------");

        uart_printf("\nSet clock: 16MHZ");
        uart_printf("\nTest put number: ");
        uart_printf("Test wirte number: %d \t %f \t %s", -161311, 3.14, "write number ok");
        uart_irq_register_callback(get_confirm);
        uart_printf("\nPlease get_confirm (press 'y')...");

        while (!g_test_ok);

        uart_printf("\nuart hardware is ok");
}

void get_confirm(void* parm)
{
        if (*(char*)parm == OK)
                g_test_ok = TRUE;
}

/*Test io*/

#define PRESS_BUTTON (10) // press button event

uint8_t LED_GREEN  = GPIO_PIN(GPIO_PC, 9);
uint8_t LED_BLUE   = GPIO_PIN(GPIO_PC, 8);
uint8_t BUTTON     = GPIO_PIN(GPIO_PA, 0);

void pulse_led(void* param);

void button_press_cb(void* param);

void gpio_test(void)
{
        uart_printf("\nTesting io ...");
        // gpio_module_init(GPIO_PC_ENABLE | GPIO_PA_ENABLE);
        gpio_init(LED_GREEN, GPIO_OUT);
        gpio_init(LED_BLUE, GPIO_OUT);

        g_test_ok = FALSE;
        uart_printf("\n    Turning on LED_GREEN...");
        delay_ms(500);
        gpio_set(LED_GREEN);
        uart_printf("\n    Please get_confirm (press 'y')...");
        while (!g_test_ok);

        g_test_ok = FALSE;
        uart_printf("\n    Turning on LED_BLUE...");
        delay_ms(500);
        gpio_set(LED_BLUE);
        uart_printf("\n    Please get_confirm (press 'y')...");
        while (!g_test_ok);

        g_test_ok = FALSE;
        uart_printf("\n    Registing button to toggle LED_GREEN ...");
        gpio_init_irq(BUTTON, GPIO_FALLING);
        gpio_irq_register_callback(button_press_cb);
        delay_ms(500);
        uart_printf("\n    Please get_confirm (press 'y')...");
        while (!g_test_ok);

        uart_printf("\n    io function is ok :D");
}

void pulse_led(void* param)
{
        uint8_t LED = CAST_VAL(uint8_t, param);
        gpio_toggle(LED);
}

void button_press_cb(void* param)
{
        /* Convert param to pin*/
        uint8_t gpio = CAST_VAL(uint8_t, param);
        if (gpio == BUTTON)
                gpio_toggle(LED_GREEN);
}

void pwm_start(void) {
        static uint16_t duty_cycle = 10;
        pwm_set_duty(PWM_CHANNEL_1, duty_cycle);
        pwm_set_duty(PWM_CHANNEL_3, duty_cycle);
        pwm_set_duty(PWM_CHANNEL_2, 1000 - duty_cycle);
        pwm_set_duty(PWM_CHANNEL_4, 1000 - duty_cycle);
        duty_cycle += 5;
        if (duty_cycle >= 1000)
                duty_cycle = 0;
        uart_printf("\n PWM duty_cycle = %d", duty_cycle);
}
/* PWM module testing - NOTE: only support 16MHZ*/
void pwm_test(void)
{
        uart_printf("\nTesting PWM ...");
        uart_printf("\n    FREQ 1kHz, duty_cycle 40%% ");

        pwm_init(PWM_CHANNEL_1 + PWM_CHANNEL_2 + PWM_CHANNEL_3 + PWM_CHANNEL_4, 50000); /* 50Khz*/
        pwm_set_duty(PWM_CHANNEL_1, 0);
        pwm_set_duty(PWM_CHANNEL_4, 1000);

        /*Generate PWM signal in 20s, sampling 15ms*/
        timer_create(MILI_TIMER_REPEAT, 15, pwm_start, NULL);
        timer_create(SECOND_TIMER_ONE_TIME, 20, timer_delete, &pwm_start);
}

/* ADC Module testing*/
void adc_sampling(void* param)
{
        uart_printf("\n     adc: %d\n", adc_read_single_channel());
}

void adc_test(void)
{
        uart_printf("\nADC module testing...");
        uart_printf("\n    Setup Pin PA4 as input signal");

        // Setup Pin1.4 as input of singgle channel adc
        adc_init(ADC_CHA4, ADC_SINGLE_CHANNEL, DISABLE_INTERRUPT_ADC);
        uart_printf("\n    Start sampling T = 500ms in 10s");
        timer_create(MILI_TIMER_REPEAT, 100, adc_sampling, NULL);
        timer_create(SECOND_TIMER_ONE_TIME, 10, timer_delete, &adc_sampling);
}

void task(void* param) {
        static uint32_t count = 0;
        uart_printf("\n    Tick of timer_hw: %d", count++);
        if(count >= 10000) count = 0;
}
void timer_hw_test(void) {
        timer_hw_init();
        uart_printf("\nTesting Timer_HW, increate ticks each 10ms in 5s");
        timer_hw_start(10000, task); /* 10000us = 10ms*/
        timer_create(SECOND_TIMER_ONE_TIME, 5, timer_hw_stop, NULL);
}


void i2c_test(void)
{
        i2c_init(I2C_MODULE_1);
        // i2c_setup(I2C1);
#define MPU6050_ADDRESS         0x68
#define MPU6050_WHO_AM_I        0x75
        uart_printf("\nTesting i2c ...");
        uart_printf("\n    Please connect device to i2c module!");

        uint8_t ret = i2c_read_byte(MPU6050_ADDRESS, MPU6050_WHO_AM_I);
        // uint8_t ret;
        // i2c_read_registers(I2C1, MPU6050_ADDRESS, 1, MPU6050_WHO_AM_I, &ret);
        uart_printf("\n    Value of mpu6050 who am i: %d", ret);
        uart_printf("\n    Please get_confirm (press 'y')...");
        while (!g_test_ok);

        uart_printf("\n    i2c function is ok :D");
}


/* Test external sensor*/

void mpu6050_sampling(void* param)
{

        acc_raw_t acc_raw;
        gyro_raw_t gyro_raw;

        mpu6050_get_acc_rawdata(&acc_raw);
        mpu6050_get_gyro_rawdata(&gyro_raw);

        uart_printf("\n%d\t %d\t %d\t %d\t %d\t %d", acc_raw.x,  acc_raw.y,  acc_raw.z, \
                                                    gyro_raw.x, gyro_raw.y, gyro_raw.z);

        /*Test convert Gyro data */
        // acc_data_t  acc_data;
        // gyro_data_t gyro_data;
        // mpu6050_get_acc_data(&acc_data);
        // mpu6050_get_acc_data(&gyro_data);

/*        uart_printf("\n%f\t %f\t %f\t %f\t %f\t %f", acc_data.x,  acc_data.y,  acc_data.z, \
        gyro_data.x, gyro_data.y, gyro_data.z);
*/

}

void mpu6050_test()
{
        uart_printf("\nmpu6050 testing...");
        uart_printf("\n   init I2C_MODULE_1");
        i2c_init(I2C_MODULE_1);

        uart_printf("\n    mpu6050 init");
        mpu6050_init(ACC_CONFIG_2G, GYRO_CONFIG_2000);

        uint8_t ret = i2c_read_byte(0x68, 0x75);
        uart_printf("\n    Value of mpu6050 who am i: %d", ret);

        /*Regist callback function for sampling data*/
        timer_create(MILI_TIMER_REPEAT, 100, mpu6050_sampling, NULL);
}

/*-------------------------------------------------------
         Service Testing
*--------------------------------------------------------*/

/* Timer testing */
void timer_test()
{
        uart_printf("\nTimer Service testing...");

        uart_printf("\n    Create timer blink LED_GREEN 100ms...");
        timer_create(MILI_TIMER_REPEAT, 100, pulse_led, &LED_GREEN);

        uart_printf("\n    Create timer blink LED_BLUE 1s");
        timer_create(SECOND_TIMER_REPEAT, 1, pulse_led, &LED_BLUE);

}

/* Event testing */
void event_test()
{
        uart_printf("\nEvent Service testing...");
        uart_printf("\n    Press button to delete timer....");
        gpio_init_irq(BUTTON, GPIO_FALLING);
        gpio_irq_register_callback(event_cb);

}

/* Create and bind and event to g_event*/
void event_cb(void* param)
{
        uint8_t gpio_irq = CAST_VAL(uint8_t, param);
        if (gpio_irq == BUTTON) {
                event_t press_button = {PRESS_BUTTON, LED_GREEN};
                bind_event(&press_button);
        }
}


void handle_event_queue()
{
        event_t event;
        while (poll_event(&event) != 0) {
                switch (event.type) {
                case PRESS_BUTTON:
                        timer_delete(pulse_led);
                        timer_delete(pulse_led);
                        gpio_clear(LED_GREEN);
                        gpio_clear(LED_BLUE);
                        break;
                }
        }
}


// /* Log raw data*/


/**
 * TEST PWM by LED
 * connect channel 1 and 2 to LED then start testing
 */
// void pwm_test_dimming_led(void)
// {
//     // system_init(FREQUENCY_16MHZ);
//     // uart_init(UART_BAUDRATE_115200, UART_ENABLE_INT);
//     uint16_t duty_cycle = 0;

//     uart_printf("\nTesting PWM by dimming led...");
//     pwm_init(PWM_CHANNEL_1 + PWM_CHANNEL_4, 100000);

//     while (1) {
//         // uart_printf("\n set duty_cycle");
//         pwm_set_duty(PWM_CHANNEL_1, duty_cycle);
//         pwm_set_duty(PWM_CHANNEL_4, 1000 - duty_cycle);
//         duty_cycle += 20;
//         if (duty_cycle >= 1000)
//             duty_cycle = 0;
//         delay_ms(50);
//     }
// }

// void lcd_5110(void) {
//     lcd_init(70,0,40);
//     lcd_clear();
//     lcd_printf("Kienltb");
// }
