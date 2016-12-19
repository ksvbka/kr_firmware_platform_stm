/*
* @Author: Trung Kien
* @Date:   2016-11-30 22:30:36
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-16 21:15:18
*/

#include "platform_test_case.h"

#define OK 'y'

static volatile bool g_test_ok = FALSE;

void platform_test_case(void)
{
        system_test();
        uart_test();
        gpio_test();

        // // /*Service testing*/
        timer_test();
        event_test();
        // pwm_test();
        // adc_test();
        // i2c_test();
        // // mpu6050_test();

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
        // uart_printf("\nPlease get_confirm (press 'y')...");

        // while (!g_test_ok);

        // uart_printf("\nuart hardware is ok");
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
// /* PWM module testing - NOTE: only support 16MHZ*/
// void pwm_test(void)
// {
//     uint8_t channel_1 = GPIO_PIN(2, 1);
//     uint8_t channel_2 = GPIO_PIN(2, 4);
//     uart_printf("\nTesting PWM ...");
//     uart_printf("\n    FREQ 1kHz, duty_cycle 40% ");

//     pwm_init(FREQUENCY_1KHZ, channel_1, channel_2);
//     pwm_set_duty(CHANNEL_1, 10);
//     pwm_set_duty(CHANNEL_2, 80);
//     uart_printf("\n    Meansure by adc!");

// }

// /* ADC Module testing*/
// void adc_sampling(void* param)
// {
//     param = NULL;
//     char buff[10];
//     itoa(adc_read_single_channel(), buff, 10);
//     uart_printf("\n");
//     uart_printf(buff);
// }

// void adc_test(void)
// {
//     uart_printf("\nADC module testing...");
//     uart_printf("\n    Setup Pin1.4 as input signal");

//     // Setup Pin1.4 as input of singgle channel adc
//     adc_init(ADC_CHA4, ADC_SINGLE_CHANNEL, ENABLE_INTERRUPT_ADC);
//     uart_printf("\n    Start sampling T = 500ms in 10s");
//     timer_create(MILI_TIMER_REPEAT, 500, adc_sampling, NULL);
//     timer_create(SECOND_TIMER_ONE_TIME, 10, timer_delete, &adc_sampling);
// }

void i2c_test(void)
{
        i2c_init(I2C_MODULE_1);
        #define MPU6050_ADDRESS         0x68
        #define MPU6050_WHO_AM_I        0x75
        uart_printf("\nTesting i2c ...");
        uart_printf("\n    Please connect device to i2c module!");

        uint8_t ret = i2c_read_byte(MPU6050_ADDRESS, MPU6050_WHO_AM_I);
        uart_printf("\n    Value of mpu6050 who am i: ");
        uart_printf(to_string(ret));
        uart_printf("\n    Please get_confirm (press 'y')...");
        while (!g_test_ok);

        uart_printf("\n    i2c function is ok :D");
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

/*
    Test external sensor
*/

// void mpu6050_test()
// {
//         i2c_init();
//         mpu6050_init(ACC_CONFIG_2G, GYRO_CONFIG_2000);

//         /*Regist callback function for sampling data*/
// }


// /* Log raw data*/

// void sampling_process(void* param);
// {
//         /*Test convert Gyro data */
//         MPU6050_GetGyroValueRaw(&gyroDataRaw);
//         MPU6050_GetAccValueRaw(&accDataRaw);

//         //BYTE I2C_ReadByte(BYTE byDeviceAddr, BYTE byRegister);

//         sprintf((char*)msg,"%4d,%4d,%4d,%4d,%4d,%4d\0\n",\
//                                     (accDataRaw.x), \
//                                     (accDataRaw.y), \
//                                     (accDataRaw.z), \
//                                     (gyroDataRaw.x), \
//                                     (gyroDataRaw.y), \
//                                     (gyroDataRaw.z));

//         uart_printf((char*)msg);

//         sprintf((char*)msg,"\n0x%x\n\n", i2c_read_byte( MPU6050_ADDRESS, MPU6050_WHO_AM_I));

//         uart_printf((char*)msg);


//         timer_create(MILI_TIMER_REPEAT,Tsample,samplingProcess, NULL);

// }

/**
 * TEST PWM by LED
 * connect channel 1 and 2 to LED then start testing
 */
// void pwm_test_dimming_led(void)
// {
//     // system_init(FREQUENCY_16MHZ);
//     // uart_init(UART_BAUDRATE_115200, UART_ENABLE_INT);

//     // uint8_t channel_1 = GPIO_PIN(2, 1);
//     // uint8_t channel_2 = GPIO_PIN(2, 4);
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
