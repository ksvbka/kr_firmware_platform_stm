/*
* @Author: Kienltb
* @Date:   2016-12-27 11:01:44
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-27 12:12:52
*/

#include "motor.h"
#include "gpio.h"
#include "pwm.h"

#define FREQUENCE_PWM 10000 /*10kHz*/

void motor_init(motor_t motor)
{
        /* Init pwm channel*/
        pwm_init(motor.pwm_channel, FREQUENCE_PWM);

        /* Configure gpio in1, in2 as output pin*/
        gpio_init(motor.in1, GPIO_OUT_PU);
        gpio_init(motor.in2, GPIO_OUT_PU);

        gpio_clear(motor.in1);
        gpio_clear(motor.in2);
}

void motor_init_param(motor_t* motor, uint8_t in1, uint8_t in2, uint8_t pwm_channel)
{
        motor->in1 = in1;
        motor->in2 = in2;
        motor->pwm_channel = pwm_channel;

        motor_init(*motor);
}

void motor_control(motor_t motor, uint8_t direction, uint16_t duty_cycle)
{
        gpio_write(motor.in1, direction & 0x01);
        gpio_write(motor.in2, direction & 0x02);

        pwm_set_duty(motor.pwm_channel, duty_cycle);
}
