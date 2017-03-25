/*
* @Author: Kienltb
* @Date:   2016-12-27 11:01:36
* @Last Modified by:   ksvbka
* @Last Modified time: 2017-01-13 14:32:21
*/

/**
 *  Implement driver for DC Motor
 *  Driver use 2 gpio IN1, IN2 for set direction and 1 PWM channel for set dutycyle
 */

typedef enum {
        STOP,      /*IN1 = 0 IN2 = 0 <=> 0x00*/
        FORWARD,   /*IN1 = 0 IN2 = 1 <=> 0x01*/
        BACKWARD   /*IN1 = 1 IN2 = 0 <=> 0x02*/
} direction_t;

typedef struct motor{
        uint8_t in1;    /* GPIO for driving motor direction*/
        uint8_t in2;    /* GPIO for driving motor direction*/
        uint8_t pwm_channel;
}motor_t;

void motor_init(motor_t motor);

void motor_init_param(motor_t* motor, uint8_t in1, uint8_t in2, uint8_t pwm_channel);

void motor_control(motor_t motor, uint8_t direction, uint16_t duty_cycle);
