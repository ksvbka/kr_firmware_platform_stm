/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:29:12
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-23 11:15:25
*/

/* Implement 2 wheels Blance robot project */

#ifndef __BALANCE_ROBOT_H__
#define __BALANCE_ROBOT_H__

/* Hardware*/
#include "system.h"
#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include "pwm.h"
#include "timer_hw.h"

/*Service*/
#include "timer.h"
#include "event.h"
#include "utility.h"
#include "angle_calculate.h"
#include "pid.h"

typedef enum {
        STOP,      /*IN1 = 0 IN2 = 0*/
        FORWARD,   /*IN1 = 0 IN2 = 1*/
        BACKWARD   /*IN1 = 1 IN2 = 0*/
} direction_t;

typedef struct motor{
        direction_t direction;
        uint8_t in1;    /* GPIO for driving motor direction*/
        uint8_t in2;    /* GPIO for driving motor direction*/
        uint8_t pwm_channel;
        uint8_t duty_cycle;
}motor_t;

typedef struct robot {
        float current_angle;
        motor_t motor_left;
        motor_t motor_right;
        uint8_t enable_pin;
        PID_t pid;
        bool state;
}robot_t;

/* Robot control */
void robot_init(robot_t* robot);
void robot_run(robot_t* robot);
void robot_stop(robot_t* robot);

#endif //__BALANCE_ROBOT_H__
