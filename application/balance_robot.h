/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:29:12
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-28 23:30:24
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
#include "motor.h"

typedef struct robot {
        float current_angle;
        motor_t motor_left;
        motor_t motor_right;
        PID_t pid;
        bool state;
}robot_t;

/* Robot control */
void robot_init(robot_t* robot);
void robot_run(robot_t* robot);
void robot_stop(robot_t* robot);

#endif //__BALANCE_ROBOT_H__
