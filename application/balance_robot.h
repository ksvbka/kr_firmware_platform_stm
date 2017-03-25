/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:29:12
* @Last Modified by:   Kienltb
* @Last Modified time: 2017-01-11 18:08:27
*/

/* Implement 2 wheels Balance robot project */

#ifndef __BALANCE_ROBOT_H__
#define __BALANCE_ROBOT_H__

/* Hardware*/
#include "system.h"
#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include "pwm.h"
#include "timer_hw.h"
#include "nrf24l01.h"

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
        uint8_t button;
        bool state;
}robot_t;

/* Define GPIO pin for NRF24L01 module */

#define RF_CE     GPIO_PIN(GPIO_PC, 8)
#define RF_CS     GPIO_PIN(GPIO_PC, 8)
#define RF_CLK    GPIO_PIN(GPIO_PC, 8)
#define RF_MOSI   GPIO_PIN(GPIO_PC, 8)
#define RF_MISO   GPIO_PIN(GPIO_PC, 8)

/* Robot control */
void robot_init(robot_t* robot);
void robot_run(robot_t* robot);
void robot_stop(robot_t* robot);

#endif //__BALANCE_ROBOT_H__
