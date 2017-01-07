/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:29:12
* @Last Modified by:   ksvbka
* @Last Modified time: 2017-01-08 01:16:54
*/

#include "balance_robot.h"
#include "string.h"

#define SAMPLE_TIME 0.002 /*s, <=> 2ms, 500hz*/

/* Robot param */
robot_t robot = {
        .current_angle = 0.0,
        .motor_left = {
                .in1 = GPIO_PIN(GPIO_PC, 8),
                .in2 = GPIO_PIN(GPIO_PC, 9),
                .pwm_channel = PWM_CHANNEL_1,
        },
        .motor_right = {
                .in1 = GPIO_PIN(GPIO_PA, 10),
                .in2 = GPIO_PIN(GPIO_PA, 11),
                .pwm_channel = PWM_CHANNEL_2,
        },
        .pid = {
                .kp  = 100.0,
                .ki  = 0,
                .kd  = 0,
                .out_min = -100,
                .out_max = 100,
                .last_error = 0,
                .error_integral = 0,
                .sample_time = SAMPLE_TIME, /*s*/
                .direction = PID_DIRECT,
        },
        .state = 0,
};

/* Helper function*/
static void robot_get_balance(robot_t* robot, int16_t pwm);
static void control_loop(void* robot);


void robot_init(robot_t* robot)
{
        /* Init Hardware */
        system_init(CLOCK_SOURCE_HSI);        /* Set clock and watchdog timer */

        uart_init(UART_BAUDRATE_115200, UART_ENABLE_INT);
        uart_printf("\n---------Robot init------");

        i2c_init(I2C_MODULE_1);
        uart_printf("\ni2c module init...");

        uart_printf("\nmpu6050 sensor init...");
        mpu6050_init(ACC_CONFIG_2G, GYRO_CONFIG_250);
        // mpu6050_calibrate();         /* Calibrate sensor */

        uart_printf("\nTimer module init...");
        timer_hw_init();

        /* Init motor, init state as STOP */
        motor_init(robot->motor_left);
        motor_init(robot->motor_right);

        uart_printf("\nSet sample timer : %f", SAMPLE_TIME);

        /* Register handler */
        /*...*/

        /* set state*/
        robot->state = 1;
}

void robot_run(robot_t* robot)
{
        /* Start control loop */
        uint32_t sample_time_us = (uint32_t)(SAMPLE_TIME *1000000);
        timer_hw_start(sample_time_us, control_loop, robot);

        while (robot->state == 1 /*run*/) {
                /* Finite state machine of Robot */
                /* Recepit command ..., handle event*/
        }
}

void robot_stop(robot_t* robot)
{
        timer_hw_stop(NULL);
}

void control_loop(void* param)
{
        robot_t* robot = (robot_t*)(param);
        static angle_t current_angle; /* Use static because need to store gyro_angle*/

        angle_complementary_getvalue(&current_angle, SAMPLE_TIME);
        // uart_printf("\n roll: %f\t pitch: %f", current_angle.roll, current_angle.pitch);

        // /* Calculate the pitch angle so that:    0 = vertical    -pi/2 = on its back    +pi/2 = on its face*/
        // float roll = angle_AHRS_get_roll(SAMPLE_TIME);
        // uart_printf("\npitch (rad): %f", roll * 57.296);
        // uart_printf("\n x: %f \t y: %f \t z: %f \t", current_angle.x, current_angle.y, current_angle.z);


        /* Test AHRS algorithm */
        // angle_AHRS_getvalue(&current_angle);
        // uart_printf("\n x: %f \t y: %f \t z: %f \t", current_angle.x * 57.296, current_angle.y * 57.296, current_angle.z * 57.296);


        // /* PID control calculate */
        // uart_printf("\n y : %f", current_angle.y);
        // float pwm = pid_compute(&(robot->pid), 0, current_angle.y);

        // /* Set output*/
        // robot_get_balance(robot, (int16_t)pwm);


        /* Get offset of motor by increate duty cycle until the motor start move*/

        // static int16_t duty_cycle = 0;
        // static uint8_t direction = FORWARD;
        // duty_cycle += 1;
        // if (duty_cycle >= 100) {
        //         direction = (direction == FORWARD) ? BACKWARD : FORWARD;
        //         duty_cycle = 0;
        // }

        // uart_printf("\nduty: %d", duty_cycle);

        // motor_control(robot->motor_left,  direction, duty_cycle);
        // motor_control(robot->motor_right, direction, duty_cycle);
}

int16_t offset = 5;
void robot_get_balance(robot_t* robot, int16_t pwm)
{
        if (pwm > offset) {
                motor_control(robot->motor_left,  BACKWARD, pwm);
                motor_control(robot->motor_right, BACKWARD, pwm);
        } else if (pwm < -offset) {
                motor_control(robot->motor_left,  FORWARD, -pwm);
                motor_control(robot->motor_right, FORWARD, -pwm);
        } else {
                motor_control(robot->motor_left,  STOP, 0);
                motor_control(robot->motor_right, STOP, 0);

        }
}
