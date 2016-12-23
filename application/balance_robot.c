/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:29:12
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-23 11:17:54
*/

#include "balance_robot.h"

#define SAMPLE_TIME 0.005f /*s, <=> 5ms*/

/* Robot param */
robot_t robot = {
        .current_angle = 0,
        .motor_left = {
                .direction = STOP,
                .in1 = GPIO_PIN(GPIO_PA, 11),
                .in2 = GPIO_PIN(GPIO_PA, 10),
                .pwm_channel = PWM_CHANNEL_2,
                .duty_cycle  = 0,
        },
        .motor_right = {
                .direction = STOP,
                .in1 = GPIO_PIN(GPIO_PC, 8),
                .in2 = GPIO_PIN(GPIO_PC, 9),
                .pwm_channel = PWM_CHANNEL_1,
                .duty_cycle  = 0,
        },
        .enable_pin = GPIO_PIN(GPIO_PC, 7),
        .pid = {
                .kp  = 55.0,
                .ki  = 0,
                .kd  = 0,
                .out_min = -1000,
                .out_max = 1000,
                .last_input = 0,
                .last_process_value = 0,
                .sample_time = SAMPLE_TIME, /*s*/
                .direction = PID_DIRECT,
        },
        .state = 0,
};

/* Helper function*/
static void robot_get_balance(robot_t* robot, int16_t pwm);
static void motor_control(motor_t motor); /* Motor control driver */
static void control_loop(void* robot);


/* Motor control */
void motor_control(motor_t motor)
{
        gpio_write(motor.in1, motor.direction & 0x01);
        gpio_write(motor.in2, motor.direction & 0x02);

        pwm_set_duty(motor.pwm_channel, motor.duty_cycle);
}

/* Robot control */

void robot_init(robot_t* robot)
{
        /* Init Hardware */
        system_init(CLOCK_SOURCE_HSI);        /* Set clock and watchdog timer */

        uart_init(UART_BAUDRATE_115200, UART_ENABLE_INT);
        uart_printf("\n---------Robot init------");

        i2c_init(I2C_MODULE_1);
        uart_printf("\ni2c module init...");

        uart_printf("\nmpu6050 sensor init...");
        mpu6050_init(ACC_CONFIG_2G, GYRO_CONFIG_2000);
        // mpu6050_calibrate();         /* Calibrate sensor */

        uart_printf("\nTimer module init...");
        timer_hw_init();

        /* Init gpio for motor control*/
        gpio_init(robot->motor_left.in1, GPIO_OUT_PU);
        gpio_init(robot->motor_left.in2, GPIO_OUT_PU);
        gpio_init(robot->motor_right.in1, GPIO_OUT_PU);
        gpio_init(robot->motor_right.in2, GPIO_OUT_PU);
        gpio_init(robot->enable_pin, GPIO_OUT_PU);

        /* Init state as STOP */
        gpio_clear(robot->motor_left.in1);
        gpio_clear(robot->motor_left.in2);
        gpio_clear(robot->motor_right.in1);
        gpio_clear(robot->motor_right.in2);
        gpio_clear(robot->enable_pin);

        pwm_init(robot->motor_left.pwm_channel + robot->motor_right.pwm_channel, 100000);

        uart_printf("\nSet sample timer : %f", SAMPLE_TIME);

        /* Register handler */
        /*...*/

        /* set state*/
        robot->state = 1;
}

void robot_run(robot_t* robot)
{
        /*Enable motor driver*/
        gpio_set(robot->enable_pin);

        /* Start control loop */
        timer_hw_start(SAMPLE_TIME, control_loop, robot);

        while (robot->state == 1 /*run*/) {
                /* Finite state machine of Robot */
                /* Recepit command ..., handle event*/
        }
}

void robot_stop(robot_t* robot)
{
        timer_hw_stop(NULL);
        gpio_clear(robot->enable_pin);
}

void control_loop(void* param)
{
        robot_t* robot = (robot_t*)(param);
        angle_t current_angle;

        angle_complementary_getvalue(&current_angle, SAMPLE_TIME);

        // uart_printf("\n x: %f \t y: %f\t z: %f", current_angle.x, current_angle.y, current_angle.z);

        // calculate the pitch angle so that:    0 = vertical    -pi/2 = on its back    +pi/2 = on its face
        // float roll = angle_AHRS_get_roll(SAMPLE_TIME);
        // uart_printf("\npitch (rad): %f", roll * 57.296);

        /* PID control calculate */
        float pwm = pid_compute(&(robot->pid), 0, current_angle.x);

        uart_printf("\n input: %f \t error: %f\t pwm: %f\t pwm_int: %d", current_angle.x, 0 - current_angle.x, pwm, (int16_t)pwm);

        // robot_get_balance(robot,(int16_t)pwm);

        /* Set output*/
}

void robot_get_balance(robot_t* robot, int16_t pwm)
{
        if (pwm > 50) {
                robot->motor_left.direction = BACKWARD;
                robot->motor_right.direction = BACKWARD;

                robot->motor_left.duty_cycle = pwm;
                robot->motor_right.duty_cycle = pwm;

                motor_control(robot->motor_left);
                motor_control(robot->motor_right);
        } else if (pwm < -50) {
                robot->motor_left.direction = FORWARD;
                robot->motor_right.direction = FORWARD;

                robot->motor_left.duty_cycle = -pwm;
                robot->motor_right.duty_cycle = -pwm;

                motor_control(robot->motor_left);
                motor_control(robot->motor_right);
        } else {
                robot->motor_left.direction = STOP;
                robot->motor_right.direction = STOP;

                robot->motor_left.duty_cycle = 0;
                robot->motor_right.duty_cycle = 0;

                motor_control(robot->motor_left);
                motor_control(robot->motor_right);

        }
}
