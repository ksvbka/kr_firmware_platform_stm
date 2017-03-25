/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:29:12
* @Last Modified by:   ksvbka
* @Last Modified time: 2017-03-25 21:32:12
*/

#include "balance_robot.h"
#include "string.h"

#define SAMPLE_TIME 0.005f /*s, <=> 5ms, 200hz*/

/* Robot param */
robot_t robot = {
        .current_angle = 0.0f,
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
                .kp  = 34.25f,
                .ki  = 0.0f,
                .kd  = 0.00f,
                .out_min = -100.0f,
                .out_max = 100.0f,
                .last_error = 0.0f,
                .error_integral = 0.0f,
                .sample_time = SAMPLE_TIME, /*s*/
                .direction = PID_DIRECT,
        },
        .button = GPIO_PIN(GPIO_PA, 0),
        .state = 0,
};

/* Setpoint of angle: 0 is balance, > 0 is forward, < 0 is backward*/
static volatile float angle_setpoint = 0.0f;

/* Control turn left or turn right of robot. > 0 is turn left, < 0 is turn right*/
static volatile float turn_value = 0.0f;

/* Helper function*/
static void robot_get_balance(robot_t* robot, int16_t pwm);
static void control_loop(void* robot);
void toogle_state(void* param);
void motor_calibrate(void);

void robot_init(robot_t* robot)
{
        /* Init Hardware */
        system_init(CLOCK_SOURCE_HSI);        /* Set clock and watchdog timer */

        uart_init(UART_BAUDRATE_115200, UART_ENABLE_INT);
        uart_printf("\n---------Robot init------");

        i2c_init(I2C_MODULE_1);
        uart_printf("\ni2c module init...");

        uart_printf("\nmpu6050 sensor init...");
        // mpu6050_init(ACC_CONFIG_2G, GYRO_CONFIG_250);
        mpu6050_init(ACC_CONFIG_4G, GYRO_CONFIG_2000);
        // mpu6050_calibrate();         /* Calibrate sensor */

        uart_printf("\nTimer module init...");
        timer_hw_init();

        /* Init motor, init state as STOP */
        motor_init(robot->motor_left);
        motor_init(robot->motor_right);

        uart_printf("\nSet sample timer : %f", SAMPLE_TIME);

        /* Init NRL24L01 module */
        // nrf24l01_init(RF_CE, RF_CS, RF_CLK, RF_MOSI, RF_MISO);
        // nrf24l01_set_tx_mode();

        /* Register handler */
        /* register button for change state */
        gpio_init_irq(robot->button, GPIO_FALLING);
        gpio_irq_register_callback(toogle_state);

        /* set state*/
        robot->state = 0;
        uart_printf("\n Please press button for start robot");

        /* cablib motor */
        // motor_calibrate();
}

void robot_run(robot_t* robot)
{
        /* Start control loop */
        uint32_t sample_time_us = (uint32_t)(SAMPLE_TIME * 1000000);
        timer_hw_start(sample_time_us, control_loop, robot);
        while (1 /*run*/) {
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

        if (robot->state == 0) {
                motor_control(robot->motor_left,  STOP, 0);
                motor_control(robot->motor_right, STOP, 0);

                return;
        }

        /* Using complementary filter */
        // angle_complementary_getvalue(&current_angle, SAMPLE_TIME);

        /* Using Kalman filter*/
        // angle_kalman_getvalue(&current_angle, SAMPLE_TIME);

        /* Using AHRS filter */
        angle_AHRS_getvalue(&current_angle, SAMPLE_TIME);

        if (ABS(current_angle.pitch) > 30) {
                uart_printf("\nCan not get balance, please press button for restart!");

                motor_control(robot->motor_left,  STOP, 0);
                motor_control(robot->motor_right, STOP, 0);

                robot->state = 0;
                return;
        }

        /* PID control calculate */
        float pwm = pid_compute(&(robot->pid), angle_setpoint, current_angle.pitch);

        /* Set output*/
        robot_get_balance(robot, (int16_t)pwm);
        uart_printf("\n pitch : %f \t pwm: %d", current_angle.pitch, (int16_t)pwm);
}


int16_t motor_left_offset =  15;
int16_t motor_right_offset = 10;
int16_t offset = 1;

void robot_get_balance(robot_t* robot, int16_t pwm)
{
        if (pwm < -offset) {
                motor_control(robot->motor_left,  BACKWARD, -pwm + motor_left_offset);
                motor_control(robot->motor_right, BACKWARD, -pwm + motor_right_offset);
        } else if (pwm > offset) {
                motor_control(robot->motor_left,  FORWARD,   pwm + motor_left_offset);
                motor_control(robot->motor_right, FORWARD,   pwm + motor_right_offset);
        } else {
                motor_control(robot->motor_left,  STOP, 0);
                motor_control(robot->motor_right, STOP, 0);
        }
}

/* Helper function */

void toogle_state(void* param)
{
        /* Delay for ... */
        uint8_t new_state;
        static uint8_t count = 0;
        count++;
        if (count >= 2) {
                new_state = (robot.state) ? 0 : 1;
                uart_printf("\n change state from %d to %d", robot.state, new_state);
                robot.state = new_state;
                count = 0;
        }
}


void motor_calibrate()
{
        /* Get offset of motor by increate duty cycle until the motor start move*/

        static int16_t duty_cycle = 0;
        static uint8_t direction = FORWARD;
        while (1) {
                duty_cycle += 1;
                if (duty_cycle >= 100) {
                        direction = (direction == FORWARD) ? BACKWARD : FORWARD;
                        duty_cycle = 0;
                }

                uart_printf("\nduty: %d", duty_cycle);

                motor_control(robot.motor_left,  direction, duty_cycle);
                motor_control(robot.motor_right, direction, duty_cycle);
                delay_ms(200);
        }

}
