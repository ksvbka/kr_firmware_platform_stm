/*
* @Author: Trung Kien
* @Date:   2016-12-21 23:34:13
* @Last Modified by:   ksvbka
* @Last Modified time: 2017-01-08 01:41:18
*/

#include "angle_calculate.h"
#include "math.h" /* For sqrt and atan2*/

/* Scale Value config for ACC of mpu6050*/
extern double g_acc_scale;

/* Scale Value config for GYRO of mpu6050*/
extern double g_gyro_scale;

#define RAD_TO_DEG (57.296)

extern float invSqrt(float x);

void angle_complementary_getvalue( angle_t* pAngle, double dt)
{
        mpu_raw_data_t imu_data;
        mpu6050_get_rawdata(&imu_data);

        // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
        // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
        // It is then converted from radians to degrees

        // use RESTRICT_PITCH => use Eq. 25 and 26
        double roll  = atan2(imu_data.ay, imu_data.az) * RAD_TO_DEG;
        double pitch = atan(-imu_data.ax / sqrt(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)) * RAD_TO_DEG;

        double gyro_xrate = imu_data.gx/g_gyro_scale; /* Convert to deg/s*/
        double gyro_yrate = imu_data.gy/g_gyro_scale; /* Convert to deg/s*/

        if (roll > 90 || roll < -90)
                gyro_yrate = -gyro_yrate;

        static uint8_t is_init = 0;
        if (is_init == 0) {
                /* run in first time, not init */
                pAngle->roll   = roll;
                pAngle->pitch  = pitch;
                is_init++;
        } else {
                /* Complement filter*/
                pAngle->roll  = 0.93 * (pAngle->roll  + gyro_xrate * dt) + 0.07 * roll;
                pAngle->pitch = 0.93 * (pAngle->pitch + gyro_yrate * dt) + 0.07 * pitch;
        }

        /*FIX ME: Cannot calcutate z => fill 0*/
        pAngle->yaw = 0;

        /* Test*/
        // uart_printf("\n%f\t %f\t %f\t %f\t   %f\t %f\t %f\t %f\t", roll, 0.0, pAngle->roll, 0.0, pitch, 0.0, pAngle->pitch, 0.0);
}

// void angle_kalman_init(kalman_t* kalman)
// {
//         /* Defaut value */
//         kalman->Q_angle     = 0.001f;
//         kalman->Q_gyroBias  = 0.0003f;
//         kalman->R           = 0.03f;

//         kalman->x_angle = 0;
//         kalman->x_bias  = 0.0;

//         kalman->P_00 = 0.0;
//         kalman->P_01 = 0.0;
//         kalman->P_10 = 0.0;
//         kalman->P_11 = 0.0;

//         kalman->K_0  = 0.0;
//         kalman->K_1  = 0.0;
// }

// double angle_kalman_getvalue(kalman_t* pKalman, double new_angle, double new_rate, double dt)
// {
//         double y, S;
//         /* Update kalman*/

//         pKalman->x_angle += dt * (new_rate - pKalman->x_bias);

//         pKalman->P_00 +=  dt * (pKalman->P_10 + pKalman->P_01) + pKalman->Q_angle * dt;
//         pKalman->P_01 += -dt * pKalman->P_11;
//         pKalman->P_10 += -dt * pKalman->P_11;
//         pKalman->P_11 +=  pKalman->Q_gyroBias * dt;

//         y = new_angle - pKalman->x_angle;
//         S = pKalman->P_00 + pKalman->R;

//         pKalman->K_0 = pKalman->P_00 / S;
//         pKalman->K_1 = pKalman->P_10 / S;

//         pKalman->x_angle +=  pKalman->K_0 * y;
//         pKalman->x_bias  +=  pKalman->K_1 * y;
//         pKalman->P_00 -= pKalman->K_0 * pKalman->P_00;
//         pKalman->P_01 -= pKalman->K_0 * pKalman->P_01;
//         pKalman->P_10 -= pKalman->K_1 * pKalman->P_00;
//         pKalman->P_11 -= pKalman->K_1 * pKalman->P_01;

//         return   pKalman->x_angle;
// }

/* NOTE: Adjust sample time in MadgwickAHRS.c to get correct value of angle*/
void angle_AHRS_getvalue(angle_t* pAngle/*, float sample_time*/)
{
        acc_data_t acc_data;
        gyro_data_t gyro_data;

        angle_t current_angle;

        mpu6050_get_acc_data(&acc_data);
        mpu6050_get_gyro_data(&gyro_data);

        /* convert gyro readings into Radians per second*/
        double gyro_x = gyro_data.x / 939.650784;
        double gyro_y = gyro_data.y / 939.650784;
        double gyro_z = gyro_data.z / 939.650784;

        MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, acc_data.x, acc_data.y, acc_data.z);

        pAngle->roll  = atan2(2.0 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * RAD_TO_DEG;
        pAngle->pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * RAD_TO_DEG;
        pAngle->yaw   = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD_TO_DEG;

        /* Test */
        uart_printf("\n%f\t %f\t %f\t", pAngle->roll, pAngle->pitch, pAngle->yaw);
}

// /* Return pitch angle - using kalman AHRS filter */
// float angle_AHRS_get_roll(float sample_time)
// {
//         acc_data_t acc_data;
//         gyro_data_t gyro_data;
//         angle_t current_angle;

//         mpu6050_get_acc_data(&acc_data);
//         mpu6050_get_gyro_data(&gyro_data);

//         MadgwickAHRSupdateIMU(gyro_data.z, gyro_data.y, -gyro_data.x, acc_data.z, acc_data.y, -acc_data.x, sample_time);

//         // float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));

//         // float yaw = atan2(2.0 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
//         float roll = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

//         // return pitch;
//         // return yaw;
//         return roll;
// }
