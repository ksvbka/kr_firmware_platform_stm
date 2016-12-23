/*
* @Author: Trung Kien
* @Date:   2016-12-21 23:34:13
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-23 10:18:15
*/

#include "angle_calculate.h"
#include "math.h" /* For sqrt and atan2*/

/* Scale Value config for ACC of mpu6050*/
extern float g_acc_scale;

/* Scale Value config for GYRO of mpu6050*/
extern float g_gyro_scale;

void angle_complementary_getvalue( angle_t* pAngle, float sample_time)
{
        acc_data_t acc_data;
        gyro_data_t gyro_data;

        mpu6050_get_acc_data(&acc_data);
        mpu6050_get_gyro_data(&gyro_data);

        float pitch_acc, roll_acc;
        /* Angle around the X axis */
        pAngle->x += ((float)gyro_data.x / g_gyro_scale) * sample_time;
        /* Angle around the Y axis*/
        pAngle->y -= ((float)gyro_data.y / g_gyro_scale) * sample_time;
        /* Angle around the Z axis*/
        pAngle->z += ((float)gyro_data.z / g_gyro_scale) * sample_time;

        /* 180/pi = 57.296 */
        /* Turning around the X axis results in a vector on the Y-a xis*/
        pitch_acc = 57.296 * atan2((float)acc_data.y, sqrt((float)acc_data.z * (float)acc_data.z + (float)acc_data.x * (float)acc_data.x));
        pAngle->x = pAngle->x * 0.95 + pitch_acc * 0.05;

        /* Turning around the Y axis results in a vector on the X-axis*/
        roll_acc = 57.296 * atan2((float)acc_data.x, sqrt((float)acc_data.z * (float)acc_data.z + (float)acc_data.y * (float)acc_data.y));
        pAngle->y = pAngle->y * 0.95 + roll_acc * 0.05;

        /* Turning around the Z axis results in a vector on the Y-axis*/
        pAngle->z = 57.296 * atan2((float)acc_data.z, sqrt((float)acc_data.x * (float)acc_data.x + (float)acc_data.y * (float)acc_data.y));
        pAngle->z = pAngle->z * 0.95 + pitch_acc * 0.05;
}

void angle_kalman_init(kalman_t* kalman)
{
        /* Defaut value */
        kalman->Q_angle     = 0.001f;
        kalman->Q_gyroBias  = 0.0003f;
        kalman->R           = 0.03f;

        kalman->x_angle = 0;
        kalman->x_bias  = 0.0;

        kalman->P_00 = 0.0;
        kalman->P_01 = 0.0;
        kalman->P_10 = 0.0;
        kalman->P_11 = 0.0;

        kalman->K_0  = 0.0;
        kalman->K_1  = 0.0;
}

float angle_kalman_getvalue(kalman_t* pKalman, float new_angle, float new_rate, float dt)
{
        float y, S;
        /* Update kalman*/

        pKalman->x_angle += dt * (new_rate - pKalman->x_bias);

        pKalman->P_00 +=  dt * (pKalman->P_10 + pKalman->P_01) + pKalman->Q_angle * dt;
        pKalman->P_01 += -dt * pKalman->P_11;
        pKalman->P_10 += -dt * pKalman->P_11;
        pKalman->P_11 +=  pKalman->Q_gyroBias * dt;

        y = new_angle - pKalman->x_angle;
        S = pKalman->P_00 + pKalman->R;

        pKalman->K_0 = pKalman->P_00 / S;
        pKalman->K_1 = pKalman->P_10 / S;

        pKalman->x_angle +=  pKalman->K_0 * y;
        pKalman->x_bias  +=  pKalman->K_1 * y;
        pKalman->P_00 -= pKalman->K_0 * pKalman->P_00;
        pKalman->P_01 -= pKalman->K_0 * pKalman->P_01;
        pKalman->P_10 -= pKalman->K_1 * pKalman->P_00;
        pKalman->P_11 -= pKalman->K_1 * pKalman->P_01;

        return   pKalman->x_angle;
}


void angle_AHRS_getvalue(angle_t* pAngle, float sample_time) {
        acc_data_t acc_data;
        gyro_data_t gyro_data;
        angle_t current_angle;

        mpu6050_get_acc_data(&acc_data);
        mpu6050_get_gyro_data(&gyro_data);

        MadgwickAHRSupdateIMU(gyro_data.z, gyro_data.y, -gyro_data.x, acc_data.z, acc_data.y, -acc_data.x, sample_time);

        pAngle->x = atan2(2.0 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        pAngle->y = asinf(-2.0f * (q1 * q3 - q0 * q2));
        pAngle->z = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

}

/* Return pitch angle - using kalman AHRS filter */
float angle_AHRS_get_roll(float sample_time)
{
        acc_data_t acc_data;
        gyro_data_t gyro_data;
        angle_t current_angle;

        mpu6050_get_acc_data(&acc_data);
        mpu6050_get_gyro_data(&gyro_data);

        MadgwickAHRSupdateIMU(gyro_data.z, gyro_data.y, -gyro_data.x, acc_data.z, acc_data.y, -acc_data.x, sample_time);

        // float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));

        // float yaw = atan2(2.0 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        float roll = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

        // return pitch;
        // return yaw;
        return roll;
}
