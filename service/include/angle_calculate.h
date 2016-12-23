/*
* @Author: Trung Kien
* @Date:   2016-12-21 23:33:50
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-22 17:29:15
*/

/* Implement angle calculate from data of acc and gyro */

#ifndef __ANGLE_CAL_H__
#define __ANGLE_CAL_H__

#include "mpu6050.h"
#include "MadgwickAHRS.h" /* Filter Algorithm for IMU*/

/* Euler angle in degre*/
typedef struct angle {
        float x;
        float y;
        float z;
}angle_t;

/* Kalman filter for MPU6050*/
typedef struct kalman
{
        float Q_angle; // Process noise
        float Q_gyroBias;
        float R; // Measurement nois

        /* State of system*/
        float x_angle; // Angle state
        float x_bias;  // Bias state

        /* Estimation error covariance */
        float P_00, P_01, P_10, P_11;

        /* Kalman gain*/
        float K_0, K_1;

}kalman_t;

void angle_complementary_getvalue( angle_t* pAngle, float sample_time);

void angle_kalman_init(kalman_t* kalman);

/* Return roll angle - using kalman filter */
float angle_kalman_getvalue(kalman_t* pKalman, float new_angle, float new_rate, float sample_time);

void angle_AHRS_getvalue(angle_t* pAngle, float sample_time);

/* Return roll angle - using kalman AHRS filter */
float angle_AHRS_get_roll(float sample_time);

#endif //__ANGLE_CAL_H__
