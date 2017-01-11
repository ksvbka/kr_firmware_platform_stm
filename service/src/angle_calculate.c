/*
* @Author: Trung Kien
* @Date:   2016-12-21 23:34:13
* @Last Modified by:   ksvbka
* @Last Modified time: 2017-01-10 20:45:26
*/

#include "angle_calculate.h"
#include "math.h" /* For sqrt and atan2*/

/* Scale Value config for ACC of mpu6050*/
extern float g_acc_scale;

/* Scale Value config for GYRO of mpu6050*/
extern float g_gyro_scale;

#define RAD_TO_DEG (57.296f)
#define DEG_TO_RAD (0.01745f)

extern float invSqrt(float x);

void angle_complementary_getvalue( angle_t* pAngle, float dt)
{
        mpu_raw_data_t imu_data;
        mpu6050_get_rawdata(&imu_data);

        // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
        // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
        // It is then converted from radians to degrees

        // use RESTRICT_PITCH => use Eq. 25 and 26
        float roll  = atan2(imu_data.ay, imu_data.az) * RAD_TO_DEG;
        float pitch = atan(-imu_data.ax / sqrt(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)) * RAD_TO_DEG;

        float gyro_xrate = imu_data.gx / g_gyro_scale; /* Convert to deg/s*/
        float gyro_yrate = imu_data.gy / g_gyro_scale; /* Convert to deg/s*/

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
                pAngle->roll  = 0.93f * (pAngle->roll  + gyro_xrate * dt) + 0.07f * roll;
                pAngle->pitch = 0.93f * (pAngle->pitch + gyro_yrate * dt) + 0.07f * pitch;
        }

        /*FIX ME: Cannot calcutate z => fill 0*/
        pAngle->yaw = 0;

        /* Test*/
        // uart_printf("\n%f\t %f\t %f\t %f\t   %f\t %f\t %f\t %f\t", roll, 0.0, pAngle->roll, 0.0, pitch, 0.0, pAngle->pitch, 0.0);
}

/* Helper function */
static void kalman_init(kalman_t* kalman);
static float kalman_calculate(kalman_t* pKalman, float new_angle, float new_rate, float dt);


/*TODO:
        Need to fix issue of slow calculate
*/
void angle_kalman_getvalue(angle_t* pAngle, float sample_time)
{
        /* Using kalman filter */
        static kalman_t kalman_roll;
        static kalman_t kalman_pitch;

        mpu_raw_data_t imu_data;
        mpu6050_get_rawdata(&imu_data);

        float roll  = atan2(imu_data.ay, imu_data.az) * RAD_TO_DEG;
        float pitch = atan(-imu_data.ax / sqrt(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)) * RAD_TO_DEG;

        float gyro_xrate = imu_data.gx / g_gyro_scale; /* Convert to deg/s*/
        float gyro_yrate = imu_data.gy / g_gyro_scale; /* Convert to deg/s*/

        if (roll > 90 || roll < -90)
                gyro_yrate = -gyro_yrate;

        /* Init for the first time*/
        static uint8_t is_init = 0;
        if (is_init == 0) {
                kalman_init(&kalman_roll);
                kalman_init(&kalman_pitch);

                /*Set stating value */
                kalman_roll.x_angle  = roll;
                kalman_pitch.x_angle = pitch;

                is_init++;
        } else {
                /* Kalman filter */
                pAngle->roll  = kalman_calculate(&kalman_roll, roll, gyro_xrate, sample_time);
                pAngle->pitch = kalman_calculate(&kalman_pitch, pitch, gyro_yrate, sample_time);
        }

        /*FIX ME: Cannot calcutate z => fill 0*/
        pAngle->yaw = 0;

        /* Test*/
        // uart_printf("\n%f\t %f\t %f\t %f\t   %f\t %f\t %f\t %f\t", roll, pAngle->roll, 0.0, 0.0, pitch, pAngle->pitch, 0.0 ,0.0);
}

void kalman_init(kalman_t* kalman)
{
        /* Defaut value */
        kalman->Q_angle     = 0.001f;
        kalman->Q_bias      = 0.003f;
        kalman->R           = 0.03f;

        kalman->x_angle = 0.0f;
        kalman->x_bias  = 0.0f;

        kalman->P_00 = 0.0f;
        kalman->P_01 = 0.0f;
        kalman->P_10 = 0.0f;
        kalman->P_11 = 0.0f;

        kalman->K_0  = 0.0f;
        kalman->K_1  = 0.0f;
}

float kalman_calculate(kalman_t* pKalman, float new_angle, float new_rate, float dt)
{
        /* Update kalman*/

        /* Step 1 */
        pKalman->x_angle += dt * (new_rate - pKalman->x_bias);

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        pKalman->P_00 += dt * (dt * pKalman->P_11 - pKalman->P_01 - pKalman->P_10 + pKalman->Q_angle);
        pKalman->P_01 -= dt * pKalman->P_11;
        pKalman->P_10 -= dt * pKalman->P_11;
        pKalman->P_11 += pKalman->Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        float S = pKalman->P_00 + pKalman->R; /* Estimate error*/

        /* Step 5 */
        pKalman->K_0 = pKalman->P_00 / S;
        pKalman->K_1 = pKalman->P_10 / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        float y = new_angle - pKalman->x_angle;
        /* Step 6 */
        pKalman->x_angle +=  pKalman->K_0 * y;
        pKalman->x_bias  +=  pKalman->K_1 * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        pKalman->P_00 -= pKalman->K_0 * pKalman->P_00;
        pKalman->P_01 -= pKalman->K_0 * pKalman->P_01;
        pKalman->P_10 -= pKalman->K_1 * pKalman->P_00;
        pKalman->P_11 -= pKalman->K_1 * pKalman->P_01;

        return   pKalman->x_angle;
}

/**
 * Madgwick's implementation of Mayhony's AHRS algorithm.
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 * Modify by Trung Kien
 */

volatile float sampleFreq  = 100.0f;    /* sample frequency in Hz*/
volatile float beta = 0.1f ;            /*2 * proportional gain (Kp)*/
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; /*quaternion of sensor frame relative to auxiliary frame*/

/* Helper function */
static void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
static void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);

void angle_AHRS_getvalue(angle_t* pAngle, float sample_time)
{
        /*Init in the first time */
        static uint8_t is_init = 0;
        if(!is_init)
                sampleFreq = 1.0/sample_time;

        acc_data_t acc_data;
        gyro_data_t gyro_data;

        mpu6050_get_acc_data(&acc_data);
        mpu6050_get_gyro_data(&gyro_data);

        /* convert gyro readings into Radians per second*/
        float gyro_x = gyro_data.x * DEG_TO_RAD;
        float gyro_y = gyro_data.y * DEG_TO_RAD;
        float gyro_z = gyro_data.z * DEG_TO_RAD;

        MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, acc_data.x, acc_data.y, acc_data.z);

        pAngle->roll   = atan2(2 * (q0 * q1 + q2 * q3) , 1 - 2 * (q2 * q2 + q1 * q1)) * RAD_TO_DEG;
        pAngle->pitch  = asin(2 * (q0 * q2 -  q1 * q3)) * RAD_TO_DEG;
        pAngle->yaw    = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

        /* Test */
        // uart_printf("\n%f\t %f\t %f\t", pAngle->roll, pAngle->pitch, pAngle->yaw);
}


void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
                MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
                return;
        }

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

                // Normalise accelerometer measurement
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Normalise magnetometer measurement
                recipNorm = invSqrt(mx * mx + my * my + mz * mz);
                mx *= recipNorm;
                my *= recipNorm;
                mz *= recipNorm;

                // Auxiliary variables to avoid repeated arithmetic
                _2q0mx = 2.0f * q0 * mx;
                _2q0my = 2.0f * q0 * my;
                _2q0mz = 2.0f * q0 * mz;
                _2q1mx = 2.0f * q1 * mx;
                _2q0 = 2.0f * q0;
                _2q1 = 2.0f * q1;
                _2q2 = 2.0f * q2;
                _2q3 = 2.0f * q3;
                _2q0q2 = 2.0f * q0 * q2;
                _2q2q3 = 2.0f * q2 * q3;
                q0q0 = q0 * q0;
                q0q1 = q0 * q1;
                q0q2 = q0 * q2;
                q0q3 = q0 * q3;
                q1q1 = q1 * q1;
                q1q2 = q1 * q2;
                q1q3 = q1 * q3;
                q2q2 = q2 * q2;
                q2q3 = q2 * q3;
                q3q3 = q3 * q3;

                // Reference direction of Earth's magnetic field
                hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
                hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
                _2bx = sqrt(hx * hx + hy * hy);
                _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
                _4bx = 2.0f * _2bx;
                _4bz = 2.0f * _2bz;

                // Gradient decent algorithm corrective step
                s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;

                // Apply feedback step
                qDot1 -= beta * s0;
                qDot2 -= beta * s1;
                qDot3 -= beta * s2;
                qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0f / sampleFreq);
        q1 += qDot2 * (1.0f / sampleFreq);
        q2 += qDot3 * (1.0f / sampleFreq);
        q3 += qDot4 * (1.0f / sampleFreq);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

                // Normalise accelerometer measurement
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Auxiliary variables to avoid repeated arithmetic
                _2q0 = 2.0f * q0;
                _2q1 = 2.0f * q1;
                _2q2 = 2.0f * q2;
                _2q3 = 2.0f * q3;
                _4q0 = 4.0f * q0;
                _4q1 = 4.0f * q1;
                _4q2 = 4.0f * q2;
                _8q1 = 8.0f * q1;
                _8q2 = 8.0f * q2;
                q0q0 = q0 * q0;
                q1q1 = q1 * q1;
                q2q2 = q2 * q2;
                q3q3 = q3 * q3;

                // Gradient decent algorithm corrective step
                s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
                s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
                s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
                s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
                recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;

                // Apply feedback step
                qDot1 -= beta * s0;
                qDot2 -= beta * s1;
                qDot3 -= beta * s2;
                qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0f / sampleFreq);
        q1 += qDot2 * (1.0f / sampleFreq);
        q2 += qDot3 * (1.0f / sampleFreq);
        q3 += qDot4 * (1.0f / sampleFreq);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
}
