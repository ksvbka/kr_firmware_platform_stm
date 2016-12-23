/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:33:44
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-21 23:41:26
*/

#ifndef __MPU6050__H__
#define __MPU6050__H__

#include "typedef.h"

#define GYRO_CONFIG_250         0
#define GYRO_CONFIG_500         1
#define GYRO_CONFIG_1000        2
#define GYRO_CONFIG_2000        3

#define ACC_CONFIG_2G           4
#define ACC_CONFIG_4G           5
#define ACC_CONFIG_8G           6
#define ACC_CONFIG_16G          7

/* mpu6050 data raw value */
typedef struct mpu_raw_data {
        int16_t ax;
        int16_t ay;
        int16_t az;
        int16_t temp;
        int16_t gx;
        int16_t gy;
        int16_t gz;
} mpu_raw_data_t;


typedef struct acc_data {
        float x; /* rad/s*/
        float y; /* rad/s*/
        float z; /* rad/s*/
} acc_data_t;


typedef struct gryro_data {
        float x; /* m/s^2*/
        float y; /* m/s^2*/
        float z; /* m/s^2*/

} gyro_data_t;

void mpu6050_init(uint8_t acc_scale_config, uint8_t gyro_scale_config);

void mpu6050_calibrate(void);

void mpu6050_get_rawdata(mpu_raw_data_t* data);

void mpu6050_get_acc_data(acc_data_t* data);

void mpu6050_get_gyro_data(gyro_data_t* data);

#endif // __MPU6050__H__
