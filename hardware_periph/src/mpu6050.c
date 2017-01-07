/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:33:44
* @Last Modified by:   ksvbka
* @Last Modified time: 2017-01-08 00:52:48
*/

#include "mpu6050.h"
#include "i2c.h"

#include "utility.h"

/* Register define */

#define MPU6050_ADDRESS         0x68 // Address MPU6050 0b01101000  0b10100100
#define MPU6050_XG_OFFS_TC      0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_YG_OFFS_TC      0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_ZG_OFFS_TC      0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_X_FINE_GAIN     0x03 //[7:0] X_FINE_GAIN
#define MPU6050_Y_FINE_GAIN     0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_Z_FINE_GAIN     0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_XA_OFFS_H       0x06 //[15:0] XA_OFFS
#define MPU6050_XA_OFFS_L_TC    0x07
#define MPU6050_YA_OFFS_H       0x08 //[15:0] YA_OFFS
#define MPU6050_YA_OFFS_L_TC    0x09
#define MPU6050_ZA_OFFS_H       0x0A //[15:0] ZA_OFFS
#define MPU6050_ZA_OFFS_L_TC    0x0B
#define MPU6050_XG_OFFS_USRH    0x13 //[15:0] XG_OFFS_USR
#define MPU6050_XG_OFFS_USRL    0x14
#define MPU6050_YG_OFFS_USRH    0x15 //[15:0] YG_OFFS_USR
#define MPU6050_YG_OFFS_USRL    0x16
#define MPU6050_ZG_OFFS_USRH    0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_ZG_OFFS_USRL    0x18
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_FF_THR          0x1D
#define MPU6050_FF_DUR          0x1E
#define MPU6050_MOT_THR         0x1F
#define MPU6050_MOT_DUR         0x20
#define MPU6050_ZRMOT_THR       0x21
#define MPU6050_ZRMOT_DUR       0x22
#define MPU6050_FIFO_EN         0x23
#define MPU6050_I2C_MST_CTRL    0x24
#define MPU6050_I2C_SLV0_ADDR   0x25
#define MPU6050_I2C_SLV0_REG    0x26
#define MPU6050_I2C_SLV0_CTRL   0x27
#define MPU6050_I2C_SLV1_ADDR   0x28
#define MPU6050_I2C_SLV1_REG    0x29
#define MPU6050_I2C_SLV1_CTRL   0x2A
#define MPU6050_I2C_SLV2_ADDR   0x2B
#define MPU6050_I2C_SLV2_REG    0x2C
#define MPU6050_I2C_SLV2_CTRL   0x2D
#define MPU6050_I2C_SLV3_ADDR   0x2E
#define MPU6050_I2C_SLV3_REG    0x2F
#define MPU6050_I2C_SLV3_CTRL   0x30
#define MPU6050_I2C_SLV4_ADDR   0x31
#define MPU6050_I2C_SLV4_REG    0x32
#define MPU6050_I2C_SLV4_DO     0x33
#define MPU6050_I2C_SLV4_CTRL   0x34
#define MPU6050_I2C_SLV4_DI     0x35
#define MPU6050_I2C_MST_STATUS  0x36
#define MPU6050_INT_PIN_CFG     0x37
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_DMP_INT_STATUS  0x39
#define MPU6050_INT_STATUS      0x3A
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
#define MPU6050_EXT_SENS_DATA_00 0x49
#define MPU6050_EXT_SENS_DATA_01 0x4A
#define MPU6050_EXT_SENS_DATA_02 0x4B
#define MPU6050_EXT_SENS_DATA_03 0x4C
#define MPU6050_EXT_SENS_DATA_04 0x4D
#define MPU6050_EXT_SENS_DATA_05 0x4E
#define MPU6050_EXT_SENS_DATA_06 0x4F
#define MPU6050_EXT_SENS_DATA_07 0x50
#define MPU6050_EXT_SENS_DATA_08 0x51
#define MPU6050_EXT_SENS_DATA_09 0x52
#define MPU6050_EXT_SENS_DATA_10 0x53
#define MPU6050_EXT_SENS_DATA_11 0x54
#define MPU6050_EXT_SENS_DATA_12 0x55
#define MPU6050_EXT_SENS_DATA_13 0x56
#define MPU6050_EXT_SENS_DATA_14 0x57
#define MPU6050_EXT_SENS_DATA_15 0x58
#define MPU6050_EXT_SENS_DATA_16 0x59
#define MPU6050_EXT_SENS_DATA_17 0x5A
#define MPU6050_EXT_SENS_DATA_18 0x5B
#define MPU6050_EXT_SENS_DATA_19 0x5C
#define MPU6050_EXT_SENS_DATA_20 0x5D
#define MPU6050_EXT_SENS_DATA_21 0x5E
#define MPU6050_EXT_SENS_DATA_22 0x5F
#define MPU6050_EXT_SENS_DATA_23 0x60
#define MPU6050_MOT_DETECT_STATUS   0x61
#define MPU6050_I2C_SLV0_DO         0x63
#define MPU6050_I2C_SLV1_DO         0x64
#define MPU6050_I2C_SLV2_DO         0x65
#define MPU6050_I2C_SLV3_DO         0x66
#define MPU6050_I2C_MST_DELAY_CTRL  0x67
#define MPU6050_SIGNAL_PATH_RESET   0x68
#define MPU6050_MOT_DETECT_CTRL     0x69
#define MPU6050_USER_CTRL           0x6A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C
#define MPU6050_BANK_SEL            0x6D
#define MPU6050_MEM_START_ADDR      0x6E
#define MPU6050_MEM_R_W             0x6F
#define MPU6050_DMP_CFG_1           0x70
#define MPU6050_DMP_CFG_2           0x71
#define MPU6050_FIFO_COUNTH         0x72
#define MPU6050_FIFO_COUNTL         0x73
#define MPU6050_FIFO_R_W            0x74
#define MPU6050_WHO_AM_I            0x75

/*---------- CONFIG VALUE----------*/
/* MPU6050_PWR_MGMT_1 REG */
#define CLKSEL_0 0x00 //Internal 8MHz Osilator
#define CLKSEL_1 0x01 //PLL with X axis gyroscope reference
#define CLKSEL_2 0x02 //PLL with Y axis gyroscope reference
#define CLKSEL_3 0x03 //PLL with Z axis gyroscope reference
#define CLKSEL_4 0x04 //PLL with external 32.768kHz reference
#define CLKSEL_5 0x05 //PLL with external 19.2MHz reference
#define CLKSEL_6 0x06 //Reserved
#define CLKSEL_7 0x07 //Stops the clock and keeps the timing generator in reset

#define TEMP_DIS     (0x01 << 3) //disables the temperature sensor
#define CYCLE        (0x01 << 5) //
#define SLEEP        (0x01 << 6) //
#define DEVICE_RESET (0x01 << 7) //

/*----------MPU6050_CONFIG----------*/
#define EXT_SYNC_SET_INPUT_DISABLE  (0x00 << 3)// Input disabled
#define EXT_SYNC_SET_TEMP_OUT       (0x01 << 3)// TEMP_OUT_L[0]
#define EXT_SYNC_SET_GYRO_XOUT      (0x02 << 3)// GYRO_XOUT_L[0]
#define EXT_SYNC_SET_GYRO_YOUT      (0x03 << 3)// GYRO_YOUT_L[0]
#define EXT_SYNC_SET_GYRO_ZOUT      (0x04 << 3)// GYRO_ZOUT_L[0]
#define EXT_SYNC_SET_ACCEL_XOUT     (0x05 << 3)// ACCEL_XOUT_L[0]
#define EXT_SYNC_SET_ACCEL_YOUT     (0x06 << 3)// ACCEL_YOUT_L[0]
#define EXT_SYNC_SET_ACCEL_ZOUT     (0x07 << 3)// ACCEL_ZOUT_L[0]

#define DLPF_CFG_BAND_WIDTH_260HZ       0x00// BandWidth 260Hz
#define DLPF_CFG_BAND_WIDTH_184HZ       0x01// BandWidth 184Hz
#define DLPF_CFG_BAND_WIDTH_94HZ        0x02// BandWidth 94Hz
#define DLPF_CFG_BAND_WIDTH_44HZ        0x03// BandWidth 44Hz
#define DLPF_CFG_BAND_WIDTH_21HZ        0x04// BandWidth 21Hz
#define DLPF_CFG_BAND_WIDTH_10HZ        0x05// BandWidth 10Hz
#define DLPF_CFG_BAND_WIDTH_5HZ         0x06// BandWidth 5Hz

/*---------MPU6050_GYRO_CONFIG-------*/
#define PS_SEL_SCALE_250    (0x00 << 3)
#define PS_SEL_SCALE_500    (0x01 << 3)
#define PS_SEL_SCALE_1000   (0x02 << 3)
#define PS_SEL_SCALE_2000   (0x03 << 3)

#define ZG_ST   (0x01 << 5)
#define YG_ST   (0x01 << 6)
#define XG_ST   (0x01 << 7)

/*---------MPU6050_ACCEL_CONFIG------*/
#define AFS_SEL_SCALE_2G    (0x00 << 3)
#define AFS_SEL_SCALE_4G    (0x01 << 3)
#define AFS_SEL_SCALE_8G    (0x02 << 3)
#define AFS_SEL_SCALE_16G   (0x03 << 3)

#define ZA_ST   (0x01 << 5)
#define YA_ST   (0x01 << 6)
#define XA_ST   (0x01 << 7)
/*---------MPU6050_SMPLRT_DIV_CONFIG---*/
#define SET_SAMPLE_RATE_1000HZ   0x07

/*-----------------------------------------------------------------------------*/
/* AFS_SEL | Full Scale Range | LSB Sensitivity
* --------+------------------+----------------
* 0       | +/- 2g           | 16384 LSB/mg
* 1       | +/- 4g           | 8192 LSB/mg
* 2       | +/- 8g           | 4096 LSB/mg
* 3       | +/- 16g          | 2043 LSB/mg
*/
#define SCALED_ACC_2G       16384.0
#define SCALED_ACC_4G       8192.0
#define SCALED_ACC_8G       4096.0
#define SCALED_ACC_16G      2043.0

/* FS_SEL | Full Scale Range   | LSB Sensitivity
* -------+--------------------+----------------
* 0      | +/- 250 degrees/s  | 131 LSB/deg/s
* 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
* 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
* 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
*/
#define SCALED_GYRO_250     131.0
#define SCALED_GYRO_500     65.5
#define SCALED_GYRO_1000    32.8
#define SCALED_GYRO_2000    16.4

/*
*   Note:
*          |   ACCELEROMETER    |           GYROSCOPE
* DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
* ---------+-----------+--------+-----------+--------+-------------
* 0        | 260Hz     | 0ms    | 256Hz     | 0.95ms | 8kHz
* 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
* 2        | 94Hz      | 3.0ms  | 95Hz      | 2.8ms  | 1kHz
* 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
* 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
* 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
* 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
* 7        |   -- Reserved --   |   -- Reserved --   | Reserved
*/


/* Off set value to calibrate Acc*/
static int16_t acc_offsetX = 1733;
static int16_t acc_offsetY = 509;
static int16_t acc_offsetZ = 338;

/*Offset value to calibrate Gyro */
static int16_t gyro_offsetX = -165;
static int16_t gyro_offsetY = 84;
static int16_t gyro_offsetZ = -150;

/* Scale Value config for ACC - default is 2G*/
double g_acc_scale  = SCALED_ACC_2G;

/* Scale Value config for GYRO - default is 250*/
double g_gyro_scale = SCALED_GYRO_250;


void mpu6050_init(uint8_t acc_scale_config, uint8_t gyro_scale_config)
{
        /* Disable FSync, 256Hz DLPF*/
        i2c_write_byte(MPU6050_ADDRESS, MPU6050_CONFIG, DLPF_CFG_BAND_WIDTH_260HZ);

        /* Config Accel*/
        uint8_t acc_config;
        switch (acc_scale_config) {
        case  ACC_CONFIG_2G:
                g_acc_scale  =   SCALED_ACC_2G;
                acc_config   =   AFS_SEL_SCALE_2G;
                break;
        case  ACC_CONFIG_4G:
                g_acc_scale  =   SCALED_ACC_4G;
                acc_config   =   AFS_SEL_SCALE_4G;
                break;
        case  ACC_CONFIG_8G:
                g_acc_scale  =   SCALED_ACC_8G;
                acc_config   =   AFS_SEL_SCALE_8G;
                break;
        case  ACC_CONFIG_16G:
                g_acc_scale  =   SCALED_ACC_16G;
                acc_config   =   AFS_SEL_SCALE_16G;
                break;
        }
        i2c_write_byte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, acc_config);

        /* Congfig Gyro*/
        uint8_t gyro_config;
        switch (gyro_scale_config) {
        case GYRO_CONFIG_250:
                g_gyro_scale  =  SCALED_GYRO_250;
                gyro_config   =  PS_SEL_SCALE_250;
                break;
        case GYRO_CONFIG_500:
                g_gyro_scale  =  SCALED_GYRO_500;
                gyro_config   =  PS_SEL_SCALE_500;
                break;
        case GYRO_CONFIG_1000:
                g_gyro_scale  =  SCALED_GYRO_1000;
                gyro_config   =  PS_SEL_SCALE_1000;
                break;
        case GYRO_CONFIG_2000:
                g_gyro_scale  =  SCALED_GYRO_2000;
                gyro_config   =  PS_SEL_SCALE_2000;
                break;
        }
        /* I2C_WriteByte(gyro_config, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG);*/
        i2c_write_byte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, gyro_config);

        /* Enable MPU6050; Clock 8MHZ*/
        i2c_write_byte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, CLKSEL_0);

        /* Set sample rate: 8MHz/(1 + 7) = 1kHz*/
        i2c_write_byte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, SET_SAMPLE_RATE_1000HZ);

        delay_ms(100);
}

void mpu6050_get_rawdata(mpu_raw_data_t* data)
{
        uint8_t buff[14];
        i2c_read_data(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, buff, 14);

        /* Acc raw data */
        data->ax   = (buff[0]  << 8 | buff[1]) - acc_offsetX;
        data->ay   = (buff[2]  << 8 | buff[3]) - acc_offsetY;
        data->az   = (buff[4]  << 8 | buff[5]) - acc_offsetZ;

        /* Temparature */
        data->temp = (buff[6]  << 8 | buff[7]);

        /* Gyro raw data */
        data->gx   = (buff[8]  << 8 | buff[9])  - gyro_offsetX;
        data->gy   = (buff[10] << 8 | buff[11]) - gyro_offsetY;
        data->gz   = (buff[12] << 8 | buff[13]) - gyro_offsetZ;
}

void mpu6050_get_acc_data(acc_data_t* data)
{
        mpu_raw_data_t raw_data;
        mpu6050_get_rawdata(&raw_data);

        data->x = raw_data.ax / g_acc_scale;
        data->y = raw_data.ay / g_acc_scale;
        data->z = raw_data.az / g_acc_scale;
}

void mpu6050_get_gyro_data(gyro_data_t* data)
{
        mpu_raw_data_t raw_data;
        mpu6050_get_rawdata(&raw_data);

        data->x = raw_data.gx / g_gyro_scale;
        data->y = raw_data.gy / g_gyro_scale;
        data->z = raw_data.gz / g_gyro_scale;
}

/* Helper function: get mean of sensor value and store to buffer*/
static int16_t acel_deadzone = 10; /* +- 10*/
static int16_t gyro_deadzone = 3;  /* +- 3*/

static mpu_raw_data_t raw_data;
int16_t mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0;
static void mean_sensor(void);

void mpu6050_calibrate(void)
{
        acc_offsetX  = mean_ax / 8;
        acc_offsetY  = mean_gy / 8;
        acc_offsetZ  = (16384 - mean_az) / 8;

        gyro_offsetX = mean_gx / 4;
        gyro_offsetY = mean_gy / 4;
        gyro_offsetZ = mean_gz / 4;

        while (1) {
                int ready = 0;
                uart_printf("\n calibrating...");

                mean_sensor();

                uart_printf("\nmean value: %d\t %d\t %d\t %d\t %d\t %d\t ", mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);

                uart_printf("\noffset    : %d\t %d\t %d\t %d\t %d\t %d\t ", acc_offsetX\
                                                                , acc_offsetY\
                                                                , acc_offsetZ\
                                                                , gyro_offsetX\
                                                                , gyro_offsetY\
                                                                , gyro_offsetZ);

                if (ABS(mean_ax) <= acel_deadzone) ready++;
                else acc_offsetX += mean_ax / acel_deadzone;

                if (ABS(mean_ay) <= acel_deadzone) ready++;
                else acc_offsetY += mean_ay / acel_deadzone;

                if (ABS(16384 - mean_az) <= acel_deadzone) ready++;
                else acc_offsetZ += -(16384 - mean_az) / acel_deadzone;


                if (ABS(mean_gx) <= gyro_deadzone) ready++;
                else gyro_offsetX  += mean_gx / (gyro_deadzone + 1);

                if (ABS(mean_gy) <= gyro_deadzone) ready++;
                else gyro_offsetY  += mean_gy / (gyro_deadzone + 1);

                if (ABS(mean_gz) <= gyro_deadzone) ready++;
                else gyro_offsetZ  += mean_gz / (gyro_deadzone + 1);

                if (ready == 6) break;

        }
}


#define BUFFER_SIZE 500

void mean_sensor(void)
{
        int16_t i = 0;
        int32_t aXvalue = 0, aYvalue = 0, aZvalue = 0, gXvalue = 0, gYvalue = 0, gZvalue = 0;

        while (i < (BUFFER_SIZE + 101)) {
                mpu6050_get_rawdata(&raw_data);
                if (i > 100  && i <= (BUFFER_SIZE + 100)) { /* First 100 measures are discarded*/
                        aXvalue  += raw_data.ax;
                        aYvalue  += raw_data.ay;
                        aZvalue  += raw_data.az;

                        gXvalue  += raw_data.gx;
                        gYvalue  += raw_data.gy;
                        gZvalue  += raw_data.gz;
                }
                if (i == BUFFER_SIZE + 100) {
                        mean_ax = (int16_t)(aXvalue / BUFFER_SIZE);
                        mean_ay = (int16_t)(aYvalue / BUFFER_SIZE);
                        mean_az = (int16_t)(aZvalue / BUFFER_SIZE);
                        mean_gx = (int16_t)(gXvalue / BUFFER_SIZE);
                        mean_gy = (int16_t)(gYvalue / BUFFER_SIZE);
                        mean_gz = (int16_t)(gZvalue / BUFFER_SIZE);
                }
                i++;
        }
}
