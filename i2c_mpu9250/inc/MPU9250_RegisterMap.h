/**
 * @file MPU9250_RegisterMap.h
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief Header file for the MPU9250 class.
 * @version 0.1
 * @date 2021-08-29
 * @copyright Copyright (c) 2021 Aniket Mazumder
 * 
 */


/**
* The MIT License (MIT)
* Copyright (c) 2016 SparkFun Electronics, Inc.
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
 * https://raw.githubusercontent.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/master/src/MPU9250_RegisterMap.h
 * This file was copied from the above link
 * Modified by:Aniket Mazumder
 */
#ifndef _MPU9250_REGISTER_MAP_H_
#define _MPU9250_REGISTER_MAP_H_

#define MPU9250_ADDRESS_AD0_HIGH 0x68
#define MPU9250_ADDRESS_AD0_LOW 0x69

#define  MPU_RAW_REG_COUNT 14

enum mpu9250_register {
	MPU9250_SELF_TEST_X_GYRO =  0x00,
	MPU9250_SELF_TEST_Y_GYRO =  0x01,
	MPU9250_SELF_TEST_Z_GYRO =  0x02,
	MPU9250_SELF_TEST_X_ACCEL = 0x0D,
	MPU9250_SELF_TEST_Y_ACCEL = 0x0E,
	MPU9250_SELF_TEST_Z_ACCEL = 0x0F,
	MPU9250_XG_OFFSET_H =       0x13,
	MPU9250_XG_OFFSET_L =       0x14,
	MPU9250_YG_OFFSET_H =       0x15,
	MPU9250_YG_OFFSET_L =       0x16,
	MPU9250_ZG_OFFSET_H =       0x17,
	MPU9250_ZG_OFFSET_L =       0x18,
	MPU9250_SMPLRT_DIV =        0x19,
	MPU9250_CONFIG =            0x1A,
	MPU9250_GYRO_CONFIG =       0x1B,
	MPU9250_ACCEL_CONFIG =      0x1C,
	MPU9250_ACCEL_CONFIG_2 =    0x1D,
	MPU9250_LP_ACCEL_ODR =      0x1E,
	MPU9250_WOM_THR =           0x1F,
	MPU9250_FIFO_EN =           0x23,
	MPU9250_I2C_MST_CTRL =      0x24,
	MPU9250_I2C_SLV0_ADDR =     0x25,
	MPU9250_I2C_SLV0_REG =      0x26,
	MPU9250_I2C_SLV0_CTRL =     0x27,
	MPU9250_I2C_SLV1_ADDR =     0x28,
	MPU9250_I2C_SLV1_REG =      0x29,
	MPU9250_I2C_SLV1_CTRL =     0x2A,
	MPU9250_I2C_SLV2_ADDR =     0x2B,
	MPU9250_I2C_SLV2_REG =      0x2C,
	MPU9250_I2C_SLV2_CTRL =     0x2D,
	MPU9250_I2C_SLV3_ADDR =     0x2E,
	MPU9250_I2C_SLV3_REG =      0x2F,
	MPU9250_I2C_SLV3_CTRL =     0x30,
	MPU9250_I2C_SLV4_ADDR =     0x31,
	MPU9250_I2C_SLV4_REG =      0x32,
	MPU9250_I2C_SLV4_DO =       0x33,
	MPU9250_I2C_SLV4_CTRL =     0x34,
	MPU9250_I2C_SLV4_DI =       0x35,
	MPU9250_I2C_MST_STATUS =    0x36,
	MPU9250_INT_PIN_CFG =       0x37,
	MPU9250_INT_ENABLE =        0x38,
	MPU9250_INT_STATUS =        0x3A,
	MPU9250_ACCEL_XOUT_H =      0x3B,
	MPU9250_ACCEL_XOUT_L =      0x3C,
	MPU9250_ACCEL_YOUT_H =      0x3D,
	MPU9250_ACCEL_YOUT_L =      0x3E,
	MPU9250_ACCEL_ZOUT_H =      0x3F,
	MPU9250_ACCEL_ZOUT_L =      0x40,
	MPU9250_TEMP_OUT_H =        0x41,
	MPU9250_TEMP_OUT_L =        0x42,
	MPU9250_GYRO_XOUT_H =       0x43,
	MPU9250_GYRO_XOUT_L =       0x44,
	MPU9250_GYRO_YOUT_H =       0x45,
	MPU9250_GYRO_YOUT_L =       0x46,
	MPU9250_GYRO_ZOUT_H =       0x47,
	MPU9250_GYRO_ZOUT_L =       0x48,
	MPU9250_EXT_SENS_DATA_00 =  0x49,
	MPU9250_EXT_SENS_DATA_01 =  0x4A,
	MPU9250_EXT_SENS_DATA_02 =  0x4B,
	MPU9250_EXT_SENS_DATA_03 =  0x4C,
	MPU9250_EXT_SENS_DATA_04 =  0x4D,
	MPU9250_EXT_SENS_DATA_05 =  0x4E,
	MPU9250_EXT_SENS_DATA_06 =  0x4F,
	MPU9250_EXT_SENS_DATA_07 =  0x50,
	MPU9250_EXT_SENS_DATA_08 =  0x51,
	MPU9250_EXT_SENS_DATA_09 =  0x52,
	MPU9250_EXT_SENS_DATA_10 =  0x53,
	MPU9250_EXT_SENS_DATA_11 =  0x54,
	MPU9250_EXT_SENS_DATA_12 =  0x55,
	MPU9250_EXT_SENS_DATA_13 =  0x56,
	MPU9250_EXT_SENS_DATA_14 =  0x57,
	MPU9250_EXT_SENS_DATA_15 =  0x58,
	MPU9250_EXT_SENS_DATA_16 =  0x59,
	MPU9250_EXT_SENS_DATA_17 =  0x5A,
	MPU9250_EXT_SENS_DATA_18 =  0x5B,
	MPU9250_EXT_SENS_DATA_19 =  0x5C,
	MPU9250_EXT_SENS_DATA_20 =  0x5D,
	MPU9250_EXT_SENS_DATA_21 =  0x5E,
	MPU9250_EXT_SENS_DATA_22 =  0x5F,
	MPU9250_EXT_SENS_DATA_23 =  0x60,
	MPU9250_I2C_SLV0_DO =       0x63,
	MPU9250_I2C_SLV1_DO =       0x64,
	MPU9250_I2C_SLV2_DO =       0x65,
	MPU9250_I2C_SLV3_DO =       0x66,
	MPU9250_I2C_MST_DELAY_CTRL =0x67,
	MPU9250_SIGNAL_PATH_RESET = 0x68,
	MPU9250_MOT_DETECT_CTRL =   0x69,
	MPU9250_USER_CTRL =         0x6A,
	MPU9250_PWR_MGMT_1 =        0x6B,
	MPU9250_PWR_MGMT_2 =        0x6C,
	MPU9250_FIFO_COUNTH =       0x72,
	MPU9250_FIFO_COUNTL =       0x73,
	MPU9250_FIFO_R_W =          0x74,
	MPU9250_WHO_AM_I =          0x75,
	MPU9250_XA_OFFSET_H =       0x77,
	MPU9250_XA_OFFSET_L =       0x78,
	MPU9250_YA_OFFSET_H =       0x7A,
	MPU9250_YA_OFFSET_L =       0x7B,
	MPU9250_ZA_OFFSET_H =       0x7D,
	MPU9250_ZA_OFFSET_L =       0x7E
};

enum interrupt_status_bits {
	INT_STATUS_RAW_DATA_RDY_INT = 0,
	INT_STATUS_FSYNC_INT = 3,
	INT_STATUS_FIFO_OVERFLOW_INT = 4,
	INT_STATUS_WOM_INT = 6,
};

enum gyro_config_bits {
	GYRO_CONFIG_FCHOICE_B = 0,
	GYRO_CONFIG_GYRO_FS_SEL = 3,
	GYRO_CONFIG_ZGYRO_CTEN = 5,
	GYRO_CONFIG_YGYRO_CTEN = 6,
	GYRO_CONFIG_XGYRO_CTEN = 7,
};
#define MPU9250_GYRO_FS_SEL_MASK 0x3
#define MPU9250_GYRO_FCHOICE_MASK 0x3

enum accel_config_bit {
	ACCEL_CONFIG_ACCEL_FS_SEL = 3,
	ACCEL_CONFIG_AZ_ST_EN = 5,
	ACCEL_CONFIG_AY_ST_EN = 6,
	ACCEL_CONFIG_AX_ST_EN = 7,
};
#define MPU9250_ACCEL_FS_SEL_MASK 0x3

enum DLP_CFG_GYRO{
    GYRO_DLP_CFG_250_8=0,
    GYRO_DLP_CFG_184_1=1,
    GYRO_DLP_CFG_92_1=2,
    GYRO_DLP_CFG_41_1=3,
    GYRO_DLP_CFG_20_1=4,
    GYRO_DLP_CFG_10_1=5,
    GYRO_DLP_CFG_5_1=6,
    GYRO_DLP_CFG_3600_8=7,
};


enum DLP_CFG_ACC{
    ACC_DLP_CONFIG_460=0,
    ACC_DLP_CONFIG_184=1,
    ACC_DLP_CONFIG_92=2,
    ACC_DLP_CONFIG_41=3,
    ACC_DLP_CONFIG_20=4,
    ACC_DLP_CONFIG_10=5,
    ACC_DLP_CONFIG_5=6,
};

 

//Accelerometer full scale range 
typedef enum{
    ACC_FS_2G=0b0,
    ACC_FS_4G=0b01,
    ACC_FS_8G=0b10,
    ACC_FS_16G=0b11,
} MPU9250_ACCEL_FULL_SCALE;

//sensitivity selectors
#define ACC_SENSITIVITY_16384 16384
#define ACC_SENSITIVITY_8192 8192
#define ACC_SENSITIVITY_4096 4096
#define ACC_SENSITIVITY_2048 2048

//Gyroscope full scale range 
typedef enum{
    GYRO_FS_250DPS=0b0,
    GYRO_FS_500DPS=0b01,
    GYRO_FS_1000DPS=0b10,
    GYRO_FS_2000DPS=0b11,
} MPU9250_GYRO_FULL_SCALE;

//sensitivity selectors
#define GYRO_SENSITIVITY_131 131
#define GYRO_SENSITIVITY_65_5 65.5
#define GYRO_SENSITIVITY_32_8 32.8 
#define GYRO_SENSITIVITY_16_4 16.4

enum accel_config_2_bits {
	ACCEL_CONFIG_2_A_DLPFCFG = 0,
	ACCEL_CONFIG_2_ACCEL_FCHOICE_B = 3,
};
	
enum pwr_mgmt_1_bits {
	PWR_MGMT_1_CLKSEL = 0,
	PWR_MGMT_1_PD_PTAT = 3,
	PWR_MGMT_1_GYRO_STANDBY = 4,
	PWR_MGMT_1_CYCLE = 5,
	PWR_MGMT_1_SLEEP = 6,
	PWR_MGMT_1_H_RESET = 7
};

enum pwr_mgmt_2_bits {
	PWR_MGMT_2_DISABLE_ZG = 0,
	PWR_MGMT_2_DISABLE_YG = 1,
	PWR_MGMT_2_DISABLE_XG = 2,
	PWR_MGMT_2_DISABLE_ZA = 3,
	PWR_MGMT_2_DISABLE_YA = 4,
	PWR_MGMT_2_DISABLE_XA = 5,
};

enum int_enable_bits {
	INT_ENABLE_RAW_RDY_EN = 0,
	INT_ENABLE_FSYNC_INT_EN = 3,
	INT_ENABLE_FIFO_OVERFLOW_EN = 4,
	INT_ENABLE_WOM_EN = 6,
};

enum int_pin_cfg_bits {
	INT_PIN_CFG_BYPASS_EN = 1,
	INT_PIN_CFG_FSYNC_INT_MODE_EN = 2,
	INT_PIN_CFG_ACTL_FSYNC = 3,
	INT_PIN_CFG_INT_ANYRD_2CLEAR = 4,
	INT_PIN_CFG_LATCH_INT_EN = 5,
	INT_PIN_CFG_OPEN = 6,
	INT_PIN_CFG_ACTL = 7,
};
#define INT_PIN_CFG_INT_MASK 0xF0

#define MPU9250_WHO_AM_I_RESULT 0x71

enum ak8963_register {
	AK8963_WIA = 0x0,
	AK8963_INFO = 0x1,
	AK8963_ST1 = 0x2,
	AK8963_HXL = 0x3,
	AK8963_HXH = 0x4,
	AK8963_HYL = 0x5,
	AK8963_HYH = 0x6,
	AK8963_HZL = 0x7,
	AK8963_HZH = 0x8,
	AK8963_ST2 = 0x9,
	AK8963_CNTL = 0xA,
	AK8963_RSV = 0xB,
	AK8963_ASTC = 0xC,
	AK8963_TS1 = 0xD,
	AK8963_TS2 = 0xE,
	AK8963_I2CDIS = 0xF,
	AK8963_ASAX = 0x10,
	AK8963_ASAY = 0x11,
	AK8963_ASAZ = 0x12,
};
#define MAG_CTRL_OP_MODE_MASK 0xF

#define AK8963_ST1_DRDY_BIT 0

#define AK8963_WHO_AM_I_RESULT 0x48

#endif // _MPU9250_REGISTER_MAP_H_

