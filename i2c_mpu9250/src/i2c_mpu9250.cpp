/**
 * @file i2c_mpu9250.cpp
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief  Class file for MPU9250 with the Raspberry pi pico
 * @version 0.1
 * @date 2021-08-29
 * @copyright Copyright (c) 2021 Aniket Mazumder
 ** 
 */
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include "hardware/i2c.h"
#include "MPU9250_RegisterMap.h"
#include "stdint.h"
#include "i2c_mpu9250.h"
#include <stdio.h>

/**
 * @brief Construct a new MPU9250::MPU9250 object
 * @param port i2c port selected 
 * @param sda sda GPIO pin 
 * @param scl scl GPIO pin
 * @param acc_scale scale of accelerometer
 * @param gyro_scale scale of gyro selected
 */
MPU9250::MPU9250(i2c_inst_t* port, uint sda, uint scl,MPU9250_ACCEL_FULL_SCALE acc_scale, MPU9250_GYRO_FULL_SCALE gyro_scale)
{
    this->PORT=port;
    this->SDA=sda;
    this->SCL=scl;
    this->current_acc_scale=acc_scale;
    this->current_gyro_scale=gyro_scale;

    /* I2C Initialisation. Using it at 400Khz.*/
    i2c_init(this->PORT, 400*1000);
    
    gpio_set_function(this->SDA, GPIO_FUNC_I2C);
    gpio_set_function(this->SCL, GPIO_FUNC_I2C);

    /*Setup pull-up registers for the i2c pins*/
    gpio_pull_up(this->SDA);
    gpio_pull_up(this->SCL);

    if(MPU_config(this->current_acc_scale, this->current_gyro_scale)==true)
	    printf("Device Configured.\n");
    else
	    printf("Config failure.\n");

}

/**
 * @brief Function to configure the IMU.
 * @param acc_scale scale of IMU
 * @param gyro_scale scale of Gyro
 * @return true if configuration is successful
 * @return false if configuration is unsuccessful
 */
bool MPU9250::MPU_config(MPU9250_ACCEL_FULL_SCALE acc_scale, MPU9250_GYRO_FULL_SCALE gyro_scale)
{

    const uint8_t temp_acc_scale=acc_scale;
    const uint8_t temp_gyro_scale=gyro_scale;

    uint8_t config_register_write[5]={
	            MPU9250_CONFIG,//beginning of register sequence to write to
	            (GYRO_DLP_CFG_5_1),//write to register 26 MPU9250_CONFIG
	            (this->current_gyro_scale<<3)|(1),//write to register 27 MPU9250_GYRO_CONFIG
	            (this->current_acc_scale<<3)|(1),//write to register 28 MPU9250_ACCEL_CONFIG
	            (0)|(ACC_DLP_CONFIG_5)//write to register 29 MPU9250_ACCEL_CONFIG2
            };
    
    if(i2c_write_blocking(this->PORT,MPU9250_ADDRESS_AD0_HIGH ,config_register_write,5,true)==PICO_ERROR_GENERIC)
	    return false;

    return true;
}



/**
 * @brief 
 * @return true 
 * @return false 
 */
bool MPU9250::get_raw_data()
{
    const uint8_t acc_reg=MPU9250_ACCEL_XOUT_H;
    uint8_t i=0;
    
    /*Write to bus the address to read from and retain control*/
    if(i2c_write_blocking(this->PORT,MPU9250_ADDRESS_AD0_HIGH ,&acc_reg,1,true)==PICO_ERROR_GENERIC)
        return false;

    /*read 2 registers from the bus*/
    if(i2c_read_blocking(this->PORT,MPU9250_ADDRESS_AD0_HIGH ,this->raw_data_buffer,MPU_RAW_REG_COUNT,false)==PICO_ERROR_GENERIC)
        return false;

    return true;
}



/**
 * @brief get all readings from the IMU and return raw values
 * @return true if reading was successfull 
 * @return false if reading was unsuccessfull
 */
bool MPU9250::get_all_readings()
{
    if(get_raw_data())
    {	
	    ax=(uint16_t)raw_data_buffer[0]<<8|(uint16_t)raw_data_buffer[1];
	    ay=(uint16_t)raw_data_buffer[2]<<8|(uint16_t)raw_data_buffer[3];
	    az=(uint16_t)raw_data_buffer[4]<<8|(uint16_t)raw_data_buffer[5];
	    temp=(uint16_t)raw_data_buffer[6]<<8|(uint16_t)raw_data_buffer[7];
	    gx=(uint16_t)raw_data_buffer[8]<<8|(uint16_t)raw_data_buffer[9];
	    gy=(uint16_t)raw_data_buffer[10]<<8|(uint16_t)raw_data_buffer[11];
	    gz=(uint16_t)raw_data_buffer[12]<<8|(uint16_t)raw_data_buffer[13];
    }
    else
        return false;

    return true;
}



/**
 * @brief get acceleration values 
 * @param accel ACCELERATION axis ACCEL_X, ACCEL_Y, ACCEL_Z
 * @return float 
 */
float MPU9250::get_acceleration(ACCELERATION_AXIS accel)
{
    if(get_raw_data())
    {
	    ax=(uint16_t)raw_data_buffer[0]<<8|(uint16_t)raw_data_buffer[1];
	    ay=(uint16_t)raw_data_buffer[2]<<8|(uint16_t)raw_data_buffer[3];
	    az=(uint16_t)raw_data_buffer[4]<<8|(uint16_t)raw_data_buffer[5];
    
	    if(accel==ACCEL_X)
	        return ax;
	    else if(accel==ACCEL_Y)
	        return ay;
	    else 
	        return az;
    }
    
    return (float)PICO_ERROR_GENERIC;
}


float MPU9250::get_temperature()
{
    if(get_raw_data())
    {
        //How to get temperature is not clear from the data sheet
	    temp=(uint16_t)raw_data_buffer[6]<<8|(uint16_t)raw_data_buffer[7];
	    return temp;
    }

    return (float)PICO_ERROR_GENERIC;
}


/**
 * @brief 
 * @param gyro selected axis of gyro GYRO_X, GYRO_Y, GYRO_Z
 * @return float 
 */
float MPU9250::get_gyro(GYROSCOPE_AXIS gyro)
{
    if(get_raw_data())
    {
	    gx=(uint16_t)raw_data_buffer[8]<<8|(uint16_t)raw_data_buffer[9];
	    gy=(uint16_t)raw_data_buffer[10]<<8|(uint16_t)raw_data_buffer[11];
	    gz=(uint16_t)raw_data_buffer[12]<<8|(uint16_t)raw_data_buffer[13];

	    if(gyro==GYRO_X)
	        return gx;
	    else if(gyro==GYRO_Y)
	        return gy;
	    else if(gyro==GYRO_Z)
	        return gz;
    }
    
    return (float)PICO_ERROR_GENERIC;
}


/**
 * @brief 
 * @param src 
 * @param dst 
 * @return true 
 * @return false 
 */
bool MPU9250::read_register(uint8_t src, uint8_t *dst)
{
    uint8_t temp=src;

    if(i2c_write_blocking(this->PORT,MPU9250_ADDRESS_AD0_HIGH ,&temp,1,true)==PICO_ERROR_GENERIC)
	    return false;

    if(i2c_read_blocking(this->PORT,MPU9250_ADDRESS_AD0_HIGH ,dst,1,false)==PICO_ERROR_GENERIC)
	    return false;

    return true;
}
