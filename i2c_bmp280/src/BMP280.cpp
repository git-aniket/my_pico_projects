/**
 * @file BMP280.cpp
 * @author Aniket Mazumder(mazumder_aniket@hotmail.com)
 * @brief Class file for BMP280 to be used with raspberry pi pico
 * @version 0.1
 * @date 2021-08-27
 * 
 * @copyright Copyright (c) 2021 Aniket Mazumder
 *  BSD license, all text above must be included in any redistribution
 */


/*!
 *  @file Adafruit_BMP280.h
 *
 *  This is a library for the Adafruit BMP280 Breakout.
 *
 *  Designed specifically to work with the Adafruit BMP280 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2651
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  BSD license, all text above must be included in any redistribution
 */



#include <stdio.h>
#include "../inc/BMP280.h"


/**
 * @brief Construct a new BMP280::BMP280 object
 * Class contructor configures the sensor and reads calibration values from
 * the sensor.
 * @param port i2c port
 * @param baud_rate i2c baudrate
 * @param sda i2c sda pin
 * @param scl i2c scl pin
 */
BMP280::BMP280(i2c_inst* port,uint baud_rate, uint sda, uint scl )
{
    this->BAUD_RATE=baud_rate;
    this->PORT=port;
    this->SCL=scl;
    this->SDA=sda;

    i2c_init(this->PORT,this->BAUD_RATE);

    gpio_set_function(this->SCL,GPIO_FUNC_I2C);
    gpio_set_function(this->SDA,GPIO_FUNC_I2C);

    gpio_pull_up(this->SDA);
    gpio_pull_up(this->SCL);

    if(config_sensor()==false)
	printf("BMP280 config fail\n");
    else
	printf("BMP280 config success\n");

    if(read_calibration_data()==false)
	printf("BMP280 calibration data read fail.\n");
    else
	printf("BMP280 calibration data read success.\n");

}

/**
 * @brief Method to configure settings for pressure and temperature measurements.
 * Set the sampling rate, filtration rate, measurement mode etc here in the correspointing 
 * registers.
 * By default it is selected to Forced measurement.
 * 
 * @return true 
 * @return false 
 */
bool BMP280::config_sensor()
{
    bool config_status=false;

    uint8_t ctrl_meas_reg_set[2]={BMP280_REGISTER_CONTROL, (SAMPLING_X1<<5)|(SAMPLING_X1<<2)|(MODE_FORCED)};
    uint8_t config_reg_set[2]={BMP280_REGISTER_CONFIG, (STANDBY_MS_1<<5)|(FILTER_X2<<2)|(0)};

    //SET Mode Forced, Sampling None for temperature and pressure
    if(i2c_write_blocking(this->PORT,BMP280_ADDRESS_ALT,ctrl_meas_reg_set,2,true)==PICO_ERROR_GENERIC)
	return config_status;
    
    //SET interface, filter configuration and standby time in normal mode
    if(i2c_write_blocking(this->PORT,BMP280_ADDRESS_ALT,config_reg_set,2,false)==PICO_ERROR_GENERIC)
	return config_status;
    
    return true;
}

/**
 * @brief Read the value of a register and save it in destination dest
 * @param reg_input 
 * @param dest Register to store the returned value
 * @return true Read successful
 * @return false  Read unsuccessful
 */
bool BMP280::read_register(uint8_t reg_input, uint8_t* dest)
{
    bool config_status=false;

    if(i2c_write_blocking(this->PORT,BMP280_ADDRESS_ALT,&reg_input,1,true)==PICO_ERROR_GENERIC)
	return config_status;
    if(i2c_read_blocking(this->PORT,BMP280_ADDRESS_ALT,dest,1,false)==PICO_ERROR_GENERIC)
	return config_status;

    return true;
}


/**
 * @brief Method to read the calibration registers in BMP280
 * @return true Read Successful
 * @return false Write Successful
 */
bool BMP280::read_calibration_data()
{
    bool cal_status=false;
    uint8_t cal_array_len=24;
    uint8_t calibration_values_8bit[cal_array_len];
    uint8_t cal_reg_start=BMP280_REGISTER_DIG_T1;

    if(i2c_write_blocking(this->PORT,BMP280_ADDRESS_ALT,&cal_reg_start,1,true)==PICO_ERROR_GENERIC)
	return cal_status;

    if(i2c_read_blocking(this->PORT,BMP280_ADDRESS_ALT,calibration_values_8bit,cal_array_len,true)==PICO_ERROR_GENERIC)
	return cal_status;

    //calibration values for temperature
    calib_data.dig_T1=uint16_t(calibration_values_8bit[1])<<8|uint16_t(calibration_values_8bit[0]);
    calib_data.dig_T2=uint16_t(calibration_values_8bit[3])<<8|uint16_t(calibration_values_8bit[2]);
    calib_data.dig_T3=uint16_t(calibration_values_8bit[5])<<8|uint16_t(calibration_values_8bit[4]);

    //calibration values for pressure
    calib_data.dig_P1=uint16_t(calibration_values_8bit[7])<<8|uint16_t(calibration_values_8bit[6]);
    calib_data.dig_P2=uint16_t(calibration_values_8bit[9])<<8|uint16_t(calibration_values_8bit[8]);
    calib_data.dig_P3=uint16_t(calibration_values_8bit[11])<<8|uint16_t(calibration_values_8bit[10]);
    calib_data.dig_P4=uint16_t(calibration_values_8bit[13])<<8|uint16_t(calibration_values_8bit[12]);
    calib_data.dig_P5=uint16_t(calibration_values_8bit[15])<<8|uint16_t(calibration_values_8bit[14]);
    calib_data.dig_P6=uint16_t(calibration_values_8bit[17])<<8|uint16_t(calibration_values_8bit[16]);
    calib_data.dig_P7=uint16_t(calibration_values_8bit[19])<<8|uint16_t(calibration_values_8bit[18]);
    calib_data.dig_P8=uint16_t(calibration_values_8bit[21])<<8|uint16_t(calibration_values_8bit[20]);
    calib_data.dig_P9=uint16_t(calibration_values_8bit[23])<<8|uint16_t(calibration_values_8bit[22]);


    return true;
}

/**
 * @brief Read the raw sensor values into data-buffer
 * @return true Read successful
 * @return false  Read failed
 */
bool BMP280::read_raw_sensor_data()
{

    bool sensor_read_status=false;
    uint8_t data_reg=BMP280_REGISTER_PRESSUREDATA;

    //write register to read from
    if(i2c_write_blocking(this->PORT,BMP280_ADDRESS_ALT,&data_reg,1,false)==PICO_ERROR_GENERIC)
	return sensor_read_status;

    //wait until measurement completes. Use in Normal mode
    //while(sensor_busy())
    //{}

    //read and store in raw_data_buffer
    if(i2c_read_blocking(this->PORT,BMP280_ADDRESS_ALT,this->raw_data_buffer,6,false)==PICO_ERROR_GENERIC)
	return sensor_read_status;
    
    return true;
}

/**
 * @brief Read and return the temperature in Degree Celcius
 * @return float 
 */
float BMP280::read_temperature()
{
    float temp=0.0;
    int32_t adc_T=0;
    int32_t var1=0,var2=0;


    if(read_raw_sensor_data()==true)
    {
	adc_T=uint32_t(raw_data_buffer[3])<<16|uint32_t(raw_data_buffer[4])<<8|uint32_t(raw_data_buffer[5]);
	adc_T>>=4;

	var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) *
		          ((int32_t)calib_data.dig_T2)) >>11;

	var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) *
			((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) *
		            ((int32_t)calib_data.dig_T3)) >> 14;

	t_fine=var1+var2;
	temp=((t_fine*5+128)>>8)/100.0;//temperature in celcius
    }
    else
	printf("sensor raw data read faliure.\n");

    return temp;
}

/**
 * @brief Read and return the temperature in pascals
 * @return float 
 */
float BMP280::read_pressure()
{
    float pressure=0.0;
    int32_t adc_P=0;
    int64_t var1,var2,p;
    
    //must be done in order to set t_fine before hand
    read_temperature();

    if(read_raw_sensor_data()==true)
    {
	adc_P=uint32_t(raw_data_buffer[0])<<16|uint32_t(raw_data_buffer[1])<<8|uint32_t(raw_data_buffer[2]);
	adc_P>>4;


	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
	var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) +
		             ((var1 * (int64_t)calib_data.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;

	if (var1 == 0) 
	{
	    return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);

    }
    else
	printf("sensor read failure.\n");

    pressure=(float)p/256.0;
    return pressure;
}

/**
 * @brief Check busy status of sensor whether measuring or not
 * @return true Sensor in measurement state
 * @return false Measurement complete
 */
bool BMP280::sensor_busy()
{
    bool busy_status=true;
    uint8_t read_reg=BMP280_REGISTER_STATUS;
    uint8_t reg_val=0x01;

    if(i2c_write_blocking(this->PORT,BMP280_ADDRESS_ALT,&read_reg,1,true)==PICO_ERROR_GENERIC)
	printf("Sensor write falure.\n");
    if(i2c_read_blocking(this->PORT,BMP280_ADDRESS_ALT,&reg_val,1,false)==PICO_ERROR_GENERIC)
	printf("Sensor, read failure.\n");

    if(reg_val!=0x00)
	return busy_status;
    
    busy_status=false;
    return busy_status;
}
