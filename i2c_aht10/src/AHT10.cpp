/**
 * @file AHT10.cpp
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief Source file for AHT10 temperature and humidity sensor.
 * @version 0.1
 * @date 2021-08-05
 * Part of this was inspired by the works of : enjoyneering79 and Thinary
 * https://github.com/enjoyneering/AHT10/blob/master/src/AHT10.h
 * https://github.com/Thinary/AHT10
 * Written by Aniket Mazumder
 * @copyright Copyright (c) Aniket Mazumder 2021
 */

#include <stdio.h>
#include "../inc/AHT10.h"

/**
 * @brief Construct a new AHT10::AHT10 object
 * Initalize pins, set pullups and initialize device with calibration data
 * @param port 
 * @param baudrate 
 * @param sda Pin
 * @param scl Pin
 */
AHT10::AHT10(i2c_inst* port, uint baudrate, uint sda, uint scl)
{
    uint8_t coupled_commands_calibrate_sensor[NUMBER_OF_BYTES_TO_WRITE]=
                        {AHT10_INIT_CMD,AHT10_INIT_CAL_ENABLE,AHT10_DATA_NOP};

    this->PORT=port;
    this->BAUD_RATE=baudrate;
    this->SDA=sda;
    this->SCL=scl;

    //set i2c funtion for the pins
    gpio_set_function(this->SDA,GPIO_FUNC_I2C);
    gpio_set_function(this->SCL, GPIO_FUNC_I2C);

    //set pull up on the i2c pins
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);

    //initialize the port
    i2c_init(this->PORT, this->BAUD_RATE);

    //sleep atleast 20ms as said in the data-sheet to get ready for read
    sleep_ms(30);
    if(i2c_write_blocking(this->PORT,AHT10_ADDRESS_0X38,coupled_commands_calibrate_sensor,NUMBER_OF_BYTES_TO_WRITE,false)==PICO_ERROR_GENERIC)
        printf("Device write error\n");

}

AHT10::~AHT10(){
    i2c_deinit(this->PORT);
}

/**
 * @brief read raw data from the sensor and return.
 * Function is blocking in nature.
 * @param measurement_type temperature or humidity
 * @return uint32_t array of data
 */
uint32_t AHT10::read_raw_data_blocking(MEASUREMENT_TYPE measurement_type)
{
    uint32_t data=0;
    uint8_t coupled_commands_measure[NUMBER_OF_BYTES_TO_WRITE]=
        {AHT10_START_MEASURMENT_CMD,AHT10_DATA_MEASURMENT_CMD,AHT10_DATA_NOP};

    if(i2c_write_blocking(this->PORT,AHT10_ADDRESS_0X38,coupled_commands_measure,NUMBER_OF_BYTES_TO_WRITE,false)==PICO_ERROR_GENERIC)
        printf("sensor write failure\n");
    
    //Sensor needs time to collect data See datasheet section 5.3
    //Don't trigger new temp/humidity measurement when sensor is busy
    while(sensor_busy_status()==true)

    if(sensor_busy_status()==false)
    {
        //read 6 bytes from the sensor consisting of the status, temp and humidity data
        if(i2c_read_blocking(this->PORT,AHT10_ADDRESS_0X38,raw_data_buffer, NUMBER_OF_BYTES_TO_READ,false)==PICO_ERROR_GENERIC)
            printf("sensor read failure\n");

        if(measurement_type== READ_TEMPERATURE)
            data=(((raw_data_buffer[3] & 0x0F) << 16) | (raw_data_buffer[4] << 8) |raw_data_buffer[5]);

        else if(measurement_type==READ_HUMIDITY)
            data=(((raw_data_buffer[1] << 16) | (raw_data_buffer[2] << 8) |raw_data_buffer[3]) >> 4);
    }
   
    return data;
}

/**
 * @brief convert raw data into temperature reading and return value
 * @return float temperature
 */
float AHT10::read_temperature()
{
    float temp= 0.0;
    temp=read_raw_data_blocking(READ_TEMPERATURE);
    temp=((200.0*(float)temp)/1048576.0)-50.0;
    return temp;
}

/**
 * @brief convert raw data into huimdity value and return
 * @return float humidity
 */
float AHT10::read_humidity()
{
    float humidity=0.0;
    humidity=read_raw_data_blocking(READ_HUMIDITY);
    humidity=(float)humidity*100.0/1048576.0;
    return humidity;
}

/**
 * @brief Check the sensor busy status bit
 * @return true 
 * @return false 
 */
bool AHT10::sensor_busy_status()
{
    bool busy=true;
    uint8_t sensor_state=0x0;
    i2c_read_blocking(this->PORT,AHT10_ADDRESS_0X38,&sensor_state,1,false);
    sensor_state=sensor_state>>7;
    if(sensor_state==1)
        busy=true;
    else
        busy =false;

    return busy;
}
