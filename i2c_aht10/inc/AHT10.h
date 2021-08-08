/**
 * @file AHT10.h
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-05
 * Part of this was written by : enjoyneering79
 * https://github.com/enjoyneering/AHT10/blob/master/src/AHT10.h
 * Modified by Aniket Mazumder
 * @copyright Copyright (c) Aniket Mazumder 2021
 */

#ifndef __AHT10__
#define __AHT10__
#include "stdbool.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define AHT10_ADDRESS_0X38         0x38  //chip I2C address no.1 for AHT10/AHT15/AHT20, address pin connected to GND
#define AHT10_ADDRESS_0X39         0x39  //chip I2C address no.2 for AHT10 only, address pin connected to Vcc

#define AHT10_INIT_CMD             0xE1  //initialization command for AHT10/AHT15
#define AHT20_INIT_CMD             0xBE  //initialization command for AHT20
#define AHT10_START_MEASURMENT_CMD 0xAC  //start measurment command
#define AHT10_NORMAL_CMD           0xA8  //normal cycle mode command, no info in datasheet!!!
#define AHT10_SOFT_RESET_CMD       0xBA  //soft reset command

#define AHT10_INIT_NORMAL_MODE     0x00  //enable normal mode
#define AHT10_INIT_CYCLE_MODE      0x20  //enable cycle mode
#define AHT10_INIT_CMD_MODE        0x40  //enable command mode
#define AHT10_INIT_CAL_ENABLE      0x08  //load factory calibration coeff


#define AHT10_DATA_MEASURMENT_CMD  0x33  //no info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHT10_DATA_NOP             0x00  //no info in datasheet!!!


#define AHT10_MEASURMENT_DELAY     80    //at least 75 milliseconds
#define AHT10_POWER_ON_DELAY       40    //at least 20..40 milliseconds
#define AHT10_CMD_DELAY            350   //at least 300 milliseconds, no info in datasheet!!!
#define AHT10_SOFT_RESET_DELAY     20    //less than 20 milliseconds

#define AHT10_FORCE_READ_DATA      true  //force to read data
#define AHT10_USE_READ_DATA        false //force to use data from previous read
#define AHT10_ERROR                0xFF  //returns 255, if communication error is occurred

#define NUMBER_OF_BYTES_TO_READ 6
#define NUMBER_OF_BYTES_TO_WRITE 3

typedef enum 
{
  AHT10_SENSOR = 0x00,
  AHT15_SENSOR = 0x01,
  AHT20_SENSOR = 0x02
}
ASAIR_I2C_SENSOR;

typedef enum{
  READ_TEMPERATURE =0,
  READ_HUMIDITY=1,
}MEASUREMENT_TYPE;

class AHT10
{
  public:
  //functions
  AHT10(i2c_inst_t *port, uint baudrate, uint SDA, uint SCL);
  ~AHT10();
  int read_register_AHT10(uint reg, uint size);

  bool sensor_busy_status();
  uint32_t read_raw_data_blocking(MEASUREMENT_TYPE);
  float read_temperature();
  float read_humidity();

  private:
  //variables
  i2c_inst_t* PORT;
  uint BAUD_RATE;
  uint SDA;
  uint SCL;

  uint8_t raw_data_buffer[NUMBER_OF_BYTES_TO_READ]={0,0,0,0,0,0};
};
#endif