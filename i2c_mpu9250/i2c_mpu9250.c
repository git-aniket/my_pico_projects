#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu9250.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

static int8_t read_buff[2]={0,0};
const int8_t acc_reg=ACCEL_ZOUT_H;
const int8_t temp_reg=TEMP_OUT_H;

int16_t acceleration_Z=0;
int16_t temp=0;

int main()
{
    sleep_ms(100);
    stdio_init_all();
    sleep_ms(100);
    printf("Initializing!");

    /* I2C Initialisation. Using it at 400Khz.*/
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    /*Setup pull-up registers for the pins*/
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    while(1)
    {
	/*Write to bus the address to read from and retain control*/
	i2c_write_blocking(i2c0,IMU_ADDRESS_LOW,&acc_reg,1,true);

	/*read 2 registers from the bus*/
	i2c_read_blocking(i2c0,IMU_ADDRESS_LOW,read_buff,2,false);

	acceleration_Z=(read_buff[0]<<8)|read_buff[1];

	printf("The accelZ is :%d\r\n",acceleration_Z);
	sleep_ms(1000);
    }

    /*Should not come to this point during normal operation*/
    return 0;

}
