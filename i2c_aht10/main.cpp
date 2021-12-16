/**
 * @file main.cpp
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief example file showing how to read data from the AHT10 sensor
 * @version 0.1
 * @date 2021-08-08
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#include "../inc/AHT10.h"
#define I2C_PORT0 i2c0
#define I2C0_SDA 8
#define I2C0_SCL 9

int main()
{
    stdio_init_all();

    //initialize sensor
    AHT10 sensor(I2C_PORT0,400*1000,I2C0_SDA,I2C0_SCL);
    
    while(1)
    {
        printf("temperature:%f\n",sensor.read_temperature());
        printf("humidity:%f\n",sensor.read_humidity());

        sleep_ms(2000);
    }
    return 0;

}
