/**
 * @file main.cpp
 * @author Aniket Mazumder mazumder_aniket@hotmail.com 
 * @brief  Example file on how to use the BMP280 library
 * @version 0.1
 * @date 2021-08-27
 * 
 * @copyright Copyright (c) 2021 Aniket Mazumder
 * 
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "BMP280.h"

// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#include "../inc/BMP280.h"
#define I2C_PORT0 i2c0
#define I2C0_SDA 8
#define I2C0_SCL 9

int main()
{
    stdio_init_all();

    //Initialize sensor
    BMP280 sensor(I2C_PORT0,400*1000,I2C0_SDA,I2C0_SCL);

    while(1)
    {
	    printf("Temperature: %f\n",sensor.read_temperature());
	    printf("Pressure: %f\n",sensor.read_pressure());
        sleep_ms(1000);
    }
    return 0;
}
