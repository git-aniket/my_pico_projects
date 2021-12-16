#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../inc/i2c_cjmcu811.h"

CJMCU811::CJMCU811(i2c_inst_t *port, uint sda, uint scl)
{
    this->PORT = port;
    this->SDA = sda;
    this->SCL = scl;

    /* I2C Initialisation. Using it at 400Khz.*/
    i2c_init(this->PORT, 400 * 1000);

    gpio_set_function(this->SDA, GPIO_FUNC_I2C);
    gpio_set_function(this->SCL, GPIO_FUNC_I2C);

    /*Setup pull-up registers for the i2c pins*/
    gpio_pull_up(this->SDA);
    gpio_pull_up(this->SCL);

    if (CJMCU811_init())
        printf("CJMCU811 Configured.\n");
    else
        printf("CJMCU811 config failure.\n");
}

bool CJMCU811::CHMCU811_init()
{
    bool var = 0;
    return var;
}

bool CJMCU811::data_available()
{
    bool var = 0;
    return var;
}

pico_float CJMCU811::get_TVOC()
{
    pico_float var = 0;
    return var;
}

pico_float CJMCU811::get_eCO2()
{
    pico_float var = 0;
    return var;
}