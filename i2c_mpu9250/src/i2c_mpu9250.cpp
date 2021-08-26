#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include "hardware/i2c.h"
#include "MPU9250_RegisterMap.h"
#include "stdint.h"

enum accel_component{
    ACCEL_X=0,
    ACCEL_Y=1,
    ACCEL_Z=2,
};

class MPU9250
{
    public:
	//functions
	MPU9250(i2c_inst_t* port, uint sda, uint scl);
	~MPU9250()
	{
	    i2c_deinit(this->PORT);
	}

	void read_raw_data(uint32_t* destination);
	float read_acceleration(accel_component);
	float read_temperature();
	
    private:
	//variables
	i2c_inst_t* PORT;
	uint SDA;
	uint SCL;
};

MPU9250::MPU9250(i2c_inst_t* port, uint sda, uint scl)
{
    this->PORT=port;
    this->SDA=sda;
    this->SCL=scl;

    /* I2C Initialisation. Using it at 400Khz.*/
    i2c_init(this->PORT, 400*1000);
    
    gpio_set_function(this->SDA, GPIO_FUNC_I2C);
    gpio_set_function(this->SCL, GPIO_FUNC_I2C);

    /*Setup pull-up registers for the pins*/
    gpio_pull_up(this->SDA);
    gpio_pull_up(this->SCL);
}



