#include <stdio.h>
#include "pico/stdio/driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU9250_RegisterMap.h"
#include "i2c_mpu9250.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

/**
 * @brief 
 * @return int 
 */
int main()
{
    sleep_ms(100);
    stdio_init_all();
    sleep_ms(100);

    //initilize sensor
    MPU9250 MPU(i2c0,
                I2C_SDA, I2C_SCL,
                ACC_FS_TWO_G,
                GYRO_FS_FIVE_HUNDRED_DPS);

    while (1)
    {
        //MPU.get_all_readings();
        printf("Accel: %f\n", MPU.get_acceleration(ACCEL_Z) / 16384.0);
        //printf("Gyro: %f\n",MPU.get_gyro(GYRO_X));
        //sleep_ms(2000);
    }

    /*Should not come to this point during normal operation*/
    return 0;
}
