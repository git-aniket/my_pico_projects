#include <stdio.h>
#include "pico/stdio/driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU9250_RegisterMap.h"
#include "i2c_mpu9250.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

volatile bool timer_fired = false;

bool repeating_timer_callback(struct repeating_timer *t)
{
    timer_fired = true;
    return true;
}

/**
 * @brief
 * @return int
 */
int main()
{
    // timer setup
    struct repeating_timer timer;
    add_repeating_timer_ms(10, repeating_timer_callback, NULL, &timer);

    sleep_ms(100);
    stdio_init_all();
    sleep_ms(100);

    // initialize sensor
    MPU9250 MPU(i2c0, I2C_SDA, I2C_SCL, ACC_FS_4G, GYRO_FS_1000DPS);
    printf("AccelX AccelY AccelZ GyroX GyroY GyroZ\n");

    while (1)
    {
        tight_loop_contents();
        if (timer_fired == true)
        {
            timer_fired = false;
            printf("%f,%f,%f,%f,%f,%f\n",
                   MPU.get_accel_g(ACCEL_X),
                   MPU.get_accel_g(ACCEL_Y),
                   MPU.get_accel_g(ACCEL_Z),
                   MPU.get_gyro_dps(GYRO_X),
                   MPU.get_gyro_dps(GYRO_Y),
                   MPU.get_gyro_dps(GYRO_Z));
        //printf("%f,%f,%f\n", MPU.get_accel_g(ACCEL_X), MPU.get_accel_g(ACCEL_Y), MPU.get_accel_g(ACCEL_Z));
        //printf("%f,%f,%f\n", MPU.get_gyro_dps(GYRO_X), MPU.get_gyro_dps(GYRO_Y), MPU.get_gyro_dps(GYRO_Z));
        }
    }

    /*Should not come to this point during normal operation*/
    return 0;
}
