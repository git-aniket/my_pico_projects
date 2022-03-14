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

    //initialize sensor
    MPU9250 MPU(i2c0,I2C_SDA, I2C_SCL,ACC_FS_2G,GYRO_FS_1000DPS);
    printf("Legend\n");
    printf("AccelX AccelY AccelZ GyroX GyroY GyroZ\n");

    while (1)
    {
	//printf("%f,%f,%f,%f,%f,%f\n",MPU.get_accel(ACCEL_X),MPU.get_accel(ACCEL_Y),MPU.get_accel(ACCEL_Z), MPU.get_gyro(GYRO_X),MPU.get_gyro(GYRO_Y),MPU.get_gyro(GYRO_Z));
	//printf("%f,%f,%f,%f,%f,%f\n",MPU.get_accel_g(ACCEL_X),MPU.get_accel_g(ACCEL_Y),MPU.get_accel_g(ACCEL_Z),MPU.get_gyro_dps(GYRO_X),MPU.get_gyro_dps(GYRO_Y),MPU.get_gyro_dps(GYRO_Z));
	printf("%f\n",MPU.get_gyro_dps(GYRO_Z));
	sleep_ms(10);
    }

    /*Should not come to this point during normal operation*/
    return 0;
}
