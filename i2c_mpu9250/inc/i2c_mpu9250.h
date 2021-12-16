#ifndef i2c_mpu9250_h
#define i2c_mpu9250_h

#include "../inc/MPU9250_RegisterMap.h"
#include "pico/stdlib.h"
#include "stdbool.h"
#include "hardware/i2c.h"

typedef enum accel_tag
{
    ACCEL_X = 1,
    ACCEL_Y = 2,
    ACCEL_Z = 3,
} ACCELERATION_AXIS;

typedef enum gyro_tag
{
    GYRO_X = 1,
    GYRO_Y = 2,
    GYRO_Z = 3,
} GYROSCOPE_AXIS;

class MPU9250
{
public:
    //variables
    float ax, ay, az;
    float gx, gy, gz;
    float mag;
    float temp;

    MPU9250_GYRO_FULL_SCALE current_gyro_scale;
    MPU9250_ACCEL_FULL_SCALE current_acc_scale;

    //functions
    MPU9250(i2c_inst_t *port, uint sda, uint scl, MPU9250_ACCEL_FULL_SCALE, MPU9250_GYRO_FULL_SCALE);
    ~MPU9250()
    {
        i2c_deinit(this->PORT);
    }

    bool MPU_config(MPU9250_ACCEL_FULL_SCALE, MPU9250_GYRO_FULL_SCALE);
    bool get_raw_data();
    bool get_all_readings();

    float get_acceleration(ACCELERATION_AXIS);
    float get_temperature();
    float get_gyro(GYROSCOPE_AXIS);
    float get_magnetometer();

private:
    //variables
    i2c_inst_t *PORT;
    uint SDA;
    uint SCL;
    uint8_t raw_data_buffer[MPU_RAW_REG_COUNT];

    //functions
    bool read_register(uint8_t src, uint8_t *dst);
};

#endif
