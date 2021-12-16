/**
 * @file i2c_cjmcu811.h
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief Header file for CJMCU811 gas sensor to be used with 
 * raspberry pico
 * @version 0.1
 * @date 2021-12-01
 * 
 * @copyright Copyright (c) 2021 Aniket Mazumder
 * the register descriptions have been taken from
 * https://github.com/adafruit/Adafruit_CCS811/blob/master/Adafruit_CCS811.h
 * 
 */
#ifndef i2c_cjmcu811_h
#define i2c_cjmcu811_h
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define CCS811_ADDRESS (0x5A)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum
{
    CCS811_STATUS = 0x00,
    CCS811_MEAS_MODE = 0x01,
    CCS811_ALG_RESULT_DATA = 0x02,
    CCS811_RAW_DATA = 0x03,
    CCS811_ENV_DATA = 0x05,
    CCS811_NTC = 0x06,
    CCS811_THRESHOLDS = 0x10,
    CCS811_BASELINE = 0x11,
    CCS811_HW_ID = 0x20,
    CCS811_HW_VERSION = 0x21,
    CCS811_FW_BOOT_VERSION = 0x23,
    CCS811_FW_APP_VERSION = 0x24,
    CCS811_ERROR_ID = 0xE0,
    CCS811_SW_RESET = 0xFF,
};

// bootloader registers
enum
{
    CCS811_BOOTLOADER_APP_ERASE = 0xF1,
    CCS811_BOOTLOADER_APP_DATA = 0xF2,
    CCS811_BOOTLOADER_APP_VERIFY = 0xF3,
    CCS811_BOOTLOADER_APP_START = 0xF4
};

enum
{
    CCS811_DRIVE_MODE_IDLE = 0x00,
    CCS811_DRIVE_MODE_1SEC = 0x01,
    CCS811_DRIVE_MODE_10SEC = 0x02,
    CCS811_DRIVE_MODE_60SEC = 0x03,
    CCS811_DRIVE_MODE_250MS = 0x04,
};

/*=========================================================================*/

#define CCS811_HW_ID_CODE 0x81

#define CCS811_REF_RESISTOR 100000

class CJMCU811
{
    CJMCU811(i2c_inst *port, uint sda, uint scl);
    ~CJMCU811();
    bool CJMCU811_init();
    bool data_available();
    bool set_thresholds();

    void get_raw_sensor_reading();
    pico_float get_TVOC();
    pico_float get_eCO2();
};

#endif