#include "FreeRTOS.h"
#include "MPU9250_RegisterMap.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "i2c_mpu9250.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>

// Define Macros here
#define LED_PIN PICO_DEFAULT_LED_PIN
// Defines for I2C
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15

// Define function prototyoes here
void vLEDTask(void *pvParameters);
void vReadIMUTask(void *pvParameters);

MPU9250 MPU(I2C_PORT, I2C_SDA, I2C_SCL, ACC_FS_4G, GYRO_FS_1000DPS);

// Main function implementation
int main()
{

  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  BaseType_t xReturned;

  TaskHandle_t xLEDHandle = NULL;
  TaskHandle_t xReadIMUHandle = NULL;

  /* Create the task, storing the handle. */
  xReturned = xTaskCreate(vLEDTask,       /* Function that implements the task. */
                          "RED LED Task", /* Text name for the task. */
                          512,            /* Stack size in words, not bytes. */
                          NULL,           /* Parameter passed into the task. */
                          tskIDLE_PRIORITY +
                              1, /* Priority at which the task is created. */
                          &xLEDHandle);
  /* Create the task, storing the handle. */
  xReturned = xTaskCreate(vReadIMUTask,    /* Function that implements the task. */
                          "Read IMU Task", /* Text name for the task. */
                          512,             /* Stack size in words, not bytes. */
                          NULL,            /* Parameter passed into the task. */
                          tskIDLE_PRIORITY +
                              2, /* Priority at which the task is created. */
                          &xReadIMUHandle);

  vTaskStartScheduler();

  while (1)
  {
    // configASSERT(0);    /* We should never get here */
  }
}

/*********************************************************
 *Task Implementation
 **********************************************************
 */

// Red LED task implementation /functionality
void vLEDTask(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    gpio_put(LED_PIN, 1);
    vTaskDelay(500);
    gpio_put(LED_PIN, 0);
    vTaskDelay(500);
  }
}

void vReadIMUTask(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    vTaskDelay(500);
    float data = MPU.get_gyro_dps(GYRO_Z);
    printf("%f\n", data);
    vTaskDelay(500);
  }
}
