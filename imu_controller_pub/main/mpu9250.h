#pragma once 
#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#define MPU9250_ADDR 0x68 

//i2c settings
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

//functions
esp_err_t mpu9250_i2c_init(void);
esp_err_t mpu9250_write_reg(uint8_t reg, uint8_t value);
esp_err_t mpu9250_read_bytes(uint8_t reg, uint8_t *buf, size_t len);
esp_err_t mpu9250_init(void);
esp_err_t mpu9250_read_imu_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *temp);