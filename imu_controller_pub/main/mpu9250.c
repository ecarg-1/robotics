#include "mpu9250.h"
#include "esp_log.h"

//created from referencing:
//https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/i2c.html#overview
//https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf 
//https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf

static const char *TAG = "MPU9250"; //identifier for logging output

esp_err_t mpu9250_i2c_init(void){
    //initialize the esp as master, mpu always a slave
    i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf)); //checks the configuration

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

//write to mpu
esp_err_t mpu9250_write_reg(uint8_t reg, uint8_t value){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //create and init i2c command link
    i2c_master_start(cmd); 
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true); //first byte [7 bit slave addr][R/W bit]
    i2c_master_write_byte(cmd, reg, true); //mpu register addr
    i2c_master_write_byte(cmd, value, true); //value for register
    i2c_master_stop(cmd); //stop bit
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS); //send command link on i2c bus
    i2c_cmd_link_delete(cmd); //delete link
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to write register: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "Successfully wrote to register.");
    return ret; //return either ESP_OK or error if it didn't work
}

//read from mpu
esp_err_t mpu9250_read_bytes(uint8_t reg, uint8_t *buf, size_t len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //first write 
    i2c_master_start(cmd); 
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    //start again
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true); //indicate it wants to read
    if (len > 1) i2c_master_read(cmd, buf, len-1, I2C_MASTER_ACK); //ack every byte
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK); //except nack the last one
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to read bytes: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "Successfully read bytes.");
    
    return ret;
}

esp_err_t mpu9250_init(void){
    esp_err_t ret;
    ret = mpu9250_write_reg(0x6B, 0x00); //set everything in PWR_MGMT_1 reg to 0 (including sleep) to wake it
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWR_MGT_1 (0x6B): %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); //delay for sensor stabilization
    ret = mpu9250_write_reg(0x1C, 0x00); //ACCEL_CONFIG to +/- 2g
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ACCEL_CONFIG (0x1C): %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mpu9250_write_reg(0x1B, 0x00); //GYRO_CONFIG to + 250dps
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GYRO_CONFIG (0x1B): %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "MPU9250 initialized!");
    return ESP_OK;
}

esp_err_t mpu9250_read_imu_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *temp){
    uint8_t data[14]; //14 bytes (accel and gyro x,y,z and temp each have 2 registers=14)
    esp_err_t ret = mpu9250_read_bytes(0x3B, data, 14); //goes through each of the 14 registers collecting the data
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to read IMU: %s", esp_err_to_name(ret));
        return ret;
    }
    *ax = (data[0] << 8) | data[1]; //high byte first, then low bite
    *ay = (data[2] << 8) | data[3];
    *az = (data[4] << 8) | data[5];
    *temp = (data[6] << 8) | data[7];
    *gx = (data[8] << 8) | data[9];
    *gy = (data[10] << 8) | data[11];
    *gz = (data[12] << 8) | data[13];
    return ESP_OK;
}