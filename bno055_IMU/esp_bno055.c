#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_bno055.h"

static const char* TAG = "i2c-bno055-IMU";


/**
 * @brief Wrietes one byte (8 bits) of data over I2C to a bno055 register
*/
esp_err_t write8((uint8_t)register, (uint8_t)data) {
    esp_err_t* err = ESP_OK;

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    if (cmd_handle == NULL) {
        return ESP_FAIL;
    }

    /** I2C start bit added to command link **/
    *err = i2c_master_start(cmd_handle);
    check_cmd_link_error(cmd_handle, err);

    /** I2C Slave address and write bit enable added to command link **/
    *err = i2c_master_write_byte(cmd_handle, (BNO055_ADDRESS_DEFAULT) | I2C_MASTER_WRITE, true);
    check_cmd_link_error(cmd_handle, err);

    /** I2C Slave register to write to added to the command link **/
    *err = i2c_master_write_byte(cmd_handle, register, true);
    check_cmd_link_error(cmd_handle, err);

    /** I2C Command to write to the slave register **/
    *err = i2c_master_write_byte(cmd_handle, data, true);
    check_cmd_link_error(cmd_handle, err);

    /** I2C stop signal added to the command link **/
    *err = i2c_master_stop(cmd_handle);
    check_cmd_link_error(cmd_handle, err);

    *err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, ticks_to_wait);
    check_cmd_link_error(cmd_handle, err);
}


esp_err_t check_cmd_link_error(i2c_cmd_handle_t cmd_handle, esp_err_t* err) {
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }
}