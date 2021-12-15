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
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    if (cmd_handle == NULL) {
        return ESP_FAIL;
    }

    /** I2C start bit added to command link **/
    err = i2c_master_start(cmd_handle);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Slave address and write bit enable added to command link **/
    err = i2c_master_write_byte(cmd_handle, (BNO055_ADDRESS_DEFAULT) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }
    /** I2C Slave register to write to added to the command link **/
    err = i2c_master_write_byte(cmd_handle, register, true);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Command to write to the slave register **/
    err = i2c_master_write_byte(cmd_handle, data, true);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C stop signal added to the command link **/
    err = i2c_master_stop(cmd_handle);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Attempt to send the command link a set number of times **/
    for (int attempt = 1; attempt <= I2C_CONNECTION_TO_TRY; attempt++) {
        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (err == ESP_OK) {
            break;
        }
        else if ((err != ESP_OK) && (attempt < I2C_CONNECTION_TO_TRY)) {
            continue;
        }
        else {
            i2c_cmd_link_delete(cmd_handle);
            return err;
        }
    }
    i2c_cmd_link_delete(cmd_handle);
    return err;
}


