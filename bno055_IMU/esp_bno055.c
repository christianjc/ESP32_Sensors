#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_bno055.h"

static const char* TAG = "i2c-bno055-IMU";


/**
 * @brief Wrietes one byte (8 bits) of data over I2C to a bno055 register
*/
esp_err_t write8(bno055_reg_t register, byte data) {
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
    err = i2c_master_write_byte(cmd_handle, (uint8_t)((BNO055_ADDRESS_DEFAULT << 1) | I2C_MASTER_WRITE), I2C_MASTER_ACK);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Slave register to write to added to the command link **/
    err = i2c_master_write_byte(cmd_handle, (uint8_t)register, I2C_MASTER_ACK);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Command to write to the slave register **/
    err = i2c_master_write_byte(cmd_handle, (uint8_t)data, I2C_MASTER_ACK);
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

uint8_t read8(bno055_reg_t register) {
    esp_err_t err = ESP_OK;
    uint8_t buffer[1];
    err = write_then_read(register, buffer, 1);
    if (err != ESP_OK) {
        return 0x00;
    }
    return (uint8_t)buffer[0];
}

esp_err_t readLen(bno055_reg_t, uint8_t* buffer, size_t len) {

}

esp_err_t write_then_read(bno055_reg_t register, uint8_t* buffer, size_t len) {
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

    /** I2C Slave address and WRITE bit enable added to command link **/
    err = i2c_master_write_byte(cmd_handle, (uint8_t)((BNO055_ADDRESS_DEFAULT << 1) | I2C_MASTER_WRITE), I2C_MASTER_ACK);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Slave register to read from added to the command link **/
    err = i2c_master_write_byte(cmd_handle, (uint8_t)register, true);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C restart bit added to command link **/
    err = i2c_master_start(cmd_handle);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Slave address and READ bit enable added to command link **/
    err = i2c_master_write_byte(cmd_handle, (uint8_t)((BNO055_ADDRESS_DEFAULT << 1) | I2C_MASTER_READ), I2C_MASTER_ACK);
    if (err != ESP_OK) {
        i2c_cmd_link_delete(cmd_handle);
        return err;
    }

    /** I2C Read command to be added to the command link **/
    if (len == 1) {
        /** If there is only one byte to read, send the byte with NACK to end transmission **/
        err = i2c_master_read_byte(cmd_handle, buffer, I2C_MASTER_LAST_NACK);
        if (err != ESP_OK) {
            i2c_cmd_link_delete(cmd_handle);
            return err;
        }
    }
    else {
        /** If there is more then one byte read all except the last one **/
        err = i2c_master_read(cmd_handle, buffer, len - 1, I2C_MASTER_ACK);
        if (err != ESP_OK) {
            i2c_cmd_link_delete(cmd_handle);
            return err;
        }
        /** Read the last byte and send a NACK to stop transmission from slave **/
        err = i2c_master_read_byte(cmd_handle, (buffer + (len - 1)), I2C_MASTER_LAST_NACK);
        if (err != ESP_OK) {
            i2c_cmd_link_delete(cmd_handle);
            return err;
        }
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

