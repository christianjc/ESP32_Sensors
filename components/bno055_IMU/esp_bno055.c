// i2c.c i2c.h i2c_hal.c i2c_hal.h i2c_ll.h reference documents

#include <stdio.h>
#include "esp_log.h"

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "esp_bno055.h"


#define INITIAL_MODE_TOUT (36000)






//esp_err_t write_then_read(bno055_reg_t register, uint8_t* buffer, size_t len);
static esp_err_t i2c_master_init(void);

static const char* TAG = "i2c-bno055-IMU";

esp_err_t bno055_begin() {
    /** Inintialize the master configuration **/
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c Master init error: %d", err);
        i2c_driver_delete(I2C_NUM_0);
        return err;
    }

    /** Set the timeout for the communication bus **/
    err = i2c_set_timeout((i2c_port_t)I2C_NUM_0, (int)INITIAL_MODE_TOUT);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGD(TAG, "This is the timeout value: %d", INITIAL_MODE_TOUT);

    /* Verify that we have the correct device */
    uint8_t bno055_id = read8(BNO055_CHIP_ID_ADDR);
    if (bno055_id != BNO055_ID) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // wait for chip to boot up
        bno055_id = read8(BNO055_CHIP_ID_ADDR);
        if (bno055_id != BNO055_ID) {
            ESP_LOGE(TAG, "Could not read chip ID: %x", bno055_id);
            return ESP_FAIL;
        }
    }
    ESP_LOGD(TAG, "CHIP ID: %x", bno055_id);

    /** Set to config mode **/
    // setmode();

    /* Reset */
    write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
    vTaskDelay(80);
    while (bno055_id != read8(BNO055_CHIP_ID_ADDR)) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    /* Set to normal power mode */


    return err;
}





/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    esp_err_t err = ESP_OK;

    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        return err;
    }

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    err = i2c_param_config(I2C_NUM_0, &config);

    return err;
}

/**
 * @brief Read one byte from the I2C slave register
*/
uint8_t read8(bno055_reg_t reg) {
    esp_err_t err = ESP_OK;
    uint8_t read_buffer[1];
    uint8_t read_reg[1];
    read_reg[0] = (uint8_t)reg;
    ESP_LOGD(TAG, "buffer address to pass: %p \n", (void*)read_buffer);
    ESP_LOGD(TAG, "buffer content %x \n", read_buffer[0]);
    err = i2c_master_write_read_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        read_reg, (size_t)1, read_buffer, (size_t)1, (TickType_t)1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error in i2c_master_write_read_device: %x", err);
        return 0x00;
    }
    return (uint8_t)read_buffer[0];
}


/**
 * @brief Wrietes one byte (8 bits) of data over I2C to a bno055 register
*/
esp_err_t write8(bno055_reg_t reg, uint8_t data) {
    esp_err_t err = ESP_OK;
    uint8_t write_buffer[2];
    write_buffer[0] = (uint8_t)reg;
    write_buffer[1] = data;
    err = i2c_master_write_to_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        write_buffer, (size_t)2, (TickType_t)1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error in i2c_master_write_to_device: %x", err);
        return err;
    }
    return err;
}


// /**
//  * @brief Wrietes one byte (8 bits) of data over I2C to a bno055 register
// */
// esp_err_t write8(bno055_reg_t reg, uint8_t data) {
//     esp_err_t err = ESP_OK;

//     i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
//     if (cmd_handle == NULL) {
//         return ESP_FAIL;
//     }

//     /** I2C start bit added to command link **/
//     err = i2c_master_start(cmd_handle);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Slave address and write bit enable added to command link **/
//     err = i2c_master_write_byte(cmd_handle, (uint8_t)((BNO055_ADDRESS_DEFAULT << 1) | I2C_MASTER_WRITE), I2C_MASTER_ACK);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Slave register to write to added to the command link **/
//     err = i2c_master_write_byte(cmd_handle, (uint8_t)reg, I2C_MASTER_ACK);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Command to write to the slave register **/
//     err = i2c_master_write_byte(cmd_handle, (uint8_t)data, I2C_MASTER_ACK);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C stop signal added to the command link **/
//     err = i2c_master_stop(cmd_handle);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Attempt to send the command link a set number of times **/
//     for (int attempt = 1; attempt <= I2C_CONNECTION_TO_TRY; attempt++) {
//         err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
//         if (err == ESP_OK) {
//             break;
//         }
//         else if ((err != ESP_OK) && (attempt < I2C_CONNECTION_TO_TRY)) {
//             continue;
//         }
//         else {
//             i2c_cmd_link_delete(cmd_handle);
//             return err;
//         }
//     }
//     i2c_cmd_link_delete(cmd_handle);
//     return err;
// }



// /**
//  * @brief Read bytes from the I2C slave register
// */
// esp_err_t readLen(bno055_reg_t reg, uint8_t* buffer, size_t len) {
//     return write_then_read(reg, buffer, len);
// }

// /**
//  * @brief Is a helper function to read data from I2C slave register
// */
// esp_err_t write_then_read(bno055_reg_t reg, uint8_t* buffer, size_t len) {
//     esp_err_t err = ESP_OK;

//     i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
//     if (cmd_handle == NULL) {
//         return ESP_FAIL;
//     }

//     printf("buffer address recieved: %p \n", (void*)buffer);
//     printf("buffer content in function %d \n", buffer[0]);

//     /** I2C start bit added to command link **/
//     err = i2c_master_start(cmd_handle);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Slave address and WRITE bit enable added to command link **/
//     err = i2c_master_write_byte(cmd_handle, (uint8_t)((BNO055_ADDRESS_DEFAULT << 1) | I2C_MASTER_WRITE), I2C_MASTER_ACK);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Slave register to read from added to the command link **/
//     err = i2c_master_write_byte(cmd_handle, (uint8_t)reg, true);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C restart bit added to command link **/
//     err = i2c_master_start(cmd_handle);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Slave address and READ bit enable added to command link **/
//     err = i2c_master_write_byte(cmd_handle, (uint8_t)((BNO055_ADDRESS_DEFAULT << 1) | I2C_MASTER_READ), I2C_MASTER_ACK);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Read command to be added to the command link **/
//     if (len == 1) {
//         /** If there is only one byte to read, send the byte with NACK to end transmission **/
//         err = i2c_master_read_byte(cmd_handle, buffer, I2C_MASTER_LAST_NACK);
//         if (err != ESP_OK) {
//             i2c_cmd_link_delete(cmd_handle);
//             return err;
//         }

//     }
//     else {
//         /** If there is more then one byte read all except the last one **/
//         err = i2c_master_read(cmd_handle, buffer, len - 1, I2C_MASTER_ACK);
//         if (err != ESP_OK) {
//             i2c_cmd_link_delete(cmd_handle);
//             return err;
//         }
//         /** Read the last byte and send a NACK to stop transmission from slave **/
//         err = i2c_master_read_byte(cmd_handle, (buffer + (len - 1)), I2C_MASTER_LAST_NACK);
//         if (err != ESP_OK) {
//             i2c_cmd_link_delete(cmd_handle);
//             return err;
//         }
//     }

//     /** I2C stop signal added to the command link **/
//     err = i2c_master_stop(cmd_handle);
//     if (err != ESP_OK) {
//         i2c_cmd_link_delete(cmd_handle);
//         return err;
//     }

//     /** I2C Attempt to send the command link a set number of times **/
//     for (int attempt = 1; attempt <= I2C_CONNECTION_TO_TRY; attempt++) {
//         err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

//         if (err == ESP_OK) {
//             printf("buffer address after read: %p \n", (void*)buffer);
//             printf("buffer content in function after read %d \n", buffer[0]);
//             break;
//         }
//         else if ((err != ESP_OK) && (attempt < I2C_CONNECTION_TO_TRY)) {
//             continue;
//         }
//         else {
//             const char* str = esp_err_to_name(err);
//             printf("Error: %s \n", str);
//             i2c_cmd_link_delete(cmd_handle);
//             return err;
//         }
//     }
//     i2c_cmd_link_delete(cmd_handle);
//     return err;
// }

// esp_err_t bno055_begin() {
//     /* Verify I2C slave bno055_id number */
//     esp_err_t err = i2c_master_init();
//     if (err != ESP_OK) {
//         ESP_LOGW(TAG, "i2c Master init error: %d", err);
//     }
//     uint8_t bno055_id = read8(BNO055_CHIP_ID_ADDR);
//     while (bno055_id != BNO055_ID) {
//         ESP_LOGW(TAG, "bno055 ID: %d", bno055_id);
//         bno055_id = read8(BNO055_CHIP_ID_ADDR);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }


//     // if (bno055_id != BNO055_ID) {
//     //     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     //     bno055_id = read8(BNO055_CHIP_ID_ADDR);
//     //     if (bno055_id != BNO055_ID) {
//     //         ESP_LOGW(TAG, "bno055 ID: %d", bno055_id);
//     //         return ESP_FAIL;
//     //     }
//     // }

//     // ESP_LOGW(TAG, "bno055 ID: %d", bno055_id);
//     // /** Set configuratio mode to make changes **/
//     // setMode(OPERATION_MODE_CONFIG);

//     // /** Test Reset Trigger **/
//     // write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
//     // /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
//     // vTaskDelay(30 / portTICK_PERIOD_MS);
//     // while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
//     //     vTaskDelay(10 / portTICK_PERIOD_MS);
//     // }
//     // vTaskDelay(50);

//     // /* Set to normal power mode */
//     // write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
//     // vTaskDelay(10);

//     // write8(BNO055_PAGE_ID_ADDR, 0);

//     /* Set the output units */
//     /*
//     uint8_t unitsel = (0 << 7) | // Orientation = Android
//                       (0 << 4) | // Temperature = Celsius
//                       (0 << 2) | // Euler = Degrees
//                       (1 << 1) | // Gyro = Rads
//                       (0 << 0);  // Accelerometer = m/s^2
//     write8(BNO055_UNIT_SEL_ADDR, unitsel);
//     */

//     /* Configure axis mapping (see section 3.4) */
//     /*
//     write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
//     delay(10);
//     write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
//     delay(10);
//     */

//     // write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
//     // vTaskDelay(10);
//     // /* Set the requested operating mode (see section 3.3) */
//     // setMode(mode);
//     // vTaskDelay(20);


//     return true;
// }