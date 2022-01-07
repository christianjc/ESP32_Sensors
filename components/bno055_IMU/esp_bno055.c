// i2c.c i2c.h i2c_hal.c i2c_hal.h i2c_ll.h reference documents

#include <stdio.h>
#include <string.h>
#include "esp_log.h"

#include "errno.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "esp_bno055.h"


#define INITIAL_MASTER_TOUT         (36000)     // This is number of clock cycles from the main clock. Here is 80,000,000 Hz. Bno055 needs at leas 500us.
#define TIME_TO_WAIT_READ_WRITE     ((TickType_t)1000)
#define TIME_TO_WAIT_OP_MODE        (30)

#define STORAGE_NAMESPACE "storage"

static const char* TAG = "i2c-bno055-IMU";


//esp_err_t write_then_read(bno055_reg_t register, uint8_t* buffer, size_t len);
static esp_err_t i2c_master_init(void);
uint8_t read8(bno055_reg_t);
/**
 * @brief This function writes one byte of data to the given register
 *
 * @param register This is the register address to write the data to
 *
 * @param data This is the data to be written in the register
*/
esp_err_t write8(bno055_reg_t reg, uint8_t data);
esp_err_t set_sensor_offset(uint8_t* calib_data);



esp_err_t bno055_begin() {
    /** Inintialize the master configuration **/
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c Master init error: %d", err);
        i2c_driver_delete(I2C_NUM_0);
        return err;
    }

    /** Set the timeout for the communication bus **/
    err = i2c_set_timeout((i2c_port_t)I2C_NUM_0, (int)INITIAL_MASTER_TOUT);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGD(TAG, "This is the timeout value: %d", INITIAL_MASTER_TOUT);

    /** Verify that we have the correct device **/
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

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bno055_reset();
    if (err != ESP_OK) return err;

    /** Wait for the chip to restart **/
    vTaskDelay(80);
    while (bno055_id != read8(BNO055_CHIP_ID_ADDR)) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // /* Set to normal power mode */
    // err = set_powermode(POWER_MODE_NORMAL);

    /** Calibrate sensor or use calibration profile **/
    err = calibrate_sensor_from_saved_profile();
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = calibrate_sensor();
            if (err != ESP_OK) return err;
        }
        else {
            return err;
        }
    }

    return err;
}

/**
 * @brief   Resets the bno055 chip
*/
esp_err_t bno055_reset(void) {
    esp_err_t err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        return err;
    }
    return write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
}


esp_err_t calibrate_sensor(void) {

    printf("SENSOR: Please start calibration\n");

    esp_err_t err = set_opmode(OPERATION_MODE_IMUPLUS);
    if (err != ESP_OK) return  err;
    // wait untill calibration is completed
    uint8_t counter = 0;
    while (!isFullyCalibrated()) {
        if (counter > 100) {
            return ESP_FAIL;
        }
        printf("sensor is not calibrated: counter %d\n", counter);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        counter++;
    }
    printf("SENSOR: sensor is now calibrated \n");
    printf("Saving calibrated profile to non-volatile memory...\n");
    uint8_t calib_data[22];
    err = get_sensor_offsets(calib_data);
    if (err != ESP_OK) return err;

    err = save_calib_profile_to_nvs(calib_data);
    return err;

}

esp_err_t calibrate_sensor_from_saved_profile(void) {
    uint8_t calib_data[NUM_BNO055_OFFSET_REGISTERS];
    memset(calib_data, 0, 22);
    esp_err_t err = get_calib_profile_from_nvs(calib_data);
    switch (err) {
    case ESP_OK:
        err = set_sensor_offset(calib_data);
        printf("check if isFullyCalibrated true after updatind sensor ofsets.....\nisFullyCalibrated:  ");
        printf((isFullyCalibrated()) ? "True!\n" : "False\n");
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        printf("No profile saved yet!\n");
        break;
    default:
        printf("Error reading calibration profile\n");
    }
    return err;
}

/**
 * @brief Gets the calibration profile from the non-volatile storate (nvs)
 *
 * @param calib_data
 *                  Pointer to a uint8_t array of size 22
 * @return  ESP_OK
 *          ESP_FAIL
 *          ESP_ERR_INVALID_ARG
*/
esp_err_t get_calib_profile_from_nvs(uint8_t* calib_data) {
    // // parameter check
    // uint8_t size_needed[NUM_BNO055_OFFSET_REGISTERS];
    // if (sizeof(calib_data) != sizeof(size_needed)) {
    //     printf("here is the error**********\n ");
    //     printf("calib_data size: %d  needed: %d\n", sizeof(calib_data), sizeof(size_needed));
    //     //return ESP_ERR_INVALID_ARG;
    // }


    // Open
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read memory space for blob if it exists
    size_t size = (size_t)(NUM_BNO055_OFFSET_REGISTERS);
    memset(calib_data, 0, NUM_BNO055_OFFSET_REGISTERS);
    err = nvs_get_blob(my_handle, "calib_data", calib_data, &size);

    nvs_close(my_handle);
    printf("Error (%s) reading!\n", esp_err_to_name(err));
    return err;
}

esp_err_t save_calib_profile_to_nvs(uint8_t* calib_data) {

    // Open
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read memory space for blob if it exists
    size_t size = (size_t)(NUM_BNO055_OFFSET_REGISTERS);
    memset(calib_data, 0, NUM_BNO055_OFFSET_REGISTERS);
    err = nvs_set_blob(my_handle, "calib_data", calib_data, size);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return err;
    }

    err = nvs_commit(my_handle);
    nvs_close(my_handle);
    return err;
}


esp_err_t print_calib_profile_from_nvs(void) {
    esp_err_t err = ESP_OK;
    uint8_t calibData[NUM_BNO055_OFFSET_REGISTERS];
    memset(calibData, 0, NUM_BNO055_OFFSET_REGISTERS);
    err = get_calib_profile_from_nvs(calibData);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    switch (err) {
    case ESP_OK:
        printf("**** Accelerometer Offsets ****\n");
        printf("Accel offset X: %d\n", ((calibData[1] << 8) | (calibData[0])));
        printf("Accel offset Y: %d\n", ((calibData[3] << 8) | (calibData[2])));
        printf("Accel offset Z: %d\n", ((calibData[5] << 8) | (calibData[4])));
        printf("**** Magnetometer Offsets ****\n");
        printf("Mag offset X: %d\n", ((calibData[7] << 8) | (calibData[6])));
        printf("Mag offset Y: %d\n", ((calibData[9] << 8) | (calibData[8])));
        printf("Mag offset Z: %d\n", ((calibData[11] << 8) | (calibData[10])));
        printf("**** Gyroscope Offsets ****\n");
        printf("Gyr offset X: %d\n", ((calibData[13] << 8) | (calibData[12])));
        printf("Gyr offset Y: %d\n", ((calibData[15] << 8) | (calibData[14])));
        printf("Gyr offset Z: %d\n", ((calibData[17] << 8) | (calibData[16])));
        printf("**** Accelerometer Radius ****\n");
        printf("Accel radius: %d\n", ((calibData[19] << 8) | (calibData[18])));
        printf("**** Magnetometer Radius ****\n");
        printf("Mag radius: %d\n", ((calibData[21] << 8) | (calibData[20])));
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        printf("Profile not initialized yet!!\n");
        break;
    default:
        printf("Error reading nvs storage\n");
    }

    return err;
}

/**
 * @brief i2c communication for master initialization and configuration
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
    err = i2c_master_write_read_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        read_reg, (size_t)1, read_buffer, (size_t)1, TIME_TO_WAIT_READ_WRITE);
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
    write_buffer[1] = (uint8_t)data;
    err = i2c_master_write_to_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        write_buffer, (size_t)2, TIME_TO_WAIT_READ_WRITE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error in i2c_master_write_to_device: %x", err);
        return err;
    }
    return err;
}

/**
 * @brief Sets bno055 operation mode
*/
esp_err_t set_opmode(bno055_opmode_t op_mode) {
    esp_err_t err = write8(BNO055_OPR_MODE_ADDR, op_mode);
    return err;
}

/**
 * @brief Gets the current bno055 operation mode
*/
bno055_opmode_t get_opmode(void) {
    return (bno055_opmode_t)read8(BNO055_OPR_MODE_ADDR);
}

/**
 * @brief Sets bno055 power mode
*/
esp_err_t set_powermode(bno055_powermode_t power_mode) {
    esp_err_t err = ESP_OK;
    err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_opmode() error in set_powermode(): %x", err);
        return err;
    }
    err = write8(BNO055_PWR_MODE_ADDR, power_mode);
    return err;
}

/**
 * @brief Gets current bno055 power mode
*/
bno055_powermode_t get_powermode() {
    return (bno055_powermode_t)read8(BNO055_PWR_MODE_ADDR);
}

/**
 * @brief Sets bno055 axis configuration
*/
esp_err_t set_axis_remap(bno055_axis_remap_config_t config) {
    bno055_opmode_t opmode = get_opmode();
    esp_err_t err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        return err;
    }
    err = write8(BNO055_AXIS_MAP_CONFIG_ADDR, config);
    if (err != ESP_OK) {
        return err;
    }
    err = set_opmode(opmode);
    return err;

}

/**
 * @brief Gets bno055 axis configuration
*/
bno055_axis_remap_config_t get_axis_remap(void) {
    bno055_opmode_t opmode = get_opmode();
    esp_err_t err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not set operation mode: %x", err);

    }
    bno055_axis_remap_config_t axis_config = read8(BNO055_AXIS_MAP_CONFIG_ADDR);
    err = set_opmode(opmode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not set operation mode: %x", err);

    }
    return axis_config;
}

/**
 * @brief Sets bno055 axis sign
*/
esp_err_t set_axis_sign(bno055_axis_remap_sign_t config) {
    bno055_opmode_t opmode = get_opmode();
    if (opmode != OPERATION_MODE_CONFIG) {
        esp_err_t err = set_opmode(OPERATION_MODE_CONFIG);
        if (err != ESP_OK) {
            return err;
        }
        err = write8(BNO055_AXIS_MAP_SIGN_ADDR, config);
        if (err != ESP_OK) {
            return err;
        }
        err = set_opmode(opmode);
        return err;
    }
    return write8(BNO055_AXIS_MAP_SIGN_ADDR, config);

}

/**
 * @brief Gets bno055 axis sign
*/
bno055_axis_remap_sign_t get_axis_sign(void) {
    bno055_opmode_t opmode = get_opmode();
    if (opmode != OPERATION_MODE_CONFIG) {
        esp_err_t err = set_opmode(OPERATION_MODE_CONFIG);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not set operation mode: %x", err);
        }
        bno055_axis_remap_sign_t axis_sign = read8(BNO055_AXIS_MAP_SIGN_ADDR);
        err = set_opmode(opmode);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not set operation mode: %x", err);
        }
        return axis_sign;
    }
    return (bno055_axis_remap_sign_t)read8(BNO055_AXIS_MAP_SIGN_ADDR);
}

/**
 * @brief Configure IMU units
*/
esp_err_t unit_config(bno055_units_config_t* config) {
    //ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Argument is NULL");
    esp_err_t err = ESP_OK;
    uint8_t unit_mask = 0x00;           // to change to pitch rotation angle range set this value to 0x80
    uint8_t accel = (config->accel);
    uint8_t angular_rate = (config->angular_rate);
    uint8_t euler_angle = (config->euler_angel);
    uint8_t temp = (config->temperature);
    uint8_t opmode = get_opmode();
    unit_mask = (accel | angular_rate | euler_angle | temp | unit_mask);
    ESP_LOGD(TAG, "unit_mask: %x", unit_mask);
    if (opmode != OPERATION_MODE_CONFIG) {
        err = set_opmode(OPERATION_MODE_CONFIG);
        if (err != ESP_OK) {
            return err;
        }
        err = write8(BNO055_UNIT_SEL_ADDR, unit_mask);
        if (err != ESP_OK) {
            return err;
        }
        err = set_opmode(opmode);
    }
    err = write8(BNO055_UNIT_SEL_ADDR, unit_mask);
    return err;
}

/**
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated. See section 34.3.54
 */
void get_calibration_state(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}

/**
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool isFullyCalibrated() {
    uint8_t system, gyro, accel, mag;
    get_calibration_state(&system, &gyro, &accel, &mag);

    switch (read8(BNO055_OPR_MODE_ADDR)) {
    case OPERATION_MODE_ACCONLY:
        return (accel == 3);
    case OPERATION_MODE_MAGONLY:
        return (mag == 3);
    case OPERATION_MODE_GYRONLY:
    case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
        return (gyro == 3);
    case OPERATION_MODE_ACCMAG:
    case OPERATION_MODE_COMPASS:
        return (accel == 3 && mag == 3);
    case OPERATION_MODE_ACCGYRO:
    case OPERATION_MODE_IMUPLUS:
        return (accel == 3 && gyro == 3);
    case OPERATION_MODE_MAGGYRO:
        return (mag == 3 && gyro == 3);
    default:
        return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}


/**
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *          Calibration offset uint8_t (buffer size should be 22)
 *  @return ESP_OK
 *          ESP_FAIL
 */
esp_err_t get_sensor_offsets(uint8_t* calibData) {
    esp_err_t err = ESP_OK;
    //if (isFullyCalibrated()) {                    // TODO: uncoment this line
    bno055_opmode_t opmode = get_opmode();
    uint8_t read_reg[1];
    read_reg[0] = (uint8_t)ACCEL_OFFSET_X_LSB_ADDR;
    err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) return err;
    err = i2c_master_write_read_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        read_reg, (size_t)1, calibData, NUM_BNO055_OFFSET_REGISTERS, TIME_TO_WAIT_READ_WRITE);
    //readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

    err = set_opmode(opmode);
    return err;
    //}
    return ESP_FAIL;
}

/**
 *  @brief  Writes the sensor's offset registers from a byte array
 *  @param  calibData
 *          Calibration offset uint8_t (buffer size should be 22)
 *  @return ESP_OK
 *          ESP_FAIL
 */
esp_err_t set_sensor_offset(uint8_t* calib_data) {
    esp_err_t err = ESP_OK;
    bno055_opmode_t opmode = get_opmode();
    err = set_opmode(OPERATION_MODE_CONFIG);
    uint8_t write_buffer[23];
    write_buffer[0] = (uint8_t)ACCEL_OFFSET_X_LSB_ADDR;
    for (int i = 1; i < 23; i++) {
        write_buffer[i] = (uint8_t)calib_data[i - 1];
    }

    err = i2c_master_write_to_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        write_buffer, (size_t)23, TIME_TO_WAIT_READ_WRITE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error in i2c_master_write_to_device: %x", err);
        return err;
    }
    err = set_opmode(opmode);
    return err;
}

/**
 *  @brief  Reads the sensor's offset registers into an offset struct to work with raw data
 *  @param  offsets type of offsets
 *  @return true if read is successful
 */
esp_err_t get_sensor_offsets_struct(bno055_offsets_t* offsets) {
    esp_err_t err = ESP_OK;
    if (isFullyCalibrated()) {
        bno055_opmode_t opmode = get_opmode();
        err = set_opmode(OPERATION_MODE_CONFIG);
        if (err != ESP_OK) return err;
        /* Accel offset range depends on the G-range:
           +/-2g  = +/- 2000 mg
           +/-4g  = +/- 4000 mg
           +/-8g  = +/- 8000 mg
           +/-1§g = +/- 16000 mg */
        offsets->accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
            (read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets->accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
            (read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets->accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
            (read8(ACCEL_OFFSET_Z_LSB_ADDR));

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets->mag_offset_x =
            (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
        offsets->mag_offset_y =
            (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets->mag_offset_z =
            (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));

        /* Gyro offset range depends on the DPS range:
          2000 dps = +/- 32000 LSB
          1000 dps = +/- 16000 LSB
           500 dps = +/- 8000 LSB
           250 dps = +/- 4000 LSB
           125 dps = +/- 2000 LSB
           ... where 1 DPS = 16 LSB */
        offsets->gyro_offset_x =
            (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets->gyro_offset_y =
            (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets->gyro_offset_z =
            (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));

        /* Accelerometer radius = +/- 1000 LSB */
        offsets->accel_radius =
            (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));

        /* Magnetometer radius = +/- 960 LSB */
        offsets->mag_radius =
            (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));

        err = set_opmode(opmode);
        return err;
    }
    return ESP_FAIL;
}



/**
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t get_temp(void) {
    int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
    return temp;
}


/**
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @return  vector from specified source
 *
 *  @p TODO: figure out the convertion units when units are changed
 */
esp_err_t get_vector(bno055_vector_type_t vector_type, int16_t* xyz) {
    esp_err_t err = ESP_OK;
    uint8_t buffer[6];
    memset(buffer, 0, 6);

    int16_t x, y, z;
    uint8_t read_reg[1];
    x = y = z = 0;
    read_reg[0] = (uint8_t)vector_type;
    /* Read vector data (6 bytes) */

    err = i2c_master_write_read_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        read_reg, (size_t)1, buffer, 6, TIME_TO_WAIT_READ_WRITE);
    //readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);
    if (err != ESP_OK) {
        return err;
    }

    x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    /*!
     * Convert the value to an appropriate range (section 3.6.4)
     * and assign the value to the Vector type
     */
    switch (vector_type) {
    case VECTOR_MAGNETOMETER:
        /* 1uT = 16 LSB */
        xyz[0] = ((double)x) / 16.0;
        xyz[1] = ((double)y) / 16.0;
        xyz[2] = ((double)z) / 16.0;
        break;
    case VECTOR_GYROSCOPE:
        /* 1dps = 16 LSB */
        /* 1rps = 900 LSB */
        xyz[0] = ((double)x) / 16.0;
        xyz[1] = ((double)y) / 16.0;
        xyz[2] = ((double)z) / 16.0;
        break;
    case VECTOR_EULER:
        /* 1 degree = 16 LSB */
        /* 1 radian = 900 LSB */
        xyz[0] = ((double)x) / 16.0;
        xyz[1] = ((double)y) / 16.0;
        xyz[2] = ((double)z) / 16.0;
        break;
    case VECTOR_ACCELEROMETER:
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        xyz[0] = ((double)x) / 100.0;
        xyz[1] = ((double)y) / 100.0;
        xyz[2] = ((double)z) / 100.0;
        break;
    case VECTOR_LINEARACCEL:
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        xyz[0] = ((double)x) / 100.0;
        xyz[1] = ((double)y) / 100.0;
        xyz[2] = ((double)z) / 100.0;
        break;
    case VECTOR_GRAVITY:
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        xyz[0] = ((double)x) / 100.0;
        xyz[1] = ((double)y) / 100.0;
        xyz[2] = ((double)z) / 100.0;
        break;
    }

    return err;
}


/**
 *  @brief  Gets a quaternion reading from the specified source
 *  @param quat pointer to an array of int16 of size 8
 *  @return quaternion reading
 */
esp_err_t get_quat(int16_t* quat) {
    esp_err_t err = ESP_OK;
    uint8_t buffer[8];
    memset(buffer, 0, 8);

    int16_t x, y, z, w;
    uint8_t read_reg[1];
    x = y = z = w = 0;
    read_reg[0] = (uint8_t)BNO055_QUATERNION_DATA_W_LSB_ADDR;

    /* Read quat data (8 bytes) */
    err = i2c_master_write_read_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
        read_reg, (size_t)1, buffer, 8, TIME_TO_WAIT_READ_WRITE);
    if (err != ESP_OK) return err;

    //readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
    z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

    /*!
     * Assign to Quaternion
     * See
     * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
     * 3.6.5.5 Orientation (Quaternion)
     */
    const double scale = (1.0 / (1 << 14));
    quat[0] = w * scale;
    quat[1] = x * scale;
    quat[2] = y * scale;
    quat[3] = z * scale;

    return err;
}


