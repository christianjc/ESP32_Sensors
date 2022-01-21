// i2c.c i2c.h i2c_hal.c i2c_hal.h i2c_ll.h reference documents

#include "esp_bno055.h"

/** The BNO055 needs at least 500us for clock stretching.
 *  The esp32 clock here is 80 MHz.
 *  We have the following calculation:
 *      Convert 500us to Hz:            1/(0.000,500) = 2000Hz
 *      Clock cycles to stretch clock:  (80 MHz) / (2000 Hz) = 40,000 cycles
 *      INITIAL_MASTER_TOUT = 40,000 clock cycles
 */
#define INITIAL_MASTER_TOUT         (40000)     
#define TIME_TO_WAIT_READ_WRITE     ((TickType_t)1000)
#define TIME_TO_WAIT_OP_MODE        (30)                    
#define STORAGE_NAMESPACE "storage"             // Partion name to store sensor offset profile

 /** Static constants **/
static const char* TAG = "i2c-bno055-IMU";

/** Static functions declaration **/
static esp_err_t i2c_master_init(void);
static uint8_t read8(bno055_reg_t reg);
static esp_err_t write8(bno055_reg_t reg, uint8_t data);
static esp_err_t print_calib_profile(uint8_t* calib_data);

/** Static functions definitions **/
/**
 * @brief   Initializes the i2c master communication with
 *          the BNO055 sensor.
 * @return  ESP_OK
 *          ESP_FAIL
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
 * @brief Reads one byte from the BNO055 (i2c slave) register
 * @param reg   Register address to read from.
 * @return  Returns one byte of data read from the given register.
*/
static uint8_t read8(bno055_reg_t reg) {
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
 * @brief   Writes one byte of data to the BNO055 given register
 * @param   register: Register address to write the data to.
 * @param   data: Data to be written in the register.
 * @return  ESP_OK
 *          ESP_FAIL
*/
static esp_err_t write8(bno055_reg_t reg, uint8_t data) {
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
 * @brief   Helper function to print sensor calibration offset values.
 * @param   calib_data: Array of uint8_t of size 22. Contains the offset
 *                      valus for the BNO055 sensors. Odd elements correspond
 *                      to MSB and even elements to LSB of one value.
 * @return  ESP_OK
 *          ESP_FAIL
*/
static esp_err_t print_calib_profile(uint8_t* calib_data) {
    if (calib_data != NULL) {
        printf("\n            **** Accelerometer Offsets ****\n");
        printf("Accel offset X: %d [m/s^2]\n", ((calib_data[1] << 8) | (calib_data[0])));
        printf("Accel offset Y: %d [m/s^2]\n", ((calib_data[3] << 8) | (calib_data[2])));
        printf("Accel offset Z: %d [m/s^2]\n", ((calib_data[5] << 8) | (calib_data[4])));
        printf("\n            **** Magnetometer Offsets ****\n");
        printf("Mag offset X: %d\n", ((calib_data[7] << 8) | (calib_data[6])));
        printf("Mag offset Y: %d\n", ((calib_data[9] << 8) | (calib_data[8])));
        printf("Mag offset Z: %d\n", ((calib_data[11] << 8) | (calib_data[10])));
        printf("\n            **** Gyroscope Offsets ****\n");
        printf("Gyr offset X: %d [Rps]\n", ((calib_data[13] << 8) | (calib_data[12])));
        printf("Gyr offset Y: %d [Rps]\n", ((calib_data[15] << 8) | (calib_data[14])));
        printf("Gyr offset Z: %d [Rps]\n", ((calib_data[17] << 8) | (calib_data[16])));
        printf("\n            **** Accelerometer Radius ****\n");
        printf("Accel radius: %d [Radians]\n", ((calib_data[19] << 8) | (calib_data[18])));
        printf("\n            **** Magnetometer Radius ****\n");
        printf("Mag radius: %d [Radians]\n", ((calib_data[21] << 8) | (calib_data[20])));
    }
    else {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/** Function definitions **/
/**
 * @brief   Initiallizes i2c communication between esp32 and bno055 sensor
 *          and calibrates the sensor fron data profile store in nvs or
 *          starts new calibration if there is no profile stored.
 *       <<!As of right now the operation mode is set to IMU in the bno055 sensor.!>>
 * @return  ESP_OK
 *          ESP_FAIL
*/
esp_err_t bno055_begin_i2c(bno055_opmode_t mode) {
    /** Inintialize the i2c master configuration **/
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c Master init error: %d", err);
        i2c_driver_delete(I2C_NUM_0);
        return err;
    }

    /** Set the timeout for the communication bus (clock stretching) **/
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

    err = set_powermode(POWER_MODE_NORMAL);
    if (err != ESP_OK) return err;

    err = set_opmode(mode);
    if (err != ESP_OK) return err;

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    return err;
}

/**
 * @brief   Resets the bno055 sensor and waits until communication is
 *          resummed.
 *
 * @return  ESP_OK - successfully reseted sensor.
 *          ESP_FAIL - sensor could not be reseted.
*/
esp_err_t bno055_reset(void) {
    uint8_t bno055_id = read8(BNO055_CHIP_ID_ADDR);
    esp_err_t err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) return err;

    err = write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (err != ESP_OK) return err;

    /** Wait for the chip to restart **/
    vTaskDelay(80);
    while (bno055_id != read8(BNO055_CHIP_ID_ADDR)) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return err;
}

/**
 * @brief   Calibrate the necessary sensors. The current setting
 *          is to calibrate for operation mode: IMU. This can be change
 *          later to include more operation modes. nvs_flash_init() must be
 *          called before this function can be used.
 *
 * @return  ESP_OK - sensor was successfully calibrated.
 *          ESP_FAIL - sensor could not be calibrated.
*/
esp_err_t calibrate_sensor(bool save_profile) {

    ESP_LOGI(TAG, "[calibrate_sensor]: Please start calibration");
    /** Reset sensor to beging calibration **/
    esp_err_t err = bno055_reset();
    if (err != ESP_OK) return  err;

    /** Change operation mode to IMU to start calibration **/
    err = set_opmode(OPERATION_MODE_IMUPLUS);
    if (err != ESP_OK) return  err;

    /** Wait until calibration is completed or return an error **/
    uint8_t counter = 0;
    while (!isFullyCalibrated()) {
        if (counter > 100) {
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "[calibrate_sensor]: is not calibrated: counter %d", counter);
        uint8_t system, gyro, accel, mag;
        get_calibration_state(&system, &gyro, &accel, &mag);
        ESP_LOGI(TAG, "[calibrate_sensor]: System: %x Gyro: %x Accel: %x Mag: %x", system, gyro, accel, mag);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        counter++;
    }
    ESP_LOGI(TAG, "[calibrate_sensor]: sensor is now calibrated!! ");

    /** Save calibrated profile to non-volitile storage **/
    if (save_profile) {
        ESP_LOGI(TAG, "[calibrate_sensor]: Saving calibrated profile to non-volatile memory...");

        uint8_t calib_data[22];
        err = get_sensor_offsets(calib_data);
        if (err != ESP_OK) return err;

        err = save_calib_profile_to_nvs(calib_data);
        if (err != ESP_OK) return err;

        ESP_LOGI(TAG, "[calibrate_sensor]: Calibrated profile was saved to nvs succesfully!");
    }

    return err;
}

/**
 * @brief   Calibrates the bno055 internal sensors with a previously
 *          calibration data saved in the non-volatile storage of the esp32.
 *          nvs_flash_init() must be called before this function can be used.
 *
 * @return  ESP_OK - calibration was succesful
 *          ESP_ERR_NVS_NOT_FOUND - There was not a saved profile in non-volitile storage (nvs)
 *          ESP_FAIL
*/
esp_err_t calibrate_sensor_from_saved_profile(void) {
    uint8_t calib_data[NUM_BNO055_OFFSET_REGISTERS];
    memset(calib_data, 0, 22);
    esp_err_t err = get_calib_profile_from_nvs(calib_data);
    switch (err) {
    case ESP_OK:
        err = set_sensor_offset(calib_data);
        printf("check if isFullyCalibrated is true after updating sensor ofsets.....\nisFullyCalibrated:  ");
        uint8_t count = 0;
        while (!isFullyCalibrated() && count < 100) {
            printf("not calibrated\n");
            count++;
            vTaskDelay(500 / portTICK_RATE_MS);
        }

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
 * @brief   Gets the calibration profile from the non-volatile storate (nvs).
 *          nvs_flash_init() must be called before this function can
 *          be used.
 *
 * @param calib_data    Pointer to a uint8_t array of size 22
 *
 * @return  ESP_OK
 *          ESP_FAIL
 *          ESP_ERR_INVALID_ARG
*/
esp_err_t get_calib_profile_from_nvs(uint8_t* calib_data) {     // TODO: could be static?
    /** Open nvs **/
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    /** Read memory space for blob if it exists **/
    size_t size = (size_t)(NUM_BNO055_OFFSET_REGISTERS);
    memset(calib_data, 0, NUM_BNO055_OFFSET_REGISTERS);
    err = nvs_get_blob(my_handle, "calib_data", calib_data, &size);

    nvs_close(my_handle);
    ESP_LOGD(TAG, "[calibrate_sensor]: Error (%s) reading!", esp_err_to_name(err));
    return err;
}

/**
 * @brief   Saves calibrated profile data to the non-volitile storage (nvs)
 *          of the esp32. nvs_flash_init() must be called before this function can
 *          be used.
 *
 * @param calib_data    A pointer to an array of uint_8 with 22 elements.
 *
 * @return  ESP_OK - calibration profile was successfully saved.
 *          ESP_FAIL -  failed to save profile to nvs.
*/
esp_err_t save_calib_profile_to_nvs(uint8_t* calib_data) {      // TODO: could be static?

    /** Open **/
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read memory space for blob if it exists
    size_t size = (size_t)(NUM_BNO055_OFFSET_REGISTERS);
    err = nvs_set_blob(my_handle, "calib_data", calib_data, size);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return err;
    }

    err = nvs_commit(my_handle);
    nvs_close(my_handle);
    return err;
}

/**
 * @brief   Sets bno055 to one of the following operation mode:
 *              OPERATION_MODE_CONFIG
 *              OPERATION_MODE_ACCONLY
 *              OPERATION_MODE_MAGONLY
 *              OPERATION_MODE_GYRONLY
 *              OPERATION_MODE_ACCMAG
 *              OPERATION_MODE_ACCGYRO
 *              OPERATION_MODE_MAGGYRO
 *              OPERATION_MODE_AMG
 *              OPERATION_MODE_IMUPLUS
 *              OPERATION_MODE_COMPASS
 *              OPERATION_MODE_M4G
 *              OPERATION_MODE_NDOF_FMC_OFF
 *              OPERATION_MODE_NDOF
 *
 * @param op_mode   mask config of type bno055_opmode_t
 *
 * @return  ESP_OK - operation set succesfully
 *          ESP_FAIL - operation could not be set
*/
esp_err_t set_opmode(bno055_opmode_t op_mode) {
    esp_err_t err = write8(BNO055_OPR_MODE_ADDR, op_mode);
    return err;
}

/**
 * @brief   Gets the current bno055 operation mode.
 *          TODO: maybe change function return type to esp_err_t
 *
 * @return  mask config of type bno055_opmode_t
*/
bno055_opmode_t get_opmode(void) {
    return (bno055_opmode_t)read8(BNO055_OPR_MODE_ADDR);
}

/**
 * @brief   Sets bno055 to one of the following power modes:
 *          POWER_MODE_NORMAL = 0X00,
 *          POWER_MODE_LOWPOWER = 0X01,
 *          POWER_MODE_SUSPEND = 0X02
 *
 * @param power_mode    one of the power modes above of type bno055_powermode_t
 *
 * @return  ESP_OK - power mode was set succesfully.
 *          ESP_FAIL - power mode could not be set.
 *
*/
esp_err_t set_powermode(bno055_powermode_t power_mode) {
    esp_err_t err = ESP_OK;
    err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[set_opmode]: error in set_powermode(): %x", err);
        return err;
    }
    err = write8(BNO055_PWR_MODE_ADDR, power_mode);
    return err;
}

/**
 * @brief   Gets current bno055 power mode.
 *
 * @return returns power mode of type bno055_powermode_t
*/
bno055_powermode_t get_powermode(void) {
    return (bno055_powermode_t)read8(BNO055_PWR_MODE_ADDR);
}

/**
 * @brief   Sets bno055 axis remap configuration.
 *          REMAP_CONFIG_P0 = 0x21,
 *          REMAP_CONFIG_P1 = 0x24, // default
 *          REMAP_CONFIG_P2 = 0x24,
 *          REMAP_CONFIG_P3 = 0x21,
 *          REMAP_CONFIG_P4 = 0x24,
 *          REMAP_CONFIG_P5 = 0x21,
 *          REMAP_CONFIG_P6 = 0x21,
 *          REMAP_CONFIG_P7 = 0x24
 *
 * @param config axis config mask of type bno055_axis_remap_config_t
 *
 * @return  ESP_OK - axis remap was successfully set.
 *          ESP_FAIL - axis remap could not be set.
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
 * @brief   Gets bno055 axis remap configuration.
 *          TODO: change return type to esp_err_t
 *
 * @return axis config of type bno055_axis_remap_config_t.
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
 * @brief   Sets bno055 axis sign config.
 *          REMAP_SIGN_P0 = 0x04,
 *          REMAP_SIGN_P1 = 0x00, // default
 *          REMAP_SIGN_P2 = 0x06,
 *          REMAP_SIGN_P3 = 0x02,
 *          REMAP_SIGN_P4 = 0x03,
 *          REMAP_SIGN_P5 = 0x01,
 *          REMAP_SIGN_P6 = 0x07,
 *          REMAP_SIGN_P7 = 0x05
 *
 * @param config axis sign config mask of type bno055_axis_remap_sign_t.
 *
 * @return  ESP_OK - axis sign was set succesfully.
 *          ESP_FAIL - axis sign could not be set.
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
 * @brief   Gets bno055 axis sign.
 *          TODO: change return type to esp_err_t.
 *
 * @return axis sign mask of type bno055_axis_remap_sign_t.
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
 * @brief   Configure IMU units
 *
 * @param config pointer of type bno055_units_config_t*
 *
* @return   ESP_OK - unit config was successfully set.
 *          ESP_FAIL - unit config could not be set.
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
 *          fully calibrated. See section 34.3.54 of bno055 datasheet.
 *
 * @param sys   systme calibration state.
 * @param gyro  gyroscope calibration state.
 * @param accel accelerometer calibration state.
 * @param mag   magnetometer calibration state.
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
 *  @brief  Checks that the operation mode is fully calibrated. The calibration state of a sensor is 3.
 *  @return TRUE - system is fully calibrated.
 *          FALSE - systme is not fully calibrated.
 */
bool isFullyCalibrated(void) {
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
 *  @brief  Reads the sensor's offset registers into a byte array.
 *
 *  @param  calib_data Calibration offset of type uint8_t (buffer size should be 22).
 *
 *  @return ESP_OK - got sensor offsets successfully.
 *          ESP_FAIL - fail to get sensor offsets.
 */
esp_err_t get_sensor_offsets(uint8_t* calib_data) {
    esp_err_t err = ESP_OK;
    if (isFullyCalibrated()) {                    // TODO: uncoment this line
        bno055_opmode_t opmode = get_opmode();
        uint8_t read_reg[1];
        read_reg[0] = (uint8_t)ACCEL_OFFSET_X_LSB_ADDR;
        err = set_opmode(OPERATION_MODE_CONFIG);
        if (err != ESP_OK) return err;
        err = i2c_master_write_read_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
            read_reg, (size_t)1, calib_data, NUM_BNO055_OFFSET_REGISTERS, TIME_TO_WAIT_READ_WRITE);
        //readLen(ACCEL_OFFSET_X_LSB_ADDR, calib_data, NUM_BNO055_OFFSET_REGISTERS);

        err = set_opmode(opmode);
        return err;
    }
    return ESP_FAIL;
}

/**
 *  @brief  Writes the sensor's offset registers from a byte array.
 *
 *  @param  calib_data Calibration offset of type uint8_t (buffer size should be 22).
 *
 *  @return ESP_OK - succesfully set sensor offset data.
 *          ESP_FAIL - fail to set sensor offset data.
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
 *  @brief  Reads the sensor's offset registers into an offset struct to work with raw data.
 *
 *  @param  offsets typedef struct of type bno055_offsets_t*
 *                      int16_t accel_offset_x;  x acceleration offset
 *                      int16_t accel_offset_y;  y acceleration offset
 *                      int16_t accel_offset_z;  z acceleration offset
 *                      int16_t mag_offset_x;    x magnetometer offset
 *                      int16_t mag_offset_y;    y magnetometer offset
 *                      int16_t mag_offset_z;    z magnetometer offset
 *                      int16_t gyro_offset_x;   x gyroscrope offset
 *                      int16_t gyro_offset_y;   y gyroscrope offset
 *                      int16_t gyro_offset_z;   z gyroscrope offset
 *                      int16_t accel_radius;    acceleration radius
 *                      int16_t mag_radius;      magnetometer radius
 *
 *  @return ESP_OK - read sensor offsets successfully.
 *          ESP_FAIL - fial to read sensor offsets.
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
 *  @brief  Gets the temperature in degrees celsius.
 *
 *  @return temperature in degrees celsius.
 */
int8_t get_temp(void) {
    int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
    return temp;
}

/**
 *  @brief  Sets to use the external crystal (32.768KHz).
 *
 *  @param  use_external_crystal use external crystal boolean.
 *
 * @return  ESP_OK - external crystal was set succesfully.
 *          ESP_FAIL - external crystal could not be set.
 */
esp_err_t set_external_crystal(bool use_external_crystal) {
    bno055_opmode_t opmode = get_opmode();

    /* Switch to config mode */
    esp_err_t err = set_opmode(OPERATION_MODE_CONFIG);
    if (err != ESP_OK) return err;

    err = write8(BNO055_PAGE_ID_ADDR, 0);
    if (err != ESP_OK) return err;

    if (use_external_crystal) {
        err = write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
        if (err != ESP_OK) return err;
    }
    else {
        err = write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
        if (err != ESP_OK) return err;
    }
    /* Set previouse operation mode */
    err = set_opmode(opmode);
    return err;
}

/**
 * @brief  Gets system status info.

 * @param  system_status    system status info: (see section 4.3.58)
 *                              0 = Idle
 *                              1 = System Error
 *                              2 = Initializing Peripherals
 *                              3 = System Iniitalization
 *                              4 = Executing Self-Test
 *                              5 = Sensor fusio algorithm running
 *                              6 = System running without fusion algorithms
 *
 * @param  self_test_result     self test result:
 *                                  1 = test passed, 0 = test failed
 *                                  Bit 0 = Accelerometer self test
 *                                  Bit 1 = Magnetometer self test
 *                                  Bit 2 = Gyroscope self test
 *                                  Bit 3 = MCU self test
 *                                  0x0F = all passed
 *
 * @param  system_error     system error info: (see section 4.3.59)
 *                              0 = No error
 *                              1 = Peripheral initialization error
 *                              2 = System initialization error
 *                              3 = Self test result failed
 *                              4 = Register map value out of range
 *                              5 = Register map address out of range
 *                              6 = Register map write error
 *                              7 = BNO low power mode not available for selected operat ion mode
 *                              8 = Accelerometer power mode not available
 *                              9 = Fusion algorithm configuration error
 *                              A = Sensor configuration error
 *
 * @return ESP_OK - read system status
 *          ESP_FAIL - fail to set page id to zero
 */
esp_err_t get_system_status(uint8_t* system_status, uint8_t* self_test_result, uint8_t* system_error) {
    esp_err_t err = write8(BNO055_PAGE_ID_ADDR, 0);
    if (err != ESP_OK) return err;

    if (system_status != NULL) {
        *system_status = read8(BNO055_SYS_STAT_ADDR);
    }

    if (self_test_result != NULL) {
        *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);
    }

    if (system_error != NULL) {
        *system_error = read8(BNO055_SYS_ERR_ADDR);
    }
    return err;
}

/**
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type possible vector type values:
 *                          VECTOR_ACCELEROMETER
 *                          VECTOR_MAGNETOMETER
 *                          VECTOR_GYROSCOPE
 *                          VECTOR_EULER
 *                          VECTOR_LINEARACCEL
 *                          VECTOR_GRAVITY
 *
 *  @return  ESP_OK - successfully read vector.
 *           ESP_FAIL - fail to get vector.
 *
 *  @p TODO: figure out the convertion units when units are changed
 */
esp_err_t get_vector(bno055_vector_type_t vector_type, double* xyz) {
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
    //printf("x: %d\n", x);

    /**
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
 *
 *  @param quat pointer to an array of int16 of size 8.
 *
 *  @return ESP_OK - succesfully read quaternion.
 *          ESP_FAIL - could not read quaternion.
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


/*** Printig helper functions ***/
/**
 * @brief   Print calibration profile from non-volitile storage (nvs).
 *
 * @return  ESP_OK - succesfully printed profile from nvs.
 *          ESP_ERR_NVS_NOT_FOUND - profile not found in nvs.
 *          ESP_FAIL - fail to print profile.
*/
esp_err_t print_calib_profile_from_nvs(void) {
    esp_err_t err = ESP_OK;
    uint8_t calib_data[NUM_BNO055_OFFSET_REGISTERS];
    memset(calib_data, 0, NUM_BNO055_OFFSET_REGISTERS);
    err = get_calib_profile_from_nvs(calib_data);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    switch (err) {
    case ESP_OK:
        printf("\nPRINTING PROFILE FROM NVS...\n");
        print_calib_profile(calib_data);
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
 * @brief   Print calibration profile from bno055 sensor.
 *
 * @return  ESP_OK - succesfully printed profile from bno055 sensor.
 *          ESP_FAIL - fail to print profile.
*/
esp_err_t print_calib_profile_from_sensor(void) {
    esp_err_t err = ESP_OK;
    uint8_t calib_data[NUM_BNO055_OFFSET_REGISTERS];
    memset(calib_data, 0, NUM_BNO055_OFFSET_REGISTERS);
    err = get_sensor_offsets(calib_data);
    if (err != ESP_OK) return err;
    printf("\nPRINTING PROFILE FROM SENSOR REGISTERS...\n");
    print_calib_profile(calib_data);
    return err;
}

/**
 * @brief   Print the given vector type:
 *              VECTOR_ACCELEROMETER
 *              VECTOR_MAGNETOMETER
 *              VECTOR_GYROSCOPE
 *              VECTOR_EULER
 *              VECTOR_LINEARACCEL
 *              VECTOR_GRAVITY
 *
 * @param vector_type   type of vector to be printed.
 * @param xyz           array of type int16_t of size 3.
 *
 * @return  ESP_OK - succesfully printed vector.
 *          ESP_FAIL - fail to print vector.
*/
esp_err_t print_vector(bno055_vector_type_t vector_type, double* xyz) {
    switch (vector_type) {
    case VECTOR_MAGNETOMETER:
        /* 1uT = 16 LSB */
        printf("\n            **** Magnetometer Vector ****\n");
        printf("Mag X: %f [Micro Tesla]\n", xyz[0]);
        printf("Mag Y: %f [Micro Tesla]\n", xyz[1]);
        printf("Mag Z: %f [Micro Tesla]\n", xyz[2]);
        break;
    case VECTOR_GYROSCOPE:
        /* 1dps = 16 LSB */
        /* 1rps = 900 LSB */
        printf("\n            **** Gyroscope Vector ****\n");
        printf("Gyro X: %f [dps]\n", xyz[0]);
        printf("Gyro Y: %f [dps]\n", xyz[1]);
        printf("Gyro Z: %f [dps]\n", xyz[2]);
        break;
    case VECTOR_EULER:
        /* 1 degree = 16 LSB */
        /* 1 radian = 900 LSB */
        printf("\n            **** Euler Vector ****\n");
        printf("Euler Heading -Z: %f [degrees]\n", xyz[0]);
        printf("Euler Roll    -Y: %f [degrees]\n", xyz[1]);
        printf("Euler Pitch   -X: %f [degrees]\n", xyz[2]);
        break;
    case VECTOR_ACCELEROMETER:
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        printf("\n            **** Accelerometer Vector ****\n");
        printf("Accelerometer X: %f [m/s^2]\n", xyz[0]);
        printf("Accelerometer Y: %f [m/s^2]\n", xyz[1]);
        printf("Accelerometer Z: %f [m/s^2]\n", xyz[2]);
        break;
    case VECTOR_LINEARACCEL:
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        printf("\n            **** Linear Acceleration Vector ****\n");
        printf("Acceleration X: %f [m/s^2]\n", xyz[0]);
        printf("Acceleration Y: %f [m/s^2]\n", xyz[1]);
        printf("Acceleration Z: %f [m/s^2]\n", xyz[2]);
        break;
    case VECTOR_GRAVITY:
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        printf("\n            **** Gravity Vector ****\n");
        printf("Grativy X: %f [m/s^2]\n", xyz[0]);
        printf("Gravity Y: %f [m/s^2]\n", xyz[1]);
        printf("Gravity Z: %f [m/s^2]\n", xyz[2]);
        break;
    }
    return ESP_OK;
}

/**
 * @brief   Helper function to print quaternions.
 *
 * @param xyz   array of type int16_t of size 3.
 *
 * @return  ESP_OK - successfully printed quaternion.
 *          ESP_FAIL - fail to print quaternion.
*/
esp_err_t print_quat(int16_t* xyz) {
    printf("\n            **** Quaternion ****\n");
    printf("W: %d\n", xyz[0]);
    printf("X: %d\n", xyz[0]);
    printf("Y: %d\n", xyz[1]);
    printf("Z: %d\n", xyz[2]);
    return ESP_OK;
}