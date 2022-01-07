#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_types.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_bno055.h"

static const char *TAG = "BNO055 testing";

uint8_t read8(bno055_reg_t);
/**
 * @brief This function writes one byte of data to the given register
 *
 * @param register This is the register address to write the data to
 *
 * @param data This is the data to be written in the register
*/
esp_err_t write8(bno055_reg_t reg, uint8_t data);
esp_err_t read8_and_write8_test(void);
esp_err_t get_set_opmod_test(void);
esp_err_t get_set_powermode_test(void);
esp_err_t get_set_axis_remap_test(void);
esp_err_t get_set_axis_sign_test(void);
esp_err_t unit_config_test(void);
esp_err_t get_calibration_state_test(void);
esp_err_t isFullyCalibrated_test(void);
esp_err_t get_sensor_offsets_test(void);
esp_err_t get_sensor_offsets_struct_test(void);
esp_err_t get_temp_test(void);
esp_err_t get_vector_test(void);
esp_err_t get_quat_test(void);

void app_main(void)
{
    /** Testing bno055_begin() **/
    esp_err_t err = bno055_begin();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not initialize communication: %x", err);
    }

    bno055_powermode_t pwr_mode = get_powermode();

    ESP_LOGD(TAG, "power mode: %x", pwr_mode);

    err = read8_and_write8_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read or write: %x", err);
    }
    err = get_set_opmod_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set or get opmode: %x", err);
    }

    err = get_set_powermode_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set or get powermode: %x", err);
    }

    err = get_set_axis_remap_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get or set axis remap %x", err);
    }

    err = get_set_axis_sign_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get or set axis sign %x", err);
    }
    err = unit_config_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not configure units %x", err);
    }
    err = get_calibration_state_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read calibration data: %x", err);
    }
    err = isFullyCalibrated_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not check if fully calibrated: %x", err);
    }
    err = get_sensor_offsets_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get sensor offset: %x", err);
    }
    err = get_sensor_offsets_struct_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get sensor offset struct: %x", err);
    }
    err = get_temp_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get temperature: %x", err);
    }
    err = get_vector_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get vector data from sensor: %x", err);
    }

    err = get_quat_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get quaternion data from sensor: %x", err);
    }
}

esp_err_t read8_and_write8_test(void)
{
    esp_err_t err = ESP_OK;
    /** Testing write8() function: write to UNIT_SEL register(0x3B) **/
    uint8_t unit_sel_val = read8(BNO055_UNIT_SEL_ADDR);
    ESP_LOGD(TAG, "unit_sel_val in register: %x", unit_sel_val);
    err = write8(BNO055_UNIT_SEL_ADDR, 0x81);
    if ((err != ESP_OK) || (unit_sel_val == read8(BNO055_UNIT_SEL_ADDR)))
    {
        ESP_LOGE(TAG, "Could not write a byte to bno055: %x", err);
    }

    if (read8(BNO055_UNIT_SEL_ADDR) != 0x81)
    {
        ESP_LOGE(TAG, "Could not write value to register - wrote: 0x81  but got: %x", unit_sel_val);
    }
    return err;
}

esp_err_t get_set_opmod_test(void)
{
    esp_err_t err = ESP_OK;
    /** Get the current opmode **/
    bno055_opmode_t op_mode = get_opmode();
    ESP_LOGD(TAG, "get operation mode: %x", (uint8_t)op_mode);
    /** Set to a new mode and again get the mode **/
    err = set_opmode(OPERATION_MODE_NDOF); // OPERATION_MODE_NDOF = 0X0C
    ESP_LOGD(TAG, "set operation mode to: %x", (uint8_t)OPERATION_MODE_NDOF);
    op_mode = get_opmode();
    ESP_LOGD(TAG, "get operation mode after setting it: %x", (uint8_t)op_mode);
    if (op_mode != OPERATION_MODE_NDOF)
    {
        ESP_LOGE(TAG, "Icorrect operation mode: %x", (uint8_t)op_mode);
        return ESP_FAIL;
    }
    return err;
}

esp_err_t get_set_powermode_test(void)
{
    esp_err_t err = ESP_OK;
    bno055_powermode_t pwrmode = get_powermode();
    ESP_LOGD(TAG, "Crrent power mode: %x", pwrmode);
    err = set_powermode(POWER_MODE_LOWPOWER); // POWER_MODE_LOWPOWER = 0X01
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get power mode, error: %x", err);
        return err;
    }
    ESP_LOGD(TAG, "Power mode to be set to: %x", POWER_MODE_LOWPOWER);
    pwrmode = get_powermode();
    ESP_LOGD(TAG, "Power mode after setting it: %x", pwrmode);
    if (pwrmode != POWER_MODE_LOWPOWER)
    {
        ESP_LOGE(TAG, "Could not get power mode, error: %x", err);
    }
    return err;
}

esp_err_t get_set_axis_remap_test(void)
{
    esp_err_t err = ESP_OK;

    bno055_axis_remap_config_t axis_config = get_axis_remap();
    ESP_LOGD(TAG, "Get axis configuration: %x", axis_config);
    ESP_LOGD(TAG, "Axis configuration to be set to: %x", REMAP_CONFIG_P3);
    err = set_axis_remap(REMAP_CONFIG_P3); // REMAP_CONFIG_P3 = 0x21
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set axis remap: %x", err);
        return err;
    }
    axis_config = get_axis_remap();
    ESP_LOGD(TAG, "Get axis configuration after setting it: %x", axis_config);
    if (axis_config != REMAP_CONFIG_P3)
    {
        ESP_LOGE(TAG, "Error setting axis configuration: %x", err);
        return err;
    }
    err = set_axis_remap(REMAP_CONFIG_P1); // REMAP_CONFIG_P1 = 0x24 Default
    return err;
}

esp_err_t get_set_axis_sign_test(void)
{
    esp_err_t err = ESP_OK;

    bno055_axis_remap_sign_t axis_sign = get_axis_sign();
    ESP_LOGD(TAG, "Get axis sign: %x", axis_sign);
    ESP_LOGD(TAG, "Axis sign to be set to: %x", REMAP_SIGN_P3);
    err = set_axis_sign(REMAP_SIGN_P3); // REMAP_SIGN_P3 = 0x02
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set axis sign: %x", err);
        return err;
    }
    axis_sign = get_axis_sign();
    ESP_LOGD(TAG, "Get axis sign after setting it: %x", axis_sign);
    if (axis_sign != REMAP_SIGN_P3)
    {
        ESP_LOGE(TAG, "Error setting axis sign: %x", err);
        return err;
    }
    err = set_axis_sign(REMAP_SIGN_P1); // REMAP_SIGN_P1 = 0x00 Default
    return err;
}

esp_err_t unit_config_test(void)
{
    esp_err_t err = ESP_OK;
    bno055_units_config_t units = {
        .accel = UNITS_MS2,
        .angular_rate = UNITS_RPS,
        .euler_angel = UNITS_RADIANS,
        .temperature = UNITS_FAHRENHEIT,
    };
    err = unit_config(&units);
    return err;
}

esp_err_t get_calibration_state_test(void)
{
    uint8_t system, gyro, accel, mag;
    get_calibration_state(&system, &gyro, &accel, &mag);
    ESP_LOGD(TAG, "System: %x Gyro: %x Accel: %x Mag: %x", system, gyro, accel, mag);
    // More test needed here

    return ESP_OK;
}

esp_err_t isFullyCalibrated_test(void)
{
    // test is fully calibrated function
    if (!isFullyCalibrated())
    {
        ESP_LOGD(TAG, "Sensor is NOT fully callibrated");
        return ESP_OK;
    }
    ESP_LOGD(TAG, "Sensor is fully callibrated");
    return ESP_OK;
}

esp_err_t get_sensor_offsets_test(void)
{
    //Test get_sensor_offset function
    uint8_t calibData[22];
    memset(calibData, 0, 22);
    esp_err_t err = get_sensor_offsets(calibData);
    if (err != ESP_OK)
    {
        return err;
    }

    for (int i = 0; i < 22; i++)
    {
        printf("value: %d  of index: %d\n", calibData[i], i);
    }
    return ESP_OK;
}

esp_err_t get_sensor_offsets_struct_test(void)
{
    //Test get_sensor_offsets_struct function
    return ESP_OK;
}

esp_err_t get_temp_test(void)
{
    // Test get_temp() function
    return ESP_OK;
}

esp_err_t get_vector_test(void)
{
    // test getting vector data from bno055
    return ESP_OK;
}

esp_err_t get_quat_test(void)
{
    // test get_quat() function
    return ESP_OK;
}
