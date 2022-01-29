/**
 *  MIT License
 *  Copyright (c) 2021 Christian Castaneda <github.com/christianjc>
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 * 
 * 
 *          https://www.bosch-sensortec.com/bst/products/all_products/bno055
 *          Reference Datasheet: BST_BNO055_DS000_14 (consulted in January 2018)
 * 
*/

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

esp_err_t get_set_opmod_test(void);
esp_err_t get_set_powermode_test(void);
esp_err_t get_set_axis_remap_test(void);
esp_err_t get_set_axis_sign_test(void);
esp_err_t unit_config_test(void);
esp_err_t get_calibration_state_test(void);
esp_err_t isFullyCalibrated_test(void);
esp_err_t print_sensor_offsets_test(void);
esp_err_t get_sensor_offsets_struct_test(void);
esp_err_t get_temp_test(void);
esp_err_t get_vector_test(void);
esp_err_t get_quat_test(void);
esp_err_t calibrate_sensor_test(void);

void app_main(void)
{
    /** Testing bno055_begin() **/
    esp_err_t err = bno055_begin_i2c(OPERATION_MODE_IMUPLUS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not initialize communication: %x", err);
    }

    bno055_powermode_t pwr_mode = get_powermode();

    ESP_LOGD(TAG, "power mode: %x", pwr_mode);

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
    err = print_sensor_offsets_test();
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

    err = calibrate_sensor_test();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not calibrate sensor: %x", err);
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

esp_err_t print_sensor_offsets_test(void)
{
    //Test get_sensor_offset function
    print_calib_profile_from_sensor();
    return ESP_OK;
}

esp_err_t get_sensor_offsets_struct_test(void)
{
    //Test get_sensor_offsets_struct function
    bno055_offsets_t *offset_struct = (bno055_offsets_t *)calloc(1, sizeof(bno055_offsets_t));
    esp_err_t err = get_sensor_offsets_struct(offset_struct);
    return err;
}

esp_err_t get_temp_test(void)
{
    // Test get_temp() function
    printf("temperature: %d [F]\n", get_temp());
    return ESP_OK;
}

esp_err_t calibrate_sensor_test(void)
{
    esp_err_t err = calibrate_sensor(true);
    if (err != ESP_OK)
    {
        printf("Calibration error\n");
        return err;
    }

    if (isFullyCalibrated())
    {
        print_calib_profile_from_sensor();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        print_calib_profile_from_nvs();
        get_calibration_state_test();
    }
    else
    {
        printf("sensor is not fully calibrated\n");
        get_calibration_state_test();
    }

    return err;
}

esp_err_t get_vector_test(void)
{
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
    // test getting vector data from bno055
    double xyz[3];

    esp_err_t err = get_vector(VECTOR_EULER, xyz);
    for (int i = 0; i < 2000; i++)
    {
        err = get_vector(VECTOR_EULER, xyz);
        if (err != ESP_OK)
        {
            printf("could not get euler vector: (%s)", esp_err_to_name(err));
            return err;
        }
        print_vector(VECTOR_EULER, xyz);
        err = get_vector(VECTOR_GYROSCOPE, xyz);
        if (err != ESP_OK)
        {
            printf("could not get gyro vector: (%s)", esp_err_to_name(err));
            return err;
        }
        print_vector(VECTOR_GYROSCOPE, xyz);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return err;
}

esp_err_t get_quat_test(void)
{
    // test get_quat() function
    //esp_err_t err = get_quat(int16_t * quat);
    return ESP_OK;
}