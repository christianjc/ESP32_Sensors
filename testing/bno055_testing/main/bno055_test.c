#include <stdio.h>
#include "esp_log.h"

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_bno055.h"

#include "esp_test_comp.h"

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