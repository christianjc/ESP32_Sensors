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

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_bno055.h"

/** Task Handles **/
static TaskHandle_t xUpdateVectorTask = NULL;
//static TaskHandle_t xUpdateFrameTask = NULL;

/** Queue Handles **/
static QueueHandle_t xVectorQueue = NULL;

static void get_vector_callback(void *arg)
{
    double vec[3];
    //memset(vec, 0, 3);
    //esp_err_t err = ESP_OK;

    // VECTOR_ACCELEROMETER
    // VECTOR_MAGNETOMETER
    // VECTOR_GYROSCOPE
    // VECTOR_EULER
    // VECTOR_LINEARACCEL
    // VECTOR_GRAVITY
    for (;;)
    {
        // get_vector(VECTOR_EULER, vec);
        // print_vector(VECTOR_EULER, vec);
        // get_vector(VECTOR_GYROSCOPE, vec);
        // print_vector(VECTOR_GYROSCOPE, vec);
        // get_vector(VECTOR_LINEARACCEL, vec);
        // print_vector(VECTOR_LINEARACCEL, vec);
        get_vector(VECTOR_ACCELEROMETER, vec);
        print_vector(VECTOR_ACCELEROMETER, vec);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void app_main(void)
{
    printf("Hello World!! \n");

    /** Create Queue Handles **/
    xVectorQueue = xQueueCreate(10, (sizeof(double) * 3));

    esp_err_t err = bno055_begin_i2c(OPERATION_MODE_IMUPLUS);
    if (err != ESP_OK)
    {
        printf("ERROR: could not initiate 12c comunication\n");
    }

    //calibrate_sensor(true);

    /** Calibrate sensor or use calibration profile **/
    err = calibrate_sensor_from_saved_profile();
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            err = calibrate_sensor(true);
            if (err != ESP_OK)
                printf("ERROR: could not calibrate sensor\n");
        }
        else
        {
            printf("ERROR: Something went wrong\n");
        }
    }

    /** Create application tasks **/
    xTaskCreate(get_vector_callback, "get_vector", 2048 * 4, NULL, 11, &xUpdateVectorTask);
}
