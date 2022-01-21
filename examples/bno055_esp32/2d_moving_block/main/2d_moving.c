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

static void get_vector_callback(void* arg) {
    double vec[3];
    //memset(vec, 0, 3);
    //esp_err_t err = ESP_OK;

// VECTOR_ACCELEROMETER
// VECTOR_MAGNETOMETER
// VECTOR_GYROSCOPE
// VECTOR_EULER
// VECTOR_LINEARACCEL
// VECTOR_GRAVITY
    for (;;) {
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

void app_main(void) {
    printf("Hello World!! \n");

    /** Create Queue Handles **/
    xVectorQueue = xQueueCreate(10, (sizeof(double) * 3));

    esp_err_t err = bno055_begin_i2c(OPERATION_MODE_IMUPLUS);
    if (err != ESP_OK) {
        printf("ERROR: could not initiate 12c comunication\n");
    }

    //calibrate_sensor(true);

    /** Calibrate sensor or use calibration profile **/
    err = calibrate_sensor_from_saved_profile();
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = calibrate_sensor(true);
            if (err != ESP_OK)
                printf("ERROR: could not calibrate sensor\n");
        }
        else {
            printf("ERROR: Something went wrong\n");
        }
    }

    /** Create application tasks **/
    xTaskCreate(get_vector_callback, "get_vector", 2048 * 4, NULL, 11, &xUpdateVectorTask);

}

