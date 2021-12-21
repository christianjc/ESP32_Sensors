#include <stdio.h>
//#include "esp_log.h"
//#include "driver/i2c.h"
//#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_test_comp.h"

void test_func(void)
{
    printf("This is a test: pringting");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
}