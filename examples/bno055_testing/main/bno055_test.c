#include <stdio.h>
#include "bno055_IMU/esp_bno055.h"

void app_main(void)
{

    bno055_begin();
    vTaskDelay(5000 / portTICK_PERIOD_MS)
}
