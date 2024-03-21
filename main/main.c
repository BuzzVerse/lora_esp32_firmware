#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"


#include "sdcard.h"

void app_main(void)
{
    sdspi_init();
    sdspi_test();
    sdspi_close();
}