#include "irrigation_system.h"
#include "main.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    esp_err_t ret = irrigation_system_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "System initialization failed");
        return;
    }
    
    // Main thread can now sleep
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}