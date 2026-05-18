#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// This is the "Tag" used for logging, helps in filtering messages
static const char *TAG = "FlowBox_Core";

void app_main(void) {
    ESP_LOGI(TAG, "===============================");
    ESP_LOGI(TAG, "   FLOWBOX NATIVE ESP-IDF      ");
    ESP_LOGI(TAG, "===============================");

    // In ESP-IDF, we use a loop inside app_main or spawn tasks
    while (1) {
        int64_t time_ms = esp_timer_get_time() / 1000;
        printf("Uptime: %lld ms | System status: STABLE\n", time_ms);

        // We don't use delay(). We use vTaskDelay to let the OS breathe.
        // pdMS_TO_TICKS converts milliseconds to OS "ticks".
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}