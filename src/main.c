#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c.h"

static const char *TAG = "FlowBox_Core";

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "      FLOWBOX HARDWARE SUBSYSTEM INITIATION    ");
    ESP_LOGI(TAG, "===============================================");

    // Initialize the low-level shared I2C master driver block
    ESP_LOGI(TAG, "Initializing hardware I2C bus lines (SDA: GPIO %d, SCL: GPIO %d)...", 
        I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    esp_err_t err = i2c_bus_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CRITICAL: Failed to install active I2C kernel driver (Code: %s)", esp_err_to_name(err));
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Perform the address diagnostic probe sweep across the shared lines
    ESP_LOGI(TAG, "Probing network lines for attached target peripherals...");
    err = i2c_bus_probe_peripherals();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SUCCESS: Shared I2C peripheral hardware bus is online and responsive.");
        
        // Wake the chip up now that we know it is listening on the bus lines
        err = i2c_sensor_init_imu();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "CRITICAL: Detected IMU could not be woken up from power-down state.");
            return;
        }
    }
    ESP_LOGI(TAG, "===============================================");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Instantiating local data containers for periodic polling transactions
    i2c_imu_data_t current_motion_frame;
    float current_ambient_lux = 0.0f;

    while (1) {
        int64_t time_ms = esp_timer_get_time() / 1000;

        // Perform transactional data updates from our hardware abstraction modules
        esp_err_t imu_status = i2c_sensor_read_imu(&current_motion_frame);
        esp_err_t lux_status = i2c_sensor_read_light(&current_ambient_lux);

        // Print aggregated diagnostics cleanly across standard output lines
        printf("[%lld ms] ", time_ms);
        
        if (imu_status == ESP_OK) {
            printf("IMU Accel: [%+.2fg, %+.2fg, %+.2fg] | ", 
                   current_motion_frame.acc_x, 
                   current_motion_frame.acc_y, 
                   current_motion_frame.acc_z);
        } else {
            printf("IMU Accel: [ READ ERROR ] | ");
        }

        if (lux_status == ESP_OK) {
            printf("Light: %6.1f Lux\n", current_ambient_lux);
        } else {
            printf("Light: [ READ ERROR ]\n");
        }

        // Relinquish processor execution execution blocks to give FreeRTOS kernel breathing space
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}