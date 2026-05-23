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

    ESP_LOGI(TAG, "Initializing hardware I2C bus lines (SDA: GPIO %d, SCL: GPIO %d)...", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    esp_err_t err = i2c_bus_master_init();
    if (err != ESP_OK) {
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Probing network lines for attached target peripherals...");
    err = i2c_bus_probe_peripherals();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WARNING: Peripheral hardware matrix reported a status failure condition.");
    }

    ESP_LOGI(TAG, "Initializing critical inertial registration parameters inside internal registers...");
    err = i2c_sensor_init_imu();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CRITICAL: Detected IMU could not be woken up from power-down state.");
        return;
    }
    ESP_LOGI(TAG, "===============================================");

    vTaskDelay(pdMS_TO_TICKS(1000));

    i2c_imu_data_t current_motion_frame;
    float current_ambient_lux = 0.0f;
    uint16_t raw_ch0 = 0;
    uint16_t raw_ch1 = 0;

    while (1) {
        int64_t time_ms = esp_timer_get_time() / 1000;

        esp_err_t imu_status = i2c_sensor_read_imu(&current_motion_frame);
        esp_err_t lux_status = i2c_sensor_read_light(&current_ambient_lux, &raw_ch0, &raw_ch1);

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
            printf("Light: [%.2f lux] (Raw CH0: %u, CH1: %u)\n", 
                   current_ambient_lux, raw_ch0, raw_ch1);
        } else {
            printf("Light: [ READ ERROR ]\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
