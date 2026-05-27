#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c.h"
#include "particle_grid.h"
#include "display.h"
#include "button.h"

#include "storage.h"

typedef enum {
    SYS_STATE_SIMULATION,
    SYS_STATE_USB_STORAGE
} system_state_t;

// Set volatile to guarantee safe access across tasks
volatile system_state_t g_system_state = SYS_STATE_SIMULATION;

static const char *TAG = "FlowBox_Core";

static particle_grid_context_t grid_ctx;
static i2c_imu_data_t sim_motion_frame;
static i2c_imu_data_t log_motion_frame;

static volatile float current_theme_param = 0.0f;
#define LIGHT_THEME_LUX 35.f

static void simulation_task(void *pvParameters)
{
    while (1) {
        if (g_system_state == SYS_STATE_SIMULATION) {
            if (i2c_sensor_read_imu(&sim_motion_frame) == ESP_OK) {
                particle_grid_step(&grid_ctx, sim_motion_frame.acc_x, sim_motion_frame.acc_y);
            }

            const uint8_t *render_source = particle_grid_get_render_buffer(&grid_ctx);
            display_render_grid(render_source, grid_ctx.active_rule_type, current_theme_param);
        } else {
            // Simulation is paused during USB Storage Mode
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void app_main(void) 
{
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

    // Set up storage tracks before display and simulation loops begin;
    storage_mount_local();

    esp_err_t disp_err = display_init();
    if (disp_err != ESP_OK) {
        ESP_LOGE(TAG, "CRITICAL: Failed to initialize ST7701/RGB display controller!");
        return;
    }

    particle_grid_init(&grid_ctx);
    particle_grid_spawn_triangle(&grid_ctx, 30, 40, 20, 30);

    button_init(&grid_ctx);

    xTaskCreatePinnedToCore(simulation_task, "sim_task", 4096, NULL, 5, NULL, 1);

    vTaskDelay(pdMS_TO_TICKS(1000));

    float current_ambient_lux = 0.0f;
    uint16_t raw_ch0 = 0;
    uint16_t raw_ch1 = 0;

    while (1) {
        int64_t time_ms = esp_timer_get_time() / 1000;

        esp_err_t imu_status = i2c_sensor_read_imu(&log_motion_frame);
        esp_err_t lux_status = i2c_sensor_read_light(&current_ambient_lux, &raw_ch0, &raw_ch1);

        if (lux_status == ESP_OK) {
            float param = current_ambient_lux / LIGHT_THEME_LUX;
            if (param > 1.0f) param = 1.0f;
            if (param < 0.0f) param = 0.0f;
            current_theme_param = param;
        }

        printf("[%lld ms] ", time_ms);
        
        if (imu_status == ESP_OK) {
            printf("IMU Accel: [%+.2fg, %+.2fg, %+.2fg] | ", 
                   log_motion_frame.acc_x, 
                   log_motion_frame.acc_y, 
                   log_motion_frame.acc_z);
        } else {
            printf("IMU Accel: [ READ ERROR ] | ");
        }

        if (lux_status == ESP_OK) {
            printf("Light: [%.2f lux] (Theme: %.2f) (Raw CH0: %u, CH1: %u)\n", 
                   current_ambient_lux, current_theme_param, raw_ch0, raw_ch1);
        } else {
            printf("Light: [ READ ERROR ]\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
