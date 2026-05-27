#include "button.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "storage.h"
#include <dirent.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "Button_Driver";
static QueueHandle_t button_evt_queue = NULL;
static particle_grid_context_t *p_grid_ctx = NULL;

typedef enum {
    SYS_STATE_SIMULATION,
    SYS_STATE_USB_STORAGE
} system_state_t;

extern volatile system_state_t g_system_state;

static int *g_valid_grid_indices = NULL;
static int g_total_grids_found = 0;
static int g_current_array_slot = 0;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(button_evt_queue, &gpio_num, NULL);
}

static void discover_sd_grids(void)
{
    DIR *dir = opendir("/sdcard");
    if (dir == NULL) {
        ESP_LOGW(TAG, "Could not open /sdcard directory for scanning.");
        g_total_grids_found = 0;
        return;
    }

    struct dirent *entry;
    int capacity = 4;
    g_valid_grid_indices = malloc(capacity * sizeof(int));
    g_total_grids_found = 0;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {
            continue;
        }

        int scanned_idx = -1;
        if (sscanf(entry->d_name, "grid_%d.txt", &scanned_idx) == 1 && scanned_idx >= 0) {
            if (g_total_grids_found >= capacity) {
                capacity *= 2;
                int *new_arr = realloc(g_valid_grid_indices, capacity * sizeof(int));
                if (new_arr == NULL) {
                    ESP_LOGE(TAG, "Out of memory allocating grid lookup maps!");
                    break;
                }
                g_valid_grid_indices = new_arr;
            }
            g_valid_grid_indices[g_total_grids_found] = scanned_idx;
            g_total_grids_found++;
        }
    }
    closedir(dir);

    for (int i = 0; i < g_total_grids_found - 1; i++) {
        for (int j = 0; j < g_total_grids_found - i - 1; j++) {
            if (g_valid_grid_indices[j] > g_valid_grid_indices[j + 1]) {
                int temp = g_valid_grid_indices[j];
                g_valid_grid_indices[j] = g_valid_grid_indices[j + 1];
                g_valid_grid_indices[j + 1] = temp;
            }
        }
    }

    ESP_LOGI(TAG, "Grid discovery complete. Located %d map profiles dynamically on media.", g_total_grids_found);
    for (int i = 0; i < g_total_grids_found; i++) {
        ESP_LOGI(TAG, " -> Found slot [%d]: grid_%d.txt", i, g_valid_grid_indices[i]);
    }
}

static void handle_restore_simulation(const char *log_message)
{
    ESP_LOGI(TAG, "%s", log_message);
    storage_disable_usb_msc();
    storage_mount_local();
    discover_sd_grids();
    g_system_state = SYS_STATE_SIMULATION;
}

static void handle_intermediate_hold(void)
{
    if (g_system_state == SYS_STATE_USB_STORAGE) {
        handle_restore_simulation("Hold threshold reached in storage session. Restoring simulation loop...");
        return;
    }

    if (p_grid_ctx == NULL) {
        return;
    }

    g_system_state = SYS_STATE_USB_STORAGE;
    vTaskDelay(pdMS_TO_TICKS(20));

    if (g_total_grids_found > 0) {
        g_current_array_slot = (g_current_array_slot + 1) % g_total_grids_found;
        int next_target_file_idx = g_valid_grid_indices[g_current_array_slot];
        
        ESP_LOGI(TAG, "Immediate 1.5s threshold met. Rotating to dynamic map index %d...", next_target_file_idx);
        particle_grid_init(p_grid_ctx, next_target_file_idx);
    } else {
        ESP_LOGW(TAG, "1.5s hold passed, but no files found on media. Loading fallback map.");
        particle_grid_init_default(p_grid_ctx);
    }

    g_system_state = SYS_STATE_SIMULATION;
}

static void handle_long_hold(void)
{
    if (g_system_state == SYS_STATE_SIMULATION) {
        ESP_LOGW(TAG, "5s Hold Met! Pausing simulation, switching to USB Storage Mode...");
        g_system_state = SYS_STATE_USB_STORAGE;
        
        storage_unmount_local();
        storage_usb_init(); 
        storage_enable_usb_msc();
    } else {
        handle_restore_simulation("Long press detected during storage session. Restoring simulation loop...");
    }
}

static void handle_short_press(void)
{
    if (g_system_state == SYS_STATE_USB_STORAGE) {
        handle_restore_simulation("Short press detected during storage session. Restoring simulation loop...");
        return;
    }

    if (p_grid_ctx == NULL) {
        return;
    }

    sim_rule_t next_rule = (p_grid_ctx->active_rule_type + 1) % 3;
    particle_grid_set_rule(p_grid_ctx, next_rule);

    const char *material_name = "UNKNOWN";
    switch (next_rule) {
        case SIM_RULE_SAND:  material_name = "SAND";  break;
        case SIM_RULE_WATER: material_name = "WATER"; break;
        case SIM_RULE_LAVA:  material_name = "LAVA";  break;
    }

    ESP_LOGI(TAG, "Button press detected. Material transitioned to: %s", material_name);
}

static void button_task(void *pvParameters)
{
    uint32_t io_num;
    int64_t last_valid_press = 0;
    const int64_t blanking_delay_ms = 300;
    const int64_t map_press_target_ms = 1500;
    const int64_t long_press_target_ms = 5000;

    discover_sd_grids();

    while (1) {
        if (!xQueueReceive(button_evt_queue, &io_num, portMAX_DELAY)) {
            continue;
        }

        int64_t current_time = esp_timer_get_time() / 1000;
        if ((current_time - last_valid_press) < blanking_delay_ms) {
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(io_num) != 0) {
            continue;
        }

        int64_t press_start_time = esp_timer_get_time() / 1000;
        bool map_loaded_during_this_press = false;
        bool usb_triggered_during_this_press = false;

        while (gpio_get_level(io_num) == 0) {
            int64_t hold_duration = (esp_timer_get_time() / 1000) - press_start_time;

            if (hold_duration >= map_press_target_ms && hold_duration < long_press_target_ms && !map_loaded_during_this_press) {
                map_loaded_during_this_press = true;
                handle_intermediate_hold();
            }

            if (hold_duration >= long_press_target_ms && !usb_triggered_during_this_press) {
                usb_triggered_during_this_press = true;
                handle_long_hold();
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }

        last_valid_press = esp_timer_get_time() / 1000;

        if (!map_loaded_during_this_press && !usb_triggered_during_this_press) {
            handle_short_press();
        }
    }
}

esp_err_t button_init(particle_grid_context_t *grid_context)
{
    if (grid_context == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    p_grid_ctx = grid_context;

    button_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    if (button_evt_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << BUTTON_GPIO_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    gpio_install_isr_service(0);

    err = gpio_isr_handler_add(BUTTON_GPIO_NUM, button_isr_handler, (void *)BUTTON_GPIO_NUM);
    if (err != ESP_OK) {
        return err;
    }

    BaseType_t ret = xTaskCreate(button_task, "button_task", 3072, NULL, 4, NULL);
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
