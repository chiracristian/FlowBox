#include "button.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "storage.h"

static const char *TAG = "Button_Driver";
static QueueHandle_t button_evt_queue = NULL;
static particle_grid_context_t *p_grid_ctx = NULL;

typedef enum {
    SYS_STATE_SIMULATION,
    SYS_STATE_USB_STORAGE
} system_state_t;

extern volatile system_state_t g_system_state;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(button_evt_queue, &gpio_num, NULL);
}

static void button_task(void *pvParameters)
{
    uint32_t io_num;
    int64_t last_valid_press = 0;
    const int64_t blanking_delay_ms = 300;
    const int64_t long_press_target_ms = 5000;

    while (1) {
        if (xQueueReceive(button_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t current_time = esp_timer_get_time() / 1000;

            if ((current_time - last_valid_press) < blanking_delay_ms) {
                continue;
            }

            vTaskDelay(pdMS_TO_TICKS(50));

            if (gpio_get_level(io_num) == 0) {
                int64_t press_start_time = esp_timer_get_time() / 1000;
                bool is_long_press = false;

                // Loop tracking to calculate active hold duration
                while (gpio_get_level(io_num) == 0) {
                    int64_t hold_duration = (esp_timer_get_time() / 1000) - press_start_time;

                    if (hold_duration >= long_press_target_ms) {
                        is_long_press = true;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }

                last_valid_press = esp_timer_get_time() / 1000;

                if (is_long_press) {
                    if (g_system_state == SYS_STATE_SIMULATION) {
                        g_system_state = SYS_STATE_USB_STORAGE;
                        
                        storage_unmount_local();
                        
                        // Initialize the USB stack only when requested
                        storage_usb_init(); 
                        storage_enable_usb_msc();
                    } else {
                        ESP_LOGW(TAG, "Long press detected during storage session. Restoring simulation loop...");
                        storage_disable_usb_msc();
                        storage_mount_local();
                        
                        g_system_state = SYS_STATE_SIMULATION;
                    }

                    // Spin-lock loop while button is still physically pushed down to block cascade edges
                    while (gpio_get_level(io_num) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                } else {
                    if (g_system_state == SYS_STATE_SIMULATION) {
                        if (p_grid_ctx != NULL) {
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
                    } else if (g_system_state == SYS_STATE_USB_STORAGE) {
                        ESP_LOGI(TAG, "Short press detected during storage session. Restoring simulation loop...");
                        storage_disable_usb_msc();
                        storage_mount_local();
                        
                        g_system_state = SYS_STATE_SIMULATION;
                    }
                }
            }
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
