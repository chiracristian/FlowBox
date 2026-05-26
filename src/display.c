#include "display.h"
#include <string.h>
#include "esp_lcd_st7701.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "lcd_bl_pwm_bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static esp_lcd_panel_handle_t panel_handle = NULL;
static SemaphoreHandle_t vsync_sem = NULL;

static uint16_t *fb_buffers[2] = {NULL, NULL};
static int current_fb_idx = 0;

static const st7701_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, (uint8_t []){0x08}, 1, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t []){0xE5, 0x02}, 2, 0},
    {0xC1, (uint8_t []){0x15, 0x0A}, 2, 0},
    {0xC2, (uint8_t []){0x07, 0x02}, 2, 0},
    {0xCC, (uint8_t []){0x10}, 1, 0},
    {0xB0, (uint8_t []){0x00, 0x08, 0x51, 0x0D, 0xCE, 0x06, 0x00, 0x08, 0x08, 0x24, 0x05, 0xD0, 0x0F, 0x6F, 0x36, 0x1F}, 16, 0},
    {0xB1, (uint8_t []){0x00, 0x10, 0x4F, 0x0C, 0x11, 0x05, 0x00, 0x07, 0x07, 0x18, 0x02, 0xD3, 0x11, 0x6E, 0x34, 0x1F}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t []){0x4D}, 1, 0},
    {0xB1, (uint8_t []){0x37}, 1, 0},
    {0xB2, (uint8_t []){0x87}, 1, 0},
    {0xB3, (uint8_t []){0x80}, 1, 0},
    {0xB5, (uint8_t []){0x4A}, 1, 0},
    {0xB7, (uint8_t []){0x85}, 1, 0},
    {0xB8, (uint8_t []){0x21}, 1, 0},
    {0xB9, (uint8_t []){0x00, 0x13}, 2, 0},
    {0xC0, (uint8_t []){0x09}, 1, 0},
    {0xC1, (uint8_t []){0x78}, 1, 0},
    {0xC2, (uint8_t []){0x78}, 1, 0},
    {0xD0, (uint8_t []){0x88}, 1, 0},
    {0xE0, (uint8_t []){0x80, 0x00, 0x02}, 3, 100},
    {0xE1, (uint8_t []){0x0F, 0xA0, 0x00, 0x00, 0x10, 0xA0, 0x00, 0x00, 0x00, 0x60, 0x60}, 11, 0},
    {0xE2, (uint8_t []){0x30, 0x30, 0x60, 0x60, 0x45, 0xA0, 0x00, 0x00, 0x46, 0xA0, 0x00, 0x00, 0x00}, 13, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t []){0x0F, 0x4A, 0xA0, 0xA0, 0x11, 0x4A, 0xA0, 0xA0, 0x13, 0x4A, 0xA0, 0xA0, 0x15, 0x4A, 0xA0, 0xA0}, 16, 0},
    {0xE6, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t []){0x10, 0x4A, 0xA0, 0xA0, 0x12, 0x4A, 0xA0, 0xA0, 0x14, 0x4A, 0xA0, 0xA0, 0x16, 0x4A, 0xA0, 0xA0}, 16, 0},
    {0xEB, (uint8_t []){0x02, 0x00, 0x4E, 0x4E, 0xEE, 0x44, 0x00}, 7, 0},
    {0xED, (uint8_t []){0xFF, 0xFF, 0x04, 0x56, 0x72, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x27, 0x65, 0x40, 0xFF, 0xFF}, 16, 0},
    {0xEF, (uint8_t []){0x08, 0x08, 0x08, 0x40, 0x3F, 0x64}, 6, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xE8, (uint8_t []){0x00, 0x0E}, 2, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xE8, (uint8_t []){0x00, 0x0C}, 2, 10},
    {0xE8, (uint8_t []){0x00, 0x00}, 2, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x29, (uint8_t []){0x00}, 0, 20},
};

static bool display_on_vsync_handler(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    if (vsync_sem != NULL) {
        xSemaphoreGiveFromISR(vsync_sem, &high_task_awoken);
    }
    return high_task_awoken == pdTRUE;
}

static inline uint16_t blend_channel_profile(uint16_t c_dark, uint16_t c_light, float factor)
{
    uint8_t r_d = UNPACK_R(c_dark);
    uint8_t g_d = UNPACK_G(c_dark);
    uint8_t b_d = UNPACK_B(c_dark);

    uint8_t r_l = UNPACK_R(c_light);
    uint8_t g_l = UNPACK_G(c_light);
    uint8_t b_l = UNPACK_B(c_light);

    uint8_t r_mixed = (uint8_t)(r_d + (r_l - r_d) * factor);
    uint8_t g_mixed = (uint8_t)(g_d + (g_l - g_d) * factor);
    uint8_t b_mixed = (uint8_t)(b_d + (b_l - b_d) * factor);

    return PACK_RGB565(r_mixed, g_mixed, b_mixed);
}

esp_err_t display_init(void)
{
    lcd_bl_pwm_bsp_init(LCD_PWM_MODE_255);

    vsync_sem = xSemaphoreCreateBinary();
    if (vsync_sem == NULL) return ESP_ERR_NO_MEM;

    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,
        .cs_gpio_num = GPIO_NUM_0,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = GPIO_NUM_2,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = GPIO_NUM_1,
        .io_expander = NULL,
    };

    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_err_t ret = esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle);
    if (ret != ESP_OK) return ret;

    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .bounce_buffer_size_px = 10 * LCD_H_RES,  
        .num_fbs = 2,
        .data_width = 16,
        .de_gpio_num = GPIO_NUM_40,
        .pclk_gpio_num = GPIO_NUM_41,
        .vsync_gpio_num = GPIO_NUM_39,
        .hsync_gpio_num = GPIO_NUM_38,
        .data_gpio_nums = {
            GPIO_NUM_21, GPIO_NUM_5,  GPIO_NUM_45, GPIO_NUM_48, GPIO_NUM_47, 
            GPIO_NUM_14, GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_10, GPIO_NUM_9, 
            GPIO_NUM_17, GPIO_NUM_46, GPIO_NUM_3,  GPIO_NUM_8,  GPIO_NUM_18, 
        },
        .timings = {
            .pclk_hz = 14 * 1000 * 1000,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_back_porch = 30,
            .hsync_front_porch = 30,
            .hsync_pulse_width = 6,
            .vsync_back_porch = 20,
            .vsync_front_porch = 20,
            .vsync_pulse_width = 40,
        },
        .flags = {
            .fb_in_psram = true,
        },
    };

    st7701_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t),
        .flags = {
            .mirror_by_cmd = 1,
            .enable_io_multiplex = 0,
        },
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_16,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };

    ret = esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) return ret;

    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) return ret;

    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) return ret;

    void *fb_0 = NULL;
    void *fb_1 = NULL;
    esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &fb_0, &fb_1);
    fb_buffers[0] = (uint16_t *)fb_0;
    fb_buffers[1] = (uint16_t *)fb_1;

    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = display_on_vsync_handler,
    };
    esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL);

    setUpduty(LCD_PWM_MODE_255);
    return ESP_OK;
}

void display_render_grid(const uint8_t *grid_buffer, sim_rule_t active_rule, float theme_val)
{
    if (panel_handle == NULL || grid_buffer == NULL) return;

    if (vsync_sem != NULL) {
        xSemaphoreTake(vsync_sem, portMAX_DELAY);
    }

    uint16_t *fb = fb_buffers[current_fb_idx];
    if (fb == NULL) return;

    uint16_t dark_particle = COLOR_DARK_SAND;
    uint16_t light_particle = COLOR_LIGHT_SAND;

    switch (active_rule) {
        case SIM_RULE_WATER:
            dark_particle = COLOR_DARK_WATER;
            light_particle = COLOR_LIGHT_WATER;
            break;
        case SIM_RULE_LAVA:
            dark_particle = COLOR_DARK_LAVA;
            light_particle = COLOR_LIGHT_LAVA;
            break;
        case SIM_RULE_SAND:
        default:
            dark_particle = COLOR_DARK_SAND;
            light_particle = COLOR_LIGHT_SAND;
            break;
    }

    uint16_t air_base    = blend_channel_profile(COLOR_DARK_AIR, COLOR_LIGHT_AIR, theme_val);
    uint16_t wall_base   = blend_channel_profile(COLOR_DARK_WALL, COLOR_LIGHT_WALL, theme_val);
    uint16_t part_base   = blend_channel_profile(dark_particle, light_particle, theme_val);

    uint8_t air_r  = UNPACK_R(air_base),  air_g  = UNPACK_G(air_base),  air_b  = UNPACK_B(air_base);
    uint8_t wall_r = UNPACK_R(wall_base), wall_g = UNPACK_G(wall_base), wall_b = UNPACK_B(wall_base);
    uint8_t part_r = UNPACK_R(part_base), part_g = UNPACK_G(part_base), part_b = UNPACK_B(part_base);

    for (int y = 0; y < GRID_HEIGHT; y++) {
        int source_cell_row = y * GRID_WIDTH;
        uint16_t target_row_buffer[GRID_WIDTH * 2];

        for (int x = 0; x < GRID_WIDTH; x++) {
            uint8_t cell = grid_buffer[source_cell_row + x];
            uint8_t r = 0, g = 0, b = 0;
            int max_noise = 0;

            switch (cell) {
                case CELL_TYPE_WALL:
                    r = wall_r; g = wall_g; b = wall_b;
                    max_noise = NOISE_INTENSITY_WALL; 
                    break;
                case CELL_TYPE_PARTICLE:
                    r = part_r; g = part_g; b = part_b;
                    max_noise = NOISE_INTENSITY_PARTICLE; 
                    break;
                case CELL_TYPE_AIR:
                default:
                    r = air_r; g = air_g; b = air_b;
                    max_noise = NOISE_INTENSITY_AIR; 
                    break;
            }

            uint32_t hash = ((uint32_t)x * HASH_PRIME_X) ^ ((uint32_t)y * HASH_PRIME_Y);
            hash = (hash ^ (hash >> HASH_SHIFT_WORD)) * HASH_MIX_STAGE_1;
            hash = (hash ^ (hash >> HASH_SHIFT_SHORT)) * HASH_MIX_STAGE_2;
            hash = hash ^ (hash >> HASH_SHIFT_WORD);

            int noise = ((int)(hash % (max_noise * 2 + 1))) - max_noise;

            int mixed_r = r + noise;
            int mixed_g = g + noise;
            int mixed_b = b + noise;

            if (mixed_r < CHANNEL_MIN_VAL) mixed_r = CHANNEL_MIN_VAL; else if (mixed_r > CHANNEL_MAX_VAL) mixed_r = CHANNEL_MAX_VAL;
            if (mixed_g < CHANNEL_MIN_VAL) mixed_g = CHANNEL_MIN_VAL; else if (mixed_g > CHANNEL_MAX_VAL) mixed_g = CHANNEL_MAX_VAL;
            if (mixed_b < CHANNEL_MIN_VAL) mixed_b = CHANNEL_MIN_VAL; else if (mixed_b > CHANNEL_MAX_VAL) mixed_b = CHANNEL_MAX_VAL;

            uint16_t final_color = PACK_RGB565((uint8_t)mixed_r, (uint8_t)mixed_g, (uint8_t)mixed_b);

            target_row_buffer[x * 2]     = final_color;
            target_row_buffer[x * 2 + 1] = final_color;
        }

        int target_pixel_row_0 = (y * 2) * LCD_H_RES;
        int target_pixel_row_1 = ((y * 2) + 1) * LCD_H_RES;

        memcpy(&fb[target_pixel_row_0], target_row_buffer, sizeof(target_row_buffer));
        memcpy(&fb[target_pixel_row_1], target_row_buffer, sizeof(target_row_buffer));
    }

    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, fb);

    current_fb_idx = !current_fb_idx;
}
