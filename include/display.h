#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_err.h"
#include "particle_grid.h"

#define LCD_H_RES               320
#define LCD_V_RES               820

/* Color conversion and channel blending macros */
#define PACK_RGB565(r, g, b)        ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3)) 
#define UNPACK_R(rgb)               (((rgb) >> 8) & 0xF8) 
#define UNPACK_G(rgb)               (((rgb) >> 3) & 0xFC) 
#define UNPACK_B(rgb)               (((rgb) << 3) & 0xF8) 

/* Spatial Hash Mixing Primes */
#define HASH_PRIME_X                159757937U
#define HASH_PRIME_Y                717285943U
#define HASH_MIX_STAGE_1            0x7feb352dU
#define HASH_MIX_STAGE_2            0x846ca68bU
#define HASH_SHIFT_SHORT            15
#define HASH_SHIFT_WORD             16

/* Texture Grain Intensities (Maximum Deviations) */
#define NOISE_INTENSITY_AIR         2
#define NOISE_INTENSITY_WALL        14
#define NOISE_INTENSITY_PARTICLE    30

/* Channel Saturation Bounds */
#define CHANNEL_MIN_VAL             0
#define CHANNEL_MAX_VAL             255

/* Dark theme color targets (lux_factor = 0.0) */
#define COLOR_DARK_AIR              0x0000    
#define COLOR_DARK_WALL             0x3186    
#define COLOR_DARK_SAND             0xED60    
#define COLOR_DARK_WATER            0x1B3D    
#define COLOR_DARK_LAVA             0xE100    

/* Light theme color targets (lux_factor = 1.0) */
#define COLOR_LIGHT_AIR             0xF7F4   
#define COLOR_LIGHT_WALL            0x52AA    
#define COLOR_LIGHT_SAND            0xBC44    
#define COLOR_LIGHT_WATER           0x3416    
#define COLOR_LIGHT_LAVA            0xD204    

/**
 * Initializes the ST7701 LCD controller over 3-wire SPI, configures the 14-bit
 * parallel RGB interface with dual DMA framebuffers in PSRAM, and powers on the backlight.
 * @return ESP_OK on successful initialization, or an error code.
 */
esp_err_t display_init(void);

/**
 * Sweeps the cellular automata grid state, scales it 2x2, maps colors dynamically 
 * using channel interpolation between light and dark profiles, and writes raw RGB565 pixels.
 * @param grid_buffer Pointer to the read-only linear array of cells from the simulation context.
 * @param active_rule The active material physics simulation rule.
 * @param theme_val A factor between 0.0 (dark theme) and 1.0 (light theme).
 */
void display_render_grid(const uint8_t *grid_buffer, sim_rule_t active_rule, float theme_val);

#endif // DISPLAY_H
