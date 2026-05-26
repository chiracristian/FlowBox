#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_err.h"
#include "particle_grid.h"

#define LCD_H_RES               320
#define LCD_V_RES               820
#define LCD_REFRESH_RATE        60

/**
 * Initializes the ST7701 LCD controller over 3-wire SPI, configures the 16-bit
 * parallel RGB interface with dual DMA framebuffers in PSRAM, and powers on the backlight.
 * @return ESP_OK on successful initialization, or an error code.
 */
esp_err_t display_init(void);

/**
 * Sweeps the cellular automata grid state, scales it 2x2, and writes raw RGB565 
 * pixels directly into the next available DMA hardware framebuffer.
 * @param grid_buffer Pointer to the read-only linear array of cells from the simulation context.
 */
void display_render_grid(const uint8_t *grid_buffer);

#endif // DISPLAY_H