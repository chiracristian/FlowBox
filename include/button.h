#ifndef BUTTON_H
#define BUTTON_H

#include "esp_err.h"
#include "particle_grid.h"

#define BUTTON_GPIO_NUM      GPIO_NUM_0

/**
 * Initializes the button hardware line with pull-up resistor settings,
 * hooks up the edge-triggered interrupt service routine, and spawns
 * the debouncing task.
 */
esp_err_t button_init(particle_grid_context_t *grid_context);

#endif // BUTTON_H
