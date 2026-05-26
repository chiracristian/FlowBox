#ifndef PARTICLE_GRID_H
#define PARTICLE_GRID_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define GRID_WIDTH                  80
#define GRID_HEIGHT                 205
#define GRID_SIZE                   (GRID_WIDTH * GRID_HEIGHT)

// Macro to linearize 2D coordinates into a 1D array index
#define CELL(x, y)                  ((y) * GRID_WIDTH + (x))

#define CELL_TYPE_AIR               0x00
#define CELL_TYPE_WALL              0x01
#define CELL_TYPE_PARTICLE          0x02

#define ACCELERATION_THRESHOLD      0.03f

typedef enum {
    SIM_RULE_SAND = 0,
    SIM_RULE_WATER,
    SIM_RULE_LAVA
} sim_rule_t;

typedef struct particle_grid_context particle_grid_context_t;

typedef void (*physics_update_rule_fn)(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y);

struct particle_grid_context {
    uint8_t grid_buffer_0[GRID_SIZE];
    uint8_t grid_buffer_1[GRID_SIZE];

    uint8_t *current_grid;         // Swappable pointer targeting buffer 0 or 1
    uint8_t *next_grid;            // Swappable pointer targeting buffer 0 or 1

    sim_rule_t active_rule_type;   
    physics_update_rule_fn update_particle; 
};

/**
 * @brief Initializes a pre-allocated grid context instance.
 * Fills the bounding perimeter edges with CELL_TYPE_WALL, injects static internal 
 * obstacle walls in the center zone, and leaves the remaining space as air.
 * @param ctx Pointer to a context structure allocated by the caller.
 */
void particle_grid_init(particle_grid_context_t *ctx);

/**
 * @brief Spawns a precise cluster of particles arranged in an upward-pointing isosceles triangle. 
 * @param ctx Pointer to the active grid context structure.
 * @param x The horizontal coordinate of the bottom-left corner of the triangle.
 * @param y The vertical coordinate of the bottom-left corner of the triangle.
 * @param length The baseline width of the triangle.
 * @param height The vertical height of the triangle from base to apex.
 */
void particle_grid_spawn_triangle(particle_grid_context_t *ctx, uint16_t x, uint16_t y, uint16_t length, uint16_t height);

/**
 * @brief Executes one complete synchronous simulation frame sweep using the attached update rules.
 * @param ctx Pointer to the active grid context structure.
 * @param acc_x Dynamic filtering X axis offset reading from the IMU.
 * @param acc_y Dynamic filtering Y axis offset reading from the IMU.
 */
void particle_grid_step(particle_grid_context_t *ctx, float acc_x, float acc_y);

/**
 * @brief Remaps the active physics function pointer and update rule identifier.
 * @param ctx Pointer to the active grid context structure.
 * @param target_rule The environmental material profile choice to attach.
 */
void particle_grid_set_rule(particle_grid_context_t *ctx, sim_rule_t target_rule);

/**
 * @brief Fetches the current active buffer configuration reference for rendering operations.
 * @param ctx Pointer to the active grid context structure.
 * @return Const pointer to the read-only linearized current_grid array of size GRID_SIZE.
 */
const uint8_t* particle_grid_get_render_buffer(const particle_grid_context_t *ctx);

#endif // PARTICLE_GRID_H
