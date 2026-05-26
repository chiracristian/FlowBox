#include "particle_grid.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Thick Map Layout Geometry Configurations (Min 2-4 units thick to prevent tunneling) */
#define FUNNEL_TOP_Y            70
#define FUNNEL_BOTTOM_Y         100
#define FUNNEL_OPENING_HALF     8
#define WALL_THICKNESS          2

#define PEG_ROW_1_Y             150
#define PEG_ROW_2_Y             190
#define PEG_SPACING_X           24
#define PEG_OFFSET_X            12
#define PEG_SIZE                3 // 3x3 block pegs provide an airtight collision footprint

#define BASIN_LEFT_X            35
#define BASIN_RIGHT_X           125
#define BASIN_Y                 270
#define BASIN_HEIGHT            30
#define BASIN_THICKNESS         3

#define SEPARATOR_X             80
#define SEPARATOR_START_Y       320
#define SEPARATOR_END_Y         390
#define SEPARATOR_THICKNESS     4

static void update_sand_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y);
static void update_water_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y);
static void update_lava_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y);

void particle_grid_init(particle_grid_context_t *ctx)
{
    if (ctx == NULL) return;

    ctx->current_grid = ctx->grid_buffer_0;
    ctx->next_grid = ctx->grid_buffer_1;

    memset(ctx->current_grid, CELL_TYPE_AIR, GRID_SIZE);

    /* Construct outer bounding container walls (2 units thick for airtight seal) */
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (x < WALL_THICKNESS || x >= (GRID_WIDTH - WALL_THICKNESS) || 
                y < WALL_THICKNESS || y >= (GRID_HEIGHT - WALL_THICKNESS)) {
                ctx->current_grid[CELL(x, y)] = CELL_TYPE_WALL;
            }
        }
    }

    /* 1. Upper Hourglass Funnel (Thickened horizontally to 2 units) */
    int mid_x = GRID_WIDTH / 2;
    for (int y = FUNNEL_TOP_Y; y < FUNNEL_BOTTOM_Y; y++) {
        int delta_y = y - FUNNEL_TOP_Y;
        int wall_gap = FUNNEL_OPENING_HALF + ((FUNNEL_BOTTOM_Y - FUNNEL_TOP_Y - delta_y) / 2);
        
        int left_wall_x = mid_x - wall_gap;
        int right_wall_x = mid_x + wall_gap;

        for (int t = 0; t < WALL_THICKNESS; t++) {
            if ((left_wall_x - t) > 0) {
                ctx->current_grid[CELL(left_wall_x - t, y)] = CELL_TYPE_WALL;
            }
            if ((right_wall_x + t) < GRID_WIDTH - 1) {
                ctx->current_grid[CELL(right_wall_x + t, y)] = CELL_TYPE_WALL;
            }
        }
    }

    /* 2. Mid-level Pachinko Peg Matrix (Row 1 - Increased to 3x3 structures) */
    for (int x = PEG_SPACING_X; x < GRID_WIDTH - WALL_THICKNESS; x += PEG_SPACING_X) {
        for (int py = 0; py < PEG_SIZE; py++) {
            for (int px = 0; px < PEG_SIZE; px++) {
                ctx->current_grid[CELL(x + px, PEG_ROW_1_Y + py)] = CELL_TYPE_WALL;
            }
        }
    }

    /* Mid-level Pachinko Peg Matrix (Row 2 - Staggered Offset 3x3 structures) */
    for (int x = PEG_OFFSET_X; x < GRID_WIDTH - WALL_THICKNESS; x += PEG_SPACING_X) {
        for (int py = 0; py < PEG_SIZE; py++) {
            for (int px = 0; px < PEG_SIZE; px++) {
                ctx->current_grid[CELL(x + px, PEG_ROW_2_Y + py)] = CELL_TYPE_WALL;
            }
        }
    }

    /* 3. Central Storage Basin (Thickened base and vertical columns to 3 units) */
    for (int x = BASIN_LEFT_X; x <= BASIN_RIGHT_X; x++) {
        for (int t = 0; t < BASIN_THICKNESS; t++) {
            ctx->current_grid[CELL(x, BASIN_Y + t)] = CELL_TYPE_WALL;
        }
    }
    for (int h = 0; h < BASIN_HEIGHT; h++) {
        for (int t = 0; t < BASIN_THICKNESS; t++) {
            ctx->current_grid[CELL(BASIN_LEFT_X - t, BASIN_Y - h)] = CELL_TYPE_WALL;
            ctx->current_grid[CELL(BASIN_RIGHT_X + t, BASIN_Y - h)] = CELL_TYPE_WALL;
        }
    }

    /* 4. Lower Binary Splitter Wall (Thickened to 4 units width) */
    for (int y = SEPARATOR_START_Y; y < SEPARATOR_END_Y; y++) {
        for (int t = 0; t < SEPARATOR_THICKNESS; t++) {
            ctx->current_grid[CELL(SEPARATOR_X - (SEPARATOR_THICKNESS / 2) + t, y)] = CELL_TYPE_WALL;
        }
    }

    memcpy(ctx->next_grid, ctx->current_grid, GRID_SIZE);

    particle_grid_set_rule(ctx, SIM_RULE_SAND);
}

void particle_grid_spawn_triangle(particle_grid_context_t *ctx, uint16_t x, uint16_t y, uint16_t length, uint16_t height)
{
    if (ctx == NULL) return;

    for (uint16_t row = 0; row < height; row++) {
        uint16_t row_width = (length * (height - row)) / height;
        if (row_width == 0) row_width = 1;

        uint16_t row_start_x = x + (length - row_width) / 2;

        for (uint16_t col = 0; col < row_width; col++) {
            uint16_t target_x = row_start_x + col;
            uint16_t target_y = y - row; 

            if (target_x < GRID_WIDTH && target_y < GRID_HEIGHT) {
                int index = CELL(target_x, target_y);
                if (ctx->current_grid[index] == CELL_TYPE_AIR) {
                    ctx->current_grid[index] = CELL_TYPE_PARTICLE;
                    ctx->next_grid[index] = CELL_TYPE_PARTICLE;
                }
            }
        }
    }
}

void particle_grid_set_rule(particle_grid_context_t *ctx, sim_rule_t target_rule)
{
    if (ctx == NULL) return;

    ctx->active_rule_type = target_rule;

    switch (target_rule) {
        case SIM_RULE_WATER:
            ctx->update_particle = update_water_physics;
            break;
        case SIM_RULE_LAVA:
            ctx->update_particle = update_lava_physics;
            break;
        case SIM_RULE_SAND:
        default:
            ctx->update_particle = update_sand_physics;
            break;
    }
}

const uint8_t* particle_grid_get_render_buffer(const particle_grid_context_t *ctx)
{
    if (ctx == NULL) return NULL;
    return ctx->current_grid;
}

void particle_grid_step(particle_grid_context_t *ctx, float acc_x, float acc_y)
{
    if (ctx == NULL || ctx->update_particle == NULL) return;

    for (int i = 0; i < GRID_SIZE; i++) {
        if (ctx->current_grid[i] == CELL_TYPE_WALL) {
            ctx->next_grid[i] = CELL_TYPE_WALL;
        } else {
            ctx->next_grid[i] = CELL_TYPE_AIR;
        }
    }

    for (int y = 1; y < GRID_HEIGHT - 1; y++) {
        for (int x = 1; x < GRID_WIDTH - 1; x++) {
            if (ctx->current_grid[CELL(x, y)] == CELL_TYPE_PARTICLE) {
                ctx->update_particle(ctx, x, y, acc_x, acc_y);
            }
        }
    }

    uint8_t *temp = ctx->current_grid;
    ctx->current_grid = ctx->next_grid;
    ctx->next_grid = temp;
}

// ============================================================================
// CORE SIMULATION KERNELS (PHYSICS FUNCTION POINTERS)
// ============================================================================

static void update_sand_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    // Container Mode check: if resting flat, maintain current position with zero drift
    if (fabsf(acc_x) <= ACCELERATION_THRESHOLD && fabsf(acc_y) <= ACCELERATION_THRESHOLD) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    int dy = (acc_y > ACCELERATION_THRESHOLD) ? 1 : ((acc_y < -ACCELERATION_THRESHOLD) ? -1 : 0);
    int dx = (acc_x > ACCELERATION_THRESHOLD) ? 1 : ((acc_x < -ACCELERATION_THRESHOLD) ? -1 : 0);

    if (ctx->current_grid[CELL(x + dx, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + dx, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + dx, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    if (dy != 0 && ctx->current_grid[CELL(x, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    int side_step = (rand() % 2 == 0) ? 1 : -1;
    
    if (ctx->current_grid[CELL(x + side_step, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + side_step, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + side_step, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }
    
    if (ctx->current_grid[CELL(x - side_step, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x - side_step, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x - side_step, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
}

static void update_water_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    if (fabsf(acc_x) <= ACCELERATION_THRESHOLD && fabsf(acc_y) <= ACCELERATION_THRESHOLD) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    int dy = (acc_y > ACCELERATION_THRESHOLD) ? 1 : ((acc_y < -ACCELERATION_THRESHOLD) ? -1 : 0);
    int dx = (acc_x > ACCELERATION_THRESHOLD) ? 1 : ((acc_x < -ACCELERATION_THRESHOLD) ? -1 : 0);

    if (ctx->current_grid[CELL(x + dx, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + dx, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + dx, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    if (dy != 0 && ctx->current_grid[CELL(x, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    int side_step = (rand() % 2 == 0) ? 1 : -1;
    if (ctx->current_grid[CELL(x + side_step, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + side_step, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + side_step, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    if (ctx->current_grid[CELL(x + side_step, y)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + side_step, y)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + side_step, y)] = CELL_TYPE_PARTICLE;
        return;
    }
    
    if (ctx->current_grid[CELL(x - side_step, y)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x - side_step, y)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x - side_step, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
}

static void update_lava_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    if (fabsf(acc_x) <= ACCELERATION_THRESHOLD && fabsf(acc_y) <= ACCELERATION_THRESHOLD) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    static uint8_t viscosity_divider = 0;
    if (x == 1 && y == 1) {
        viscosity_divider++;
    }

    if ((viscosity_divider % 6) != 0) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    int dy = (acc_y > ACCELERATION_THRESHOLD) ? 1 : ((acc_y < -ACCELERATION_THRESHOLD) ? -1 : 0);
    int dx = (acc_x > ACCELERATION_THRESHOLD) ? 1 : ((acc_x < -ACCELERATION_THRESHOLD) ? -1 : 0);

    // Standard heavy fluid checks
    if (ctx->current_grid[CELL(x + dx, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + dx, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + dx, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    if (dy != 0 && ctx->current_grid[CELL(x, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    int side_step = (rand() % 2 == 0) ? 1 : -1;
    if (ctx->current_grid[CELL(x + side_step, y + dy)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + side_step, y + dy)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + side_step, y + dy)] = CELL_TYPE_PARTICLE;
        return;
    }

    if (ctx->current_grid[CELL(x + side_step, y)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x + side_step, y)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x + side_step, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    if (ctx->current_grid[CELL(x - side_step, y)] == CELL_TYPE_AIR &&
        ctx->next_grid[CELL(x - side_step, y)] == CELL_TYPE_AIR) 
    {
        ctx->next_grid[CELL(x - side_step, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    // CHARMING EXCURSION: Viscous Magma Creep Rule
    // If blocked on all down/lateral moves, introduce a 5% chaotic chance to surge upwards or climb walls
    if (rand() % 100 < 5) {
        int climb_y = y - dy; // Push opposite to gravity vector direction
        int climb_x = x + side_step;

        if (climb_y > 0 && climb_y < GRID_HEIGHT - 1 && climb_x > 0 && climb_x < GRID_WIDTH - 1) {
            if (ctx->current_grid[CELL(climb_x, climb_y)] == CELL_TYPE_AIR &&
                ctx->next_grid[CELL(climb_x, climb_y)] == CELL_TYPE_AIR) 
            {
                ctx->next_grid[CELL(climb_x, climb_y)] = CELL_TYPE_PARTICLE;
                return;
            }
        }
    }

    ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
}
