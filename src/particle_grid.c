#include "particle_grid.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Map Layout Geometry Scaled for 80x205 Grid */
#define FUNNEL_TOP_Y            35
#define FUNNEL_BOTTOM_Y         50
#define FUNNEL_OPENING_HALF     4
#define WALL_THICKNESS          2
#define BORDER_WALL_THICKNESS   1

#define PEG_ROW_1_Y             75
#define PEG_ROW_2_Y             95
#define PEG_SPACING_X           12
#define PEG_OFFSET_X            6
#define PEG_SIZE                2 

#define BASIN_LEFT_X            17
#define BASIN_RIGHT_X           62
#define BASIN_Y                 135
#define BASIN_HEIGHT            15
#define BASIN_THICKNESS         2

#define SEPARATOR_X             40
#define SEPARATOR_START_Y       160
#define SEPARATOR_END_Y         195
#define SEPARATOR_THICKNESS     2

/* Vastly increased substepping limits for smoother ASMR fluid physics */
#define WATER_SUBSTEPS          5 
#define SAND_SUBSTEPS           3 

static void update_sand_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y);
static void update_water_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y);
static void update_lava_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y);

void particle_grid_init(particle_grid_context_t *ctx)
{
    if (ctx == NULL) return;

    ctx->current_grid = ctx->grid_buffer_0;
    ctx->next_grid = ctx->grid_buffer_1;

    memset(ctx->current_grid, CELL_TYPE_AIR, GRID_SIZE);

    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (x < BORDER_WALL_THICKNESS || x >= (GRID_WIDTH - BORDER_WALL_THICKNESS) || 
                y < BORDER_WALL_THICKNESS || y >= (GRID_HEIGHT - BORDER_WALL_THICKNESS)) {
                ctx->current_grid[CELL(x, y)] = CELL_TYPE_WALL;
            }
        }
    }

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

    for (int x = PEG_SPACING_X; x < GRID_WIDTH - WALL_THICKNESS; x += PEG_SPACING_X) {
        for (int py = 0; py < PEG_SIZE; py++) {
            for (int px = 0; px < PEG_SIZE; px++) {
                ctx->current_grid[CELL(x + px, PEG_ROW_1_Y + py)] = CELL_TYPE_WALL;
            }
        }
    }

    for (int x = PEG_OFFSET_X; x < GRID_WIDTH - WALL_THICKNESS; x += PEG_SPACING_X) {
        for (int py = 0; py < PEG_SIZE; py++) {
            for (int px = 0; px < PEG_SIZE; px++) {
                ctx->current_grid[CELL(x + px, PEG_ROW_2_Y + py)] = CELL_TYPE_WALL;
            }
        }
    }

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

static void execute_physics_substep(particle_grid_context_t *ctx, float acc_x, float acc_y)
{
    /* 1. Copy the snapshot to work on it in-place */
    memcpy(ctx->next_grid, ctx->current_grid, GRID_SIZE);

    /* 2. Dynamically align the traversal order with the gravity vector to prevent "boiling" gaps */
    int start_y = 1, end_y = GRID_HEIGHT - 1, step_y = 1;
    if (acc_y > 0.0f) {
        start_y = GRID_HEIGHT - 2;
        end_y = 0;
        step_y = -1;
    }

    int start_x = 1, end_x = GRID_WIDTH - 1, step_x = 1;
    if (acc_x > 0.0f) {
        start_x = GRID_WIDTH - 2;
        end_x = 0;
        step_x = -1;
    }

    for (int y = start_y; y != end_y; y += step_y) {
        for (int x = start_x; x != end_x; x += step_x) {
            /* Only process if it was a particle at the start, AND hasn't already moved this pass */
            if (ctx->current_grid[CELL(x, y)] == CELL_TYPE_PARTICLE && 
                ctx->next_grid[CELL(x, y)] == CELL_TYPE_PARTICLE) 
            {
                ctx->update_particle(ctx, x, y, acc_x, acc_y);
            }
        }
    }

    /* 3. Commit the in-place modifications to the active buffer */
    uint8_t *temp = ctx->current_grid;
    ctx->current_grid = ctx->next_grid;
    ctx->next_grid = temp;
}

void particle_grid_step(particle_grid_context_t *ctx, float acc_x, float acc_y)
{
    if (ctx == NULL || ctx->update_particle == NULL) return;

    int steps = 1;
    if (ctx->active_rule_type == SIM_RULE_WATER) {
        steps = WATER_SUBSTEPS;
    } else if (ctx->active_rule_type == SIM_RULE_SAND) {
        steps = SAND_SUBSTEPS;
    }

    for (int s = 0; s < steps; s++) {
        execute_physics_substep(ctx, acc_x, acc_y);
    }
}

/* Fast In-Place Swapping Macro */
#define TRY_MOVE(dx, dy) \
    if (ctx->next_grid[CELL(x + (dx), y + (dy))] == CELL_TYPE_AIR) { \
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_AIR; \
        ctx->next_grid[CELL(x + (dx), y + (dy))] = CELL_TYPE_PARTICLE; \
        return; \
    }

/* Bounded Lateral Sprinting Macro */
#define TRY_SPRINT(dx, dy, block_flag) \
    if (x + (dx) > 0 && x + (dx) < GRID_WIDTH - 1 && y + (dy) > 0 && y + (dy) < GRID_HEIGHT - 1) { \
        if (ctx->next_grid[CELL(x + (dx), y + (dy))] == CELL_TYPE_AIR) { \
            ctx->next_grid[CELL(x, y)] = CELL_TYPE_AIR; \
            ctx->next_grid[CELL(x + (dx), y + (dy))] = CELL_TYPE_PARTICLE; \
            return; \
        } else { block_flag = true; } \
    } else { block_flag = true; }

static void update_sand_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    float abs_x = fabsf(acc_x);
    float abs_y = fabsf(acc_y);
    
    if (abs_x <= ACCELERATION_THRESHOLD && abs_y <= ACCELERATION_THRESHOLD) return;

    int sign_x = (acc_x > 0.0f) ? 1 : ((acc_x < 0.0f) ? -1 : 0);
    int sign_y = (acc_y > 0.0f) ? 1 : ((acc_y < 0.0f) ? -1 : 0);

    float wx = abs_x * abs_x;
    float wy = abs_y * abs_y;
    float total_w = wx + wy;

    bool y_is_primary = true;
    if (total_w > 0.0001f) {
        float rnd = (float)rand() / (float)RAND_MAX * total_w;
        y_is_primary = (rnd <= wy);
    }

    int prim_dx = 0, prim_dy = 0, sec_dx = 0, sec_dy = 0, tert_dx = 0, tert_dy = 0;
    int side_step;

    if (y_is_primary) {
        prim_dy = sign_y;
        sec_dx  = sign_x;  sec_dy  = sign_y;
        tert_dx = -sign_x; tert_dy = sign_y;
        side_step = (y % 2 == 0) ? 1 : -1; 
    } else {
        prim_dx = sign_x;
        sec_dx  = sign_x;  sec_dy  = sign_y;
        tert_dx = sign_x;  tert_dy = -sign_y;
        side_step = (x % 2 == 0) ? 1 : -1; 
    }

    if (y_is_primary && sign_x == 0) {
        sec_dx = side_step; tert_dx = -side_step;
    } else if (!y_is_primary && sign_y == 0) {
        sec_dy = side_step; tert_dy = -side_step;
    }

    if (prim_dx != 0 || prim_dy != 0) { TRY_MOVE(prim_dx, prim_dy); }
    if (sec_dx != 0 || sec_dy != 0) { TRY_MOVE(sec_dx, sec_dy); }
    if (tert_dx != 0 || tert_dy != 0) { TRY_MOVE(tert_dx, tert_dy); }
}

static void update_water_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    float abs_x = fabsf(acc_x);
    float abs_y = fabsf(acc_y);
    
    if (abs_x <= ACCELERATION_THRESHOLD && abs_y <= ACCELERATION_THRESHOLD) return;

    int sign_x = (acc_x > 0.0f) ? 1 : ((acc_x < 0.0f) ? -1 : 0);
    int sign_y = (acc_y > 0.0f) ? 1 : ((acc_y < 0.0f) ? -1 : 0);

    float wx = abs_x * abs_x;
    float wy = abs_y * abs_y;
    float total_w = wx + wy;

    bool y_is_primary = true;
    if (total_w > 0.0001f) {
        float rnd = (float)rand() / (float)RAND_MAX * total_w;
        y_is_primary = (rnd <= wy);
    }

    int prim_dx = 0, prim_dy = 0, sec_dx = 0, sec_dy = 0, tert_dx = 0, tert_dy = 0, sprint_dx = 0, sprint_dy = 0;
    int side_step;

    if (y_is_primary) {
        prim_dy = sign_y;
        sec_dx  = sign_x;  sec_dy  = sign_y;
        tert_dx = -sign_x; tert_dy = sign_y;
        sprint_dx = 1;
        side_step = (y % 2 == 0) ? 1 : -1; 
    } else {
        prim_dx = sign_x;
        sec_dx  = sign_x;  sec_dy  = sign_y;
        tert_dx = sign_x;  tert_dy = -sign_y;
        sprint_dy = 1;
        side_step = (x % 2 == 0) ? 1 : -1; 
    }

    if (y_is_primary && sign_x == 0) {
        sec_dx = side_step; tert_dx = -side_step;
    } else if (!y_is_primary && sign_y == 0) {
        sec_dy = side_step; tert_dy = -side_step;
    }

    if (prim_dx != 0 || prim_dy != 0) { TRY_MOVE(prim_dx, prim_dy); }
    if (sec_dx != 0 || sec_dy != 0) { TRY_MOVE(sec_dx, sec_dy); }
    if (tert_dx != 0 || tert_dy != 0) { TRY_MOVE(tert_dx, tert_dy); }

    int sprint_dir_1 = y_is_primary ? sign_x : sign_y;
    int max_spread = 1;
    bool allow_reverse = true;

    if (sprint_dir_1 == 0) {
        sprint_dir_1 = side_step; 
        max_spread = 2; 
        allow_reverse = false;
    } else {
        float lateral_acc = y_is_primary ? abs_x : abs_y;
        max_spread = (lateral_acc > 0.3f) ? 3 : 2;
    }

    int sprint_dir_2 = -sprint_dir_1;
    bool blocked_1 = false;
    bool blocked_2 = !allow_reverse;

    for (int spread = 1; spread <= max_spread; spread++) {
        if (!blocked_1) {
            TRY_SPRINT(sprint_dx * sprint_dir_1 * spread, sprint_dy * sprint_dir_1 * spread, blocked_1);
        }
        if (!blocked_2) {
            TRY_SPRINT(sprint_dx * sprint_dir_2 * spread, sprint_dy * sprint_dir_2 * spread, blocked_2);
        }
    }
}

static void update_lava_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    float abs_x = fabsf(acc_x);
    float abs_y = fabsf(acc_y);
    
    if (abs_x <= ACCELERATION_THRESHOLD && abs_y <= ACCELERATION_THRESHOLD) return;

    if (rand() % 5 == 0) return; 

    int sign_x = (acc_x > 0.0f) ? 1 : ((acc_x < 0.0f) ? -1 : 0);
    int sign_y = (acc_y > 0.0f) ? 1 : ((acc_y < 0.0f) ? -1 : 0);

    float wx = abs_x * abs_x;
    float wy = abs_y * abs_y;
    float total_w = wx + wy;

    bool y_is_primary = true;
    if (total_w > 0.0001f) {
        float rnd = (float)rand() / (float)RAND_MAX * total_w;
        y_is_primary = (rnd <= wy);
    }

    int prim_dx = 0, prim_dy = 0, sec_dx = 0, sec_dy = 0, tert_dx = 0, tert_dy = 0, sprint_dx = 0, sprint_dy = 0;
    int side_step;

    if (y_is_primary) {
        prim_dy = sign_y;
        sec_dx  = sign_x;  sec_dy  = sign_y;
        tert_dx = -sign_x; tert_dy = sign_y;
        sprint_dx = 1;
        side_step = (y % 2 == 0) ? 1 : -1;
    } else {
        prim_dx = sign_x;
        sec_dx  = sign_x;  sec_dy  = sign_y;
        tert_dx = sign_x;  tert_dy = -sign_y;
        sprint_dy = 1;
        side_step = (x % 2 == 0) ? 1 : -1;
    }

    if (y_is_primary && sign_x == 0) {
        sec_dx = side_step; tert_dx = -side_step;
    } else if (!y_is_primary && sign_y == 0) {
        sec_dy = side_step; tert_dy = -side_step;
    }

    if (prim_dx != 0 || prim_dy != 0) { TRY_MOVE(prim_dx, prim_dy); }
    if (sec_dx != 0 || sec_dy != 0) { TRY_MOVE(sec_dx, sec_dy); }
    if (tert_dx != 0 || tert_dy != 0) { TRY_MOVE(tert_dx, tert_dy); }

    int sprint_dir_1 = y_is_primary ? sign_x : sign_y;
    int max_spread = 1;
    bool allow_reverse = true;

    if (sprint_dir_1 == 0) {
        sprint_dir_1 = side_step; 
        max_spread = 1; 
        allow_reverse = false;
    } else {
        float lateral_acc = y_is_primary ? abs_x : abs_y;
        max_spread = (lateral_acc > 0.4f) ? 2 : 1;
    }

    int sprint_dir_2 = -sprint_dir_1;
    bool blocked_1 = false;
    bool blocked_2 = !allow_reverse;

    for (int spread = 1; spread <= max_spread; spread++) {
        if (!blocked_1) {
            TRY_SPRINT(sprint_dx * sprint_dir_1 * spread, sprint_dy * sprint_dir_1 * spread, blocked_1);
        }
        if (!blocked_2) {
            TRY_SPRINT(sprint_dx * sprint_dir_2 * spread, sprint_dy * sprint_dir_2 * spread, blocked_2);
        }
    }

    if (rand() % 4 == 0) {
        int climb_x = -prim_dx;
        int climb_y = -prim_dy;

        if (x + climb_x > 0 && x + climb_x < GRID_WIDTH - 1 && y + climb_y > 0 && y + climb_y < GRID_HEIGHT - 1) {
            TRY_MOVE(climb_x, climb_y);
        }
    }
}
