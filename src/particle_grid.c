#include "particle_grid.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Map Layout Geometry Scaled for 80x205 Grid */
#define FUNNEL_TOP_Y            35
#define FUNNEL_BOTTOM_Y         50
#define FUNNEL_OPENING_HALF     4
#define WALL_THICKNESS          1

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
            if (x < WALL_THICKNESS || x >= (GRID_WIDTH - WALL_THICKNESS) || 
                y < WALL_THICKNESS || y >= (GRID_HEIGHT - WALL_THICKNESS)) {
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

static void update_sand_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    float abs_x = fabsf(acc_x);
    float abs_y = fabsf(acc_y);
    
    if (abs_x <= ACCELERATION_THRESHOLD && abs_y <= ACCELERATION_THRESHOLD) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    int sign_x = (acc_x > 0.0f) ? 1 : ((acc_x < 0.0f) ? -1 : 0);
    int sign_y = (acc_y > 0.0f) ? 1 : ((acc_y < 0.0f) ? -1 : 0);

    int prim_dx, prim_dy, sec_dx, sec_dy, tert_dx, tert_dy;

    if (abs_y >= abs_x) {
        prim_dx = 0;             prim_dy = sign_y;
        sec_dx  = sign_x;        sec_dy  = sign_y;
        tert_dx = -sign_x;       tert_dy = sign_y;
    } else {
        prim_dx = sign_x;        prim_dy = 0;
        sec_dx  = sign_x;        sec_dy  = sign_y;
        tert_dx = sign_x;        tert_dy = -sign_y;
    }

    if (abs_y >= abs_x && sign_x == 0) {
        int r = (rand() % 2 == 0) ? 1 : -1;
        sec_dx = r; tert_dx = -r;
    } else if (abs_x > abs_y && sign_y == 0) {
        int r = (rand() % 2 == 0) ? 1 : -1;
        sec_dy = r; tert_dy = -r;
    }

    if (prim_dx != 0 || prim_dy != 0) {
        if (ctx->current_grid[CELL(x + prim_dx, y + prim_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + prim_dx, y + prim_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + prim_dx, y + prim_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    if (sec_dx != 0 || sec_dy != 0) {
        if (ctx->current_grid[CELL(x + sec_dx, y + sec_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + sec_dx, y + sec_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + sec_dx, y + sec_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    if (tert_dx != 0 || tert_dy != 0) {
        if (ctx->current_grid[CELL(x + tert_dx, y + tert_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + tert_dx, y + tert_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + tert_dx, y + tert_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
}

static void update_water_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    float abs_x = fabsf(acc_x);
    float abs_y = fabsf(acc_y);
    
    if (abs_x <= ACCELERATION_THRESHOLD && abs_y <= ACCELERATION_THRESHOLD) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    int sign_x = (acc_x > 0.0f) ? 1 : ((acc_x < 0.0f) ? -1 : 0);
    int sign_y = (acc_y > 0.0f) ? 1 : ((acc_y < 0.0f) ? -1 : 0);

    int prim_dx, prim_dy, sec_dx, sec_dy, tert_dx, tert_dy, sprint_dx, sprint_dy;

    if (abs_y >= abs_x) {
        prim_dx = 0;             prim_dy = sign_y;
        sec_dx  = sign_x;        sec_dy  = sign_y;
        tert_dx = -sign_x;       tert_dy = sign_y;
        sprint_dx = 1;           sprint_dy = 0;
    } else {
        prim_dx = sign_x;        prim_dy = 0;
        sec_dx  = sign_x;        sec_dy  = sign_y;
        tert_dx = sign_x;        tert_dy = -sign_y;
        sprint_dx = 0;           sprint_dy = 1;
    }

    if (abs_y >= abs_x && sign_x == 0) {
        int r = (rand() % 2 == 0) ? 1 : -1;
        sec_dx = r; tert_dx = -r;
    } else if (abs_x > abs_y && sign_y == 0) {
        int r = (rand() % 2 == 0) ? 1 : -1;
        sec_dy = r; tert_dy = -r;
    }

    if (prim_dx != 0 || prim_dy != 0) {
        if (ctx->current_grid[CELL(x + prim_dx, y + prim_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + prim_dx, y + prim_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + prim_dx, y + prim_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    if (sec_dx != 0 || sec_dy != 0) {
        if (ctx->current_grid[CELL(x + sec_dx, y + sec_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + sec_dx, y + sec_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + sec_dx, y + sec_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    if (tert_dx != 0 || tert_dy != 0) {
        if (ctx->current_grid[CELL(x + tert_dx, y + tert_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + tert_dx, y + tert_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + tert_dx, y + tert_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    int sprint_dir_1 = (abs_y >= abs_x) ? sign_x : sign_y;
    if (sprint_dir_1 == 0) sprint_dir_1 = (rand() % 2 == 0) ? 1 : -1;
    int sprint_dir_2 = -sprint_dir_1;

    bool blocked_1 = false;
    bool blocked_2 = false;

    for (int spread = 1; spread <= 5; spread++) {
        if (!blocked_1) {
            int cx1 = x + sprint_dx * sprint_dir_1 * spread;
            int cy1 = y + sprint_dy * sprint_dir_1 * spread;
            if (cx1 > 0 && cx1 < GRID_WIDTH - 1 && cy1 > 0 && cy1 < GRID_HEIGHT - 1) {
                if (ctx->current_grid[CELL(cx1, cy1)] == CELL_TYPE_AIR && ctx->next_grid[CELL(cx1, cy1)] == CELL_TYPE_AIR) {
                    ctx->next_grid[CELL(cx1, cy1)] = CELL_TYPE_PARTICLE; 
                    return;
                } else {
                    blocked_1 = true;
                }
            } else { blocked_1 = true; }
        }

        if (!blocked_2) {
            int cx2 = x + sprint_dx * sprint_dir_2 * spread;
            int cy2 = y + sprint_dy * sprint_dir_2 * spread;
            if (cx2 > 0 && cx2 < GRID_WIDTH - 1 && cy2 > 0 && cy2 < GRID_HEIGHT - 1) {
                if (ctx->current_grid[CELL(cx2, cy2)] == CELL_TYPE_AIR && ctx->next_grid[CELL(cx2, cy2)] == CELL_TYPE_AIR) {
                    ctx->next_grid[CELL(cx2, cy2)] = CELL_TYPE_PARTICLE; 
                    return;
                } else {
                    blocked_2 = true;
                }
            } else { blocked_2 = true; }
        }
    }

    ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
}

static void update_lava_physics(particle_grid_context_t *ctx, int x, int y, float acc_x, float acc_y)
{
    float abs_x = fabsf(acc_x);
    float abs_y = fabsf(acc_y);
    
    if (abs_x <= ACCELERATION_THRESHOLD && abs_y <= ACCELERATION_THRESHOLD) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    static uint8_t viscosity_divider = 0;
    if (x == 1 && y == 1) {
        viscosity_divider++;
    }

    if ((viscosity_divider % 2) != 0) {
        ctx->next_grid[CELL(x, y)] = CELL_TYPE_PARTICLE;
        return;
    }

    int sign_x = (acc_x > 0.0f) ? 1 : ((acc_x < 0.0f) ? -1 : 0);
    int sign_y = (acc_y > 0.0f) ? 1 : ((acc_y < 0.0f) ? -1 : 0);

    int prim_dx, prim_dy, sec_dx, sec_dy, tert_dx, tert_dy, sprint_dx, sprint_dy;

    if (abs_y >= abs_x) {
        prim_dx = 0;             prim_dy = sign_y;
        sec_dx  = sign_x;        sec_dy  = sign_y;
        tert_dx = -sign_x;       tert_dy = sign_y;
        sprint_dx = 1;           sprint_dy = 0;
    } else {
        prim_dx = sign_x;        prim_dy = 0;
        sec_dx  = sign_x;        sec_dy  = sign_y;
        tert_dx = sign_x;        tert_dy = -sign_y;
        sprint_dx = 0;           sprint_dy = 1;
    }

    if (abs_y >= abs_x && sign_x == 0) {
        int r = (rand() % 2 == 0) ? 1 : -1;
        sec_dx = r; tert_dx = -r;
    } else if (abs_x > abs_y && sign_y == 0) {
        int r = (rand() % 2 == 0) ? 1 : -1;
        sec_dy = r; tert_dy = -r;
    }

    if (prim_dx != 0 || prim_dy != 0) {
        if (ctx->current_grid[CELL(x + prim_dx, y + prim_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + prim_dx, y + prim_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + prim_dx, y + prim_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    if (sec_dx != 0 || sec_dy != 0) {
        if (ctx->current_grid[CELL(x + sec_dx, y + sec_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + sec_dx, y + sec_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + sec_dx, y + sec_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    if (tert_dx != 0 || tert_dy != 0) {
        if (ctx->current_grid[CELL(x + tert_dx, y + tert_dy)] == CELL_TYPE_AIR &&
            ctx->next_grid[CELL(x + tert_dx, y + tert_dy)] == CELL_TYPE_AIR) 
        {
            ctx->next_grid[CELL(x + tert_dx, y + tert_dy)] = CELL_TYPE_PARTICLE;
            return;
        }
    }

    int sprint_dir_1 = (abs_y >= abs_x) ? sign_x : sign_y;
    if (sprint_dir_1 == 0) sprint_dir_1 = (rand() % 2 == 0) ? 1 : -1;
    int sprint_dir_2 = -sprint_dir_1;

    bool blocked_1 = false;
    bool blocked_2 = false;

    for (int spread = 1; spread <= 2; spread++) {
        if (!blocked_1) {
            int cx1 = x + sprint_dx * sprint_dir_1 * spread;
            int cy1 = y + sprint_dy * sprint_dir_1 * spread;
            if (cx1 > 0 && cx1 < GRID_WIDTH - 1 && cy1 > 0 && cy1 < GRID_HEIGHT - 1) {
                if (ctx->current_grid[CELL(cx1, cy1)] == CELL_TYPE_AIR && ctx->next_grid[CELL(cx1, cy1)] == CELL_TYPE_AIR) {
                    ctx->next_grid[CELL(cx1, cy1)] = CELL_TYPE_PARTICLE; 
                    return;
                } else {
                    blocked_1 = true;
                }
            } else { blocked_1 = true; }
        }

        if (!blocked_2) {
            int cx2 = x + sprint_dx * sprint_dir_2 * spread;
            int cy2 = y + sprint_dy * sprint_dir_2 * spread;
            if (cx2 > 0 && cx2 < GRID_WIDTH - 1 && cy2 > 0 && cy2 < GRID_HEIGHT - 1) {
                if (ctx->current_grid[CELL(cx2, cy2)] == CELL_TYPE_AIR && ctx->next_grid[CELL(cx2, cy2)] == CELL_TYPE_AIR) {
                    ctx->next_grid[CELL(cx2, cy2)] = CELL_TYPE_PARTICLE; 
                    return;
                } else {
                    blocked_2 = true;
                }
            } else { blocked_2 = true; }
        }
    }

    if (rand() % 4 == 0) {
        int climb_x = x - prim_dx;
        int climb_y = y - prim_dy;

        if (climb_x > 0 && climb_x < GRID_WIDTH - 1 && climb_y > 0 && climb_y < GRID_HEIGHT - 1) {
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
