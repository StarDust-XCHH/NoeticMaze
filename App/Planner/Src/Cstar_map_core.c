//
// Created by lmtgy on 2026/4/15.
//
#include "Cstar_map_core.h"
#include <string.h>
#include "planner_config.h"
#include "map_core.h"

ServerMap g_map = {NULL, PLANNER_MAP_RES, MAX_GRID_SIZE};

// 【修复】：从 planner_core.c 暴露写指针
extern uint8_t* volatile s_write_map_ptr;

static inline uint8_t aggregate_slam_block(int planner_x, int planner_y) {
    int slam_x0 = planner_x * PLANNER_DOWNSAMPLE_SCALE;
    int slam_y0 = planner_y * PLANNER_DOWNSAMPLE_SCALE;
    bool has_free = false;

    for (int dy = 0; dy < PLANNER_DOWNSAMPLE_SCALE; ++dy) {
        for (int dx = 0; dx < PLANNER_DOWNSAMPLE_SCALE; ++dx) {
            uint8_t cell = GetMapState(slam_x0 + dx, slam_y0 + dy);
            if (cell == MAP_OCCUPIED) {
                return 1;
            }
            if (cell == MAP_FREE) {
                has_free = true;
            }
        }
    }

    return has_free ? 2 : 0;
}

static void copy_map_cells(const ServerMap* src_map, ServerMap* dst_map) {
    for (int x = 0; x < src_map->grid_size; ++x) {
        for (int y = 0; y < src_map->grid_size; ++y) {
            set_grid_val(dst_map, x, y, get_grid_val((ServerMap*)src_map, x, y));
        }
    }
}

void build_blocked_view_from_base(const ServerMap* src_map, ServerMap* dst_map) {
    uint8_t occupied_mask[MAX_CELLS];
    memset(occupied_mask, 0, sizeof(occupied_mask));
    memset(dst_map->grid, 0, MAX_MAP_BYTES);

    copy_map_cells(src_map, dst_map);

    for (int x = 0; x < src_map->grid_size; ++x) {
        for (int y = 0; y < src_map->grid_size; ++y) {
            if (get_grid_val((ServerMap*)src_map, x, y) == 1) {
                occupied_mask[x * src_map->grid_size + y] = 1;
            }
        }
    }

    for (int x = 0; x < src_map->grid_size; ++x) {
        for (int y = 0; y < src_map->grid_size; ++y) {
            if (occupied_mask[x * src_map->grid_size + y] == 0) {
                continue;
            }

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= src_map->grid_size || ny < 0 || ny >= src_map->grid_size) {
                        continue;
                    }
                    set_grid_val(dst_map, nx, ny, 1);
                }
            }
        }
    }
}

void process_map_update(ServerMap* planner_map_out) {
    if (s_write_map_ptr == NULL) {
        return;
    }

    memset((void*)s_write_map_ptr, 0, MAX_MAP_BYTES);

    ServerMap write_target = {(uint8_t*)s_write_map_ptr, PLANNER_MAP_RES, MAX_GRID_SIZE};

    for (int x = 0; x < MAX_GRID_SIZE; ++x) {
        for (int y = 0; y < MAX_GRID_SIZE; ++y) {
            uint8_t planner_val = aggregate_slam_block(x, y);
            set_grid_val(&write_target, x, y, planner_val);
        }
    }

    if (planner_map_out != NULL) {
        *planner_map_out = write_target;
    }
}
