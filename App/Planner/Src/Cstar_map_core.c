//
// Created by lmtgy on 2026/4/15.
//
#include "Cstar_map_core.h"
#include <string.h>
#include "planner_config.h"

static uint8_t s_map_grid[MAX_MAP_BYTES];
ServerMap g_map = {s_map_grid, 0.0f, 0};

void process_map_update(uint8_t* payload, uint32_t delta_count, int grid_size, float res) {
    if (grid_size > MAX_GRID_SIZE) return;

    g_map.grid_size = grid_size;
    g_map.res = res;
    int required_bytes = (grid_size * grid_size + 1) / 2;

    if (delta_count == 0xFFFFFFFF) {
        memcpy(g_map.grid, payload, required_bytes);
    } else if (delta_count > 0) {
        // 优化 5：移除 memcpy，使用单指针遍历提取，避免每次循环计算 i*5
        uint8_t* p = payload;
        for (uint32_t i = 0; i < delta_count; i++) {
            // 通过小端移位安全拼接，避免强制转换造成的未对齐异常 (Unaligned Fault)
            uint16_t x = p[0] | (p[1] << 8);
            uint16_t y = p[2] | (p[3] << 8);
            uint8_t val = p[4];
            p += 5; // 指针步进

            if (x < grid_size && y < grid_size) {
                set_grid_val(&g_map, x, y, val);
            }
        }
    }
}