//
// Created by lmtgy on 2026/4/15.
//
#include "Cstar_map_core.h"
#include <string.h>
#include "planner_config.h"

ServerMap g_map = {NULL,PLANNER_MAP_RES, MAX_GRID_SIZE};

// 【修复】：从 planner_core.c 暴露写指针
extern uint8_t* volatile s_write_map_ptr;

void process_map_update(uint8_t* payload, uint32_t delta_count, int grid_size, float res) {
    if (grid_size > MAX_GRID_SIZE) return;

    // 动态同步最新系统级分辨率和尺寸 (但不干扰正在寻路的 g_map)
    int required_bytes = (grid_size * grid_size + 1) / 2;

    // 【修复】：建立一个局部靶向结构体，让 set_grid_val 操作安全内存
    ServerMap write_target = {s_write_map_ptr, res, grid_size};

    if (delta_count == 0xFFFFFFFF) {
        // 直接整片覆写在 s_write_map_ptr 上
        memcpy(s_write_map_ptr, payload, required_bytes);
    } else if (delta_count > 0) {
        uint8_t* p = payload;
        for (uint32_t i = 0; i < delta_count; i++) {
            uint16_t x = p[0] | (p[1] << 8);
            uint16_t y = p[2] | (p[3] << 8);
            uint8_t val = p[4];
            p += 5;

            if (x < grid_size && y < grid_size) {
                // 【修复】：这里传入 write_target 而不是 &g_map
                set_grid_val(&write_target, x, y, val);
            }
        }
    }
}