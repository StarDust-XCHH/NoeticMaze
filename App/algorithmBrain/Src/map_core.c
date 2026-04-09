//
// Created by lmtgy on 2026/4/9.
//

#include "map_core.h"

#include <stdint.h>
#include <stdlib.h>
uint32_t diff_cnt = 0;
uint8_t diff_payload[MAX_MAP_DIFF * 3];
uint8_t global_map[15625];


// 物理坐标转地图格点坐标
int world_to_grid(float world_val, int *grid_idx) {
    *grid_idx = (int)((world_val + MAP_OFFSET) / MAP_RES);
    // 关键安全检查
    if (*grid_idx < 0) { *grid_idx = 0; return 0; }
    if (*grid_idx >= MAP_SIZE) { *grid_idx = MAP_SIZE - 1; return 0; }
    return 1;
}


// 替换你原有的 Bresenham 函数
void trace_ray_bresenham_diff(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    // 为了防止把起点(机器人自己)标为空闲引起误判，跳过起点的更新
    int is_first = 1;

    while (1) {
        // 到达终点(击中点)直接退出。
        // 因为终点必须是 MAP_OCCUPIED，不能在 Bresenham 里被标记为 MAP_FREE
        if (x0 == x1 && y0 == y1) break;

        if (!is_first) {
            // 尝试将途经点标记为空闲
            UpdateAndRecordMap(x0, y0, MAP_FREE);
        }
        is_first = 0;

        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}



