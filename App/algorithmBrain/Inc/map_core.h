//
// Created by lmtgy on 2026/4/10.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\algorithmBrain\Inc\map_core.h

#ifndef NOETICMAZE_MAP_CORE_H
#define NOETICMAZE_MAP_CORE_H

#include <stdint.h>
#include "bt_protocol.h"
#include "cmsis_os2.h"

// 地图基础参数
#define MAP_UNKNOWN  0x00
#define MAP_FREE     0x01
#define MAP_OCCUPIED 0x02
#define MAP_OFFSET   0.0f
#define MAP_RES      0.02f
#define MAP_SIZE     250

int world_to_grid(float world_val, int *grid_idx);

// ⚠️ 注意：暴露 global_map 是为了让内联的 GetMapState 能够高速运行。
// 我们依靠架构纪律和 write.h 封装来防止其他线程恶意写入。
extern uint8_t global_map[15625];

// 栅格地图蓝牙发送相关
extern osMutexId_t MapDataMutexHandle;
extern MapIcp_Packet_t g_Shared_MapIcp_Data;
extern volatile uint8_t g_MapIcp_Ready;

/**
 * @brief  读取栅格状态 (2-bit 压缩版) - 任何线程均可安全调用
 */
static inline uint8_t GetMapState(int x, int y) {
    if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return MAP_UNKNOWN;

    uint32_t flat_idx = y * MAP_SIZE + x;
    uint32_t byte_idx = flat_idx >> 2;
    uint8_t bit_shift = (flat_idx & 0x03) << 1;

    return (global_map[byte_idx] >> bit_shift) & 0x03;
}

#endif //NOETICMAZE_MAP_CORE_H