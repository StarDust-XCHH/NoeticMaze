//
// Created by lmtgy on 2026/4/15.
// 3. 核心算法层



#ifndef STM32_PLANNER_PC_TEST_PLANNER_ALGO_H
#define STM32_PLANNER_PC_TEST_PLANNER_ALGO_H
#include <stdbool.h>

#include "Cstar_map_core.h"
#include "planner_config.h"

// 对外保留原始的物理碰撞检查接口
bool check_collision(ServerMap* map, Point2D pos, float radius, bool is_return);

// 核心规划器
int astar_plan(ServerMap* map, Point2D start, Point2D goal, Point2D* prev_path, int prev_path_len, bool is_return, Point2D* out_path);

#endif //STM32_PLANNER_PC_TEST_PLANNER_ALGO_H