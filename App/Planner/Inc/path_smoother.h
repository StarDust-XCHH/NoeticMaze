//
// Created by lmtgy on 2026/4/15.
//

#ifndef STM32_PLANNER_PC_TEST_PATH_SMOOTHER_H
#define STM32_PLANNER_PC_TEST_PATH_SMOOTHER_H
#include <stdbool.h>

#include "Cstar_map_core.h"
#include "planner_config.h"



bool bresenham_line_check(ServerMap* map, Point2D p1, Point2D p2, float radius, float ignore_start_dist, bool is_return);
int double_smooth_path(ServerMap* map, Point2D* path, int path_len, float safe_radius, bool is_return, Point2D* out_path);
int generate_safe_corner_path(ServerMap* map, Point2D* path, int path_len, float max_corner_radius, float point_dist, bool is_return, Point2D* out_path);



#endif //STM32_PLANNER_PC_TEST_PATH_SMOOTHER_H