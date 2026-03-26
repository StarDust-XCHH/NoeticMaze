//
// Created by lmtgy on 2026/3/27.
//

#include "global_vault.h"

#include <stdint.h>

// 严格分配在 .bss 段（确保这些变量在编译时就划定好地盘）
uint8_t LidarBuffer_A[2900];
uint8_t LidarBuffer_B[2900];
uint8_t slam_map_2cm[250][250];
uint8_t nav_map_20cm[25][25];
RobotState_t global_robot_state;
Point2D_t GlobalPath[200];
uint8_t TxBuffer_A[150];
uint8_t TxBuffer_B[150];