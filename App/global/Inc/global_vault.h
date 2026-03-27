//
// Created by lmtgy on 2026/3/27.
//
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\global\Inc\global_vault.h
#ifndef NOETICMAZE_GLOBAL_VAULT_H
#define NOETICMAZE_GLOBAL_VAULT_H

#include <stdint.h>

// --- 数据结构定义 ---
typedef struct {
    float x_icp;
    float y_icp;
    float x_rawInEncoder;
    float y_rawInEncoder;
    float theta_icp;
    float linear_vel;
    float angular_vel;
    float yaw;
    float yaw_rate;
} RobotState_t;

typedef struct {
    float x;
    float y;
} Point2D_t;

// --- 外部变量声明 (extern) ---
extern uint8_t LidarBuffer_A[2900];
extern uint8_t LidarBuffer_B[2900];
extern uint8_t slam_map_2cm[250][250];
extern uint8_t nav_map_20cm[25][25];
extern RobotState_t global_robot_state;
extern Point2D_t GlobalPathFromeAStar[200];
extern uint8_t TxBuffer_A[150];
extern uint8_t TxBuffer_B[150];
#endif //NOETICMAZE_GLOBAL_VAULT_H