//
// Created by lmtgy on 2026/4/8.
//

#ifndef NOETICMAZE_ROBOT_STATE_H
#define NOETICMAZE_ROBOT_STATE_H

#include <stdint.h>

// 全局机器人状态结构体 (按需扩充)
typedef struct {
    // --- 姿态与角速度 (通常由 IMU 任务更新) ---
    float yaw;          // 当前偏航角 (0-360)
    float yaw_rate;     // 当前角速度 (Z轴)

    // --- 位置与线速度 (通常由 里程计/编码器 任务更新) ---
    float x_encoder;            // 全局 X 坐标 (米)
    float y_encoder;            // 全局 Y 坐标 (米)
    float linear_vel_encoder;   // 当前线速度 (米/秒)
} RobotState_t;

// ==========================================
// 全局状态读取接口
// ==========================================
/**
 * @brief 获取机器人完整状态的“时间冻结快照”
 * @param out_state 指向接收数据的结构体指针
 */
void Get_Robot_State_Snapshot(RobotState_t *out_state);

// ==========================================
// 全局状态更新接口 (解耦更新)
// ==========================================
/**
 * @brief 仅更新 IMU 相关的状态 (供 IMU 解析任务调用)
 */
void Update_Robot_IMU_State(float new_yaw, float new_yaw_rate);

/**
 * @brief 仅更新里程计相关的状态 (供 100Hz 运动控制任务调用)
 */
void Update_Robot_Odom_State(float new_x, float new_y, float new_v);


// 保留你原有的单个读取接口（如果某些地方确实只需要单个值且不在乎空间关联性）
float Get_Global_Yaw(void);
float Get_Global_Yaw_Rate(void);

#endif //NOETICMAZE_ROBOT_STATE_H