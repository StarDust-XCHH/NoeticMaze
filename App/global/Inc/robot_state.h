//
// Created by lmtgy on 2026/4/8.
//

#ifndef NOETICMAZE_ROBOT_STATE_H
#define NOETICMAZE_ROBOT_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "planner_config.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

#define DEG_TO_RAD_F(x) ((x) * (PI / 180.0f))
#define RAD_TO_DEG_F(x) ((x) * (180.0f / PI))

// 全局机器人状态结构体 (按需扩充)
typedef struct {
    // --- 姿态与角速度 (通常由 IMU 任务更新) ---
    float yaw_deg;          // 当前偏航角 (deg, 0~360)
    float yaw_rate_deg_s;   // 当前角速度 (Z轴 deg/s)
    uint8_t imu_ready;      // IMU 是否校准完成标志

    // --- 原始 odom (通常由 里程计/编码器 任务更新) ---
    float x_encoder_m;            // odom 系 X 坐标 (m)
    float y_encoder_m;            // odom 系 Y 坐标 (m)
    float linear_vel_encoder_m_s; // 当前线速度 (m/s)

    // --- map <- odom TF 漂移补偿 (由 ICP 线程维护) ---
    float tf_map_odom_x_m;        // map<-odom 平移 x (m)
    float tf_map_odom_y_m;        // map<-odom 平移 y (m)
    float tf_map_odom_theta_rad;  // map<-odom 旋转角 (rad)
    uint32_t tf_map_odom_version; // TF 发布版本号

    // --- 高频精准全局位姿 (由运动线程维护) ---
    float global_fast_x_m;        // map 系高频 X 坐标 (m)
    float global_fast_y_m;        // map 系高频 Y 坐标 (m)
    float global_fast_theta_rad;  // map 系高频朝向 (rad)
} RobotState_t;

typedef struct {
    Point2D path_points[MAX_PATH_LEN];
    uint16_t path_len;
    float progress_s_m;
    uint32_t source_path_sequence;
    uint32_t tf_version;
    uint32_t trimmed_sequence;
    uint8_t valid;
} TrimmedPathState;

typedef struct {
    Point2D* path_ptr;
    uint16_t path_len;
    float progress_s_m;
    uint32_t source_path_sequence;
    uint32_t tf_version;
    uint32_t trimmed_sequence;
    bool valid;
} TrimmedPathSnapshot;

// ==========================================
// 角度/姿态辅助接口
// ==========================================
float Normalize_Angle_Rad(float angle_rad);
float Robot_DegToRad(float angle_deg);
float Robot_RadToDeg(float angle_rad);

// ==========================================
// 全局状态读取接口
// ==========================================
/**
 * @brief 获取机器人完整状态的“时间冻结快照”
 * @param out_state 指向接收数据的结构体指针
 */
void Get_Robot_State_Snapshot(RobotState_t *out_state);
bool Get_Trimmed_Path_Snapshot(TrimmedPathSnapshot *out_snapshot);

// ==========================================
// 全局状态更新接口 (解耦更新)
// ==========================================
/**
 * @brief 仅更新 IMU 相关的状态 (供 IMU 解析任务调用)
 * @param new_yaw_deg 当前偏航角，单位 deg
 * @param new_yaw_rate_deg_s 当前角速度，单位 deg/s
 * @param ready IMU 是否校准完成
 */
void Update_Robot_IMU_State(float new_yaw_deg, float new_yaw_rate_deg_s, uint8_t ready);

/**
 * @brief 仅更新原始里程计相关的状态 (供 100Hz 运动控制任务调用)
 * @param new_x_m odom 系 X 坐标，单位 m
 * @param new_y_m odom 系 Y 坐标，单位 m
 * @param new_v_m_s 当前线速度，单位 m/s
 */
void Update_Robot_Odom_State(float new_x_m, float new_y_m, float new_v_m_s);

/**
 * @brief 更新 map<-odom TF 漂移补偿 (供 ICP 线程调用)
 * @param tf_x_m map<-odom 平移 x，单位 m
 * @param tf_y_m map<-odom 平移 y，单位 m
 * @param tf_theta_rad map<-odom 旋转角，单位 rad
 */
void Update_Robot_TF_MapOdom(float tf_x_m, float tf_y_m, float tf_theta_rad);

/**
 * @brief 更新高频精准全局位姿 (供 100Hz 运动控制任务调用)
 * @param x_m map 系 X 坐标，单位 m
 * @param y_m map 系 Y 坐标，单位 m
 * @param theta_rad map 系朝向，单位 rad
 */
void Update_Robot_Global_Fast_Pose(float x_m, float y_m, float theta_rad);
void Update_Trimmed_Path_State(const Point2D* path_points,
                               uint16_t path_len,
                               float progress_s_m,
                               uint32_t source_path_sequence,
                               uint32_t tf_version,
                               uint8_t valid);
void Reset_Trimmed_Path_State(void);

// 保留兼容接口（单位保持原语义）
float Get_Global_Yaw(void);          // 返回 yaw_deg
float Get_Global_Yaw_Rate(void);     // 返回 yaw_rate_deg_s

#endif //NOETICMAZE_ROBOT_STATE_H
