//
// Created by lmtgy on 2026/3/20.
//

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"  // 引入 htim3 和 htim4 的声明






// ==========================================
// 3. 调度周期配置
// ==========================================
// 注意：这个值必须与你在 Scheduler_AddTask 中设置的周期绝对一致！
// 例如调度器设为 10ms 执行一次，这里就是 0.01f 秒
#define ENCODER_UPDATE_DT  0.01f

// ==========================================
// 4. 数据结构体声明
// ==========================================
// 编码器内部单轮状态结构体
typedef struct {
    uint32_t last_count;   // 上一次的定时器计数值
    int32_t  delta_ticks;  // 周期内的脉冲差值 (带符号)
    float    speed_rps;    // 当前转速 (转/秒)
    float    speed_ms;     // 新增：当前线速度 (米/秒)
} Encoder_Data_t;

// 里程计全局状态结构体
typedef struct {
    float x;           // 全局坐标系 X 坐标 (米)
    float y;           // 全局坐标系 Y 坐标 (米)
    float linear_vel;  // 机器人中心当前线速度 (米/秒)
} Odometry_Data_t;

// ==========================================
// 5. 对外接口函数 (API)
// ==========================================

// --- 基础初始化与更新 ---
void Encoder_Init(void);
void Encoder_Update(void *param);

// --- 速度获取接口 ---
/**
 * @brief 获取左右轮当前转速 (转/秒 RPS)
 */
float Encoder_GetLeftRPS(void);
float Encoder_GetRightRPS(void);

/**
 * @brief 获取左右轮当前线速度 (米/秒 m/s)
 */
float Encoder_GetLeftMS(void);
float Encoder_GetRightMS(void);

// --- 里程计获取接口 ---
/**
 * @brief 获取当前机器人的全局里程计位姿
 */
Odometry_Data_t Encoder_GetOdometry(void);

/**
 * @brief 将里程计的 X、Y 坐标清零 (常用于上电或收到复位指令时)
 */
void Encoder_ResetOdometry(void);

// --- 打印与调试 ---
void Task_ReportRps(void *argument);

// --- 电机 PID 角度控制相关 ---
void Motor_AnglePID_Init(void);
void Motor_SetTargetAngle(float angle, float linear_rps);
float _Calculate_Angle_Error_Correction(float current_yaw);

#endif /* ENCODER_H */