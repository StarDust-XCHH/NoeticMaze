//
// Created by lmtgy on 2026/4/8.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\global\Inc\roboConifg.h

#ifndef NOETICMAZE_ROBOCONIFG_H
#define NOETICMAZE_ROBOCONIFG_H

// imu相关
#define YAW_DRIFT_RATE  0.00391974f



// ==========================================
// 1. 编码器与电机物理参数配置 (MC520P30_12V)
// ==========================================
#define ENCODER_LINES      13.0f   // 电机原始线数
#define REDUCTION_RATIO    30.0f   // 减速比
#define ENCODER_MULTIPLIER 4.0f    // 硬件定时器 4倍频模式
// 轮子转一圈的总脉冲数 = 1560
#define TICKS_PER_REV      (ENCODER_LINES * REDUCTION_RATIO * ENCODER_MULTIPLIER)

// ==========================================
// 2. 机器人运动学参数配置
// ==========================================
#define WHEEL_DIAMETER     0.07f    // 轮子直径 0.07 米 (7cm)
#define WHEEL_CIRCUM       (WHEEL_DIAMETER * 3.1415926f) // 轮子周长
// 【新增】定义里程计的起始位置 (单位：米)
#define INITIAL_ODOM_X     2.5f     // 例如：起始 X 设为 1.5 米
#define INITIAL_ODOM_Y     2.5f     // 例如：起始 Y 设为 2.0 米
// 速度限制 (安全第一)
#define MAX_LINEAR_VEL     0.8f                 // 最大线速度限制 0.8m/s
#define MAX_ANGULAR_DELTA  0.15f                // 转向引起的差速补偿最大值 0.15m/s
#define STOP_DEADBAND 0.01f   // 停车死区：当转速小于此值(圈/秒)时认为已停稳
#define motor_Kp 950.0f
#define motor_Ki 250.0f
#define motor_Kd 0.0f
#define angle_Kp 0.005f
#define angle_Ki 0.0f
#define angle_Kd 0.001f




// 已经于App\motor\Src\encoder.c中执行DWT_CYCCNT = 0;

#endif //NOETICMAZE_ROBOCONIFG_H