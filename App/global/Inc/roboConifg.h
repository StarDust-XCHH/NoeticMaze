//
// Created by lmtgy on 2026/4/8.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\global\Inc\roboConifg.h

#ifndef NOETICMAZE_ROBOCONIFG_H
#define NOETICMAZE_ROBOCONIFG_H


// 蓝牙相关

// ======================================================================
// 宏定义配置：是否通过蓝牙发送雷达帧数据
// 0: 关闭 (让出雷达队列给 ICP 线程，蓝牙仅以50Hz发送机器人状态)
// 1: 开启 (阻塞读取雷达队列，同步发送雷达与机器人状态)
// ======================================================================
#define ENABLE_LIDAR_BT_TX  0



// icp相关
#define MIN_VALID_POINTS_FOR_INIT 280 // 需至少有 280 个有效点才认为icp初始化完成
#define ICP_REF_UPDATE_FRAMES 5

// imu相关
#define YAW_DRIFT_RATE  0.00391974f

// lidar相关
#define MIN_VALID_DIST 30 // 最小有效距离3cm
#define MAX_VALID_DIST 8000 // 最大有效距离8m

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
// 【新增】轮距 (两个驱动轮中心之间的距离)
#define WHEEL_TRACK        0.18f    // 18cm = 0.18米
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
// 【修改】角度环 PID 参数 -> 角速度环 PID 参数
// 注意：因为输入变成了角速度误差(deg/s)，原来的参数不适用了，建议从较小的值重新开始整定
#define yaw_rate_Kp 0.0035f
#define yaw_rate_Ki 0.0f
#define yaw_rate_Kd 0.0005f

#define DIED_YAW_RATE 1.0f // 角速度死区:死区优化：当角速度误差很小（如小于 1 deg/s）时，忽略不计，防止电机轻微抖动



// 已经于App\motor\Src\encoder.c中执行DWT_CYCCNT = 0;

#endif //NOETICMAZE_ROBOCONIFG_H