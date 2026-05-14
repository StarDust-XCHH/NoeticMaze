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

// 是否发布流到Astar线程进行规划

#define IS_ICP2ASTAR 1

// 自动返航：到达用户目标点后，自动规划回 INITIAL_ODOM_X/Y
// 0: 关闭  1: 开启
#define ENABLE_AUTO_RETURN_HOME 0

#define jumpOutIcp 0 // 是否跳过icp算法
#define MIN_VALID_POINTS_FOR_INIT 280 // 需至少有 280 个有效点才认为icp初始化完成
// --- 空间关键帧更新阈值 (KeyFrame Update) ---
// 决定了小车走多远、转多大角度才更新一次参考帧地图
// 提示: 设得太大容易匹配丢步，设得太小(比如0)会导致静止发散
#define ICP_KF_UPDATE_DIST_SQ    0.01f      // 距离平方阈值：0.01f 代表 0.1m (10cm) 的平方
#define ICP_KF_UPDATE_ANGLE_RAD  0.1f       // 角度阈值：0.1f rad 约等于 5.7度

// --- 里程计信任死区阈值 (Zero-Velocity Update / Deadzone) ---
// 当编码器认为单帧位移小于这个值时，直接信任编码器，跳过 ICP
// 提示: 根据你底层电机的抖动和编码器分辨率来微调，通常 1~5mm 即可
#define ZUPT_DEADZONE_DIST_SQ    0.000004f  // 死区距离平方：0.000004f 代表 0.002m (2mm) 的平方
#define ZUPT_DEADZONE_ANGLE_RAD  0.0025f     // 死区角度：0.005f rad 约等于 0.14度

// --- 路径裁切保护参数 ---
// 说明：裁切基于路径弧长推进，而不是基于车前/车后；这样才能兼容倒车等效场景


#define PATH_PROJECTION_MAX_DIST_M     0.22f   // 最近投影距离过大时视为不可信，不更新裁切进度
#define PATH_PROGRESS_MAX_STEP_M       0.06f   // 单个 10ms 周期允许的最大弧长推进，防止 U 型弯跳段
#define PATH_TRIM_BACK_MARGIN_M        0.10f   // 裁切时保留少量历史余量，避免首点/显示突变
#define PATH_SEARCH_BACK_WINDOW        12      // 最近投影索引局部回看窗口
#define PATH_SEARCH_FORWARD_WINDOW     20      // 最近投影索引局部前看窗口
#define PATH_MIN_SEG_LEN_M             0.002f  // 路径线段最小有效长度，避免除零与数值放大

// --- 线路追踪参数 ---
#define TRACK_ENABLE_AUTONOMOUS_FOLLOW   1
#define TRACK_LOOKAHEAD_DIST_M           0.18f   // 预瞄距离
#define TRACK_HEADING_KP                 0.90f   // 航向误差P控制输出到角速度(deg/s per deg)
#define TRACK_CRUISE_LINEAR_M_S          0.65f   // 默认巡航线速度
#define TRACK_MAX_LINEAR_M_S             1.0f   // 上层最终输出线速度限制
#define TRACK_MAX_YAW_RATE_DEG_S         60.0f   // 上层最终输出角速度限制
#define TRACK_TURN_SLOWDOWN_START_DEG    10.0f   // 超过该航向误差开始降线速
#define TRACK_TURN_SLOWDOWN_STOP_DEG     60.0f   // 超过该航向误差线速度归零
#define TRACK_GOAL_SLOWDOWN_DIST_M       0.25f   // 接近终点开始降速
#define TRACK_GOAL_STOP_DIST_M           0.05f   // 终点停车距离
#define AUTO_RETURN_HOME_TRIGGER_DIST_M  TRACK_GOAL_STOP_DIST_M
#define TRACK_HEADING_STOP_DEG           8.0f    // 终点附近允许停止的航向误差
#define TRACK_MIN_LINEAR_M_S             0.08f   // 非停车状态下的最小前进线速度
#define TRACK_ENABLE_LINEAR_SLEW         1
#define TRACK_LINEAR_ACCEL_LIMIT_M_S2    1.60f   // 线速度斜坡限速


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
// 【新增】定义里程计的起始位置 (单位：米)——>右下角原点的右手系
#define INITIAL_ODOM_X     1.05f     // 例如：起始 X 设为 1.5 米
#define INITIAL_ODOM_Y     3.15f     // 例如：起始 Y 设为 2.0 米
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
