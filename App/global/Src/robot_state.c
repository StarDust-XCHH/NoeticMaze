//
// Created by lmtgy on 2026/4/8.
//

#include "robot_state.h"
#include "FreeRTOS.h"
#include "roboConifg.h"
#include "task.h"

// static 关键字将变量作用域限制在本文件内，彻底杜绝外部 extern 绕过保护
// static 关键字将变量作用域限制在本文件内，彻底杜绝外部 extern 绕过保护
// 使用 C99 语法，只给 x_encoder 和 y_encoder 赋初值，结构体中未指定的其他成员会自动初始化为 0
static RobotState_t global_robot_state = {
    .x_encoder = INITIAL_ODOM_X,
    .y_encoder = INITIAL_ODOM_Y
};
// ==========================================
// 1. 快照读取 (最安全的读取方式)
// ==========================================
void Get_Robot_State_Snapshot(RobotState_t *out_state) {
    if (out_state == NULL) return;

    // 进入临界区：暂停任务调度和中断，保证拷贝过程中数据不被篡改
    taskENTER_CRITICAL();
    // 结构体直接赋值，编译器会优化为高效的内存块拷贝 (类似 memcpy)
    *out_state = global_robot_state;
    taskEXIT_CRITICAL();
}

// ==========================================
// 2. 分类更新 (防止不同任务互相覆盖数据)
// ==========================================
void Update_Robot_IMU_State(float new_yaw, float new_yaw_rate, uint8_t ready) {
    taskENTER_CRITICAL();
    global_robot_state.yaw = new_yaw;
    global_robot_state.yaw_rate = new_yaw_rate;
    global_robot_state.imu_ready = ready; // 同步状态
    taskEXIT_CRITICAL();
}

void Update_Robot_Odom_State(float new_x, float new_y, float new_v) {
    taskENTER_CRITICAL();
    global_robot_state.x_encoder = new_x;
    global_robot_state.y_encoder = new_y;
    global_robot_state.linear_vel_encoder = new_v;
    taskEXIT_CRITICAL();
}

// ==========================================
// 3. 兼容旧的单个参数读取接口
// ==========================================
float Get_Global_Yaw(void) {
    float temp;
    taskENTER_CRITICAL();
    temp = global_robot_state.yaw;
    taskEXIT_CRITICAL();
    return temp;
}

float Get_Global_Yaw_Rate(void) {
    float temp;
    taskENTER_CRITICAL();
    temp = global_robot_state.yaw_rate;
    taskEXIT_CRITICAL();
    return temp;
}