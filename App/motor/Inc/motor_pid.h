#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>
#include "arm_math.h"

// 定义电机运行状态
typedef enum {
    MOTOR_STATE_IDLE = 0,    // 待机/完全停止状态 (无动作，PWM=0，无PID运算)
    MOTOR_STATE_RUNNING,     // 正常闭环运行状态
    MOTOR_STATE_STOPPING,    // 平滑停车中 (目标速度设为0，正在向速度0收敛)
    MOTOR_STATE_ESTOP        // 紧急停车状态 (直接切断PID，PWM=0)
} Motor_State_t;

// --- 初始化与定时任务 ---
void Motor_PID_Init(void);
void Task_MotorPID_Update(void *param); // 放入调度器，10ms执行一次

// --- 速度控制接口 ---
void Motor_SetTargetRPS(float left_rps, float right_rps);
void Motor_SetTargetAngle(float angle, float linear_rps);

void Motor_Resume(void); // 从停止状态恢复运行

// --- 停车控制接口 ---
void Motor_NormalStop(void);    // 平滑停车：PID控制收敛到0后关闭输出
void Motor_EmergencyStop(void); // 紧急停车：立即PWM=0并停止PID
// --- 状态获取 ---
Motor_State_t Motor_GetState(void);

#endif // MOTOR_PID_H