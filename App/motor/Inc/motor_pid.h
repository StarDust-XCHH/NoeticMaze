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
void Motor_YawRatePID_Init(void);
void Task_MotorPID_Update(void); // 放入调度器，10ms执行一次

// --- 速度控制接口 ---
// 【修改】将原先的设置目标角度改为设置目标线速度和角速度
// linear_v_ms: 目标线速度 (m/s)
// angular_v_degs: 目标角速度 (deg/s)
void Motor_SetTargetVelocity(float linear_v_ms, float angular_v_degs);
float _Calculate_YawRate_Error_Correction(float current_yaw_rate);
void Motor_Resume(void); // 从停止状态恢复运行
float Motor_GetTargetYawRate(void);


// --- 停车控制接口 ---
void Motor_NormalStop(void);    // 平滑停车：PID控制收敛到0后关闭输出
void Motor_EmergencyStop(void); // 紧急停车：立即PWM=0并停止PID
// --- 状态获取 ---
Motor_State_t Motor_GetState(void);

#endif // MOTOR_PID_H
