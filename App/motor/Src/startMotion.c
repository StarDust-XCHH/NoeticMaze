//
// Created by lmtgy on 2026/3/27.
//

// 在 freertos.c 或对应的 task.c 中

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

// 引入你的各个驱动头文件
#include "pwm.h"
#include "encoder.h"
#include "motor_pid.h"
#include "robot_state.h"

// 假设 IMU 校准标志位为全局变量（可以通过信号量或事件标志组优化，此处先保持简单）
extern uint8_t g_imu_is_calibrated;

void StartMotionTask(void *argument)
{
    // 1. 硬件外设与算法的初始化 (务必在死循环前完成)
    PWM_Init();
    Encoder_Init();
    Motor_PID_Init();
    Motor_AnglePID_Init();

    // 2. 初始化绝对延时所需的时间戳变量
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 严格 10ms 周期

    // 获取当前时间
    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        // ==========================================
        // 1. 数据采集层
        // ==========================================
        // 更新编码器与里程计 (内部依赖 dt = 0.01s)
        Encoder_Update(NULL);

        // ==========================================
        // 2. 运动学与安全层 (Priority 4 的核心)
        // ==========================================
        // [TODO] 极速防撞逻辑：读取雷达/超声波标志位
        // if (Check_Collision_Danger()) { Motor_EmergencyStop(); }

        // [TODO] TF 坐标变换与 Pure Pursuit 轨迹跟踪
        // float desired_v, desired_w;
        // Run_Pure_Pursuit(&desired_v, &desired_w);

        // ==========================================
        // 3. 底层闭环控制层
        // ==========================================
        // 执行 PID 运算并输出 PWM (传入 IMU 校准状态以供拦截)
        Task_MotorPID_Update(&g_imu_is_calibrated);

        // ==========================================
        // 4. 严格绝对延时
        // ==========================================
        // 任务会被挂起，直到下一个 10ms 周期到来
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
