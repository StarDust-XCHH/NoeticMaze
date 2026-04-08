//
// Created by lmtgy on 2026/3/27.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\imu\Src\imu.c

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MPU6500.h"       // 你的 IMU 头文件
#include "roboConifg.h"
#include "robot_state.h"   // 引入全局状态 API

// 实例化一个局部的 IMU 句柄，只在这个 Task 里用
IMU_Yaw_Handler_t my_imu_handler = {0};

// 定义一个供外部只读的校准标志位 (加上 volatile 防止编译器过度优化缓存)
volatile uint8_t g_imu_is_calibrated = 0;

void StartImuTask(void *argument)
{
    // 1. 硬件初始化 (如果还没在 main 里初始化的话)
    // if(MPU6500Init() != HAL_OK) { Error_Handler(); }

    // 初始化句柄参数
    my_imu_handler.drift_rate = YAW_DRIFT_RATE;
    my_imu_handler.is_calibrated = 0;

    // 2. 准备绝对延时
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 严格 5ms = 200Hz

    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        // ==========================================
        // 1. DSP 数据解算 (无锁执行)
        // ==========================================
        IMU_Update(&my_imu_handler);

        // ==========================================
        // 2. 更新校准状态标志位 (单字节写入天然原子，无需加锁)
        // ==========================================
        g_imu_is_calibrated = my_imu_handler.is_calibrated;

        // ==========================================
        // 3. 使用 API 安全更新全局状态中枢
        // ==========================================
        if (my_imu_handler.is_calibrated) {
            // API 内部已经封装了 taskENTER_CRITICAL，这里直接调用即可
            Update_Robot_IMU_State(my_imu_handler.compensated_yaw,
                                   my_imu_handler.gyro_z,
                                   my_imu_handler.is_calibrated);
        } else {
            // 没校准完时，偏航角传 0，角速度正常传
            Update_Robot_IMU_State(0.0f, my_imu_handler.gyro_z,my_imu_handler.is_calibrated);
        }

        // ==========================================
        // 4. 精确延时
        // ==========================================
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}