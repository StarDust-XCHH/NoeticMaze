//
// Created by lmtgy on 2026/3/27.
//

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

void StartImuTask(void *argument)
{
    // 你的 200Hz IMU 读取与解算逻辑
    for(;;)
    {
        osDelay(5);
    }
}


// 预留接口
float Get_Imu_Angular_Velocity_Z(void)
{
    return 0.0f;
}