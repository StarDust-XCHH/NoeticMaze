//
// Created by lmtgy on 2026/3/27.
//


#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

// 注意：函数名必须和报错日志里缺少的名称一模一样
void StartMotionTask(void *argument)
{
    // 你的 100Hz 控制逻辑写在这里
    for(;;)
    {
        // 极速防撞、纯追踪、PID 等
        osDelay(10);
    }
}
