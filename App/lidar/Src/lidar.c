//
// Created by lmtgy on 2026/3/27.
//

#include "../Inc/lidar.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

void StartLidarRouteTask(void *argument)
{
    // 你的雷达跳帧决策与队列投递逻辑
    for(;;)
    {
        osDelay(100); // 暂时代替，后续用队列阻塞
    }
}