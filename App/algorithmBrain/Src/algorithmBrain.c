//
// Created by lmtgy on 2026/3/27.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\algorithmBrain\Src\algorithmBrain.c

#include "algorithmBrain.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

void StartAlgorithmBrain(void *argument)
{
    // 你的 ICP匹配、双层地图更新、A*寻路逻辑
    for(;;)
    {

        osDelay(100); // 暂时代替，后续用队列阻塞
    }
}