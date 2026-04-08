//
// Created by lmtgy on 2026/3/27.
//

#include "printfDebug.h"


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "arm_math.h"
#include <stdio.h>
#include "lidar.h" // 确保包含了 LidarMap_t 和队列句柄
#include "usart.h"
#include "robot_state.h"  // <--- 新增：引入机器人状态头文件

extern osMutexId_t PrintfMutexHandle; // 引入 CubeMX 生成的 Mutex 句柄

// GCC 编译器的 printf 底层输出函数重定向
// GCC 编译器的 printf 底层输出函数重定向
int _write(int file, char *ptr, int len) {
    (void)file;

    // 1. 检查是否在中断上下文中 (ISR 内严禁使用 Mutex)
    if (__get_IPSR() != 0) {
        // 中断中直接轮询发送，不使用操作系统 API
        HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
        return len;
    }

    // 2. 检查 RTOS 调度器是否已经运行，且 Mutex 是否已经成功创建
    if (osKernelGetState() == osKernelRunning && PrintfMutexHandle != NULL) {
        // 系统运行中，使用互斥锁保护
        if (osMutexAcquire(PrintfMutexHandle, 100) == osOK) {
            HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
            osMutexRelease(PrintfMutexHandle);
            return len;
        } else {
            return 0; // 获取锁失败，丢弃打印
        }
    } else {
        // 调度器尚未启动（例如在 main 函数的硬件初始化阶段调用了 printf）
        HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
        return len;
    }
}

// 本地打印缓冲区（不在堆栈中分配，防止爆栈）
static uint16_t local_print_buffer[360];

/* * 任务：雷达数据队列测试与打印
 * 行为：作为标准消费者，严格遵守内存池契约获取雷达数据
 */
void StartTaskPrint(void *argument) {
    LidarMap_t *received_map = NULL;
    osStatus_t status;
    // 准备一个本地变量，用于存放状态快照
    RobotState_t current_robot_state;

    // --- 新增：线程启动心跳 ---
    printf("\r\n[DEBUG] Print Task is running, waiting for Lidar data...\r\n");

    for(;;) {
        // ==========================================================
        // 1. [契约第一步：阻塞借出] 等待雷达送来完整的一圈数据
        // ==========================================================
        status = osMessageQueueGet(LidarQueueHandle, &received_map, NULL, osWaitForever);

        if (status == osOK && received_map != NULL) {

            // ==========================================================
            // 2. [业务逻辑：高速拷贝] 把数据拿到自己的地盘
            // ==========================================================
            arm_copy_q15((q15_t*)received_map->distance, (q15_t*)local_print_buffer, 360);
            uint32_t current_sweep = received_map->sweep_count;

            // ==========================================================
            // 3. [契约第二步：立刻归还] 核心强制约束！
            // ==========================================================
            status = osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);

            if (status != osOK) {
                printf("\r\n[ERROR] Failed to return Lidar Memory Block!\r\n");
            }
            received_map = NULL; // 斩断联系

            // ==========================================================
            // 3. 安全获取里程计快照
            // ==========================================================
            // 调用你定义的快照接口，它内部使用了临界区保护，确保 X, Y, Yaw 是同一时刻的数据
            Get_Robot_State_Snapshot(&current_robot_state);

            // 4. 执行打印 (利用你已有的 Mutex 保护的 printf)
            printf("\r\n--- [Snapshot Sweep: %lu] ---", current_sweep);

            // 打印位置与速度 (保留 3 位小数，对应毫米级精度)
            // 注意：若打印显示为 0.000，请确认是否在工程设置中开启了 printf float 支持
            printf("\r\n[Odom] X: %.3f m | Y: %.3f m | Vel: %.2f m/s",
                   current_robot_state.x_encoder,
                   current_robot_state.y_encoder,
                   current_robot_state.linear_vel_encoder);

            // 打印姿态
            printf("\r\n[Pose] Yaw: %.1f deg | Rate: %.1f deg/s\r\n",
                   current_robot_state.yaw,
                   current_robot_state.yaw_rate);

            printf("-----------------------------\r\n");

            // for (uint16_t i = 0; i < 360; i++) {
            //     if (local_print_buffer[i] > 0) {
            //         printf("[%3d:%4d] ", i, local_print_buffer[i]);
            //     }
            //
            //     if (i % 10 == 9) {
            //         printf("\r\n");
            //     }
            //
            //     // RTOS 友好的时间切片：每打印 20 个点让出一下 CPU 和 Mutex
            //     if (i % 20 == 19) {
            //         osDelay(2);
            //     }
            // }
            printf("\r\n=== End of Sweep ===\r\n");

        }
    }
}

