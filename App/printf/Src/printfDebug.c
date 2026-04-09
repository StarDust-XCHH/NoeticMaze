//
// Created by lmtgy on 2026/3/27.
//
#include "printfDebug.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "arm_math.h"
#include "usart.h"         // 包含 huart3
#include "lidar.h"
#include "robot_state.h"
#include "bt_protocol.h"
#include "motor_pid.h"
#include "roboConifg.h"



// 引入外部句柄
extern osMutexId_t PrintfMutexHandle;


#if ENABLE_LIDAR_BT_TX
extern osMessageQueueId_t LidarQueueHandle;
extern osMessageQueueId_t LidarFreeQueueHandle;
#endif

// 定义静态的发送包，放在全局数据区
static RobotState_Packet_t state_pkg;

#if ENABLE_LIDAR_BT_TX
static LidarData_Packet_t  lidar_pkg;
#endif


// --- 新增：蓝牙接收相关宏与变量 ---
uint8_t bt_rx_raw_buf[BT_RX_BUF_SIZE]; // DMA 直接写入的原始缓冲区
char    bt_process_buf[BT_RX_BUF_SIZE+1]; // 解析用的转储缓冲区
volatile uint8_t bt_frame_ready = 0;    // 帧就绪标志位

// GCC 编译器的 printf 底层输出函数重定向
int _write(int file, char *ptr, int len) {
    (void)file;
    return 0 ;
}

/**
 * @brief 计算单字节校验和
 */
static uint8_t Calc_Checksum(uint8_t *data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 等待串口 DMA 空闲的安全封装
 * @return 0: 成功, -1: 超时失败
 */
static int Wait_UART_Ready(UART_HandleTypeDef *huart, uint32_t timeout_ms) {
    uint32_t start_time = osKernelGetTickCount();
    // 轮询检查 UART 状态
    while (huart->gState != HAL_UART_STATE_READY) {
        if ((osKernelGetTickCount() - start_time) > pdMS_TO_TICKS(timeout_ms)) {
            return -1; // 超时
        }
        osDelay(1); // 让出 CPU 权限给其他任务
    }
    return 0;
}

/**
 * @brief 启动蓝牙 DMA 接收 (建议在任务循环开始前调用一次)
 */
static void Bluetooth_Start_Receive(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, bt_rx_raw_buf, BT_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

/**
 * @brief 蓝牙指令解析逻辑 (线程安全版)
 */
static void Process_Bluetooth_Command(void) {
    if (!bt_frame_ready) return;

    char local_buf[BT_RX_BUF_SIZE + 1];

    taskENTER_CRITICAL();
    memcpy(local_buf, bt_process_buf, BT_RX_BUF_SIZE);
    bt_frame_ready = 0;
    taskEXIT_CRITICAL();

    for (uint16_t i = 0; i <= (BT_RX_BUF_SIZE - sizeof(ControlPacket_t)); i++) {
        if (local_buf[i] == 0x5A && local_buf[i+1] == 0x5A) {
            ControlPacket_t *pkt = (ControlPacket_t *)&local_buf[i];

            if (pkt->type != 0x03) continue;

            uint8_t sum = 0;
            uint8_t *ptr = (uint8_t *)pkt;
            for (int j = 0; j < sizeof(ControlPacket_t) - 1; j++) {
                sum += ptr[j];
            }

            if (sum == pkt->checksum) {
                Motor_SetTargetVelocity(pkt->linear_vel, pkt->yaw_rate);

                static AckPacket_t ack;
                ack.header = 0x5A5A;
                ack.type = 0x04;
                ack.yaw_rate = pkt->yaw_rate;
                ack.linear_vel = pkt->linear_vel;

                uint8_t ack_sum = 0;
                uint8_t *ack_ptr = (uint8_t *)&ack;
                for (int j = 0; j < sizeof(AckPacket_t) - 1; j++) {
                    ack_sum += ack_ptr[j];
                }
                ack.checksum = ack_sum;

                if (Wait_UART_Ready(&huart3, 5) == 0) {
                    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&ack, sizeof(AckPacket_t));
                }
                break;
            }
        }
    }
}

/**
 * @brief 蓝牙遥测与控制任务
 */
void StartTaskPrint(void *argument) {
    RobotState_t current_robot_state;

    // 预填状态包固定包头
    state_pkg.header = 0x55AA;
    state_pkg.type   = 0x01;

#if ENABLE_LIDAR_BT_TX
    LidarMap_t *received_map = NULL;
    osStatus_t status;

    // 预填雷达包固定包头
    lidar_pkg.header = 0x55AA;
    lidar_pkg.type   = 0x02;
#endif

    for(;;) {
        // 1. 处理控制指令下发
        Process_Bluetooth_Command();

#if ENABLE_LIDAR_BT_TX
        // ==========================================================
        // 【模式一：启用雷达发送】基于雷达频率进行阻塞发送
        // ==========================================================
        status = osMessageQueueGet(LidarQueueHandle, &received_map, NULL, pdMS_TO_TICKS(5));
        if (status == osOK && received_map != NULL) {

            arm_copy_q15((q15_t*)received_map->distance, (q15_t*)lidar_pkg.distance, 360);
            uint32_t current_sweep = received_map->sweep_count;

            osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
            received_map = NULL;

            Get_Robot_State_Snapshot(&current_robot_state);

            // 打包并发送状态
            state_pkg.sweep_count = current_sweep;
            state_pkg.x           = current_robot_state.x_encoder;
            state_pkg.y           = current_robot_state.y_encoder;
            state_pkg.linear_vel  = current_robot_state.linear_vel_encoder;
            state_pkg.yaw         = current_robot_state.yaw;
            state_pkg.yaw_rate    = current_robot_state.yaw_rate;
            state_pkg.target_yaw_rate = Motor_GetTargetYawRate();

            state_pkg.checksum = Calc_Checksum((uint8_t*)&state_pkg + 3, 28);

            if (Wait_UART_Ready(&huart3, 10) == 0) {
                HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&state_pkg, sizeof(RobotState_Packet_t));
            }

            // 打包并发送雷达
            lidar_pkg.checksum = Calc_Checksum((uint8_t*)lidar_pkg.distance, 720);

            if (Wait_UART_Ready(&huart3, 20) == 0) {
                HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&lidar_pkg, sizeof(LidarData_Packet_t));
            }
        }
#else
        // ==========================================================
        // 【模式二：禁用雷达发送】按 50Hz 独立时序发送状态包
        // ==========================================================
        Get_Robot_State_Snapshot(&current_robot_state);

        state_pkg.sweep_count = osKernelGetTickCount(); // 替用系统 Tick 时间戳
        state_pkg.x           = current_robot_state.x_encoder;
        state_pkg.y           = current_robot_state.y_encoder;
        state_pkg.linear_vel  = current_robot_state.linear_vel_encoder;
        state_pkg.yaw         = current_robot_state.yaw;
        state_pkg.yaw_rate    = current_robot_state.yaw_rate;
        state_pkg.target_yaw_rate = Motor_GetTargetYawRate();

        state_pkg.checksum = Calc_Checksum((uint8_t*)&state_pkg + 3, 28);

        if (Wait_UART_Ready(&huart3, 5) == 0) {
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&state_pkg, sizeof(RobotState_Packet_t));
        }

        osDelay(20); // 释放 CPU，维持 50Hz 控制和遥测频率
#endif
    }
}