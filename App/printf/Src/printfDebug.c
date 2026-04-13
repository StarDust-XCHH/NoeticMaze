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
#include "map_core.h"
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


// 蓝牙发送缓冲区

// 【重点】：静态分配 DMA 发送缓冲区，彻底杜绝爆栈
// DMA 必须使用这块稳定的内存，直到发送完成前不能被修改
static MapIcp_Packet_t tx_map_pkg;


// GCC 编译器的 printf 底层输出函数重定向
int _write(int file, char *ptr, int len) {
    (void)file;
    return 0 ;
}


void Send_MapIcp_Data_DMA(void) {
    if (g_MapIcp_Ready == 1)
    {
        if (Wait_UART_Ready(&huart3, 10) == 0) {
            // 1. 尝试加锁
            if (osMutexAcquire(MapDataMutexHandle, pdMS_TO_TICKS(5)) == osOK) {

                // 计算有效载荷长度 (12字节Pose + 2字节count + N字节地图)
                uint16_t payload_len = 12 + 2 + (g_Shared_MapIcp_Data.diff_count * 3);

                // 填充固定字段
                tx_map_pkg.header = 0x55AA;
                tx_map_pkg.type = 0x05;
                tx_map_pkg.data_len = payload_len;

                // 拷贝有效载荷部分 (icp_x 开始到 diff_payload 结束)
                // 地址偏移 5 字节 = header(2) + type(1) + data_len(2)
                memcpy(((uint8_t*)&tx_map_pkg) + 5, ((uint8_t*)&g_Shared_MapIcp_Data) + 5, payload_len);

                g_MapIcp_Ready = 0; // 释放生产信号
                osMutexRelease(MapDataMutexHandle);

                // 2. 计算校验和 (从 type 开始到 payload 结束，不包含 header)
                // 范围：Type(1) + DataLen(2) + Payload(payload_len)
                uint32_t check_range_len = 3 + payload_len;
                uint8_t sum = Calc_Checksum(((uint8_t*)&tx_map_pkg) + 2, check_range_len);

                // 3. 将校验位塞进数据的屁股后面
                uint8_t* p_raw = (uint8_t*)&tx_map_pkg;
                p_raw[5 + payload_len] = sum;

                // 4. 计算 DMA 最终发送长度: Header(2) + Type(1) + Len(2) + Payload(payload_len) + Checksum(1)
                uint32_t total_tx_size = 5 + payload_len + 1;

                // 5. 启动非阻塞 DMA 发送
                if (Wait_UART_Ready(&huart3, 10) == 0) {
                    HAL_UART_Transmit_DMA(&huart3, p_raw, total_tx_size);
                }
            }
        }
    }
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
    while (huart->gState != HAL_UART_STATE_READY) {
        if ((osKernelGetTickCount() - start_time) > pdMS_TO_TICKS(timeout_ms)) {
            // 【关键修复】超时后强行终止可能卡死的 DMA/UART 状态机，使其恢复 READY
            HAL_UART_AbortTransmit(huart);
            return -1;
        }
        osDelay(1);
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

            // 【新增 1】：提取精准测算的时间并赋给发送包
            lidar_pkg.scan_time = received_map->scan_time;
            // 【新增】：拷贝点数统计
            lidar_pkg.point_count = received_map->point_count;

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

            // 【新增 2】：更新雷达包的校验和计算长度
            // 1. 计算范围：distance(720) + scan_time(4) + point_count(4) = 728 字节
            lidar_pkg.checksum = Calc_Checksum((uint8_t*)lidar_pkg.distance, 728);

            // 2. 发送长度会自动根据 sizeof(LidarData_Packet_t) 增加
            // 总长度应该是 2(header) + 1(type) + 720(dist) + 4(time) + 4(count) + 1(sum) = 732 字节
            if (Wait_UART_Ready(&huart3, 20) == 0) {
                HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&lidar_pkg, sizeof(LidarData_Packet_t));
            }

        }
#else


        // 2. [新增] 检查是否有新的地图增量和ICP位姿准备就绪

        Send_MapIcp_Data_DMA();



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