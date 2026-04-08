//
// Created by lmtgy on 2026/3/27.
//
#include "printfDebug.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "arm_math.h"
#include "usart.h"         // 包含 huart3
#include "lidar.h"
#include "robot_state.h"
#include "bt_protocol.h"
#include "motor_pid.h"

// 引入外部句柄
extern osMessageQueueId_t LidarQueueHandle;
extern osMessageQueueId_t LidarFreeQueueHandle;

// 定义静态的发送包，放在全局数据区，防止爆栈，同时也保证 DMA 传输期间内存的绝对稳定
static RobotState_Packet_t state_pkg;
static LidarData_Packet_t  lidar_pkg;

extern osMutexId_t PrintfMutexHandle; // 引入 CubeMX 生成的 Mutex 句柄


// --- 新增：蓝牙接收相关宏与变量 ---
uint8_t bt_rx_raw_buf[BT_RX_BUF_SIZE]; // DMA 直接写入的原始缓冲区
char    bt_process_buf[BT_RX_BUF_SIZE]; // 解析用的转储缓冲区
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
    // 使用 HAL_UARTEx_ReceiveToIdle_DMA 可以在收到 IDLE 信号时自动触发回调
    // 这是 2026 年 STM32 开发中最推荐的变长数据接收方式
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, bt_rx_raw_buf, BT_RX_BUF_SIZE);
    // 关闭 DMA 半传输中断，防止干扰
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}


/**
 * @brief 蓝牙指令解析逻辑
 */
static void Process_Bluetooth_Command(void) {
    if (!bt_frame_ready) return;

    for (uint16_t i = 0; i <= (BT_RX_BUF_SIZE - sizeof(ControlPacket_t)); i++) {
        if (bt_process_buf[i] == 0x5A && bt_process_buf[i+1] == 0x5A) {
            ControlPacket_t *pkt = (ControlPacket_t *)&bt_process_buf[i];

            // 1. 校验和计算
            uint8_t sum = 0;
            uint8_t *ptr = (uint8_t *)pkt;
            for (int j = 0; j < sizeof(ControlPacket_t) - 1; j++) {
                sum += ptr[j];
            }

            if (sum == pkt->checksum) {
                // 2. 执行指令
                Motor_SetTargetAngle(pkt->angle, pkt->speed);

                // 3. 构造 ACK 包回发
                static AckPacket_t ack; // 静态变量防止栈溢出
                ack.header = 0x5A5A;
                ack.type = 0x04;
                ack.angle = pkt->angle;
                ack.speed = pkt->speed;

                // 计算 ACK 的校验和
                uint8_t ack_sum = 0;
                uint8_t *ack_ptr = (uint8_t *)&ack;
                for (int j = 0; j < sizeof(AckPacket_t) - 1; j++) ack_sum += ack_ptr[j];
                ack.checksum = ack_sum;

                // 4. 安全发送：检查串口是否忙碌
                // 如果 DMA 正在发状态包，这里可以等待或者使用阻塞方式发送(对于 10 字节的小包很快)
                if (huart3.gState == HAL_UART_STATE_READY) {
                    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&ack, sizeof(AckPacket_t));
                } else {
                    // 如果 DMA 忙，为了保证及时性，可以使用阻塞模式（非 DMA）
                    HAL_UART_Transmit(&huart3, (uint8_t*)&ack, sizeof(AckPacket_t), 10);
                }
            }
            break;
        }
    }
    bt_frame_ready = 0;
}


/**
 * @brief 蓝牙遥测任务 (替代原有的 Print 任务)
 */
void StartTaskPrint(void *argument) {
    LidarMap_t *received_map = NULL;
    RobotState_t current_robot_state;
    osStatus_t status;

    // 预填固定包头信息
    state_pkg.header = 0x55AA;
    state_pkg.type   = 0x01;

    lidar_pkg.header = 0x55AA;
    lidar_pkg.type   = 0x02;

    for(;;) {

        // ==========================================================
        // 1. [新增] 检查并处理蓝牙指令
        // ==========================================================
        Process_Bluetooth_Command();

        // ==========================================================
        // 1. [契约获取] 阻塞等待雷达一圈数据就绪
        // ==========================================================
        // 将等待时间缩短，比如 5ms，这样即使没雷达数据，指令也能被响应
        status = osMessageQueueGet(LidarQueueHandle, &received_map, NULL, pdMS_TO_TICKS(5));
        if (status == osOK && received_map != NULL) {

            // ==========================================================
            // 2. [数据拷贝] 提取雷达数据与圈数
            // ==========================================================
            arm_copy_q15((q15_t*)received_map->distance, (q15_t*)lidar_pkg.distance, 360);
            uint32_t current_sweep = received_map->sweep_count;

            // ==========================================================
            // 3. [契约归还] 立即释放内存池，不阻碍雷达底层继续接收
            // ==========================================================
            osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
            received_map = NULL;

            // ==========================================================
            // 4. [状态快照] 获取与当前雷达数据时间同步的 IMU/Odom 状态
            // ==========================================================
            Get_Robot_State_Snapshot(&current_robot_state);

            // ==========================================================
            // 5. [打包发送 A：机器人状态包]
            // ==========================================================
            state_pkg.sweep_count = current_sweep; // 用圈数作为时间戳同步对齐
            state_pkg.x           = current_robot_state.x_encoder;
            state_pkg.y           = current_robot_state.y_encoder;
            state_pkg.linear_vel  = current_robot_state.linear_vel_encoder;
            state_pkg.yaw         = current_robot_state.yaw;
            state_pkg.yaw_rate    = current_robot_state.yaw_rate;

            // 计算校验和：跳过 header(2) 和 type(1)，载荷长度 = 24字节
            // (sweep_count=4, x=4, y=4, vel=4, yaw=4, rate=4)
            state_pkg.checksum = Calc_Checksum((uint8_t*)&state_pkg + 3, 24);

            // 确保上次 DMA 发送完毕，超时设置 10ms
            if (Wait_UART_Ready(&huart3, 10) == 0) {
                HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&state_pkg, sizeof(RobotState_Packet_t));
            }

            // ==========================================================
            // 6. [打包发送 B：雷达数据包]
            // ==========================================================
            // 计算雷达校验和：仅计算 720 字节的载荷
            lidar_pkg.checksum = Calc_Checksum((uint8_t*)lidar_pkg.distance, 720);

            // 必须再次等待前一个状态包的 DMA 传输完成！
            // 状态包约 28 字节，460800 波特率下需要约 0.6ms
            if (Wait_UART_Ready(&huart3, 20) == 0) {
                HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&lidar_pkg, sizeof(LidarData_Packet_t));
            }
        }
    }
}