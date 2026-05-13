//
// Created by lmtgy on 2026/3/27.
//
#include "printfDebug.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

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
#include "planner_core.h"
#include "roboConifg.h"
#include "algorithmBrain.h" // <--- 新增：为了调用 Set_New_Target_Goal()

#define BT_UART_STUCK_TIMEOUT_MS 200U
#define BT_STATE_PERIOD_MS      20U
#define BT_STATE_MAX_GAP_MS     40U
#define BT_PRINT_LOOP_MS        5U


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
static bool s_lidar_pkg_pending = false;
#endif


// --- 新增：蓝牙接收相关宏与变量 ---
uint8_t bt_rx_raw_buf[BT_RX_BUF_SIZE]; // DMA 直接写入的原始缓冲区
char    bt_process_buf[BT_RX_BUF_SIZE+1]; // 解析用的转储缓冲区
volatile uint8_t bt_frame_ready = 0;    // 帧就绪标志位


// 蓝牙发送缓冲区

// 【重点】：静态分配 DMA 发送缓冲区，彻底杜绝爆栈
// DMA 必须使用这块稳定的内存，直到发送完成前不能被修改
static MapIcp_Packet_t tx_map_pkg;

// 静态分配 A* 路径 DMA 发送缓冲区
static PathData_Packet_t tx_path_pkg;
// 记录上位机已经发送过的最新路径序列号
static uint32_t last_sent_path_seq = 0;

static uint32_t s_bt_uart_busy_since = 0U;
static uint32_t s_last_state_tx_tick = 0U;
static uint32_t s_next_state_tx_tick = 0U;

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

static bool BtUart_IsReady(UART_HandleTypeDef *huart)
{
    uint32_t now = osKernelGetTickCount();

    if (huart->gState == HAL_UART_STATE_READY) {
        s_bt_uart_busy_since = 0U;
        return true;
    }

    if (s_bt_uart_busy_since == 0U) {
        s_bt_uart_busy_since = now;
    } else if ((now - s_bt_uart_busy_since) >= pdMS_TO_TICKS(BT_UART_STUCK_TIMEOUT_MS)) {
        HAL_UART_AbortTransmit(huart);
        s_bt_uart_busy_since = 0U;
    }

    return false;
}

static bool BtUart_TryTransmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return false;
    }

    if (!BtUart_IsReady(huart)) {
        return false;
    }

    if (HAL_UART_Transmit_DMA(huart, data, len) == HAL_OK) {
        return true;
    }

    return false;
}

static bool Send_Robot_State_DMA(uint32_t sweep_count)
{
    RobotState_t current_robot_state;

    if (!BtUart_IsReady(&huart3)) {
        return false;
    }

    Get_Robot_State_Snapshot(&current_robot_state);

    state_pkg.header = 0x55AA;
    state_pkg.type   = 0x01;
    state_pkg.sweep_count = sweep_count;
    state_pkg.x           = current_robot_state.global_fast_x_m;
    state_pkg.y           = current_robot_state.global_fast_y_m;
    state_pkg.linear_vel  = current_robot_state.linear_vel_encoder_m_s;
    state_pkg.yaw         = Robot_RadToDeg(current_robot_state.global_fast_theta_rad);
    state_pkg.yaw_rate    = current_robot_state.yaw_rate_deg_s;
    state_pkg.target_yaw_rate = Motor_GetTargetYawRate();
    state_pkg.checksum = Calc_Checksum((uint8_t*)&state_pkg + 3, 28);

    if (BtUart_TryTransmit(&huart3, (uint8_t*)&state_pkg, sizeof(RobotState_Packet_t))) {
        s_last_state_tx_tick = osKernelGetTickCount();
        s_next_state_tx_tick = s_last_state_tx_tick + pdMS_TO_TICKS(BT_STATE_PERIOD_MS);
        return true;
    }

    return false;
}


/**
 * @brief 检查全局公告板，如果有新路径则通过 DMA 发送
 */
void Send_Astar_Path_DMA(void) {
    TrimmedPathSnapshot path_snapshot;

    if (!BtUart_IsReady(&huart3)) {
        return;
    }

    // 发送执行层已经安全裁切后的路径视图，而不是原始全路径
    if (Get_Trimmed_Path_Snapshot(&path_snapshot) && path_snapshot.trimmed_sequence != last_sent_path_seq) {
        uint16_t pts = path_snapshot.path_len;
        Point2D* safe_path_ptr = path_snapshot.path_ptr;

        // 安全截断，防止越界
        if (pts > MAX_PATH_LEN) pts = MAX_PATH_LEN;

        // 3. 填充固定字段
        tx_path_pkg.header = 0x55AA;
        tx_path_pkg.type   = 0x06;
        tx_path_pkg.point_count = pts;

        // 计算变长载荷长度：2字节的 point_count + N个点的坐标(每个点 float x,y 共8字节)
        uint16_t points_bytes = pts * sizeof(Point2D);
        uint16_t payload_len = 2 + points_bytes;
        tx_path_pkg.data_len = payload_len;

        // 4. 将实际路径数据拷贝到 DMA 发送缓冲区
        if (pts > 0 && safe_path_ptr != NULL) {
            memcpy(tx_path_pkg.path_points, safe_path_ptr, points_bytes);
        }

        // 5. 计算校验和 (范围：Type[1] + DataLen[2] + Payload[payload_len])
        uint32_t check_range_len = 3 + payload_len;
        uint8_t sum = Calc_Checksum(((uint8_t*)&tx_path_pkg) + 2, check_range_len);

        // 6. 将校验和塞到数据的末尾
        uint8_t* p_raw = (uint8_t*)&tx_path_pkg;
        // 偏移量：Header(2) + Type(1) + DataLen(2) = 5，再加上载荷长度
        p_raw[5 + payload_len] = sum;

        // 7. 计算 DMA 实际要发送的总字节数
        uint16_t total_tx_size = (uint16_t)(5 + payload_len + 1);

        // 8. 启动 DMA 发送
        if (BtUart_TryTransmit(&huart3, p_raw, total_tx_size)) {
            // 9. 更新历史序列号，表示该帧已发送！
            last_sent_path_seq = path_snapshot.trimmed_sequence;
        }
    }
}

void Send_MapIcp_Data_DMA(void) {
    if (g_MapIcp_Ready == 1)
    {
        if (!BtUart_IsReady(&huart3)) {
            return;
        }

        // 1. 尝试加锁
        if (osMutexAcquire(MapDataMutexHandle, 0) == osOK) {

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
            uint16_t total_tx_size = (uint16_t)(5 + payload_len + 1);

            // 5. 启动非阻塞 DMA 发送
            (void)BtUart_TryTransmit(&huart3, p_raw, total_tx_size);
        }
    }
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

    // 考虑到 GoalPacket_t 和 ControlPacket_t 大小一致 (12字节)，统一减去 12
    for (uint16_t i = 0; i <= (BT_RX_BUF_SIZE - 12); i++) {
        if (local_buf[i] == 0x5A && local_buf[i+1] == 0x5A) {
            uint8_t type = local_buf[i+2];

            if (type == 0x03) {
                ControlPacket_t *pkt = (ControlPacket_t *)&local_buf[i];
                uint8_t sum = 0;
                uint8_t *ptr = (uint8_t *)pkt;
                for (int j = 0; j < sizeof(ControlPacket_t) - 1; j++) sum += ptr[j];

                if (sum == pkt->checksum) {
                    Motor_SetTargetVelocity(pkt->linear_vel, pkt->yaw_rate);

                    if (BtUart_IsReady(&huart3)) {
                        static AckPacket_t ack;
                        ack.header = 0x5A5A;
                        ack.type = 0x04;
                        ack.yaw_rate = pkt->yaw_rate;
                        ack.linear_vel = pkt->linear_vel;

                        uint8_t ack_sum = 0;
                        uint8_t *ack_ptr = (uint8_t *)&ack;
                        for (int j = 0; j < sizeof(AckPacket_t) - 1; j++) ack_sum += ack_ptr[j];
                        ack.checksum = ack_sum;

                        (void)BtUart_TryTransmit(&huart3, (uint8_t*)&ack, sizeof(AckPacket_t));
                    }
                    break;
                }
            }
            // <--- 新增：截获目标点指令
            else if (type == 0x07) {
                GoalPacket_t *pkt = (GoalPacket_t *)&local_buf[i];
                uint8_t sum = 0;
                uint8_t *ptr = (uint8_t *)pkt;

                // 计算 checksum (跳过结构体最后一个字节的自身 checksum 字段)
                for (int j = 0; j < sizeof(GoalPacket_t) - 1; j++) sum += ptr[j];

                if (sum == pkt->checksum) {
                    // 调用算法大脑抛出的安全更新接口
                    Set_New_Target_Goal(pkt->goal_x, pkt->goal_y);
                    break;
                }
            }
        }
    }
}

/**
 * @brief 蓝牙遥测与控制任务
 */
void StartTaskPrint(void *argument) {
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
        if (s_lidar_pkg_pending && BtUart_IsReady(&huart3)) {
            if (BtUart_TryTransmit(&huart3, (uint8_t*)&lidar_pkg, sizeof(LidarData_Packet_t))) {
                s_lidar_pkg_pending = false;
            }
        }

        status = osMessageQueueGet(LidarQueueHandle, &received_map, NULL, pdMS_TO_TICKS(5));
        if (status == osOK && received_map != NULL) {
            uint32_t current_sweep = received_map->sweep_count;

            // 打包并发送状态
            (void)Send_Robot_State_DMA(current_sweep);

            // 打包并发送雷达
            if (!s_lidar_pkg_pending && BtUart_IsReady(&huart3)) {
                arm_copy_q15((q15_t*)received_map->distance, (q15_t*)lidar_pkg.distance, 360);
                // 【新增 1】：提取精准测算的时间并赋给发送包
                lidar_pkg.scan_time = received_map->scan_time;
                // 【新增】：拷贝点数统计
                lidar_pkg.point_count = received_map->point_count;

                // 【新增 2】：更新雷达包的校验和计算长度
                // 1. 计算范围：distance(720) + scan_time(4) + point_count(4) = 728 字节
                lidar_pkg.checksum = Calc_Checksum((uint8_t*)lidar_pkg.distance, 728);

                // 2. 发送长度会自动根据 sizeof(LidarData_Packet_t) 增加
                // 总长度应该是 2(header) + 1(type) + 720(dist) + 4(time) + 4(count) + 1(sum) = 732 字节
                s_lidar_pkg_pending = !BtUart_TryTransmit(&huart3, (uint8_t*)&lidar_pkg, sizeof(LidarData_Packet_t));
            }

            osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
            received_map = NULL;
        }
#else
        uint32_t now = osKernelGetTickCount();
        bool state_due = (s_last_state_tx_tick == 0U) ||
                         ((int32_t)(now - s_next_state_tx_tick) >= 0);
        bool state_overdue = (s_last_state_tx_tick == 0U) ||
                             ((now - s_last_state_tx_tick) >= pdMS_TO_TICKS(BT_STATE_MAX_GAP_MS));
        bool tx_started = false;


        // 2. 【模式二：禁用雷达发送】轻量发送调度器，每轮最多启动一个DMA包
        if (state_overdue) {
            tx_started = Send_Robot_State_DMA(now);
        }

        if (!tx_started && g_MapIcp_Ready == 1) {
            Send_MapIcp_Data_DMA();
            tx_started = (huart3.gState != HAL_UART_STATE_READY);
        }

        if (!tx_started) {
            Send_Astar_Path_DMA();
            tx_started = (huart3.gState != HAL_UART_STATE_READY);
        }

        if (!tx_started && state_due) {
            (void)Send_Robot_State_DMA(now);
        }

        osDelay(BT_PRINT_LOOP_MS); // 释放 CPU，由发送调度器控制实际包频率
#endif
    }
}
