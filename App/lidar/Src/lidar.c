//
// Created by lmtgy on 2026/3/27.
//

#include "lidar.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

Lidar_HandleTypeDef hlidar1;
osMessageQueueId_t LidarFreeQueueHandle; // 空闲缓冲池队列

// 静态内存池分配，防止动态分配造成内存碎片
static LidarMap_t LidarMapPool[MAP_POOL_SIZE];

// 预先计算好 40 个点相对于起始角度的偏移量 (DSP 加速用)
static const float32_t Lidar_Angle_Offsets[40] = {
    0.000f,  0.705f,  1.410f,  2.115f,  2.820f,
    3.525f,  4.230f,  4.935f,  5.640f,  6.345f,
    7.050f,  7.755f,  8.460f,  9.165f,  9.870f,
   10.575f, 11.280f, 11.985f, 12.690f, 13.395f,
   14.100f, 14.805f, 15.510f, 16.215f, 16.920f,
   17.625f, 18.330f, 19.035f, 19.740f, 20.445f,
   21.150f, 21.855f, 22.560f, 23.265f, 23.970f,
   24.675f, 25.380f, 26.085f, 26.790f, 27.495f
};

// 外部陀螺仪数据获取接口 (假设你有这个接口，用于判断是否转弯)
extern float Get_Imu_Angular_Velocity_Z(void);

/* ==========================================
 * 底层驱动与中断
 * ========================================== */

void Lidar_Init(Lidar_HandleTypeDef *hlidar, UART_HandleTypeDef *huart) {
    memset(hlidar, 0, sizeof(Lidar_HandleTypeDef));
    hlidar->huart = huart;
    hlidar->last_start_angle = 0.0f;

    // 创建一个深度相同的空闲队列用于管理内存池
    // 注意：CubeMX 已经生成了 LidarQueueHandle，这里我们只需创建 FreeQueue
    osMessageQueueAttr_t free_q_attr = { .name = "LidarFreeQ" };
    LidarFreeQueueHandle = osMessageQueueNew(MAP_POOL_SIZE, sizeof(LidarMap_t*), &free_q_attr);

    // 将所有内存块指针投入空闲队列
    for (int i = 0; i < MAP_POOL_SIZE; i++) {
        LidarMap_t* ptr = &LidarMapPool[i];
        osMessageQueuePut(LidarFreeQueueHandle, &ptr, 0, osWaitForever);
    }

    // 从空闲队列取出一个作为当前正在解析的 Buffer
    osMessageQueueGet(LidarFreeQueueHandle, &hlidar->current_map, NULL, osWaitForever);
}

void Lidar_Start(Lidar_HandleTypeDef *hlidar) {
    uint8_t start_cmd[9] = {0xA5, 0x82, 05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
    HAL_UART_Transmit(hlidar->huart, start_cmd, sizeof(start_cmd), 100);
    HAL_UARTEx_ReceiveToIdle_DMA(hlidar->huart, hlidar->rx_dma_buf, sizeof(hlidar->rx_dma_buf));

    // 关闭过半中断，减少系统开销
    __HAL_DMA_DISABLE_IT(hlidar->huart->hdmarx, DMA_IT_HT);
}

// 在 USART 的 HAL_UARTEx_RxEventCallback 中调用此函数
void Lidar_ParseDMA_ISR(Lidar_HandleTypeDef *hlidar, uint16_t size) {
    // 1. 将数据写入 FIFO
    for (uint16_t i = 0; i < size; i++) {
        uint16_t next = (hlidar->head + 1) % LIDAR_FIFO_SIZE;
        if (next != hlidar->tail) {
            hlidar->fifo[hlidar->head] = hlidar->rx_dma_buf[i];
            hlidar->head = next;
        }
    }

    // 2. 重启 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(hlidar->huart, hlidar->rx_dma_buf, sizeof(hlidar->rx_dma_buf));
    __HAL_DMA_DISABLE_IT(hlidar->huart->hdmarx, DMA_IT_HT);

    // 3. 发送任务通知唤醒 Task3 (非阻塞，极速返回)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(LidarRouteTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ==========================================
 * 解析核心函数 (Task 内联调用)
 * ========================================== */

// 解析一帧数据，如果检测到新的一圈完成，返回 1，否则返回 0
// 解析一帧数据，如果检测到新的一圈完成，返回 1，否则返回 0
static uint8_t Lidar_DecodeFrame_DSP(Lidar_HandleTypeDef *hlidar, const uint8_t *payload) {
    // --- 1. 将所有变量声明统一放在作用域的最顶部 (兼容所有编译器) ---
    uint8_t sweep_completed = 0;
    uint16_t angle_raw;
    float32_t start_angle;
    float32_t abs_angles[40];
    uint16_t i;
    uint16_t base;
    uint16_t dis;
    float32_t final_angle;
    int angle_idx;

    // --- 2. 开始执行逻辑 ---
    angle_raw = (uint16_t)(((uint16_t)(payload[3] & 0x7FU) << 8) | payload[2]);
    start_angle = (float32_t)angle_raw * 0.015625f;

    // 批量计算 40 个点的绝对角度
    arm_offset_f32((float32_t*)Lidar_Angle_Offsets, start_angle, abs_angles, 40);

    // 检测一圈是否结束 (角度发生突变)
    if (start_angle < hlidar->last_start_angle && (hlidar->last_start_angle - start_angle) > 100.0f) {
        hlidar->sweep_count++;
        // 确保 current_map 不为空时再写入属性，防止硬错误
        if (hlidar->current_map != NULL) {
            hlidar->current_map->sweep_count = hlidar->sweep_count;
            hlidar->current_map->timestamp = osKernelGetTickCount();
        }
        sweep_completed = 1;
    }
    hlidar->last_start_angle = start_angle;

    // 写入当前 Map
    if (hlidar->current_map != NULL) {
        for (i = 0; i < LIDAR_POINTS_PER_FRAME; i++) {
            base = 4 + (i << 1);
            dis = (uint16_t)(((uint16_t)payload[base + 1] << 8) | payload[base]);

            if (dis == 0) continue;

            final_angle = abs_angles[i]; // 这里只做赋值，声明已经移到了顶部
            if (final_angle >= 360.0f) final_angle -= 360.0f;

            angle_idx = (int)(final_angle + 0.5f) % 360;
            hlidar->current_map->distance[angle_idx] = dis;
        }
    }

    return sweep_completed;
}
/* ==========================================
 * Task 3: Lidar DMA 路由任务
 * ========================================== */
void StartLidarRouteTask(void *argument) {
    uint32_t straight_frame_counter = 0;
    const float TURN_THRESHOLD = 0.2f; // 角速度转弯判定阈值 (根据实际单位调整)

    for(;;) {
        // 1. 等待串口 IDLE 中断唤醒 (超时时间 100ms 兜底)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        uint16_t available = (hlidar1.head >= hlidar1.tail) ?
                             (hlidar1.head - hlidar1.tail) :
                             (LIDAR_FIFO_SIZE - hlidar1.tail + hlidar1.head);

        // 2. 从 FIFO 中提取并校验数据帧
        while (available >= LIDAR_FRAME_LEN) {
            if ((hlidar1.fifo[hlidar1.tail] & 0xF0) == 0xA0 &&
                (hlidar1.fifo[(hlidar1.tail + 1) % LIDAR_FIFO_SIZE] & 0xF0) == 0x50) {

                uint8_t frame[LIDAR_FRAME_LEN];
                uint8_t checksum = 0;

                for (uint8_t i = 0; i < LIDAR_FRAME_LEN; i++) {
                    frame[i] = hlidar1.fifo[(hlidar1.tail + i) % LIDAR_FIFO_SIZE];
                    if (i >= 2) checksum ^= frame[i];
                }

                uint8_t expected = ((frame[1] & 0x0F) << 4) | (frame[0] & 0x0F);

                if (checksum == expected) {
                    // 解析当前帧，判断是否完成了一整圈 360°
                    uint8_t is_sweep_done = Lidar_DecodeFrame_DSP(&hlidar1, frame);

                    if (is_sweep_done && hlidar1.current_map != NULL) {
                        // --------------------------------------------------
                        // 运动学跳帧决策逻辑 (视觉视神经过滤)
                        // --------------------------------------------------
                        float angular_z = Get_Imu_Angular_Velocity_Z(); // 获取实时角速度
                        uint8_t should_send = 0;

                        if (fabs(angular_z) > TURN_THRESHOLD) {
                            // 转弯状态：逐帧发送
                            should_send = 1;
                            straight_frame_counter = 0;
                        } else {
                            // 直行状态：直行 10 帧发 1 帧
                            straight_frame_counter++;
                            if (straight_frame_counter >= KINEMATIC_FILTER_JUMPING_FORWARD) {
                                should_send = 1;
                                straight_frame_counter = 0;
                            }
                        }

                        if (should_send) {
                            // a. 将装满数据的当前 Map 指针投递给视觉算法处理队列
                            osStatus_t status = osMessageQueuePut(LidarQueueHandle, &hlidar1.current_map, 0, 0);

                            if (status == osOK) {
                                // 投递成功，当前指针已被队列接管，置空
                                hlidar1.current_map = NULL;
                            }
                        } else {
                            // 如果跳帧，不清空当前指针，但在下一圈解析前应当把旧数据清除
                            memset(hlidar1.current_map->distance, 0, sizeof(hlidar1.current_map->distance));
                        }

                        // b. 如果当前指针为空 (已投递)，则从空闲队列取回一个新指针交还给 DMA 解析逻辑
                        if (hlidar1.current_map == NULL) {
                            // 如果空闲队列没取到（算法任务处理太慢），先等待，避免数据被覆盖
                            if (osMessageQueueGet(LidarFreeQueueHandle, &hlidar1.current_map, NULL, pdMS_TO_TICKS(10)) != osOK) {
                                // 处理异常：丢帧处理（在此处你也可以选择不挂起，直接丢弃新一圈数据）
                            } else {
                                // 拿到新 Buffer，清零准备接客
                                memset(hlidar1.current_map->distance, 0, sizeof(hlidar1.current_map->distance));
                            }
                        }
                    } // end of is_sweep_done

                    hlidar1.tail = (hlidar1.tail + LIDAR_FRAME_LEN) % LIDAR_FIFO_SIZE;
                    available -= LIDAR_FRAME_LEN;
                    continue;
                }
            }
            hlidar1.tail = (hlidar1.tail + 1) % LIDAR_FIFO_SIZE;
            available--;
        }
    }
}