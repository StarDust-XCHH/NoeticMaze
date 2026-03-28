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
    hlidar->last_dma_pos = 0; // [新增：初始化 DMA 位置]

    osMessageQueueAttr_t free_q_attr = { .name = "LidarFreeQ" };
    LidarFreeQueueHandle = osMessageQueueNew(MAP_POOL_SIZE, sizeof(LidarMap_t*), &free_q_attr);

    for (int i = 0; i < MAP_POOL_SIZE; i++) {
        LidarMap_t* ptr = &LidarMapPool[i];
        osMessageQueuePut(LidarFreeQueueHandle, &ptr, 0, osWaitForever);
    }

    // [修改：修复隐患三]
    // 在系统启动前（调度器未运行），调用带延时的 FreeRTOS API 会触发 HardFault
    // 因为前面刚塞了数据，这里必然有指针，直接将超时设为 0 即可安全取出
    osMessageQueueGet(LidarFreeQueueHandle, &hlidar->current_map, NULL, 0);
}

void Lidar_Start(Lidar_HandleTypeDef *hlidar) {
    uint8_t start_cmd[9] = {0xA5, 0x82, 05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
    HAL_UART_Transmit(hlidar->huart, start_cmd, sizeof(start_cmd), 100);

    // 启动空闲中断 + DMA 接收 (仅启动一次，后续交由 Circular 模式硬件托管)
    HAL_UARTEx_ReceiveToIdle_DMA(hlidar->huart, hlidar->rx_dma_buf, sizeof(hlidar->rx_dma_buf));
    __HAL_DMA_DISABLE_IT(hlidar->huart->hdmarx, DMA_IT_HT);
}



// 在 USART 的 HAL_UARTEx_RxEventCallback 中调用此函数
// [修改：重写 ISR 回调以支持 Circular DMA，修复隐患二]
void Lidar_ParseDMA_ISR(Lidar_HandleTypeDef *hlidar, uint16_t Size) {
    // 在 ReceiveToIdle_DMA 配合 Circular 模式时，Size 表示当前写入缓冲区的绝对偏移量
    uint16_t curr_pos = Size;

    if (curr_pos == hlidar->last_dma_pos) return; // 无新数据

    // 1. 从 rx_dma_buf 提取新到达的数据并送入软件 FIFO
    if (curr_pos > hlidar->last_dma_pos) {
        // 没有发生缓冲区回环，直接线性拷贝
        for (uint16_t i = hlidar->last_dma_pos; i < curr_pos; i++) {
            uint16_t next = (hlidar->head + 1) & LIDAR_FIFO_MASK;
            if (next != hlidar->tail) {
                hlidar->fifo[hlidar->head] = hlidar->rx_dma_buf[i];
                hlidar->head = next;
            }
        }
    } else {
        // 发生了缓冲区回环 (curr_pos < last_dma_pos)，分两段拷贝
        // a. 拷贝尾部剩余数据
        for (uint16_t i = hlidar->last_dma_pos; i < sizeof(hlidar->rx_dma_buf); i++) {
            uint16_t next = (hlidar->head + 1) & LIDAR_FIFO_MASK;
            if (next != hlidar->tail) {
                hlidar->fifo[hlidar->head] = hlidar->rx_dma_buf[i];
                hlidar->head = next;
            }
        }
        // b. 拷贝头部新数据
        for (uint16_t i = 0; i < curr_pos; i++) {
            uint16_t next = (hlidar->head + 1) & LIDAR_FIFO_MASK;
            if (next != hlidar->tail) {
                hlidar->fifo[hlidar->head] = hlidar->rx_dma_buf[i];
                hlidar->head = next;
            }
        }
    }

    // 更新游标，处理 DMA 恰好写满边界的情况
    hlidar->last_dma_pos = curr_pos;
    if (hlidar->last_dma_pos >= sizeof(hlidar->rx_dma_buf)) {
        hlidar->last_dma_pos = 0;
    }

    // 注意：这里删除了原先的 HAL_UARTEx_ReceiveToIdle_DMA 重启代码！硬件会自动继续循环！

    // 2. 发送任务通知唤醒 Task3
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

            // --- 修改后的过滤逻辑 ---
            // 只有在 [MIN, MAX] 范围内的点才被视为有效
            if (dis < LIDAR_MIN_RANGE || dis > LIDAR_MAX_RANGE) {
                continue;
            }
            // -----------------------

            final_angle = abs_angles[i]; // 这里只做赋值，声明已经移到了顶部
            // 归一化到 360 度以内 (因为极限值只有 539，减一次必进 360 以内)
            if (final_angle >= 360.0f) {
                final_angle -= 360.0f;
            }

            // 直接呼叫硬件 VCVTR 指令，单周期完成浮点转整型+四舍五入
            angle_idx = lrintf(final_angle);

            // 干掉 % 360，省下十几个 CPU 周期
            if (angle_idx >= 360) {
                angle_idx -= 360;
            }
            hlidar->current_map->distance[angle_idx] = dis;
        }
    }

    return sweep_completed;
}
/* ==========================================
 * Task 3: Lidar DMA 路由任务
 * ========================================== */
/* ==========================================
 * Task 3: Lidar DMA 路由任务
 * ========================================== */
void StartLidarRouteTask(void *argument) {
    uint32_t straight_frame_counter = 0;

    for(;;) {
        // 1. 等待串口 IDLE 中断唤醒
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        // [修改：修复隐患一]
        // 将 available 的计算放进死循环，动态判断是否还有剩余数据未处理
        // 防止多次中断累积导致的 available 没刷新，从而造成数据滞后
while (1) {
            // 🔥 优化 1：无分支计算可用数据量，强制使用无符号运算防止隐式提升
            uint16_t available = ((uint16_t)(hlidar1.head - hlidar1.tail)) & LIDAR_FIFO_MASK;

            // 如果积累的数据不足一帧，跳出 while(1) 循环，回到 for(;;) 顶部继续挂起等待
            if (available < LIDAR_FRAME_LEN) break;

            // 2. 校验数据帧
            // 🔥 优化 2.1：帧头校验干掉 %
            if ((hlidar1.fifo[hlidar1.tail] & 0xF0) == 0xA0 &&
                (hlidar1.fifo[(hlidar1.tail + 1) & LIDAR_FIFO_MASK] & 0xF0) == 0x50) {

                uint8_t frame[LIDAR_FRAME_LEN];
                uint8_t checksum = 0;

                // 🔥 优化 2.2：极速提取 Frame 数组，干掉内部的 %
                for (uint8_t i = 0; i < LIDAR_FRAME_LEN; i++) {
                    frame[i] = hlidar1.fifo[(hlidar1.tail + i) & LIDAR_FIFO_MASK];
                    if (i >= 2) checksum ^= frame[i];
                }

                uint8_t expected = ((frame[1] & 0x0F) << 4) | (frame[0] & 0x0F);

                if (checksum == expected) {
                    // 解析当前帧，判断是否完成了一整圈 360°
                    uint8_t is_sweep_done = Lidar_DecodeFrame_DSP(&hlidar1, frame);

                    if (is_sweep_done && hlidar1.current_map != NULL) {
                        // 1. 直接尝试发送给 ICP 任务队列
                        osStatus_t status = osMessageQueuePut(LidarQueueHandle, &hlidar1.current_map, 0, 0);

                        if (status == osOK) {
                            // 发送成功，将指针置空，稍后去申请新的内存块
                            hlidar1.current_map = NULL;
                        } else {
                            // 如果队列满了（ICP算不过来），说明处理能力达到极限
                            // 此时必须清空当前内存块以便下一圈重新填充
                            memset(hlidar1.current_map->distance, 0, sizeof(hlidar1.current_map->distance));
                        }

                        // 2. 确保手里始终有一个可用的空白内存块
                        if (hlidar1.current_map == NULL) {
                            if (osMessageQueueGet(LidarFreeQueueHandle, &hlidar1.current_map, NULL, 0) != osOK) {
                                // 如果内存池也干了，那只能等下一圈了
                            } else {
                                memset(hlidar1.current_map->distance, 0, sizeof(hlidar1.current_map->distance));
                            }
                        }
                    } // end of is_sweep_done

                    // 🔥 优化 3.1：极速推进游标
                    hlidar1.tail = (hlidar1.tail + LIDAR_FRAME_LEN) & LIDAR_FIFO_MASK;
                    continue;
                }
            }
            // 没对齐帧头或者校验失败，tail 步进1，丢弃当前错位字节
            // 🔥 优化 3.2：错位游标步进干掉 %
            hlidar1.tail = (hlidar1.tail + 1) & LIDAR_FIFO_MASK;
        } // end of while(1)
    } // end of for(;;)
}