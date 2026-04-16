//
// Created by lmtgy on 2026/3/27.
// E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\lidar\Src\lidar.c

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


/* ==========================================
 * 底层驱动与中断
 * ========================================== */

void Lidar_Init(Lidar_HandleTypeDef *hlidar, UART_HandleTypeDef *huart) {
    memset(hlidar, 0, sizeof(Lidar_HandleTypeDef));
    hlidar->huart = huart;
    // 【新增】初始化时缓存无效
    hlidar->cache.is_valid = 0;
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

/**
 * @brief  工业级两帧插值解算器 (动态补偿电机波动)
 * @return 1:触发了跨圈(360°) 0:正常解析
 */
static uint8_t Lidar_DecodeFrame_Dynamic(Lidar_HandleTypeDef *hlidar, const uint8_t *payload) {
    uint8_t sweep_completed = 0;
    uint16_t angle_raw;
    float current_start_angle;

    // 1. 提取当前包(Frame i+1)的起始角度
    angle_raw = (uint16_t)(((uint16_t)(payload[3] & 0x7FU) << 8) | payload[2]);
    current_start_angle = (float)angle_raw * 0.015625f;

    // 2. 核心逻辑：如果手里有缓存的“上一帧(Frame i)”，才进行解算
    if (hlidar->cache.is_valid) {
        float cached_angle = hlidar->cache.start_angle;

        // 【极其重要】：计算真实物理夹角 AngleDiff
        float angle_diff = current_start_angle - cached_angle;

        // 处理 360° 跨界问题
        // 例如：上一帧是 358°，当前帧跨越了零点变成 2°
        // 2 - 358 = -356°，这显然不对。加上 360 得到真实的 4° 夹角。
        if (angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }

        // 【异常防爆保护】：如果电机被卡住，或者串口丢包严重，导致角度差畸形
        // 正常情况下，一帧跨度在 28° 左右。如果出现负数或超过 45°，说明物理层异常！
        // 此时放弃动态插值，强制使用默认步长保底，防止后续数组越界或全零覆盖。
        if (angle_diff < 0.0f || angle_diff > 45.0f) {
            angle_diff = 28.2f; // 使用标准期待值 (0.705 * 40)
        }

        // 计算出针对这一帧的绝对精确单点步长
        float dynamic_step = angle_diff / (float)LIDAR_POINTS_PER_FRAME;

        // 【跨圈判定】：
        // 如果当前角度(2°) 小于 缓存角度(358°)，并且差距巨大，说明肯定跨越了 0 度线！
        // 这一步判定放在这里，是因为我们要画完“上一帧”之后，再去封闭“上一圈”的地图。
        uint8_t is_wrap_around = 0;
        if (current_start_angle < cached_angle && (cached_angle - current_start_angle) > 100.0f) {
            is_wrap_around = 1;
        }

        // 渲染上一帧的 40 个点到 Map 中
        if (hlidar->current_map != NULL) {
            for (int i = 0; i < LIDAR_POINTS_PER_FRAME; i++) {
                uint16_t dis = hlidar->cache.distances[i];

                if (dis >= LIDAR_MIN_RANGE && dis <= LIDAR_MAX_RANGE) {
                    // 动态插值计算当前点的绝对角度
                    float final_angle = cached_angle + (float)i * dynamic_step;

                    // 归一化到 360 度以内
                    if (final_angle >= 360.0f) final_angle -= 360.0f;

                    // 调用硬件指令快速转整型
                    int angle_idx = lrintf(final_angle);

                    // 二次防越界保护
                    if (angle_idx >= 360) angle_idx -= 360;
                    if (angle_idx < 0) angle_idx = 0;

                    hlidar->current_map->distance[angle_idx] = dis;
                }
            }
        }

        // 累加时间同步的点数 (代表我们真实画完了 40 个点)
        hlidar->points_since_last_sweep += LIDAR_POINTS_PER_FRAME;

        // ==========================================================
        // 【封圈逻辑】：如果你画的这帧跨越了 0 度，说明【上一圈】正式圆满了！
        // ==========================================================
        if (is_wrap_around) {
            hlidar->sweep_count++;
            if (hlidar->current_map != NULL) {
                hlidar->current_map->sweep_count = hlidar->sweep_count;
                // 这里的 Timestamp 确实晚了一帧(约10ms)，但在低速自动驾驶中完全可接受
                hlidar->current_map->timestamp = osKernelGetTickCount();

                float real_scan_time = (float)hlidar->points_since_last_sweep / 5000.0f;
                if (real_scan_time > 0.07f && real_scan_time < 0.15f) {
                    hlidar->current_map->scan_time = real_scan_time;
                } else {
                    hlidar->current_map->scan_time = 0.1f;
                }

                hlidar->current_map->point_count = hlidar->points_since_last_sweep;
                // 清零计数器，因为下一帧(当前刚收到的包)属于新的一圈
                hlidar->points_since_last_sweep = 0;
            }
            sweep_completed = 1; // 告诉 Task 3，赶紧把这一圈丢进队列里发走！
        }
    }

    // 3. 将刚刚收到的【当前帧】存入缓存，它将成为下一次解析的【上一帧】
    hlidar->cache.start_angle = current_start_angle;
    for (int i = 0; i < LIDAR_POINTS_PER_FRAME; i++) {
        int base = 4 + (i << 1);
        hlidar->cache.distances[i] = (uint16_t)(((uint16_t)payload[base + 1] << 8) | payload[base]);
    }
    // 标记缓存已填满，下一包到来时就可以开始插值计算了
    hlidar->cache.is_valid = 1;

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
                    uint8_t is_sweep_done = Lidar_DecodeFrame_Dynamic(&hlidar1, frame);

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