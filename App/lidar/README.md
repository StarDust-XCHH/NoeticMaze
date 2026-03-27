这是一份为你量身定制的、符合标准软件工程规范的 `README` 文档。它详细记录了该雷达路由节点的设计哲学、数据结构以及与其他外设/线程的交互契约。

你可以直接将其复制到你的工程目录下的 `README_LIDAR_NODE.md` 文件中。

---

# 📡 模块文档：雷达 DMA 路由与预处理节点 (Lidar Route Node)

## 1. 节点概述
* **任务名称:** `StartLidarRouteTask` (Task 3)
* **执行性质:** 事件驱动型 (非阻塞)，由硬件外设中断唤醒。
* **优先级:** 高 (Priority 2 / High)，需确保雷达数据不溢出。
* **核心职责:** 1. 承接 DMA 高速接收的底层字节流，进行协议校验与 DSP 加速解析。
    2. 整合完整的 360° 点云地图 (Map)。
    3. 结合 IMU 偏航角速度，执行**“运动学跳帧/视神经过滤”**，剔除冗余帧。
    4. 采用**零拷贝 (Zero-Copy) 内存池机制**，将点云指针路由至下游算法节点。

---

## 2. 核心数据结构

为了在 FreeRTOS 中极压 RAM 消耗，本系统采用 **定长内存池 + 指针传递** 的架构。队列中流转的永远是 4 字节的内存地址，而非庞大的数组。

### 2.1 一帧完整点云：`LidarMap_t`
这是系统中雷达数据的最小逻辑单元，占用约 728 Bytes。
```c
typedef struct {
    uint16_t distance[360];  // 360度距离数据 (单位: 毫米, 索引0~359对应绝对角度)
    uint32_t timestamp;      // 帧生成时的系统 OS Tick 时间戳 (用于算法延时补偿)
    uint32_t sweep_count;    // 雷达运行的绝对圈数 (用于丢帧检测)
} LidarMap_t;
```

### 2.2 线程间通讯凭证 (Queues)
系统维护了两个互逆的消息队列，深度均为 `MAP_POOL_SIZE` (当前设为 3，即三缓冲)。
* **`LidarFreeQueueHandle` (空闲缓冲池)**: 存放可被写入的空白 `LidarMap_t` 指针。
* **`LidarQueueHandle` (就绪数据池)**: 存放已装载完毕，等待下游算法提取的 `LidarMap_t` 指针。

---

## 3. 节点功能与工作流

本线程的内部工作流严格遵循以下时间线：

1. **休眠与唤醒:** 任务平时处于 `Blocked` 状态，直到串口空闲中断 (IDLE) 触发 `Lidar_ParseDMA_ISR`，通过任务通知 (`vTaskNotifyGiveFromISR`) 将其瞬间唤醒。
2. **协议解包:** 从软件 FIFO 提取数据，校验帧头 (`0xA0 0x50`) 和异或校验和。
3. **DSP 坐标系解算:** 调用 CMSIS-DSP 库的向量加法，将每帧的局部角度转换为 0-359 度的绝对极坐标，并存入当前的 `LidarMap_t` 结构体。
4. **运动学动态跳帧 (Kinematic Filter):** * 当检测到完整一圈 (360°) 拼装完毕后，查询系统全局的 Z 轴角速度 (`Get_Imu_Angular_Velocity_Z()`)。
    * **转弯期 ($>0.2\, \text{rad/s}$)**：小车位姿变化剧烈，**逐帧全部投递**，确保 SLAM 算法不丢失特征。
    * **直行/静止期 ($\le0.2\, \text{rad/s}$)**：环境相对静态，**直行 KINEMATIC_FILTER_JUMPING_FORWARD 帧仅投递 1 帧**，大幅降低下游 CPU 算力消耗；被跳过的帧会原地清空，复用内存。

---

## 4. 线程节点交互拓扑 (Interaction Topology)

本节点在系统中起到“承上启下”的网关作用，其交互关系如下：

```text
[硬件层] UART1 + DMA
       | (产生中断)
       v
[ISR层] Lidar_ParseDMA_ISR 
       | (写入 FIFO, 发送 Task Notification)
       |
==================== [边界：跨入 Task 3] ====================
       v
[本节点] StartLidarRouteTask <------- (查询角速度) ------- [节点] IMU 数据解算任务
       |
       |  (Put 指针)
       | --------------------------> [ LidarQueue (就绪队列) ]
       |                                       |
       |                                       | (Get 指针)
       |                                       v
       |                             [下游节点] SLAM / 导航避障算法任务
       |                                       |
       |                                       | (处理完毕，Put 指针)
       |                                       v
       ^---------------------------- [ LidarFreeQueue (空闲队列) ]
          (Get 指针，获取新空白内存)
```

---

## 5. ⚠️ 下游算法节点消费指南 (Contract)

下游任务（如避障、建图）在消费雷达数据时，**必须严格遵守内存借还契约**，否则会导致内存池干涸，整个雷达系统假死！

**标准处理流程规范：**
1. 阻塞等待 `LidarQueueHandle` 中的有效指针。
2. 读取指针指向的 `distance` 数组进行算法运算。
3. **(核心强制约束)** 运算结束后，必须将该指针 `Put` 回 `LidarFreeQueueHandle`。

**错误示范：**
* ❌ 从队列取回指针后，只用不还 -> 系统运行 3 圈后卡死。
* ❌ 试图 `free()` 该指针 -> 内存池是静态分配的，会导致 HardFault。

---


## 6. 下游算法节点代码模板 (Downstream Consumer Template)

为了确保内存池的稳定循环，任何需要读取雷达数据的下游任务（如 SLAM 建图、避障、特征提取等）都应参考以下标准模板进行编写。

### 核心设计原则
1. **阻塞等待**：任务平时应处于挂起状态，不占用 CPU，直到拿到雷达数据。
2. **只读借用**：拿到指针后，只读取数据，**绝对不要**修改内部数据或尝试释放（`free()`）该指针。
3. **按时归还**：算法执行完毕后，**必须第一时间内**将指针原样投递回 `LidarFreeQueue`。

### C 语言参考代码 (`slam_node.c` 示例)

```c
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "lidar.h" // 包含 LidarMap_t 和队列句柄声明

// 假设这是你的 SLAM 或避障任务
void StartSlamAlgorithmTask(void *argument) {
    LidarMap_t *received_map = NULL;
    osStatus_t status;

    for(;;) {
        // 1. [阻塞等待] 从就绪队列获取最新一帧雷达地图的指针
        // 使用 osWaitForever 意味着如果没有新数据，本任务会自动挂起让出 CPU
        status = osMessageQueueGet(LidarQueueHandle, &received_map, NULL, osWaitForever);

        if (status == osOK && received_map != NULL) {
            
            // ==========================================================
            // 2. [业务逻辑] 在这里执行你的核心算法 (只读操作)
            // ==========================================================
            
            // 示例：遍历 360 度寻找最近的障碍物
            uint16_t min_distance = 65535;
            int min_angle = -1;
            
            for (int i = 0; i < LIDAR_MAP_SIZE; i++) {
                uint16_t dist = received_map->distance[i];
                if (dist > 0 && dist < min_distance) { // 过滤掉 0 (无效点)
                    min_distance = dist;
                    min_angle = i;
                }
            }
            
            // 示例：结合时间戳进行位姿推算补偿
            // uint32_t frame_time = received_map->timestamp;
            // Run_Slam_Backend(received_map->distance, frame_time);

            // ==========================================================
            // 3. [强制归还] 算法执行完毕，务必将内存指针还给缓冲池
            // ==========================================================
            status = osMessageQueuePut(LidarFreeQueueHandle, &received_map, 0, 0);
            
            if (status != osOK) {
                // 严重错误：空闲队列已满（通常是因为有其他地方错误地塞入了多余的指针）
                // 可以在这里加入断言 (assert) 或错误日志打印
            }
            
            // 安全起见，归还后将本地指针置空，防止后续手误产生悬垂指针(Dangling Pointer)
            received_map = NULL; 
        }
    }
}
```

### 常见避坑指南 (Troubleshooting)

* **Q: 如果我的算法处理时间太长（比如一帧要算 300ms），会导致什么后果？**
    * **A:** 由于我们配置了跳帧机制，雷达本身并不会堆积太多数据。但如果算法真的极慢，导致 3 个缓冲块全被拿走且没来得及还，雷达底层的 DMA 解析任务（Task 3）在拿不到新空白内存时，会**主动丢弃**新来的圈数以保护系统不崩溃。你的算法节点依然安全，只是会感受到雷达刷新率变低。
* **Q: 我可以把这帧数据拷贝下来慢慢算吗？**
    * **A:** 尽量不要。拷贝 `LidarMap_t` 需要消耗额外约 728 字节的 RAM 和 CPU 搬运时间。如果确实需要长期持有历史帧用于比对，建议扩大 `MAP_POOL_SIZE`（例如增至 5），让指针在多个算法节点间流转后再归还。

---

这份 README 现在包含了从底层驱动架构到上层应用消费的完整契约。需要我帮你检查其他模块与这个雷达节点的配合逻辑吗？