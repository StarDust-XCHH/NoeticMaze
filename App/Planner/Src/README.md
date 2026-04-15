
### 第四步：编写消费者线程的接收逻辑 (执行器 / 通信任务)

现在，任何需要路径的线程都可以优雅地订阅这个事件。当事件发生时，它们会被**同时唤醒**，并且都能通过指针安全地读取同一个乒乓缓冲片里的数据。

以**底层电机控制线程 (Actuator Task)** 为例：

```c
#include "planner_core.h" // 引入全局公告板和事件句柄

void StartActuatorTask(void *argument) {
    uint32_t last_processed_seq = 0; // 记录自己处理过的最新序列号

    for(;;) {
        // 1. 阻塞等待路径更新事件发生
        // 参数：句柄, 等待的Bit, 等待后不清除该Bit(osFlagsNoClear)或清除(默认), 永远等待
        osEventFlagsWait(PathEventHandle, EVENT_PATH_UPDATED, osFlagsWaitAny, osWaitForever);

        // 2. 被唤醒后，检查序列号是否真的是新的 (防止事件标志的误触发)
        if (g_current_safe_path.sequence != last_processed_seq) {
            
            // 3. 快速拷贝或直接使用指针
            // 【极致安全提示】：由于 Planner 已经把输出缓冲翻转了，
            // 此时 g_current_safe_path.path_ptr 指向的内存是绝对静止且安全的！
            Point2D* current_path = g_current_safe_path.path_ptr;
            int path_len = g_current_safe_path.path_len;

            // ... 将 current_path 送入你的 MPC 或 Pure Pursuit 算法中执行 ...
            
            // 4. 更新自己的历史记录
            last_processed_seq = g_current_safe_path.sequence;
        }
    }
}
```

以**通信线程 (Comm Task)** 为例，代码几乎一模一样：

```c
#include "planner_core.h"

void StartCommTask(void *argument) {
    uint32_t last_sent_seq = 0;

    for(;;) {
        // 同样等待这个大钟敲响
        osEventFlagsWait(PathEventHandle, EVENT_PATH_UPDATED, osFlagsWaitAny, osWaitForever);

        if (g_current_safe_path.sequence != last_sent_seq) {
            // 通过串口 DMA 将 g_current_safe_path.path_ptr 里的数据打包发给上位机
            // ... DMA 发送代码 ...

            last_sent_seq = g_current_safe_path.sequence;
        }
    }
}
```

### 为什么这套方案是完美的？

1. **零内存拷贝广播：** 不管你有 2 个还是 10 个消费者线程，传递的仅仅是一个带有指针的事件通知。所有线程底层读取的都是同一块物理内存（当前的乒乓缓冲片），极大节省了 RAM。
2. **绝对安全：** 当消费者线程被唤醒去读取 `Buffer_C[0]` 时，生产者 A* 线程已经通过 `out_ping_pong_idx ^= 1` 将自己的枪口对准了 `Buffer_C[1]`。在接下来的几十毫秒内，两边互不干扰。
3. **序列号防呆机制：** `last_processed_seq` 保证了即使某个线程跑得太快，也不会把同一条路径重复发给电机执行两次。