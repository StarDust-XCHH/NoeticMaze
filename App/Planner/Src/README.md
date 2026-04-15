### 架构级

已经重新通读了 [`App/Planner`](App/Planner) 与 [`App/algorithmBrain`](App/algorithmBrain) 当前实现。基于现状，我给你的最终结论是：

**现在最需要做的，不是继续局部补 patch，而是把系统明确成 双地图双线程架构，并顺着这条主线一次性修掉剩余 bug。**

你的方向已经基本正确：
- [`MAX_GRID_SIZE`](App/Planner/Inc/planner_config.h:35) 改成 `40`
- 在 [`StartAlgorithmBrain()`](App/algorithmBrain/Src/algorithmBrain.c:49) 中完成 `2cm -> 10cm` 降采样
- 由 SLAM 线程判断是否需要重规划
- [`StartPlannerTask()`](App/Planner/Src/planner_core.c:62) 专注做 A* 与平滑

下面给你一份**完整修复 bug + 对齐 SLAM 与 A* 线程的落地方案**。

---

# 一、我重新审查后确认的现状

## 1. 分辨率和物理范围现在已经基本被你纠正到正确方向
当前：
- [`MAP_RES`](App/algorithmBrain/Inc/map_core.h:17) = `0.02f`
- [`MAP_SIZE`](App/algorithmBrain/Inc/map_core.h:18) = `200`
- 高精图覆盖 `4m x 4m`

而 planner 已经改为：
- [`PLANNER_MAP_RES`](App/Planner/Inc/planner_config.h:14) = `0.1f`
- [`MAX_GRID_SIZE`](App/Planner/Inc/planner_config.h:35) = `40`

这意味着：
- planner 低精图物理范围也变成 `4m x 4m`
- 映射倍率固定为 `5 x 5`

这一点是整个架构成立的基础。

---

## 2. 但 [`process_map_update()`](App/Planner/Src/Cstar_map_core.c:13) 现在仍然是旧协议逻辑，不适合当前系统
你现在的实现仍然是：
- `full map` 时走 [`memcpy()`](App/Planner/Src/Cstar_map_core.c:24)
- `delta` 时按单格 set planner map

这对当前系统是**结构性错误**，原因有三条：

### 错误 A
它默认输入图和输出图同分辨率。
但现在真实情况是：
- 输入 SLAM 图 2cm
- 输出 planner 图 10cm

### 错误 B
它默认 payload 的编码和 planner packed grid 兼容。
但当前：
- SLAM 图是 2bit/cell，见 [`GetMapState()`](App/algorithmBrain/Inc/map_core.h:35)
- planner 图是 4bit/cell 语义，见 [`set_grid_val()`](App/Planner/Inc/Cstar_map_core.h:26)

### 错误 C
它把 `grid_size` 和 `res` 当作 planner 可变运行参数，但 planner 在当前板端架构里应该是固定配置。

所以这个函数必须重写，不是微调。

---

## 3. Planner 主线程的核心框架是可保留的，但地图写入契约必须改
[`StartPlannerTask()`](App/Planner/Src/planner_core.c:62) 现在做了三件重要的事：
- 从 [`ReqQueueHandle`](Core/Src/freertos.c:178) 取最新请求
- 交换 [`s_write_map_ptr`](App/Planner/Src/planner_core.c:53) / [`s_read_map_ptr`](App/Planner/Src/planner_core.c:54)
- 用 [`g_map`](App/Planner/Src/Cstar_map_core.c:8) 指向读缓冲后做 [`astar_plan()`](App/Planner/Src/planner_algo.c:82)

这个结构本身是可以保留的。

但它的前提是：

**SLAM 线程写入 [`s_write_map_ptr`](App/Planner/Src/planner_core.c:53) 时，写进去的必须已经是最终可供 planner 使用的 10cm 图。**

也就是说，planner 不负责再理解 SLAM 原图。

---

## 4. A* 与平滑器本体基本可用，但还残留一个关键 silent-failure bug
[`pq_push()`](App/Planner/Src/priority_queue.c:21) 仍然是：
- 满了就 [`return`](App/Planner/Src/priority_queue.c:22)

虽然你已经把 [`MAX_PQ_SIZE`](App/Planner/Inc/planner_config.h:43) 改到 `2500`
这在 `40x40=1600 cell` 下大概率够用，风险比之前小很多。

但这个逻辑仍然有一个坏处：
- 一旦未来参数变化
- 或启发式导致 open set 峰值偏大
- 仍然会 silently fail

所以这个点仍然建议修。

---

# 二、重新定义最终正确架构

你现在系统最正确、最稳的架构应该是下面这样：

```mermaid
flowchart LR
A[激光与里程计输入] --> B[SLAM线程维护2cm global_map]
B --> C[SLAM线程降采样成10cm planner_map]
C --> D[SLAM线程对10cm图做膨胀]
D --> E[SLAM线程判断是否触发重规划]
E --> F[ReqQueue发送规划请求]
F --> G[Planner线程交换地图缓冲]
G --> H[Astar]
H --> I[路径平滑]
I --> J[发布全局路径公告板]
```

这个架构下，职责边界非常清晰：

## SLAM 线程负责
在 [`StartAlgorithmBrain()`](App/algorithmBrain/Src/algorithmBrain.c:49) 中负责：
- 维护 2cm 高精地图
- 生成 10cm 规划图
- 做障碍膨胀
- 检测旧路径是否失效
- 判断是否重规划
- 发送规划请求

## Planner 线程负责
在 [`StartPlannerTask()`](App/Planner/Src/planner_core.c:62) 中负责：
- 接收规划请求
- 使用最新 10cm 图做 A*
- 做路径平滑
- 发布路径

## 严格禁止
- planner 线程直接读 SLAM 原始 2cm 图
- [`process_map_update()`](App/Planner/Src/Cstar_map_core.c:13) 再去兼容旧仿真协议 full copy
- 外部 payload 直接覆盖 planner packed bytes

---

# 三、当前代码中仍然存在的 bug 和错位点

下面是我建议你纳入这次整改的一次性修复清单。

---

## Bug 1: [`process_map_update()`](App/Planner/Src/Cstar_map_core.c:13) 仍是错误语义
### 现状
- full-map 走 [`memcpy()`](App/Planner/Src/Cstar_map_core.c:24)
- delta 走单格 set

### 风险
- 与 2cm -> 10cm 桥接模型完全不一致
- 一旦继续沿用，会把错误地图写进 planner 缓冲
- affinity bit 语义也可能被脏写破坏

### 修复原则
把 [`process_map_update()`](App/Planner/Src/Cstar_map_core.c:13) 改成：
- **不再接收旧协议 payload**
- **只做 2cm SLAM 图到 10cm planner 图的整图降采样/重建**

我建议它最终语义变成类似：
- 输入：SLAM 图指针或直接读取 [`global_map`](App/algorithmBrain/Inc/map_core.h:25)
- 输出：写入 [`s_write_map_ptr`](App/Planner/Src/planner_core.c:53)

它的内部循环应是：
- 遍历 planner `40 x 40`
- 每格对应 SLAM `5 x 5`
- 聚合 25 个高精 cell
- 用 [`set_grid_val()`](App/Planner/Inc/Cstar_map_core.h:26) 写 occupancy
- 保留 affinity bit 由 planner 自己控制

---

## Bug 2: 当前 planner 与 SLAM 之间还没有正式接入桥接调用
### 现状
[`algorithmBrain.c`](App/algorithmBrain/Src/algorithmBrain.c:340) 和 [`algorithmBrain.c`](App/algorithmBrain/Src/algorithmBrain.c:343) 仍然是 TODO。

### 风险
- 现在 SLAM 线程虽然维护了高精图
- 但 planner 线程没有稳定、清晰、同尺度的数据来源
- 最后重规划条件无法成立

### 修复原则
在 [`StartAlgorithmBrain()`](App/algorithmBrain/Src/algorithmBrain.c:272) 地图更新后，正式接入：
1. 生成 planner 10cm 图
2. 对 planner 图做膨胀
3. 判断是否需要规划
4. 若需要则向 [`ReqQueueHandle`](Core/Src/freertos.c:178) 投递 [`PlannerReqMsg`](App/Planner/Inc/planner_core.h:22)

---

## Bug 3: [`pq_push()`](App/Planner/Src/priority_queue.c:21) 静默失败
### 现状
满了直接 [`return`](App/Planner/Src/priority_queue.c:22)

### 风险
- A* 失败没有诊断信息
- 会变成“偶现找不到路径”而不是明确报错

### 修复原则
建议最少做以下一种：
- 给 [`PriorityQueue`](App/Planner/Inc/priority_queue.h) 增加 overflow 标志
- 或给 [`astar_plan()`](App/Planner/Src/planner_algo.c:82) 增加“open set overflow -> 直接失败码”

这样后续调试时能分清：
- 真无路
- 还是队列容量问题

---

## Bug 4: planner 图上的障碍语义与实际机器人安全边界还未完全统一
### 现状
A* 内部通过 [`evaluate_node_cost()`](App/Planner/Src/planner_algo.c:50) 做动态碰撞和分级惩罚。

### 问题
如果 SLAM 线程只做降采样、不做 planner blocking 图一致化，会出现两个判断体系：
- 重规划触发时一套
- A* 内部评估时一套

### 修复原则
必须统一：
- **路径阻断检测所用地图**
- **A* 搜索所用地图**

最稳妥的办法是：
- SLAM 线程输出的是 planner occupancy 图
- 路径阻断检测基于这个 occupancy 图 + 同一套 inflation 规则
- A* 继续用 [`evaluate_node_cost()`](App/Planner/Src/planner_algo.c:50) 做细化代价

也就是说：
- “是否完全堵死” 与 “路径代价高低” 分层处理
- 但 blocking 判断要统一基准

---

## Bug 5: 路径平滑会放大贴墙风险，因此 planner 图必须先安全化
### 现状
[`double_smooth_path()`](App/Planner/Src/path_smoother.c:98) 与 [`generate_safe_corner_path()`](App/Planner/Src/path_smoother.c:135) 都会把折线进一步拉直、倒角。

### 风险
若原始 A* 路径已经靠墙：
- 平滑后更容易擦边
- 甚至穿过原本仅靠离散判定可通的狭窄通道

### 修复原则
因此本次架构里必须明确：
- **在 10cm planner 图上做膨胀**
- 路径阻断检测也基于膨胀后的 blocking 语义

---

# 四、推荐的最终地图桥接方案

这是整个整改的核心。

## 1. 双图模型
### SLAM 高精图
见 [`global_map`](App/algorithmBrain/Inc/map_core.h:25)
- 2cm
- `200 x 200`
- 2bit occupancy

### Planner 低精图
见 [`s_map_ping`](App/Planner/Src/planner_core.c:51) / [`s_map_pong`](App/Planner/Src/planner_core.c:52)
- 10cm
- `40 x 40`
- 4bit/cell 语义

---

## 2. 降采样规则
对 planner 每个 cell `(px, py)`：
- 对应 SLAM 范围：`[px*5 .. px*5+4] x [py*5 .. py*5+4]`
- 聚合这 25 个细格

### 建议规则
- 只要存在 [`MAP_OCCUPIED`](App/algorithmBrain/Inc/map_core.h:15) -> planner = occupied
- 否则只要存在 [`MAP_FREE`](App/algorithmBrain/Inc/map_core.h:14) -> planner = free
- 否则 -> planner = unknown

这是最保守、最不容易漏障碍的规则。

---

## 3. 膨胀规则
建议在 **10cm planner 图** 上做。

也就是：
- 先得到基础 occupancy planner 图
- 再在 planner 尺度上额外生成一个 blocking 语义

推荐先简化为：
- 半径 `1 cell` 的硬膨胀
- 后续若太保守再调

原因：
- 你的 [`PHYSICAL_RADIUS`](App/Planner/Inc/planner_config.h:16) = `0.10f`
- 而 planner res = `0.1f`
- 1 cell 膨胀就已经很贴近车体尺度

如果再叠加 [`SAFE_SMOOTH_RAD`](App/Planner/Inc/planner_config.h:23) = `0.15f`
整体会更稳。

---

# 五、推荐的重规划触发机制

你前面提出的触发条件是对的，我这里给出最终定稿版。

## 触发条件 1: 目标变化
只要新目标和当前路径对应目标不一致，就立即重规划。

建议：
- 目标格变化超过 `1 planner cell` 就算变化

---

## 触发条件 2: 当前无有效路径
如果：
- [`g_current_safe_path.path_len`](App/Planner/Src/planner_core.c:139) 为 0
- 或当前根本没有成功路径

则直接触发规划。

这个条件必须补上，不能只靠“终点变化 / 路径阻断 / 位移超阈值”。

---

## 触发条件 3: 当前路径被新障碍阻断
这个条件必须保留，而且要用**膨胀后的 10cm planner 图**去判断。

建议做法：
- 沿当前全局路径采样路径点
- 映射到 planner cell
- 只要命中 occupied 或 inflated occupied，就判定路径失效

这应是高优先级触发。

---

## 触发条件 4: 起点偏离旧规划起点过大
你原先说 `1m`
我现在仍建议你改成：
- `4~5 个 planner cell`
- 即 `0.4m ~ 0.5m`

原因：
- 对 4m 局部图来说 1m 太大
- 会让路径拖得太久才刷新

---

## 触发条件 5: debounce
建议增加一个小抖动抑制：
- 连续若干帧判定 blocked 才触发
- 或重规划之间留最小间隔

这样可避免单帧噪声不断打断 A*。

---

# 六、线程协作边界应如何改

这部分要讲清楚，否则以后还会乱。

---

## 1. SLAM 线程可以写什么
[`StartAlgorithmBrain()`](App/algorithmBrain/Src/algorithmBrain.c:49) 可以写：
- [`global_map`](App/algorithmBrain/Inc/map_core.h:25)
- planner 写缓冲 [`s_write_map_ptr`](App/Planner/Src/planner_core.c:53)
- 重规划请求队列 [`ReqQueueHandle`](Core/Src/freertos.c:178)
- 必要时置位 [`g_abort_astar`](App/Planner/Src/planner_core.c:57)

但它不能：
- 直接改 [`s_read_map_ptr`](App/Planner/Src/planner_core.c:54)
- 直接改 [`g_current_safe_path`](App/Planner/Src/planner_core.c:33)

---

## 2. Planner 线程可以读什么
[`StartPlannerTask()`](App/Planner/Src/planner_core.c:62) 只读：
- 交换后的 [`s_read_map_ptr`](App/Planner/Src/planner_core.c:54)
- 请求里的 start/goal
- 历史路径 [`s_prev_path`](App/Planner/Src/planner_core.c:42)

并负责：
- 更新 [`g_current_safe_path`](App/Planner/Src/planner_core.c:33)
- 广播 [`EVENT_PATH_UPDATED`](App/Planner/Inc/planner_core.h:54)

---

## 3. `s_write_map_ptr` / `s_read_map_ptr` 的正确契约
正确契约是：
- SLAM 永远写 [`s_write_map_ptr`](App/Planner/Src/planner_core.c:53)
- Planner 永远读 [`s_read_map_ptr`](App/Planner/Src/planner_core.c:54)
- 交换只允许 [`StartPlannerTask()`](App/Planner/Src/planner_core.c:62) 做

这个契约你当前已经基本有了，应该继续保留。

---

# 七、我建议你这次整改涉及的文件与具体动作

---

## 1. [`App/Planner/Inc/planner_config.h`](App/Planner/Inc/planner_config.h)
### 已完成方向
- [`MAX_GRID_SIZE`](App/Planner/Inc/planner_config.h:35) 改为 `40`

### 还要确认
- 注释改掉，当前注释还写着 `5m/0.1=50`，这已经和实际配置冲突
- [`MAX_CELLS`](App/Planner/Inc/planner_config.h:36) 注释也应同步改成 `1600`
- [`MAX_MAP_BYTES`](App/Planner/Inc/planner_config.h:40) 注释同步成 `1600 * 4bit = 800 bytes`

这虽然是注释问题，但必须修，不然后面会继续误导。

---

## 2. [`App/Planner/Inc/Cstar_map_core.h`](App/Planner/Inc/Cstar_map_core.h)
### 需要改
- 重写 [`process_map_update()`](App/Planner/Inc/Cstar_map_core.h:63) 的接口语义
- 不再保留 old sim delta 的注释/语义
- 如果可以，直接改函数签名

建议最终让这个接口名字和职责更一致，例如偏向：
- planner map rebuild
- slam to planner sync

如果你暂时不想重命名，至少要彻底改注释语义。

---

## 3. [`App/Planner/Src/Cstar_map_core.c`](App/Planner/Src/Cstar_map_core.c)
### 这是最关键的整改点
需要重写为：
- planner 写缓冲清空
- 从 SLAM 图按 `5x5` 聚合
- 写入 planner occupancy
- 可选叠加膨胀

当前这份实现里的 [`memcpy()`](App/Planner/Src/Cstar_map_core.c:24) 必须删掉。

---

## 4. [`App/Planner/Src/priority_queue.c`](App/Planner/Src/priority_queue.c)
### 需要改
- 处理 [`pq_push()`](App/Planner/Src/priority_queue.c:21) overflow
- 最少给出可观测失败路径

---

## 5. [`App/Planner/Src/planner_core.c`](App/Planner/Src/planner_core.c)
### 保留主结构，但建议补两点
- 若收到的是重复且过期请求，继续保留“抽干队列只处理最后一条”逻辑，这很好
- 但建议把“重规划原因”编码进 [`PlannerReqMsg`](App/Planner/Inc/planner_core.h:22)，方便后期调试

例如：
- goal changed
- path blocked
- drift exceeded

不是必须，但强烈建议。

---

## 6. [`App/Planner/Inc/planner_core.h`](App/Planner/Inc/planner_core.h)
### 需要改
建议为 [`PlannerReqMsg`](App/Planner/Inc/planner_core.h:22) 增加：
- 触发原因字段
- 本次请求序号或地图版本号字段

这样可以避免后期完全不知道本次重规划是为什么触发的。

---

## 7. [`App/algorithmBrain/Src/algorithmBrain.c`](App/algorithmBrain/Src/algorithmBrain.c)
### 这是另一处核心整改点
你需要在 [`algorithmBrain.c`](App/algorithmBrain/Src/algorithmBrain.c:340) / [`algorithmBrain.c`](App/algorithmBrain/Src/algorithmBrain.c:343) 两个 TODO 位置正式落地：
- 降采样生成 planner 图
- 路径阻断检查
- 位移阈值检查
- 目标变化检查
- 发送规划请求
- 必要时打断 A*

这部分应成为你新架构的总控。

---

# 八、建议的最终执行顺序

为了降低一次性改动风险，我建议按下面顺序实施。

## 第一批：先把地图桥接改对
1. 修 [`planner_config.h`](App/Planner/Inc/planner_config.h) 注释与常量一致性
2. 重写 [`process_map_update()`](App/Planner/Src/Cstar_map_core.c:13)
3. 明确 planner 图只接受 10cm 结果图

## 第二批：把 SLAM 与 planner 正式接上
4. 在 [`algorithmBrain.c`](App/algorithmBrain/Src/algorithmBrain.c:272) 后增加整图降采样
5. 增加 planner 图膨胀
6. 增加路径阻断检测
7. 增加重规划请求发送

## 第三批：收尾 bug 修复
8. 修 [`pq_push()`](App/Planner/Src/priority_queue.c:21) silent failure
9. 给 [`PlannerReqMsg`](App/Planner/Inc/planner_core.h:22) 加触发原因字段
10. 必要时补 debug 打印或状态码

---

# 九、最终结论

这次完整整改的目标可以概括成一句话：

**把系统从 旧仿真协议驱动的单图假设，重构成 板端固定参数下的 双地图双线程架构。**

最终形态应该是：
- [`App/algorithmBrain`](App/algorithmBrain) 维护 2cm 高精图
- [`App/Planner`](App/Planner) 只消费 10cm 规划图
- [`process_map_update()`](App/Planner/Src/Cstar_map_core.c:13) 不再做旧协议搬运，而是做 2cm -> 10cm 的桥接
- SLAM 线程负责判断何时重规划
- Planner 线程只负责高效求解与路径发布

我对你现在这个方向的最终判断是：

**架构方向已经对了，剩下的问题主要集中在：旧的 [`process_map_update()`](App/Planner/Src/Cstar_map_core.c:13) 语义没有彻底删除、SLAM 到 planner 的桥接还没真正接上、以及少数 silent-failure 细节未收尾。把这三类问题按上面的顺序一次性改完，这套系统就会从“勉强可跑”进入“结构正确且可维护”的状态。**

接下来的实施建议已经明确：按这份方案切换到 [`code`](code) 模式，优先修改 [`App/Planner/Src/Cstar_map_core.c`](App/Planner/Src/Cstar_map_core.c)、[`App/algorithmBrain/Src/algorithmBrain.c`](App/algorithmBrain/Src/algorithmBrain.c)、[`App/Planner/Inc/planner_core.h`](App/Planner/Inc/planner_core.h)、[`App/Planner/Src/priority_queue.c`](App/Planner/Src/priority_queue.c) 这几处。



### 编写消费者线程的接收逻辑 (执行器 / 通信任务)

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