# 接线图

![img.png](asset/img.png)

## imu

- VCC,3.3V,电源正极,供电
- GND,GND,电源地,共地
- SCL,PB13,SPI_SCK (时钟),提供通信时钟
- SDA,PC1,SPI_MOSI (主机输出),STM32 发数据给 MPU
- AD0,PC2,SPI_MISO (主机输入),MPU 发数据给 STM32
- NCS (或 CS),PC0,MPU_CS (片选),拉低时激活 SPI 模式

## lidar
- VCC——>5v
- GND——>GND
- 黄TX——>板载RX——>PA10
- 绿RX——>板载TX——>PA9

## motor

- TIM2·ch1ch2左轮——>PA0、PA1
- TIM2·ch3ch4右轮——>PB2、PB10
- TIM3·左轮——>PA6、PA7
- TIM4·右轮——>PB6、PB7



## bluetooth

- VCC——>3.3v
- GND——>GND
- TXD——>板载RX——>PC5
- RXD——>板载TX——>PC10

# 运行卡死排查

本节记录当前代码审查中发现的可能卡死根因、建议调试包设计，以及后续上位机可视化定位方案。这里只作为诊断记录，不代表已经修改固件协议或上位机代码。

## 可能卡死根因

按优先级从高到低排查：

1. **`MotionTask` 栈溢出**
   - `App/motor/Src/startMotion.c` 中 `Motion_Publish_Trimmed_Path()` 使用了局部数组 `Point2D trimmed_points[MAX_PATH_LEN]`。
   - `Point2D` 为两个 `float`，约 `8B`；`MAX_PATH_LEN=400` 时该数组固定约 `3200B`。
   - `Core/Src/freertos.c` 中 `MotionTask` 栈大小约为 `256 * 4 = 1024B`。
   - 该数组单独就超过任务栈大小，长路径时更容易写坏相邻内存，表现为卡死、HardFault、串口异常或任务不再调度。

2. **`MotionTask` 职责污染**
   - `MotionTask` 本应只做高频位姿合成、路径跟踪和速度输出。
   - 当前还承担“路径裁切”和“上位机剩余路径发布视图”的维护。
   - 这会把显示/遥测职责混入实时控制任务，增加栈压力和周期抖动风险。

3. **大路径蓝牙包发送超时**
   - `MAX_PATH_LEN=400` 时，路径包负载约为 `400 * sizeof(Point2D) = 3200B`，再加协议头和校验。
   - 921600 bps 下，实际发送 3200B 以上通常需要 30ms 级时间。
   - 当前 `Send_Astar_Path_DMA()`、`Send_MapIcp_Data_DMA()` 等路径中多处只等待 `10ms` 左右，可能导致 DMA 忙、发送失败、包丢失或遥测断流。

4. **SLAM/map 坐标越界**
   - 下位机 SLAM 地图参数为 `MAP_RES=0.02f`、`MAP_SIZE=200`、`MAP_OFFSET=0.0f`，物理覆盖范围是 `[0, 4)m`。
   - 若机器人位姿或雷达点落到该范围外，`world_to_grid()` 会夹到边界或返回失败。
   - 位姿长期越界可能导致地图更新异常、规划失败或表现为“卡住不动”。

5. **上下位机地图参数不一致**
   - 下位机地图为 `MAP_SIZE=200, MAP_OFFSET=0.0f`。
   - 上位机 `asset/Util/btData.py` 中使用 `STM32_MAP_SIZE=250, MAP_OFFSET=3.5`。
   - 这不会必然导致固件卡死，但会让目标点、地图边界和显示坐标产生误判，增加定位问题难度。

6. **A* 静默失败**
   - `astar_plan()` 返回 `0` 时，上位机无法区分失败原因。
   - 可能原因包括起点/终点越界、碰撞、无可行路径、优先队列 `PQ` 溢出、被 `g_abort_astar` 打断等。
   - 当前缺少失败原因上报，导致“大地图卡死”时很难判断是否卡在 Planner。

7. **`AlgorithmBrain` 栈压力**
   - `build_blocked_view_from_base()` 内部有约 `MAX_CELLS=1600` 字节的局部数组。
   - 叠加 SLAM、ICP、地图更新、planner bridge 调用链后，需要监控 `AlgorithmBrain` 栈余量。

8. **UART/DMA 忙导致遥测丢失**
   - 地图包、路径包、状态包共用 UART3 DMA。
   - 当路径包或地图增量包较大时，状态包和调试包可能被挤掉。
   - 如果没有独立调试阶段上报，现象只会是上位机“突然没数据”。

9. **HardFault/Error_Handler 无上报**
   - 当前 `Error_Handler()` 和 `HardFault_Handler()` 进入后，上位机通常只能看到数据停止。
   - 没有最后模块、最后阶段、错误码记录，无法区分栈溢出、非法访问、DMA 状态异常或断言失败。

## 建议调试包

建议新增一个轻量二进制调试包，类型号暂定：

```c
#define TYPE_DEBUG 0x08
```

建议包结构：

```c
#pragma pack(push, 1)
typedef struct {
    uint16_t header;     // 0x55AA
    uint8_t  type;       // 0x08
    uint32_t tick;       // osKernelGetTickCount()
    uint8_t  module;     // SLAM / PLANNER / MOTION / UART / FAULT
    uint8_t  stage;      // 当前执行阶段
    int16_t  code;       // 错误码、失败原因或状态码
    float    a;          // 通用参数1
    float    b;          // 通用参数2
    float    c;          // 通用参数3
    float    d;          // 通用参数4
    uint8_t  checksum;   // 与现有协议一致的单字节校验
} Debug_Packet_t;
#pragma pack(pop)
```

字段语义：

- `module + stage + code` 用于表示“当前执行到哪里”和“为什么失败”。
- `a/b/c/d` 根据模块复用，例如耗时、坐标、路径长度、栈余量、UART 超时计数。
- 调试包应低频发送，避免刷爆 UART3；建议先做 heartbeat，再逐步增加关键阶段打点。

建议模块枚举：

```c
enum {
    DBG_MOD_SLAM    = 1,
    DBG_MOD_PLANNER = 2,
    DBG_MOD_MOTION  = 3,
    DBG_MOD_UART    = 4,
    DBG_MOD_FAULT   = 5,
};
```

## 关键打点位置

### SLAM / AlgorithmBrain

- 取到雷达帧：记录 `sweep_count`、有效点数、雷达队列状态。
- ICP 开始/结束：记录 ICP 耗时、`result.x/y/theta`、匹配是否跳过。
- 地图更新结束：记录 `diff_cnt`、位姿是否越界、地图更新耗时。
- planner 请求投递：记录 start、goal、replan reason、`ReqQueue` 投递结果。
- 栈余量：周期性上报 `uxTaskGetStackHighWaterMark(AlgorithmBrainHandle)`。

### Planner

- 收到请求：记录 `cmd_type`、`reason`、start、goal。
- A* 开始/结束：记录耗时、raw path length、是否返回 `0`。
- 失败原因：区分起终点越界、碰撞、PQ overflow、无路径、被 abort。
- 平滑结束：记录 smoothed length、final path length、总耗时。
- 栈余量：周期性上报 `uxTaskGetStackHighWaterMark(PlannerTaskHandle)`。

### Motion

- 新路径加载：记录 path length、sequence、总弧长。
- 投影结果：记录 projection index、projection distance、progress。
- 裁切发布：记录 trimmed length、`tf_version`、`s_progress_s_m`。
- 追踪输出：记录 lookahead、heading error、linear command、yaw-rate command。
- 栈余量：重点上报 `uxTaskGetStackHighWaterMark(MotionTaskHandle)`。

### UART / TaskPrint

- 地图包发送：记录 payload length、DMA 启动是否成功、超时次数。
- 路径包发送：记录 point count、payload length、预计发送时间、超时次数。
- 状态包发送：记录发送周期是否稳定。
- DMA abort：记录发生次数和当时正在发送的包类型。

### Fault

- 在进入 `Error_Handler()` 或 HardFault 死循环前，尽量保存最后一次 `module/stage/code`。
- 如果来得及发送调试包，发送 `DBG_MOD_FAULT`。
- 如果来不及发送，至少保存在全局变量，方便调试器查看。

## 上位机显示建议

`asset/Util/btData.py` 建议增加 `TYPE_DEBUG = 0x08` 解析。

日志显示格式建议：

```text
[tick] module.stage code a b c d
```

同时维护每个模块最近一次阶段：

```text
SLAM:    last stage / tick / code
Planner: last stage / tick / code
Motion:  last stage / tick / code
UART:    last stage / tick / code
Fault:   last stage / tick / code
```

当系统卡死时，上位机最后一条 debug log 和各模块最后 stage 可以作为第一定位线索。

## 验证顺序

1. 先只发送低频 debug heartbeat，确认上位机能稳定解析。
2. 打开 `MotionTask` 栈余量上报，优先验证 `trimmed_points[MAX_PATH_LEN]` 是否导致栈风险。
3. 打开 UART 路径包和地图包超时统计，确认大路径包是否阻塞遥测。
4. 打开 Planner 失败原因上报，区分无路径、越界、PQ overflow 和 abort。
5. 用小地图和大地图分别测试，对比最后 debug stage、任务栈余量、路径长度和 UART 超时次数。

# 前端控制仲裁建议

本节只记录后续修改建议，不代表本轮已经修改固件协议、上位机代码或蓝牙解析逻辑。

## 当前风险

1. **`0x03` 手动控制包会直接写电机目标**
   - `App/printf/Src/printfDebug.c` 收到 `ControlPacket_t` 后会直接调用 `Motor_SetTargetVelocity(pkt->linear_vel, pkt->yaw_rate)`。
   - 自动路径跟踪线程也会周期性调用 `Motor_SetTargetVelocity()`。
   - 如果前端在展示或调试时周期性发送 `0x03`，手动控制和自动跟踪会抢写同一组目标速度。

2. **角速度单位注释存在风险**
   - `App/printf/Inc/bt_protocol.h` 中 `ControlPacket_t.yaw_rate` 注释写的是 `rad/s`。
   - `Motor_SetTargetVelocity()` 的接口语义是 `deg/s`。
   - 如果上位机按 `rad/s` 发送而固件按 `deg/s` 使用，手动控制的转向量会与自动跟踪不一致。

3. **前端开关可能改变实车行为**
   - 前端打开时若持续发送控制包，可能覆盖或扰动自动跟踪目标。
   - 前端关闭后自动跟踪独占电机目标，问题表现会不同。

## 后续建议

1. 增加明确的 `AUTO/MANUAL` 控制模式仲裁。
2. 手动控制包只在 `MANUAL` 模式生效，自动路径跟踪只在 `AUTO` 模式写电机目标。
3. 给手动控制增加超时，例如 200ms 内没有新 `0x03` 包则自动停车或回到安全状态。
4. 统一协议单位：建议将 `ControlPacket_t.yaw_rate` 明确为 `deg/s`，或在接收端显式从 `rad/s` 转为 `deg/s`。
5. 上位机显示页面与手动遥控页面分离，单纯可视化时不要发送 `0x03` 控制包。
