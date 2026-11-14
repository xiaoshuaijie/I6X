# I6X

## 1. 模块作用 / Purpose

本模块从专用 UART 接收 Flysky 遥控链路，使用一个配置参数选择 SBUS 或 iBus，校验并发布统一的 `I6X::Data` Topic。SBUS 帧支持 16 个 11 位通道和链路状态；iBus 帧支持 14 个基础通道、校验和与在线状态。

This module receives a Flysky receiver link over a dedicated UART. One configuration parameter selects SBUS or iBus, and validated frames are published as a unified `I6X::Data` Topic. SBUS provides 16 11-bit channels and link flags; iBus provides 14 raw channels, a checksum and link status.

## 2. 协议与设备支持 / Protocol and Device Support

| 协议 / Protocol | UART | 帧 / Frame | 通道 / Channels |
| --- | --- | --- | --- |
| SBUS | 100000 baud, 8E2 | 25 bytes, header `0x0F` | 16 x 11-bit |
| iBus | 115200 baud, 8N1 | 32 bytes, length `0x20`, command `0x40` | 14 x 16-bit |

SBUS 标准信号为反相信号；BSP 必须提供反相输入或使用接收端的非反相输出。iBus 为非反相信号。SBUS 没有校验和，只检查帧头和帧尾；iBus 使用 `0xFFFF` 减去前 30 字节的校验和。

Standard SBUS is inverted; the BSP must provide an inverter or a receiver output that is already non-inverted. iBus is non-inverted. SBUS has no checksum and validates only its header and footer; iBus uses `0xFFFF` minus the first 30 bytes as its checksum.

接收机和发射机的协议能力必须按实际型号配置：

The protocol capability depends on the exact transmitter and receiver model:

I6 不支持 SBUS，只支持 iBus。6 通道 I6X 支持 SBUS 和 iBus；IA6 不支持 SBUS、只支持 iBus；6 通道 IA6B 支持 SBUS 和 iBus；IA10B 支持 SBUS 和 iBus，并提供 10 个通道。

I6 supports iBus only. The 6-channel I6X and 6-channel IA6B support both SBUS and iBus; IA6 supports iBus only; IA10B supports both protocols with 10 channels.

| 设备 / Device | SBUS | iBus | 通道 / Channels |
| --- | --- | --- | --- |
| I6 发射机 / I6 transmitter | 不支持 / No | 支持 / Yes | — |
| 6 通道 I6X 接收机 / 6-channel I6X receiver | 支持 / Yes | 支持 / Yes | 6 |
| IA6 接收机 / IA6 receiver | 不支持 / No | 支持 / Yes | 6 |
| 6 通道 IA6B 接收机 / 6-channel IA6B receiver | 支持 / Yes | 支持 / Yes | 6 |
| IA10B 接收机 / IA10B receiver | 支持 / Yes | 支持 / Yes | 10 |

模块会为协议帧提供各自协议的最大通道数组；`Data::channel_count` 表示协议帧中解析出的通道数（SBUS 为 16、iBus 为 14）。6 或 10 通道接收机按设备型号使用前 N 路，未使用的数组元素为零。

The module exposes the maximum channel array for the selected protocol. `Data::channel_count` is the number decoded from the protocol frame (16 for SBUS and 14 for iBus); a 6-channel or 10-channel receiver uses the first N channels for its model. Unused array elements remain zero.

## 3. 构造接口 / Constructor

```cpp
I6X(LibXR::UART& uart,
    const I6X::Param& param = {.topic_name = "i6x",
                               .offline_timeout_ms = 20,
                               .task_stack_depth_uart = 2048,
                               .thread_priority_uart = LibXR::Thread::Priority::HIGH,
                               .calibration = I6X::Calibration{},
                               .protocol = I6X::Protocol::IBUS});
```

`uart` 是必选依赖，且只能由一个接收消费者拥有。`param.protocol` 是协议切换参数：`I6X::Protocol::IBUS` 配置 115200 8N1，`I6X::Protocol::SBUS` 配置 100000 8E2。`topic_name` 必须非空且每个实例唯一；超时和线程栈大小必须大于零。

`uart` is a required dependency and its RX path must have one consumer. `param.protocol` is the protocol selector: `I6X::Protocol::IBUS` configures 115200 8N1 and `I6X::Protocol::SBUS` configures 100000 8E2. `topic_name` must be nonempty and unique per instance; timeout and stack size must be positive.

帧解析也可脱离硬件调用：

The decoder can also be called without hardware:

```cpp
I6X::Data data{};
auto result = I6X::ParseFrame(frame, data, sizeof(frame), I6X::Protocol::SBUS);
```

解析失败时输出保持不变。SBUS 故障帧仍返回 `OK`，但 `online` 为 `false`；iBus 校验失败或通道超范围时返回 `CHECK_ERR` 或 `OUT_OF_RANGE`。`Data::ch[]`、`Data::channel[]` 和 `Data::channels[]` 是同一数组的兼容别名。

The output is unchanged on a decode failure. An SBUS failsafe or lost-frame packet returns `OK` with `online == false`; an iBus checksum or range failure returns `CHECK_ERR` or `OUT_OF_RANGE`. `Data::ch[]`, `Data::channel[]` and `Data::channels[]` are aliases of the same storage.

## 4. Topic

| 名称 / Name | 方向 / Direction | 类型 / Type | 用途 / Meaning |
| --- | --- | --- | --- |
| `param.topic_name`（默认 / default `i6x`） | 发布 / Publish | `I6X::Data` | 通道值、协议、通道数量、校验和与在线状态 / Channels, protocol, channel count, checksum and link status |

消费者只在 `online == true` 时使用通道值；失联超时每次只发布一次全零离线消息，恢复后继续发布。回调运行在 UART 接收线程中，应及时返回。

Consumers should use channel values only when `online == true`. One zero-initialized offline message is published per timeout period, and publication resumes after a valid frame. Callbacks run in the UART RX thread and should return promptly.

## 5. 配置示例 / Configuration Example

I6X 是可直接放入 BSP 的Module。下面的实例选择 SBUS；切换到 iBus 时只需把 `protocol` 改为 `I6X::Protocol::IBUS`。

I6X can be included as a Module in a BSP. This instance selects SBUS; switch to iBus by changing only `protocol` to `I6X::Protocol::IBUS`.

```cpp
#include "I6X.hpp"

static I6X i6x_0(
    uart5,
    {.topic_name = "i6x",
     .offline_timeout_ms = 20,
     .task_stack_depth_uart = 2048,
     .thread_priority_uart = LibXR::Thread::Priority::HIGH,
     .calibration = I6X::Calibration{},
     .protocol = I6X::Protocol::SBUS});
```

## 6. 依赖与硬件 / Dependencies and Hardware

模块只依赖 LibXR 的 UART、Topic、Thread 和时间基准，不依赖其他 XRobot Module。BSP 必须先初始化 LibXR，注册匹配协议 UART，并保证接收队列至少能容纳 32 字节。

The module depends only on LibXR UART, Topic, Thread and timebase APIs. The BSP must initialize LibXR first, register a UART matching the selected protocol, and provide an RX queue that can hold at least 32 bytes.

使用 SBUS 时还必须处理信号反相；使用 iBus 时连接非反相 RX。不要让其他任务、驱动或调试输出同时读取同一个 UART 接收端。

SBUS installations must handle signal inversion; iBus uses a non-inverted RX connection. Do not let another task, driver or debug stream read the same UART RX path.
