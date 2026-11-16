# I6X

## 1. 模块作用 / Purpose

I6X 从独占的 UART 接收 Flysky 遥控器链路，支持 iBus 和 SBUS 两种协议。模块校验并解析输入帧，把通道值归一化到 `-1..1`，然后通过一个 Topic 发布 `I6X::Data`。

I6X reads a Flysky receiver link from a dedicated UART. It supports iBus and SBUS, validates and decodes each frame, normalizes channel values to `-1..1`, and publishes them as `I6X::Data` through one Topic.

模块实例和 UART 必须在接收线程运行期间持续存在，UART RX 只能由一个消费者使用。

The module instance and UART must outlive the receiver thread, and the UART RX path must have one consumer.

## 2. 协议与设备支持 / Protocol and Device Support

| 协议 / Protocol | UART 格式 / UART format | 帧 / Frame | 解析通道 / Decoded channels |
| --- | --- | --- | --- |
| SBUS | 100000 baud, 8E2 | 25 bytes, header `0x0F` | 16 channels, 11-bit each |
| iBus | 115200 baud, 8N1 | 32 bytes, length `0x20`, command `0x40` | 14 channels, 16-bit each |

SBUS 标准信号通常是反相的。BSP 必须提供硬件反相、启用 UART RX 反相，或使用接收机提供的非反相 SBUS 输出。iBus 使用非反相信号。

Standard SBUS is usually inverted. The BSP must provide an inverter, enable UART RX inversion, or use a receiver output that is already non-inverted. iBus is non-inverted.

SBUS 没有校验和，解析器检查帧头、帧尾以及丢帧/失控保护标志。iBus 使用 `0xFFFF` 减去前 30 个字节的结果作为校验和。

SBUS has no checksum; the parser checks the header, footer, and frame-lost/failsafe flags. iBus uses `0xFFFF` minus the first 30 bytes as its checksum.

设备的协议能力取决于具体型号和固件，请先确认接收机支持所选协议：

Protocol support depends on the exact model and firmware. Confirm that the receiver supports the selected protocol:

| 设备 / Device | SBUS | iBus | 通道 / Channels |
| --- | --- | --- | --- |
| I6 发射机 / I6 transmitter | 不支持 / No | 支持 / Yes | — |
| 6 通道 I6X 接收机 / 6-channel I6X receiver | 支持 / Yes | 支持 / Yes | 6 |
| IA6 接收机 / IA6 receiver | 不支持 / No | 支持 / Yes | 6 |
| 6 通道 IA6B 接收机 / 6-channel IA6B receiver | 支持 / Yes | 支持 / Yes | 6 |
| IA10B 接收机 / IA10B receiver | 支持 / Yes | 支持 / Yes | 10 |

模块按协议最大通道数解析：SBUS 解析 16 路，iBus 解析前 14 路。接收机实际通道数不足时，未使用的 `Data::normalized[]` 元素保持为零。

The parser handles the protocol maximum: 16 SBUS channels and the first 14 iBus channels. Unused `Data::normalized[]` elements remain zero when a receiver has fewer channels.

### 接收机串行总线输出设置 / Receiver Serial-Bus Output Setup

使用 iBus 或 SBUS 前，需要在 i6 或 i6x 遥控器中选择与 `I6X::Param::protocol` 一致的输出协议：

Before using iBus or SBUS, select the output protocol that matches `I6X::Param::protocol` on the i6 or i6x transmitter:

1. 长按 `OK` 进入系统菜单。
2. 进入“接收机设置”。
3. 进入“输出模式”。
4. 在“串行总线”中选择 `IBUS` 或 `SBUS`。
5. 长按 `CANCEL` 保存设置并退出菜单。

1. Press and hold `OK` to enter the system menu.
2. Open the receiver settings.
3. Open the output-mode settings.
4. Select `IBUS` or `SBUS` under the serial-bus option.
5. Press and hold `CANCEL` to save the setting and leave the menu.

## 3. 通道值与校准 / Channel Values and Calibration

`I6X::Data::normalized[]` 中的每个元素范围为 `-1..1`。Topic 只发布归一化值，原始帧、校验和和链路状态保存在模块内部。

Each element of `I6X::Data::normalized[]` is in the range `-1..1`. The Topic publishes normalized values only; raw frames, checksums, and link state remain internal to the module.

SBUS 默认使用以下校准范围，每个通道都可以通过 `Param::calibration` 单独调整：

SBUS uses the following default calibration ranges. Each channel can be adjusted independently through `Param::calibration`:

| 协议 / Protocol | 最小值 / Minimum | 中位值 / Center | 最大值 / Maximum | 配置 / Configuration |
| --- | ---: | ---: | ---: | --- |
| SBUS | `SBUS_CHANNEL_MIN = 240` | `SBUS_CHANNEL_CENTER = 1024` | `SBUS_CHANNEL_MAX = 1807` | 每通道可调 / Per-channel |
| iBus | `IBUS_CHANNEL_MIN = 1000` | `IBUS_CHANNEL_CENTER = 1500` | `IBUS_CHANNEL_MAX = 2000` | 固定 / Fixed |

SBUS 通道值按各通道的最小值、中位值和最大值归一化。iBus 固定使用 `1000 / 1500 / 2000` 进行归一化。

SBUS channels use each channel's minimum, center, and maximum calibration values. iBus always uses `1000 / 1500 / 2000`.

## 4. 构造接口 / Constructor

```cpp
I6X(LibXR::UART& uart,
    const I6X::Param& param = {.topic_name = "i6x",
                               .offline_timeout_ms = 20,
                               .task_stack_depth_uart = 2048,
                               .thread_priority_uart = LibXR::Thread::Priority::HIGH,
                               .calibration = I6X::Calibration{},
                               .protocol = I6X::Protocol::SBUS});
```

构造函数会：

- 创建 `param.topic_name` 指定的 Topic。
- 按协议配置 UART：SBUS 为 `100000, 8E2`，iBus 为 `115200, 8N1`。
- 创建 UART 接收线程。

The constructor:

- Creates the Topic named by `param.topic_name`.
- Configures the UART as `100000, 8E2` for SBUS or `115200, 8N1` for iBus.
- Creates the UART receiver thread.

`topic_name` 必须非空且每个实例唯一；`offline_timeout_ms` 和 `task_stack_depth_uart` 必须大于零；传入的校准范围必须满足 `minimum < center < maximum <= 2047`。

`topic_name` must be non-empty and unique per instance. `offline_timeout_ms` and `task_stack_depth_uart` must be greater than zero. Each calibration range must satisfy `minimum < center < maximum <= 2047`.

## 5. 脱离硬件解析帧 / Parsing a Frame Without Hardware

`ParseFrame` 的第三个参数选择协议，省略时默认使用 SBUS；`size == 0` 时使用协议默认帧长。

The third `ParseFrame` argument selects the protocol. It defaults to SBUS, and `size == 0` selects the protocol's default frame length.

```cpp
#include "I6X.hpp"

I6X::Data data{};
auto result = I6X::ParseFrame(frame, data, I6X::Protocol::SBUS,
                              sizeof(frame));
```

解析失败时，`output_data` 保持调用前的内容不变。SBUS 丢帧或失控保护帧返回 `OK`，但归一化输出全为零。iBus 校验失败返回 `CHECK_ERR`，通道值超出 `1000..2000` 返回 `OUT_OF_RANGE`。

On a parse failure, `output_data` remains unchanged. An SBUS frame-lost or failsafe frame returns `OK` with all normalized outputs set to zero. An iBus checksum failure returns `CHECK_ERR`; an iBus channel outside `1000..2000` returns `OUT_OF_RANGE`.

## 6. Topic 数据 / Topic Data

| 名称 / Name | 方向 / Direction | 类型 / Type | 内容 / Contents |
| --- | --- | --- | --- |
| `param.topic_name`，默认 `i6x` | 发布 / Publish | `I6X::Data` | `normalized[16]`, range `-1..1` |

当有效帧在 `offline_timeout_ms` 内没有到达时，模块发布一次全零的离线数据。收到有效帧后恢复正常发布。

When no valid frame arrives within `offline_timeout_ms`, the module publishes one all-zero offline message. Publishing resumes after a valid frame is received.

Topic 回调运行在 UART 接收线程中，应尽快返回。模块不预设油门、转向、模式或舵机等通道功能，应用应根据遥控器配置解释通道顺序。

Topic callbacks run in the UART receiver thread and should return quickly. The module does not assign meanings such as throttle, steering, mode, or servo to channels; applications should interpret channel order according to the transmitter configuration.

## 7. 配置示例 / Configuration Example

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

切换到 iBus 时，将 `protocol` 改为 `I6X::Protocol::IBUS`，并确保遥控器输出模式、UART 接线和 UART 参数同步切换。

To use iBus, change `protocol` to `I6X::Protocol::IBUS` and change the transmitter output mode, wiring, and UART configuration accordingly.

## 8. BSP 与硬件要求 / BSP and Hardware Requirements

- BSP 必须先初始化 LibXR 的时间基准、线程和 UART。
- UART RX 必须独占，接收队列至少能够容纳 32 字节。
- DMA 接收缓冲区必须位于 DMA 可访问的内存，并满足 BSP 的缓存维护和对齐要求。
- 使用 SBUS 时必须处理信号反相；使用 iBus 时连接非反相 RX。
- 检查接收机供电、UART RX/TX 方向、公共地线和目标 UART 引脚复用配置。

- The BSP must initialize the LibXR timebase, threading, and UART first.
- UART RX must be exclusive, and the receive queue must hold at least 32 bytes.
- DMA receive buffers must be in DMA-accessible memory and meet the BSP's cache-maintenance and alignment requirements.
- Handle SBUS inversion; connect iBus to a non-inverted RX input.
- Check receiver power, UART RX/TX direction, common ground, and the target UART pin-multiplexing configuration.

如果没有数据，请依次检查：遥控器是否已选择正确的串行总线协议、接收机型号是否支持该协议、UART 参数是否匹配、SBUS 是否反相、RX 是否被其他任务占用，以及 DMA 缓冲区地址是否可被 DMA 访问。

If no data is received, check in this order: transmitter serial-bus selection, receiver protocol support, matching UART parameters, SBUS inversion, exclusive RX ownership, and DMA-buffer placement.
