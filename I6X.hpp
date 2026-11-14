#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: I6X 遥控接收机：可配置解析 UART SBUS 或 iBus 帧并发布统一通道状态 / I6X receiver with configurable UART SBUS or iBus decoding and unified channel status
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "message.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/**
 * @brief 从专用 UART 接收 SBUS 或 iBus，并发布统一的遥控通道 Topic。
 *        Receive SBUS or iBus on a dedicated UART and publish a unified
 *        receiver Topic.
 * @note 通过 Param::protocol 选择协议；实例与 UART 须长期存在且独占 RX。
 *       Select the protocol with Param::protocol; the instance and UART must
 *       outlive the RX thread and own UART RX exclusively.
 */
class I6X
{
 public:
  enum class Protocol : uint8_t
  {
    IBUS,  ///< iBus 协议
           ///< iBus protocol
    SBUS,  ///< SBUS 协议
           ///< SBUS protocol
  };

  /// SBUS 帧字节数
  /// SBUS frame size in bytes
  static constexpr std::size_t SBUS_FRAME_SIZE = 25;
  /// iBus 帧字节数
  /// iBus frame size in bytes
  static constexpr std::size_t IBUS_FRAME_SIZE = 32;
  /// SBUS 帧通道数
  /// Number of channels in an SBUS frame
  static constexpr std::size_t SBUS_CHANNEL_COUNT = 16;
  /// iBus 帧通道数
  /// Number of channels in an iBus frame
  static constexpr std::size_t IBUS_CHANNEL_COUNT = 14;
  /// SBUS 帧头
  /// SBUS frame header
  static constexpr uint8_t SBUS_HEADER = 0x0F;
  /// SBUS 帧尾
  /// SBUS frame footer
  static constexpr uint8_t SBUS_FOOTER = 0x00;
  /// Some receivers use 0x7E as an alternate SBUS footer / 部分接收机使用 0x7E 作为 SBUS 备用帧尾
  static constexpr uint8_t SBUS_ALT_FOOTER = 0x7E;
  /// SBUS status flag for digital channel 17 / SBUS 数字通道 17 状态位
  static constexpr uint8_t SBUS_STATUS_CH17 = 0x01;
  /// SBUS status flag for digital channel 18 / SBUS 数字通道 18 状态位
  static constexpr uint8_t SBUS_STATUS_CH18 = 0x02;
  /// SBUS status flag for a lost frame / SBUS 丢帧状态位
  static constexpr uint8_t SBUS_STATUS_FRAME_LOST = 0x04;
  /// SBUS status flag for failsafe / SBUS 失控保护状态位
  static constexpr uint8_t SBUS_STATUS_FAILSAFE = 0x08;
  /// SBUS status byte offset / SBUS 状态字节偏移
  static constexpr std::size_t SBUS_STATUS_OFFSET = SBUS_FRAME_SIZE - 2;
  /// iBus 帧长度字节
  /// iBus frame length byte
  static constexpr uint8_t IBUS_LENGTH = 0x20;
  /// iBus 命令字节
  /// iBus command byte
  static constexpr uint8_t IBUS_COMMAND = 0x40;
  /// iBus 通道最小有效值
  /// Minimum valid iBus channel value
  static constexpr uint16_t IBUS_CHANNEL_MIN = 1000;
  /// iBus 通道最大有效值
  /// Maximum valid iBus channel value
  static constexpr uint16_t IBUS_CHANNEL_MAX = 2000;
  /// iBus checksum byte count / iBus 校验和字节数
  static constexpr std::size_t IBUS_CHECKSUM_SIZE = 2;
  /// iBus checksum start offset / iBus 校验和起始偏移
  static constexpr std::size_t IBUS_CHECKSUM_OFFSET = IBUS_FRAME_SIZE - IBUS_CHECKSUM_SIZE;
  /// iBus channel data start offset / iBus 通道数据起始偏移
  static constexpr std::size_t IBUS_CHANNEL_DATA_OFFSET = 2;
  /// iBus checksum initial value / iBus 校验和初始值
  static constexpr uint16_t IBUS_CHECKSUM_INITIAL = 0xFFFF;
  /**
   * @brief 单通道归一化量程。
   *        Per-channel range used for -1..1 normalization.
   */
  struct ChannelCalibration
  {
    uint16_t minimum;  ///< 原始通道最小值
                        ///< Minimum raw channel value
    uint16_t center;   ///< 原始通道中位值
                       ///< Center raw channel value
    uint16_t maximum;  ///< 原始通道最大值
                       ///< Maximum raw channel value

    constexpr ChannelCalibration(uint16_t min_value = 240, uint16_t center_value = 1024,
                                 uint16_t max_value = 1807)
        : minimum(min_value), center(center_value), maximum(max_value)
    {
    }
  };

  using Calibration = std::array<ChannelCalibration, SBUS_CHANNEL_COUNT>;

  /**
   * @brief 两种协议共用的 Topic 数据。
   *        Topic payload shared by both protocols.
   * @note iBus 只填充前 14 个通道；SBUS 填充 16 个通道。未使用通道为零。
   *       iBus fills the first 14 channels; SBUS fills 16 channels. Unused
   *       channels are zero.
   */
  struct Data
  {
    uint16_t channels[SBUS_CHANNEL_COUNT]{};  ///< 通道值
                                              ///< Channel values

    uint16_t checksum_calculated = 0;  ///< 计算得到的 iBus 校验和
                                         ///< Calculated iBus checksum
    uint16_t checksum_received = 0;  ///< 接收到的 iBus 校验和
                                       ///< Received iBus checksum

    struct
    {
      uint8_t length = 0;  ///< 帧长度字节
                           ///< Frame length byte
      uint8_t marker = 0;  ///< 协议标识字节：SBUS 为帧头，iBus 为命令字节
                           ///< Protocol marker: SBUS header or iBus command
      bool valid = false;  ///< 帧格式和校验有效
                          ///< Frame format and checksum are valid
    } frame;

    Protocol protocol = Protocol::SBUS;  ///< 本帧使用的协议
                                         ///< Protocol used for this frame
    uint8_t channel_count = 0;  ///< 已解析的通道数
                                 ///< Number of decoded channels
    bool ch17 = false;  ///< SBUS 数字通道 17
                        ///< SBUS digital channel 17
    bool ch18 = false;  ///< SBUS 数字通道 18
                        ///< SBUS digital channel 18
    bool frame_lost = false;  ///< 接收机报告丢帧
                               ///< Receiver reported a lost frame
    bool failsafe = false;  ///< 接收机报告失控保护
                            ///< Receiver reported failsafe
    bool online = false;  ///< 本帧可用于控制
                          ///< Frame is valid for control
    float normalized[SBUS_CHANNEL_COUNT]{};  ///< 校准后的 -1..1 值
                                              ///< Calibrated values in -1..1
  };

  /**
   * @brief 模块配置；协议字段是唯一的协议切换参数。
   *        Module configuration; protocol is the single protocol selector.
   */
  struct Param
  {
    const char* topic_name;  ///< 发布的 Topic 名称
                              ///< Published Topic name
    uint32_t offline_timeout_ms;  ///< 离线超时毫秒数
                                    ///< Offline timeout in milliseconds
    uint32_t task_stack_depth_uart;  ///< UART 线程栈深
                                      ///< UART thread stack depth
    LibXR::Thread::Priority thread_priority_uart;  ///< UART 线程优先级
                                                     ///< UART thread priority
    Calibration calibration;  ///< 各通道校准参数
                              ///< Per-channel calibration
    Protocol protocol;  ///< 输入协议
                        ///< Input protocol
  };

  /**
   * @brief 按选择的协议配置 UART，创建 Topic 和接收线程。
   *        Configure the UART, create the Topic and start the RX thread.
   * @param uart 独占接收的 UART / UART with exclusive RX ownership.
   * @param param Topic、超时、栈、校准和协议配置 / Topic, timeout, stack,
   * calibration and protocol configuration.
   */
  I6X(LibXR::UART& uart,
      const Param& param = {.topic_name = "i6x",
                            .offline_timeout_ms = 20,
                            .task_stack_depth_uart = 2048,
                            .thread_priority_uart = LibXR::Thread::Priority::HIGH,
                            .calibration = Calibration{},
                            .protocol = Protocol::SBUS})
      : uart_(uart),
        offline_timeout_ms_(param.offline_timeout_ms),
        calibration_(param.calibration),
        protocol_(param.protocol),
        sem_(0),
        op_(sem_, 64)
  {
    ASSERT(param.topic_name != nullptr && param.topic_name[0] != '\0');
    ASSERT(param.offline_timeout_ms > 0 && param.task_stack_depth_uart > 0);
    ASSERT(param.protocol == Protocol::IBUS || param.protocol == Protocol::SBUS);
    ASSERT(ValidCalibration(calibration_));

    topic_ = LibXR::Topic::CreateTopic<Data>(param.topic_name);
    const auto result = uart_.SetConfig(UartConfig(param.protocol));
    REQUIRE(result == LibXR::ErrorCode::OK);
    thread_uart_.Create(this, ThreadI6X, "uart_i6x", param.task_stack_depth_uart,
                        param.thread_priority_uart);
  }

  /**
   * @brief 解析一帧；默认按 SBUS 解析。
   *        Decode one frame; the default protocol is SBUS.
   * @param raw_data 输入帧 / Input frame.
   * @param output_data 解析结果，失败时保持不变 / Decoded result, unchanged
   * on failure.
   * @param size 输入长度；传零时使用协议默认长度 / Input size; zero selects
   * the protocol default.
   * @param protocol 帧协议 / Frame protocol.
   */
  static LibXR::ErrorCode ParseFrame(const uint8_t* raw_data, Data& output_data,
                                     std::size_t size = 0,
                                     Protocol protocol = Protocol::SBUS)
  {
    return ParseFrame(raw_data, output_data, size, protocol, Calibration{});
  }

  /**
   * @brief 使用指定校准表解析帧。
   *        Decode a frame with an explicit calibration table.
   */
  static LibXR::ErrorCode ParseFrame(const uint8_t* raw_data, Data& output_data,
                                     std::size_t size, Protocol protocol,
                                     const Calibration& calibration)
  {
    const std::size_t expected_size = FrameSize(protocol);
    if (raw_data == nullptr)
    {
      return LibXR::ErrorCode::PTR_NULL;
    }
    if (size == 0)
    {
      size = expected_size;
    }
    if (size != expected_size)
    {
      return LibXR::ErrorCode::SIZE_ERR;
    }
    if (protocol == Protocol::SBUS)
    {
      if (!ValidCalibration(calibration))
      {
        return LibXR::ErrorCode::ARG_ERR;
      }
      return ParseSbusFrame(raw_data, output_data, calibration);
    }
    if (protocol == Protocol::IBUS)
    {
      return ParseIbusFrame(raw_data, output_data);
    }
    return LibXR::ErrorCode::ARG_ERR;
  }

  /**
   * @brief 允许把协议作为第三个参数传入的重载。
   *        Overload that accepts the protocol as the third argument.
   */
  static LibXR::ErrorCode ParseFrame(const uint8_t* raw_data, Data& output_data,
                                     Protocol protocol, std::size_t size = 0)
  {
    return ParseFrame(raw_data, output_data, size, protocol);
  }

  static LibXR::ErrorCode ParseFrame(const uint8_t* raw_data, Data& output_data,
                                     std::size_t size, const Calibration& calibration)
  {
    return ParseFrame(raw_data, output_data, size, Protocol::SBUS, calibration);
  }

  static LibXR::ErrorCode ParseFrame(const uint8_t* raw_data, Data& output_data,
                                     Protocol protocol, std::size_t size,
                                     const Calibration& calibration)
  {
    return ParseFrame(raw_data, output_data, size, protocol, calibration);
  }

 private:
  LibXR::UART& uart_;
  uint32_t offline_timeout_ms_;
  Calibration calibration_;
  Protocol protocol_;
  LibXR::Topic topic_;
  LibXR::Semaphore sem_;
  LibXR::ReadOperation op_;

  static constexpr std::size_t FrameSize(Protocol protocol)
  {
    return protocol == Protocol::SBUS ? SBUS_FRAME_SIZE : IBUS_FRAME_SIZE;
  }

  static constexpr LibXR::UART::Configuration UartConfig(Protocol protocol)
  {
    if (protocol == Protocol::SBUS)
    {
      return {100000, LibXR::UART::Parity::EVEN, 8, 2};
    }
    return {115200, LibXR::UART::Parity::NO_PARITY, 8, 1};
  }

  static constexpr bool ValidCalibration(const Calibration& calibration)
  {
    for (const auto& range : calibration)
    {
      if (range.minimum >= range.center || range.center >= range.maximum ||
          range.maximum > 2047)
      {
        return false;
      }
    }
    return true;
  }

  static void Normalize(Data& data, const Calibration& calibration, std::size_t count)
  {
    for (std::size_t channel = 0; channel < count; ++channel)
    {
      const auto& range = calibration[channel];
      const auto raw = data.channels[channel];
      if (raw <= range.minimum)
      {
        data.normalized[channel] = -1.0F;
      }
      else if (raw >= range.maximum)
      {
        data.normalized[channel] = 1.0F;
      }
      else
      {
        const float span = raw < range.center ? range.center - range.minimum
                                              : range.maximum - range.center;
        data.normalized[channel] = (static_cast<float>(raw) - range.center) / span;
      }
    }
  }

  static void NormalizeIbus(Data& data)
  {
    constexpr float midpoint =
        (static_cast<float>(IBUS_CHANNEL_MIN) + IBUS_CHANNEL_MAX) * 0.5F;
    constexpr float half_range =
        (static_cast<float>(IBUS_CHANNEL_MAX) - IBUS_CHANNEL_MIN) * 0.5F;
    for (std::size_t channel = 0; channel < IBUS_CHANNEL_COUNT; ++channel)
    {
      const float normalized = (static_cast<float>(data.channels[channel]) - midpoint) /
                               half_range;
      data.normalized[channel] = std::clamp(normalized, -1.0F, 1.0F);
    }
  }

  static LibXR::ErrorCode ParseSbusFrame(const uint8_t* raw_data, Data& output_data,
                                         const Calibration& calibration)
  {
    if (raw_data[0] != SBUS_HEADER || (raw_data[SBUS_FRAME_SIZE - 1] != SBUS_FOOTER &&
                                       raw_data[SBUS_FRAME_SIZE - 1] != SBUS_ALT_FOOTER))
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    Data data{};
    data.protocol = Protocol::SBUS;
    data.channel_count = static_cast<uint8_t>(SBUS_CHANNEL_COUNT);
    data.frame.length = static_cast<uint8_t>(SBUS_FRAME_SIZE);
    data.frame.marker = SBUS_HEADER;
    for (std::size_t channel = 0; channel < SBUS_CHANNEL_COUNT; ++channel)
    {
      const std::size_t bit = channel * 11;
      const std::size_t byte = 1 + bit / 8;
      const std::size_t shift = bit % 8;
      uint32_t value = static_cast<uint32_t>(raw_data[byte]) |
                       (static_cast<uint32_t>(raw_data[byte + 1]) << 8U);
      if (shift > 5)
      {
        value |= static_cast<uint32_t>(raw_data[byte + 2]) << 16U;
      }
      data.channels[channel] = static_cast<uint16_t>((value >> shift) & 0x07FFU);
    }
    data.ch17 = (raw_data[SBUS_STATUS_OFFSET] & SBUS_STATUS_CH17) != 0;
    data.ch18 = (raw_data[SBUS_STATUS_OFFSET] & SBUS_STATUS_CH18) != 0;
    data.frame_lost = (raw_data[SBUS_STATUS_OFFSET] & SBUS_STATUS_FRAME_LOST) != 0;
    data.failsafe = (raw_data[SBUS_STATUS_OFFSET] & SBUS_STATUS_FAILSAFE) != 0;
    data.online = !data.frame_lost && !data.failsafe;
    data.frame.valid = true;
    if (data.online)
    {
      Normalize(data, calibration, SBUS_CHANNEL_COUNT);
    }
    output_data = data;
    return LibXR::ErrorCode::OK;
  }

  static LibXR::ErrorCode ParseIbusFrame(const uint8_t* raw_data, Data& output_data)
  {
    if (raw_data[0] != IBUS_LENGTH || raw_data[1] != IBUS_COMMAND)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    const uint16_t checksum_received =
        static_cast<uint16_t>(raw_data[IBUS_CHECKSUM_OFFSET]) |
        (static_cast<uint16_t>(raw_data[IBUS_CHECKSUM_OFFSET + 1]) << 8U);
    uint16_t checksum_calculated = IBUS_CHECKSUM_INITIAL;
    for (std::size_t i = 0; i < IBUS_CHECKSUM_OFFSET; ++i)
    {
      checksum_calculated = static_cast<uint16_t>(checksum_calculated - raw_data[i]);
    }
    if (checksum_calculated != checksum_received)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    Data data{};
    data.protocol = Protocol::IBUS;
    data.channel_count = static_cast<uint8_t>(IBUS_CHANNEL_COUNT);
    data.frame.length = raw_data[0];
    data.frame.marker = raw_data[1];
    data.checksum_calculated = checksum_calculated;
    data.checksum_received = checksum_received;
    for (std::size_t channel = 0; channel < IBUS_CHANNEL_COUNT; ++channel)
    {
      const std::size_t byte = IBUS_CHANNEL_DATA_OFFSET + channel * 2U;
      const uint16_t value = static_cast<uint16_t>(raw_data[byte]) |
                             (static_cast<uint16_t>(raw_data[byte + 1U]) << 8U);
      if (value < IBUS_CHANNEL_MIN || value > IBUS_CHANNEL_MAX)
      {
        return LibXR::ErrorCode::OUT_OF_RANGE;
      }
      data.channels[channel] = value;
    }
    data.frame.valid = true;
    data.online = true;
    NormalizeIbus(data);
    output_data = data;
    return LibXR::ErrorCode::OK;
  }

  static void PublishFrame(I6X* i6x, uint8_t* frame, std::size_t frame_size,
                           std::size_t& used,
                           decltype(LibXR::Timebase::GetMilliseconds())& last_frame_time,
                           bool& timeout_published)
  {
    Data data{};
    if (ParseFrame(frame, data, frame_size, i6x->protocol_, i6x->calibration_) ==
        LibXR::ErrorCode::OK)
    {
      last_frame_time = LibXR::Timebase::GetMilliseconds();
      timeout_published = false;
      i6x->topic_.Publish(data);
      used = 0;
    }
  }

  static void ThreadI6X(I6X* i6x)
  {
    constexpr std::size_t RX_BUFFER_SIZE = 64;
    uint8_t rx_buffer[RX_BUFFER_SIZE]{};
    uint8_t frame[IBUS_FRAME_SIZE]{};
    const std::size_t frame_size = FrameSize(i6x->protocol_);
    std::size_t used = 0;
    auto last_frame_time = LibXR::Timebase::GetMilliseconds();
    bool timeout_published = false;

    while (true)
    {
      const auto result = i6x->uart_.Read({rx_buffer, RX_BUFFER_SIZE}, i6x->op_);
      if (result == LibXR::ErrorCode::OK)
      {
        for (std::size_t index = 0; index < RX_BUFFER_SIZE; ++index)
        {
          const uint8_t byte = rx_buffer[index];
          if (i6x->protocol_ == Protocol::SBUS)
          {
            if (used == 0)
            {
              if (byte == SBUS_HEADER)
              {
                frame[used++] = byte;
              }
            }
            else
            {
              frame[used++] = byte;
              if (used == frame_size)
              {
                const std::size_t previous_used = used;
                PublishFrame(i6x, frame, frame_size, used, last_frame_time,
                             timeout_published);
                if (used == previous_used)
                {
                  std::size_t next = 1;
                  while (next < frame_size && frame[next] != SBUS_HEADER)
                  {
                    ++next;
                  }
                  used = frame_size - next;
                  if (used != 0)
                  {
                    std::memmove(frame, frame + next, used);
                  }
                }
              }
            }
          }
          else
          {
            if (used == 0)
            {
              if (byte == IBUS_LENGTH)
              {
                frame[used++] = byte;
              }
            }
            else if (used == 1)
            {
              if (byte == IBUS_COMMAND)
              {
                frame[used++] = byte;
              }
              else
              {
                used = byte == IBUS_LENGTH ? 1 : 0;
                if (used != 0)
                {
                  frame[0] = byte;
                }
              }
            }
            else
            {
              frame[used++] = byte;
              if (used == frame_size)
              {
                const std::size_t previous_used = used;
                PublishFrame(i6x, frame, frame_size, used, last_frame_time,
                             timeout_published);
                if (used == previous_used)
                {
                  std::size_t next = 1;
                  while (next + 1 < frame_size &&
                         !(frame[next] == IBUS_LENGTH && frame[next + 1] == IBUS_COMMAND))
                  {
                    ++next;
                  }
                  if (next + 1 < frame_size)
                  {
                    used = frame_size - next;
                    std::memmove(frame, frame + next, used);
                  }
                  else if (next < frame_size && frame[next] == IBUS_LENGTH)
                  {
                    frame[0] = IBUS_LENGTH;
                    used = 1;
                  }
                  else
                  {
                    used = 0;
                  }
                }
              }
            }
          }
        }
      }
      else
      {
        used = 0;
        LibXR::Thread::Sleep(1);
      }

      const auto now = LibXR::Timebase::GetMilliseconds();
      if (!timeout_published &&
          (now - last_frame_time).ToMillisecond() >= i6x->offline_timeout_ms_)
      {
        Data offline{};
        offline.protocol = i6x->protocol_;
        i6x->topic_.Publish(offline);
        timeout_published = true;
        used = 0;
      }
    }
  }

  LibXR::Thread thread_uart_;
};
