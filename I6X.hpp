#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: I6X 遥控接收机：可配置解析 UART SBUS 或 iBus 帧并发布统一通道状态 / I6X receiver with configurable UART SBUS or iBus decoding and unified channel status
depends: []
=== END MANIFEST === */
// clang-format on

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
    IBUS,
    SBUS,
  };

  static constexpr std::size_t SBUS_FRAME_SIZE = 25;
  static constexpr std::size_t IBUS_FRAME_SIZE = 32;
  static constexpr std::size_t FRAME_SIZE = IBUS_FRAME_SIZE;
  static constexpr std::size_t SBUS_CHANNEL_COUNT = 16;
  static constexpr std::size_t IBUS_CHANNEL_COUNT = 14;
  static constexpr std::size_t CHANNEL_COUNT = SBUS_CHANNEL_COUNT;
  static constexpr uint8_t SBUS_HEADER = 0x0F;
  static constexpr uint8_t SBUS_FOOTER = 0x00;
  static constexpr uint8_t IBUS_LENGTH = 0x20;
  static constexpr uint8_t IBUS_COMMAND = 0x40;
  static constexpr uint16_t IBUS_CHANNEL_MIN = 1000;
  static constexpr uint16_t IBUS_CHANNEL_MAX = 2000;
  static constexpr uint8_t FRAME_LENGTH = IBUS_LENGTH;
  static constexpr uint8_t FRAME_COMMAND = IBUS_COMMAND;
  static constexpr uint16_t CHANNEL_MIN = IBUS_CHANNEL_MIN;
  static constexpr uint16_t CHANNEL_MAX = IBUS_CHANNEL_MAX;

  /**
   * @brief 单通道归一化量程。
   *        Per-channel range used for -1..1 normalization.
   */
  struct ChannelCalibration
  {
    uint16_t minimum;
    uint16_t center;
    uint16_t maximum;

    constexpr ChannelCalibration(uint16_t min_value = 240, uint16_t center_value = 1024,
                                 uint16_t max_value = 1807)
        : minimum(min_value), center(center_value), maximum(max_value)
    {
    }
  };

  using Calibration = std::array<ChannelCalibration, CHANNEL_COUNT>;

  /**
   * @brief 两种协议共用的 Topic 数据。
   *        Topic payload shared by both protocols.
   * @note iBus 只填充前 14 个通道；SBUS 填充 16 个通道。未使用通道为零。
   *       iBus fills the first 14 channels; SBUS fills 16 channels. Unused
   *       channels are zero.
   */
  struct Data
  {
    union
    {
      uint16_t ch[CHANNEL_COUNT];
      uint16_t channel[CHANNEL_COUNT];
      uint16_t channels[CHANNEL_COUNT];
    };

    uint16_t checksum_calculated = 0;
    uint16_t checksum_received = 0;

    struct
    {
      uint8_t length = 0;
      uint8_t command = 0;
      bool valid = false;
    } frame;

    Protocol protocol = Protocol::SBUS;
    uint8_t channel_count = 0;
    bool ch17 = false;
    bool ch18 = false;
    bool frame_lost = false;
    bool failsafe = false;
    bool online = false;
    float normalized[CHANNEL_COUNT]{};
  };

  /**
   * @brief 模块配置；协议字段是唯一的协议切换参数。
   *        Module configuration; protocol is the single protocol selector.
   */
  struct Param
  {
    const char* topic_name;
    uint32_t offline_timeout_ms;
    uint32_t task_stack_depth_uart;
    LibXR::Thread::Priority thread_priority_uart;
    Calibration calibration;
    Protocol protocol;
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
        op_(sem_, 5)
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
      const auto raw = data.ch[channel];
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

  static LibXR::ErrorCode ParseSbusFrame(const uint8_t* raw_data, Data& output_data,
                                         const Calibration& calibration)
  {
    if (raw_data[0] != SBUS_HEADER || (raw_data[SBUS_FRAME_SIZE - 1] != SBUS_FOOTER &&
                                       raw_data[SBUS_FRAME_SIZE - 1] != 0x7E))
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    Data data{};
    data.protocol = Protocol::SBUS;
    data.channel_count = static_cast<uint8_t>(SBUS_CHANNEL_COUNT);
    data.frame.length = static_cast<uint8_t>(SBUS_FRAME_SIZE);
    data.frame.command = SBUS_HEADER;
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
      data.ch[channel] = static_cast<uint16_t>((value >> shift) & 0x07FFU);
    }
    data.ch17 = (raw_data[23] & 0x01U) != 0;
    data.ch18 = (raw_data[23] & 0x02U) != 0;
    data.frame_lost = (raw_data[23] & 0x04U) != 0;
    data.failsafe = (raw_data[23] & 0x08U) != 0;
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
        static_cast<uint16_t>(raw_data[30]) | (static_cast<uint16_t>(raw_data[31]) << 8U);
    uint16_t checksum_calculated = 0xFFFFU;
    for (std::size_t i = 0; i < IBUS_FRAME_SIZE - 2; ++i)
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
    data.frame.command = raw_data[1];
    data.checksum_calculated = checksum_calculated;
    data.checksum_received = checksum_received;
    for (std::size_t channel = 0; channel < IBUS_CHANNEL_COUNT; ++channel)
    {
      const std::size_t byte = 2U + channel * 2U;
      const uint16_t value = static_cast<uint16_t>(raw_data[byte]) |
                             (static_cast<uint16_t>(raw_data[byte + 1U]) << 8U);
      if (value < IBUS_CHANNEL_MIN || value > IBUS_CHANNEL_MAX)
      {
        return LibXR::ErrorCode::OUT_OF_RANGE;
      }
      data.ch[channel] = value;
    }
    data.frame.valid = true;
    data.online = true;
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
    uint8_t frame[FRAME_SIZE]{};
    const std::size_t frame_size = FrameSize(i6x->protocol_);
    std::size_t used = 0;
    auto last_frame_time = LibXR::Timebase::GetMilliseconds();
    bool timeout_published = false;

    while (true)
    {
      uint8_t byte = 0;
      const auto result = i6x->uart_.Read({&byte, sizeof(byte)}, i6x->op_);
      if (result == LibXR::ErrorCode::OK)
      {
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
