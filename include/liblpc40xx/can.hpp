#pragma once

#include <string_view>

#include <libarmcortex/interrupt.hpp>
#include <libhal/can/interface.hpp>
#include <libhal/static_callable.hpp>
#include <libxbitset/bitset.hpp>

#include "constants.hpp"
#include "internal/pin.hpp"
#include "system_controller.hpp"

namespace hal::lpc40xx {
class can final : public hal::can
{
public:
  struct acceptance_filter_ram_t
  {
    /// Mask IDs
    volatile uint32_t mask[512];
  };

  struct acceptance_filter_t
  {
    /// Offset: 0x00000000 - Acceptance Filter Register
    volatile uint32_t acceptance_filter;
    /// Offset: 0x00000004 - Standard Frame Individual Start Address Register
    volatile uint32_t SFF_sa;
    /// Offset: 0x00000008 - Standard Frame Group Start Address Register
    volatile uint32_t SFF_GRP_sa;
    /// Offset: 0x0000000C - Extended Frame Start Address Register
    volatile uint32_t EFF_sa;
    /// Offset: 0x00000010 - Extended Frame Group Start Address Register
    volatile uint32_t EFF_GRP_sa;
    /// Offset: 0x00000014 - End of AF Tables register
    volatile uint32_t ENDofTable;
    /// Offset: 0x00000018 - LUT Error Address register
    const volatile uint32_t LUTerrAd;
    /// Offset: 0x0000001C - LUT Error Register
    const volatile uint32_t LUTerr;
    /// Offset: 0x00000020 - CAN Central Transmit Status Register
    volatile uint32_t full_can_transmit_status;
    /// Offset: 0x00000024 - FullCAN Interrupt and Capture registers 0
    volatile uint32_t FCANIC0;
    /// Offset: 0x00000028 - FullCAN Interrupt and Capture registers 1
    volatile uint32_t FCANIC1;
  };

  struct central_reg_t
  {
    const volatile uint32_t TxSR;
    const volatile uint32_t RxSR;
    const volatile uint32_t MSR;
  };

  struct reg_t
  {
    /// Offset: 0x00000000 - Controls the operating mode of the CAN Controller
    volatile uint32_t MOD;
    /// Offset: 0x00000004 - Command bits that affect the state
    volatile uint32_t CMR;
    /// Offset: 0x00000008 - Global Controller Status and Error Counters
    volatile uint32_t GSR;
    /// Offset: 0x0000000C - Interrupt status, Arbitration Lost Capture, Error
    /// Code Capture
    const volatile uint32_t ICR;
    /// Offset: 0x00000010 - Interrupt Enable Register
    volatile uint32_t IER;
    /// Offset: 0x00000014 - Bus Timing Register
    volatile uint32_t BTR;
    /// Offset: 0x00000018 - Error Warning Limit
    volatile uint32_t EWL;
    /// Offset: 0x0000001C - Status Register
    const volatile uint32_t SR;
    /// Offset: 0x00000020 - Receive frame status
    volatile uint32_t RFS;
    /// Offset: 0x00000024 - Received Identifier
    volatile uint32_t RID;
    /// Offset: 0x00000028 - Received data bytes 1-4
    volatile uint32_t RDA;
    /// Offset: 0x0000002C - Received data bytes 5-8
    volatile uint32_t RDB;
    /// Offset: 0x00000030 - Transmit frame info (Tx Buffer 1)
    volatile uint32_t TFI1;
    /// Offset: 0x00000034 - Transmit Identifier (Tx Buffer 1)
    volatile uint32_t TID1;
    /// Offset: 0x00000038 - Transmit data bytes 1-4 (Tx Buffer 1)
    volatile uint32_t TDA1;
    /// Offset: 0x0000003C - Transmit data bytes 5-8 (Tx Buffer 1)
    volatile uint32_t TDB1;
    /// Offset: 0x00000040 - Transmit frame info (Tx Buffer 2)
    volatile uint32_t TFI2;
    /// Offset: 0x00000044 - Transmit Identifier (Tx Buffer 2)
    volatile uint32_t TID2;
    /// Offset: 0x00000048 - Transmit data bytes 1-4 (Tx Buffer 2)
    volatile uint32_t TDA2;
    /// Offset: 0x0000004C - Transmit data bytes 5-8 (Tx Buffer 2)
    volatile uint32_t TDB2;
    /// Offset: 0x00000050 - Transmit frame info (Tx Buffer 3)
    volatile uint32_t TFI3;
    /// Offset: 0x00000054 - Transmit Identifier (Tx Buffer 3)
    volatile uint32_t TID3;
    /// Offset: 0x00000058 - Transmit data bytes 1-4 (Tx Buffer 3)
    volatile uint32_t TDA3;
    /// Offset: 0x0000005C - Transmit data bytes 5-8 (Tx Buffer 3)
    volatile uint32_t TDB3;
  };

  /// This struct holds bit timing values. It is used to configure the CAN bus
  /// clock. It is HW mapped to a 32-bit register: BTR (pg. 562)
  struct bus_timing
  {
    /// The peripheral bus clock is divided by this value
    static constexpr auto prescalar = xstd::bitrange::from<0, 9>();

    /// Used to compensate for positive and negative edge phase errors
    static constexpr auto sync_jump_width = xstd::bitrange::from<14, 15>();
    /// The delay from the nominal Sync point to the sample point is (this value
    /// plus one) CAN clocks.
    static constexpr auto time_segment1 = xstd::bitrange::from<16, 19>();

    /// The delay from the sample point to the next nominal sync point isCAN
    /// clocks. The nominal CAN bit time is (this value plus the value in
    /// time_segment1 plus 3) CAN clocks.
    static constexpr auto time_segment2 = xstd::bitrange::from<20, 22>();

    /// How many times the bus is sampled; 0 == once, 1 == 3 times
    static constexpr auto sampling = xstd::bitrange::from<23>();
  };

  /// This struct holds interrupt flags and capture flag status. It is HW mapped
  /// to a 16-bit register: ICR (pg. 557)
  struct interrupts
  {
    // ICR - Interrupt and Capture Register
    // NOTE: Bits 1-10 are cleared by the CAN controller
    //       as soon as they are read.
    //       Bits 16-23 & 24-31 are released by the CAN
    //       controller as soon as they are read.

    /// Assert interrupt when a new message has been received
    static constexpr auto received_message = xstd::bitrange::from<0>();

    /// Assert interrupt when TX Buffer 1 has finished or aborted its
    /// transmission.
    static constexpr auto tx1_ready = xstd::bitrange::from<1>();

    /// Assert interrupt when bus status or error status is asserted.
    static constexpr auto error_warning = xstd::bitrange::from<2>();

    /// Assert interrupt on data overrun occurs
    static constexpr auto data_overrun = xstd::bitrange::from<3>();

    /// Assert interrupt when CAN controller is sleeping and was woken up from
    /// bus activity.
    static constexpr auto wakeup = xstd::bitrange::from<4>();

    /// Assert interrupt when the CAN Controller has reached the Error Passive
    /// Status (error counter exceeds 127)
    static constexpr auto error_passive = xstd::bitrange::from<5>();

    /// Assert interrupt when arbitration is lost
    static constexpr auto arbitration_lost = xstd::bitrange::from<6>();

    /// Assert interrupt on bus error
    static constexpr auto bus_error = xstd::bitrange::from<7>();

    /// Assert interrupt when any message has been successfully transmitted.
    static constexpr auto identifier_ready = xstd::bitrange::from<8>();

    /// Assert interrupt when TX Buffer 2 has finished or aborted its
    /// transmission.
    static constexpr auto tx2_ready = xstd::bitrange::from<9>();

    /// Assert interrupt when TX Buffer 3 has finished or aborted its
    /// transmission.
    static constexpr auto tx3_ready = xstd::bitrange::from<10>();

    /// Error Code Capture status bits to be read during an interrupt
    static constexpr auto error_code_location = xstd::bitrange::from<16, 20>();
    /// Indicates if the error occurred during transmission (0) or receiving (1)
    static constexpr auto error_code_direction = xstd::bitrange::from<21>();
    /// The type of bus error that occurred such as bit error, stuff error, etc
    static constexpr auto error_code_type = xstd::bitrange::from<22, 23>();
    /// Bit location of where arbitration was lost.
    static constexpr auto arbitration_lost_loc = xstd::bitrange::from<24, 31>();
  };

  /// This struct holds CAN controller global status information.
  /// It is a condensed version of the status register.
  /// It is HW mapped to a 32-bit register: GSR (pg. 555)
  struct global_status
  {
    /// If 1, receive buffer has at least 1 complete message stored
    static constexpr auto receive_buffer = xstd::bitrange::from<0>();

    /// Bus status bit. If this is '1' then the bus is active, otherwise the bus
    /// is bus off.
    static constexpr auto bus_error = xstd::bitrange::from<7>();
  };

  /// This struct holds CAN controller status information. It is HW mapped to a
  /// 32-bit register: SR (pg. 564). Many of them are not here because they have
  /// counter parts in GSR (global status register).
  struct buffer_status
  {
    /// TX1 Buffer has been released
    static constexpr auto tx1_released = xstd::bitrange::from<2>();

    /// TX2 Buffer has been released
    static constexpr auto tx2_released = xstd::bitrange::from<10>();

    /// TX3 Buffer has been released
    static constexpr auto tx3_released = xstd::bitrange::from<18>();
  };

  /// CAN BUS modes
  struct mode
  {
    /// Reset CAN Controller, allows configuration registers to be modified.
    static constexpr auto reset = xstd::bitrange::from<0>();

    /// Put device into Listen Only Mode, device will not acknowledge, messages.
    static constexpr auto listen_only = xstd::bitrange::from<1>();

    /// Put device on self test mode.
    static constexpr auto self_test = xstd::bitrange::from<2>();

    /// Enable transmit priority control. When enabled, allows a particular
    static constexpr auto tx_priority = xstd::bitrange::from<3>();

    /// Put device to Sleep Mode.
    static constexpr auto sleep_mode = xstd::bitrange::from<4>();

    /// Receive polarity mode. If 1 RD input is active high
    static constexpr auto rx_polarity = xstd::bitrange::from<5>();

    /// Put CAN into test mode, which allows the TD pin to reflect its bits ot
    /// the RD pin.
    static constexpr auto test = xstd::bitrange::from<7>();
  };

  /// CAN Bus frame bit masks for the TFM and RFM registers
  struct frame_info
  {
    /// The message priority bits (not used in this implementation)
    static constexpr auto priority = xstd::bitrange::from<0, 7>();

    /// The length of the data
    static constexpr auto length = xstd::bitrange::from<16, 19>();

    /// If set to 1, the message becomes a remote request message
    static constexpr auto remote_request = xstd::bitrange::from<30>();

    /// If 0, the ID is 11-bits, if 1, the ID is 29-bits.
    static constexpr auto format = xstd::bitrange::from<31>();
  };

  /// https://www.nxp.com/docs/en/user-guide/UM10562.pdf (pg. 554)
  enum class commands : uint32_t
  {
    release_rx_buffer = 0x04,
    send_tx_buffer1 = 0x21,
    send_tx_buffer2 = 0x41,
    send_tx_buffer3 = 0x81,
    self_reception_send_tx_buffer1 = 0x30,
    accept_all_messages = 0x02,
  };

  /// Contains all of the information for to control and configure a CAN BUS bus
  /// on the LPC40xx platform.
  struct port
  {
    /// Reference to transmit pin object
    internal::pin td;

    /// Pin function code for transmit
    uint8_t td_function_code;

    /// Reference to read pin object
    internal::pin rd;

    /// Pin function code for receive
    uint8_t rd_function_code;

    /// Pointer to the LPC CAN peripheral in memory
    reg_t* reg;

    /// Peripheral's ID
    peripheral id;

    /// IRQ
    irq irq_number;

    /// Number of time quanta for sync bits - 1
    std::uint8_t sync_jump = 0;

    /// Number of time quanta for tseg1 - 1
    std::uint8_t tseg1 = 6;

    /// Number of time quanta for tseg2 - 1
    std::uint8_t tseg2 = 1;
  };

  /// Container for the LPC40xx CAN BUS registers
  struct lpc_message
  {
    /// TFI register contents
    uint32_t frame = 0;
    /// TID register contents
    uint32_t id = 0;
    /// TDA register contents
    uint32_t data_a = 0;
    /// TDB register contents
    uint32_t data_b = 0;
  };

  /// Pointer to the LPC CAN BUS acceptance filter peripheral in memory
  inline static auto& acceptance_filter()
  {
    if constexpr (hal::is_platform("lpc40")) {
      return *reinterpret_cast<acceptance_filter_t*>(0x4003'C000);
    } else if constexpr (hal::is_a_test()) {
      static acceptance_filter_t dummy{};
      return dummy;
    }
  }

  template<int PortNumber>
  static result<can&> get(can::settings p_settings = {})
  {
    compile_time_platform_check();
    static_assert(PortNumber == 1 || PortNumber == 2,
                  "\n\n"
                  "LPC40xx Compile Time Error:\n"
                  "    LPC40xx only supports CAN port numbers from 1 and 2. \n"
                  "\n");

    can::port port;

    if constexpr (PortNumber == 1) {
      port = can::port{
        .td = internal::pin(0, 1),
        .td_function_code = 1,
        .rd = internal::pin(0, 0),
        .rd_function_code = 1,
        .reg = reinterpret_cast<can::reg_t*>(0x4004'4000),
        .id = peripheral::can1,
        .irq_number = irq::can,
      };
    } else if constexpr (PortNumber == 2) {
      port = can::port{
        .td = internal::pin(2, 8),
        .td_function_code = 1,
        .rd = internal::pin(2, 7),
        .rd_function_code = 1,
        .reg = reinterpret_cast<can::reg_t*>(0x4004'8000),
        .id = peripheral::can2,
        .irq_number = irq::can,
      };
    } else {
      static_assert(hal::error::invalid_option<port>,
                    "Support can ports for LPC40xx are can1 and can2.");
    }

    if constexpr (hal::is_a_test()) {
      static std::array<can::reg_t, 2> registers{};
      port.reg = &registers[PortNumber - 1];
    }

    HAL_CHECK(setup(port, p_settings));

    static can can_channel(port);
    return can_channel;
  }

  /**
   * @brief Construct a new can object
   *
   * @param p_port - CAN port information
   */
  can(port p_port)
    : m_port(p_port)
  {
    cortex_m::interrupt::initialize<value(irq::max)>();
  }

  status driver_configure(const settings& p_settings) noexcept override;
  status driver_send(const message_t& p_message) noexcept override;

  /**
   * @note This interrupt handler is used by both CAN1 and CAN2. This should
   *     only be called for 1 can port to service both receive handlers.
   */
  status driver_on_receive([[maybe_unused]] std::function<can::handler>
                             p_receive_handler) noexcept override;

  /**
   * @brief Get the port details object
   *
   * @return auto& reference to the port details object
   */
  auto& get_port_details()
  {
    return m_port;
  }

  ~can()
  {
    // Disable generating an interrupt request by this CAN peripheral, but leave
    // the interrupt enabled. We must NOT disable the interrupt as it could be
    // used by the other CAN peripheral.
    xstd::bitmanip(m_port.reg->IER).reset(interrupts::received_message);
  }

private:
  /**
   * @brief Set the baud rate based on the can settings baud_rate.
   *
   */
  static status configure_baud_rate(const can::port& p_port,
                                    const settings& p_settings) noexcept;
  static status setup(const can::port& p_port, settings p_settings) noexcept;
  message_t receive() noexcept;
  bool has_data() noexcept;
  /**
   * @brief Convert can message into LPC40xx can bus register format.
   *
   * @param message message to convert
   * @return lpc_message the values to insert directly into the CAN register
   */
  lpc_message message_to_registers(const message_t& p_message) const;
  /**
   * @brief Accept all messages when called (by default the CAN peripheral will
   *     ignore all messages)
   *
   */
  static void enable_acceptance_filter();

  port m_port;
  std::function<can::handler> m_receive_handler;
};

// TODO: this needs to return a bool if the baud rate cannot be achieved
inline status can::configure_baud_rate(const can::port& p_port,
                                       const settings& p_settings) noexcept
{
  using namespace hal::literals;

  if (p_settings.baud_rate > 100.0_kHz &&
      !clock::get().config().use_external_oscillator) {
    return hal::new_error(std::errc::invalid_argument,
                          error_t::requires_usage_of_external_oscillator);
  }

  const auto frequency = clock::get().get_frequency(p_port.id);
  auto baud_rate_prescalar = p_settings.is_valid(frequency);

  if (!baud_rate_prescalar) {
    return hal::new_error(std::errc::invalid_argument,
                          error_t::baud_rate_impossible);
  }

  // Hold the results in RAM rather than altering the register directly
  // multiple times.
  xstd::bitmanip bus_timing(p_port.reg->BTR);

  const auto sync_jump = p_settings.synchronization_jump_width - 1;
  const auto tseg1 =
    (p_settings.propagation_delay + p_settings.phase_segment1) - 1;
  const auto tseg2 = p_settings.phase_segment2 - 1;
  const auto final_baudrate_prescale = baud_rate_prescalar.value() - 1;

  // Used to compensate for positive and negative edge phase errors. Defines
  // how much the sample point can be shifted.
  // These time segments determine the location of the "sample point".
  bus_timing.insert<bus_timing::sync_jump_width>(sync_jump)
    .insert<bus_timing::time_segment1>(tseg1)
    .insert<bus_timing::time_segment2>(tseg2)
    .insert<bus_timing::prescalar>(final_baudrate_prescale);

  if (p_settings.baud_rate < 100.0_kHz) {
    // The bus is sampled 3 times (recommended for low speeds, 100kHz is
    // considered HIGH).
    bus_timing.insert<bus_timing::sampling>(1);
  } else {
    bus_timing.insert<bus_timing::sampling>(0);
  }

  return success();
}

inline status can::setup(const can::port& p_port, settings p_settings) noexcept
{
  /// Power on CAN BUS peripheral
  power(p_port.id).on();

  /// Configure pins
  p_port.td.function(p_port.td_function_code);
  p_port.rd.function(p_port.rd_function_code);

  // Enable reset mode in order to write to CAN registers.
  xstd::bitmanip(p_port.reg->MOD).set(mode::reset);

  HAL_CHECK(configure_baud_rate(p_port, p_settings));
  enable_acceptance_filter();

  // Disable reset mode, enabling the device
  xstd::bitmanip(p_port.reg->MOD).reset(mode::reset);

  return success();
}

inline status can::driver_configure(const settings& p_settings) noexcept
{
  return configure_baud_rate(m_port, p_settings);
}

inline status can::driver_send(const message_t& p_message) noexcept
{
  lpc_message registers = message_to_registers(p_message);

  // Wait for one of the buffers to be free so we can transmit a message
  // through it.
  bool sent = false;
  while (!sent) {
    const auto status_register = xstd::bitset(m_port.reg->SR);
    // Check if any buffer is available.
    if (status_register.test(buffer_status::tx1_released)) {
      m_port.reg->TFI1 = registers.frame;
      m_port.reg->TID1 = registers.id;
      m_port.reg->TDA1 = registers.data_a;
      m_port.reg->TDB1 = registers.data_b;
      m_port.reg->CMR = value(commands::send_tx_buffer1);
      sent = true;
    } else if (status_register.test(buffer_status::tx2_released)) {
      m_port.reg->TFI2 = registers.frame;
      m_port.reg->TID2 = registers.id;
      m_port.reg->TDA2 = registers.data_a;
      m_port.reg->TDB2 = registers.data_b;
      m_port.reg->CMR = value(commands::send_tx_buffer2);
      sent = true;
    } else if (status_register.test(buffer_status::tx3_released)) {
      m_port.reg->TFI3 = registers.frame;
      m_port.reg->TID3 = registers.id;
      m_port.reg->TDA3 = registers.data_a;
      m_port.reg->TDB3 = registers.data_b;
      m_port.reg->CMR = value(commands::send_tx_buffer3);
      sent = true;
    }
  }

  return success();
}

inline bool can::has_data() noexcept
{
  return xstd::bitmanip(m_port.reg->GSR).test(global_status::receive_buffer);
}

inline status can::driver_on_receive(
  std::function<can::handler> p_receive_handler) noexcept
{
  if (p_receive_handler) {
    // Save the handler
    m_receive_handler = p_receive_handler;
    // Create a lambda that passes this object's reference to the stored handler
    auto isr = [this]() {
      auto message = receive();
      m_receive_handler(message);
    };
    auto handler = static_callable<can, 0, void(void)>(isr).get_handler();
    HAL_CHECK(cortex_m::interrupt(value(irq::can)).enable(handler));

    xstd::bitmanip(m_port.reg->IER).set(interrupts::received_message);
  } else {
    // Disable CAN interrupt
    xstd::bitmanip(m_port.reg->IER).reset(interrupts::received_message);
    // Disable the Cortex-M interrupt
    HAL_CHECK(cortex_m::interrupt(value(irq::can)).disable());
  }

  return success();
}

inline can::message_t can::receive() noexcept
{
  message_t message;

  // Extract all of the information from the message frame
  auto frame = xstd::bitset(m_port.reg->RFS);
  auto remote_request = frame.test(frame_info::remote_request);
  auto length = frame.extract<frame_info::length>().to<std::uint32_t>();

  message.is_remote_request = remote_request;
  message.length = static_cast<uint8_t>(length);

  // Get the frame ID
  message.id = xstd::bitset(m_port.reg->RID)
                 .extract<xstd::bitrange::from<0, 28>()>()
                 .to<decltype(message.id)>();

  // Pull the bytes from RDA into the payload array
  message.payload[0] = (m_port.reg->RDA >> (0 * 8)) & 0xFF;
  message.payload[1] = (m_port.reg->RDA >> (1 * 8)) & 0xFF;
  message.payload[2] = (m_port.reg->RDA >> (2 * 8)) & 0xFF;
  message.payload[3] = (m_port.reg->RDA >> (3 * 8)) & 0xFF;

  // Pull the bytes from RDB into the payload array
  message.payload[4] = (m_port.reg->RDB >> (0 * 8)) & 0xFF;
  message.payload[5] = (m_port.reg->RDB >> (1 * 8)) & 0xFF;
  message.payload[6] = (m_port.reg->RDB >> (2 * 8)) & 0xFF;
  message.payload[7] = (m_port.reg->RDB >> (3 * 8)) & 0xFF;

  // Release the RX buffer and allow another buffer to be read.
  m_port.reg->CMR = value(commands::release_rx_buffer);

  return message;
}

/// Convert message into the registers LPC40xx can bus registers.
///
/// @param message - message to convert.
inline can::lpc_message can::message_to_registers(
  const message_t& message) const
{
  static constexpr auto highest_11_bit_number = 2048UL;
  lpc_message registers;

  uint32_t frame_info = 0;

  if (message.id < highest_11_bit_number) {
    frame_info =
      xstd::bitset(0)
        .insert<frame_info::length>(message.length)
        .insert<frame_info::remote_request>(message.is_remote_request)
        .insert<frame_info::format>(0)
        .to<std::uint32_t>();
  } else {
    frame_info =
      xstd::bitset(0)
        .insert<frame_info::length>(message.length)
        .insert<frame_info::remote_request>(message.is_remote_request)
        .insert<frame_info::format>(1)
        .to<std::uint32_t>();
  }

  uint32_t data_a = 0;
  data_a |= message.payload[0] << (0 * 8);
  data_a |= message.payload[1] << (1 * 8);
  data_a |= message.payload[2] << (2 * 8);
  data_a |= message.payload[3] << (3 * 8);

  uint32_t data_b = 0;
  data_b |= message.payload[4] << (0 * 8);
  data_b |= message.payload[5] << (1 * 8);
  data_b |= message.payload[6] << (2 * 8);
  data_b |= message.payload[7] << (3 * 8);

  registers.frame = frame_info;
  registers.id = message.id;
  registers.data_a = data_a;
  registers.data_b = data_b;

  return registers;
}

inline void can::enable_acceptance_filter()
{
  acceptance_filter().acceptance_filter = value(commands::accept_all_messages);
}
}  // namespace hal::lpc40xx
