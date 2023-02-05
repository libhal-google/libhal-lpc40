#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <span>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal/serial.hpp>
#include <nonstd/ring_span.hpp>

#include "constants.hpp"
#include "internal/pin.hpp"
#include "internal/platform_check.hpp"
#include "internal/uart.hpp"
#include "system_controller.hpp"

namespace hal::lpc40xx {
/**
 * @brief Implementation of the UART peripheral for the LPC40xx family of
 * microcontrollers.
 *
 * @note that the baud rates less than or equal to the peripheral clock
 * frequency / 48. Otherwise this peripheral cannot guarantee proper
 * transmission or receive of bytes.
 */
class uart : public hal::serial
{
public:
  /// peripheral register map
  struct reg_t
  {
    /// Union of registers overlapping offset address 0x000
    union union1
    {
      /// (R/ ) Contains the next received character to be read
      const volatile uint8_t receive_buffer;
      /// ( /W) The next character to be transmitted is written here (DLAB = 0)
      volatile uint8_t transmit_buffer;
      /// (R/W) Least significant byte of the baud rate divisor value. The full
      /// divisor is used to generate a baud rate from the fractional rate
      /// divider (DLAB = 1)
      volatile uint8_t divisor_latch_lsb;
      /// Simply here to expand the size of the first union to 32-bits
      uint32_t reserved0;
    };
    /// Offset: 0x000 Registers overlapping offset address 0x000
    union1 group1;
    /// Union of registers overlapping offset address 0x004
    union union2
    {
      /// (R/W) Most significant byte of the baud rate divisor value. The full
      /// divisor is used to generate a baud rate from the fractional rate
      /// divider (DLAB = 1)
      volatile uint8_t divisor_latch_msb;
      /// (R/W) Contains individual interrupt enable bits for the 7 potential
      /// UART interrupts (DLAB =0).
      volatile uint32_t interrupt_enable;
    };
    /// Offset: 0x004 Registers overlapping offset address 0x004
    union2 group2;
    /// Union of registers overlapping offset address 0x008
    union union3
    {
      /// (R/ ) Identifies which interrupt(s) are pending.
      const volatile uint32_t interrupt_id;
      /// ( /W) Controls UART FIFO usage and modes.
      volatile uint8_t fifo_control;
    };
    /// Offset: 0x008 Registers overlapping offset address 0x000
    union3 group3;
    /// Offset: 0x00C (R/W) Contains controls for frame formatting and break
    /// generation
    volatile uint8_t line_control;
    /// reserved 1
    std::array<uint8_t, 7> reserved1;
    /// Offset: 0x014 (R/ ) Contains flags for transmit and receive status,
    /// including line errors
    const volatile uint8_t line_status;
    /// reserved 2
    std::array<uint8_t, 7> reserved2;
    /// Offset: 0x01C (R/W) 8-bit temporary storage for software
    volatile uint8_t scratch_pad;
    /// reserved 3
    std::array<uint8_t, 3> reserved3;
    /// Offset: 0x020 (R/W) Contains controls for the auto-baud feature.
    volatile uint32_t autobaud_control;
    /// Offset:
    volatile uint8_t icr;
    /// reserved 4
    std::array<uint8_t, 3> reserved4;
    /// Offset: 0x028 (R/W) Generates a clock input for the baud rate divider.
    volatile uint8_t fractional_divider;
    /// reserved 5
    std::array<uint8_t, 7> reserved5;
    /// Offset: 0x030 (R/W) Turns off UART transmitter for use with software
    /// flow control.
    volatile uint8_t transmit_enable;
  };

  /// Port contains all of the information that the lpc40xx uart port needs to
  /// operate.
  struct port
  {
    /// Address of the LPC_UART peripheral in memory
    reg_t* reg;
    /// Resource ID of the UART peripheral to power on at initialization.
    peripheral id;
    /// Interrupt request number
    irq irq_number;
    /// Reference to a uart transmitter pin
    internal::pin tx;
    /// Reference to a uart receiver pin
    internal::pin rx;
    /// Function code to set the transmit pin to uart transmitter
    uint8_t tx_function;
    /// Function code to set the receive pin to uart receiver
    uint8_t rx_function;
  };

  /// Line control bit fields
  struct line_control
  {
    /// Word Length Select: Reset = 0
    /// - 0x0 5-bit character
    /// - 0x1 6-bit character
    /// - 0x2 7-bit character
    /// - 0x3 8-bit character
    static constexpr auto word_length = bit::mask::from<0, 1>();
    /// Stop Bit Select: Reset 0
    /// - 0 1 stop bit.
    /// - 1 2 stop bits. (1.5 if UnLCR[1:0]=00).)
    static constexpr auto stop = bit::mask::from<2>();
    /// Parity Enable: Reset 0
    /// - 0 Disable parity generation and checking.
    /// - 1 Enable parity generation and checking.
    static constexpr auto parity_enable = bit::mask::from<3>();
    /// Parity Select 0
    /// - 0x0 Odd parity. Number of 1s in the transmitted character and the
    ///   attached parity bit will be odd.
    /// - 0x1 Even Parity. Number of 1s in the transmitted character and the
    ///   attached parity bit will be even.
    /// - 0x2 Forced 1 stick parity.
    /// - 0x3 Forced 0 stick parity.
    static constexpr auto parity = bit::mask::from<4, 5>();
  };

  /// Interrupt enable bit fields
  struct interrupt_enable
  {
    /// RBR Interrupt Enable. Enables the Receive Data Available interrupt for
    /// UARTn: Reset 0 It also controls the Character Receive Time-out
    /// interrupt.
    /// - 0 Disable the RDA interrupts.
    /// - 1 Enable the RDA interrupts.
    static constexpr auto receive_interrupt = bit::mask::from<0>();
  };

  /// Interrupt ID bit fields
  struct interrupt_id
  {
    /// Interrupt identification. UnIER[3:1] identifies an interrupt
    /// corresponding to the UARTn Rx or TX FIFO. All other combinations of
    /// UnIER[3:1] not listed below are reserved (000,100,101,111).
    /// - 0x3 1 - Receive Line Status (RLS).
    /// - 0x2 2a - Receive Data Available (RDA).
    /// - 0x6 2b - Character Time-out Indicator (CTI).
    /// - 0x1 3 - THRE Interrupt
    static constexpr auto id = bit::mask::from<1, 3>();
  };

  /// FIFO control bit fields
  struct fifo_control
  {
    /// FIFO Enable: Reset 0
    /// - 0 UARTn FIFOs are disabled. Must not be used in the application.
    /// - 1 Active high enable for both UARTn Rx and TX FIFOs and UnFCR[7:1]
    /// access. This bit must be set for proper UART operation. Any transition
    /// on this bit will automatically clear the related UART FIFOs.
    static constexpr auto fifo_enable = bit::mask::from<0>();
    /// RX FIFO Reset: Reset 0
    /// - 0 No impact on either of UARTn FIFOs.
    /// - 1 Writing a logic 1 to UnFCR[1] will clear all bytes in UARTn Rx FIFO,
    /// reset the pointer logic. This bit is self-clearing.
    static constexpr auto rx_fifo_clear = bit::mask::from<1>();
    /// TX FIFO Reset: Reset 0
    /// - 0 No impact on either of UARTn FIFOs.
    /// - 1 Writing a logic 1 to UnFCR[2] will clear all bytes in UARTn TX FIFO,
    /// reset the pointer logic. This bit is self-clearing.
    static constexpr auto tx_fifo_clear = bit::mask::from<2>();
    /// RX Trigger Level. These two bits determine how many receiver UARTn FIFO
    /// characters must be written before an interrupt or DMA request is
    /// activated: Reset 0
    /// - 0x0 Trigger level 0 (1 character or 0x01).
    /// - 0x1 Trigger level 1 (4 characters or 0x04).
    /// - 0x2 Trigger level 2 (8 characters or 0x08).
    /// - 0x3 Trigger level 3 (14 characters or 0x0E).
    static constexpr auto rx_trigger_level = bit::mask::from<6, 7>();
  };

  /**
   * @brief Retrieve a UART serial port
   *
   * @tparam PortNumber - which uart port number to return
   * @tparam BufferSize - the size of the reception working buffer
   * @param p_settings - the initial settings for the uart driver
   * @return uart& - reference of the uart serial driver
   */
  template<int PortNumber, size_t BufferSize = 512>
  static result<uart&> get(serial::settings p_settings = {})
  {
    compile_time_platform_check();

    static uart::port port;
    if constexpr (PortNumber == 0) {
      port = uart::port{
        // NOTE: required since LPC_UART0 is of type LPC_UART0_TypeDef in
        // lpc17xx
        // and LPC_UART_TypeDef in lpc40xx causing a "useless cast" warning when
        // compiled for, some odd reason, for either one being compiled, which
        // would make more sense if it only warned us with lpc40xx.
        .reg = reinterpret_cast<uart::reg_t*>(0x4000'C000),
        .id = peripheral::uart0,
        .irq_number = irq::uart0,
        .tx = internal::pin(0, 2),
        .rx = internal::pin(0, 3),
        .tx_function = 0b001,
        .rx_function = 0b001,
      };
    } else if constexpr (PortNumber == 1) {
      port = uart::port{
        .reg = reinterpret_cast<uart::reg_t*>(0x4001'0000),
        .id = peripheral::uart1,
        .irq_number = irq::uart1,
        .tx = internal::pin(2, 0),
        .rx = internal::pin(2, 1),
        .tx_function = 0b010,
        .rx_function = 0b010,
      };
    } else if constexpr (PortNumber == 2) {
      port = uart::port{
        .reg = reinterpret_cast<uart::reg_t*>(0x4008'8000),
        .id = peripheral::uart2,
        .irq_number = irq::uart2,
        .tx = internal::pin(2, 8),
        .rx = internal::pin(2, 9),
        .tx_function = 0b010,
        .rx_function = 0b010,
      };
    } else if constexpr (PortNumber == 3) {
      port = uart::port{
        .reg = reinterpret_cast<uart::reg_t*>(0x4009'C000),
        .id = peripheral::uart3,
        .irq_number = irq::uart3,
        .tx = internal::pin(4, 28),
        .rx = internal::pin(4, 29),
        .tx_function = 0b010,
        .rx_function = 0b010,
      };
    } else if constexpr (PortNumber == 4) {
      port = uart::port{
        .reg = reinterpret_cast<uart::reg_t*>(0x400A'4000),
        .id = peripheral::uart4,
        .irq_number = irq::uart4,
        .tx = internal::pin(1, 28),
        .rx = internal::pin(2, 9),
        .tx_function = 0b101,
        .rx_function = 0b011,
      };
    } else {
      static_assert(
        hal::error::invalid_option<PortNumber>,
        "Support UART ports for LPC40xx are UART0, UART2, UART3, and UART4.");
    }

    cortex_m::interrupt::initialize<value(irq::max)>();

    static std::array<hal::byte, BufferSize> receive_buffer;
    static uart uart_object(port, receive_buffer);

    HAL_CHECK(uart_object.driver_configure(p_settings));

    return uart_object;
  }

  static result<uart> construct_custom(
    uart::port p_port,
    std::span<hal::byte> p_receive_working_buffer,
    serial::settings p_settings = {})
  {
    compile_time_platform_check();

    cortex_m::interrupt::initialize<value(irq::max)>();
    uart uart_object(p_port, p_receive_working_buffer);

    HAL_CHECK(uart_object.driver_configure(p_settings));

    return uart_object;
  }

private:
  explicit uart(const port& p_port, std::span<hal::byte> p_receive_buffer)
    : m_port(&p_port)
    , m_receive_buffer(p_receive_buffer.begin(), p_receive_buffer.end())
  {
  }

  status driver_configure(const settings& p_settings) override;
  result<write_t> driver_write(std::span<const hal::byte> p_data) override;
  result<read_t> driver_read(std::span<hal::byte> p_data) override;
  status driver_flush() override;

  void configure_baud_rate(internal::uart_baud_t p_calibration);
  void reset_uart_queue();
  void interrupt();
  uint8_t get_line_control(const settings& p_settings);
  void setup_receive_interrupt();
  bool has_data()
  {
    return bit::extract<bit::mask::from<0U>()>(m_port->reg->line_status);
  }
  bool finished_sending()
  {
    return bit::extract<bit::mask::from<5U>()>(m_port->reg->line_status);
  }

  const port* m_port;
  nonstd::ring_span<hal::byte> m_receive_buffer;
};

inline status uart::driver_configure(const settings& p_settings)
{
  // Validate the settings before configuring any hardware
  auto baud_rate = static_cast<std::uint32_t>(p_settings.baud_rate);
  auto uart_frequency = clock::get().get_frequency(m_port->id);
  auto uart_frequency_hz = static_cast<std::uint32_t>(uart_frequency);
  auto baud_settings = internal::calculate_baud(baud_rate, uart_frequency_hz);

  // For proper operation of the UART port, the divider must be greater than 2
  // If it is not the cause that means that the baud rate is too high for this
  // device.
  if (baud_settings.divider <= 2) {
    return hal::new_error(std::errc::invalid_argument);
  }

  // Power on UART peripheral
  power(m_port->id).on();

  // Enable fifo for receiving bytes and to enable full access of the FCR
  // register.
  bit::modify(m_port->reg->group3.fifo_control).set(fifo_control::fifo_enable);
  m_port->reg->line_control = get_line_control(p_settings);

  configure_baud_rate(baud_settings);

  internal::pin(m_port->tx).function(m_port->tx_function);
  internal::pin(m_port->rx)
    .function(m_port->rx_function)
    .resistor(hal::pin_resistor::pull_up);

  setup_receive_interrupt();

  // Clear the buffer
  driver_flush();

  // Reset the UART queues
  reset_uart_queue();

  return hal::success();
}

inline result<serial::write_t> uart::driver_write(
  std::span<const hal::byte> p_data)
{
  for (const auto& byte : p_data) {
    m_port->reg->group1.transmit_buffer = byte;
    while (!finished_sending()) {
      continue;
    }
  }

  return write_t{ .data = p_data };
}

inline result<serial::read_t> uart::driver_read(std::span<hal::byte> p_data)
{
  size_t count = 0;
  for (auto& byte : p_data) {
    if (m_receive_buffer.empty()) {
      break;
    }

    byte = m_receive_buffer.pop_front();
    count++;
  }

  return read_t{
    .data = p_data.subspan(0, count),
    .available = m_receive_buffer.size(),
    .capacity = m_receive_buffer.capacity(),
  };
}

inline status uart::driver_flush()
{
  while (!m_receive_buffer.empty()) {
    m_receive_buffer.pop_back();
  }
  return success();
}

inline void uart::configure_baud_rate(internal::uart_baud_t p_calibration)
{
  static constexpr auto divisor_access = bit::mask::from<7>();

  uint8_t divisor_latch_msb =
    static_cast<uint8_t>((p_calibration.divider >> 8) & 0xFF);
  uint8_t divisor_latch_lsb =
    static_cast<uint8_t>(p_calibration.divider & 0xFF);
  uint8_t fractional_divider = static_cast<uint8_t>(
    (p_calibration.numerator & 0xF) | (p_calibration.denominator & 0xF) << 4);

  bit::modify(m_port->reg->line_control).set(divisor_access);
  m_port->reg->group1.divisor_latch_lsb = divisor_latch_lsb;
  m_port->reg->group2.divisor_latch_msb = divisor_latch_msb;
  m_port->reg->fractional_divider = fractional_divider;
  bit::modify(m_port->reg->line_control).clear(divisor_access);
}

inline void uart::reset_uart_queue()
{
  bit::modify(m_port->reg->group3.fifo_control)
    .set(fifo_control::rx_fifo_clear)
    .set(fifo_control::tx_fifo_clear);
}

inline void uart::interrupt()
{
  [[maybe_unused]] auto line_status_value = m_port->reg->line_status;
  auto interrupt_type =
    bit::extract<interrupt_id::id>(m_port->reg->group3.interrupt_id);
  if (interrupt_type == 0x2 || interrupt_type == 0x6) {
    while (has_data()) {
      hal::byte new_byte{ m_port->reg->group1.receive_buffer };
      if (!m_receive_buffer.full()) {
        m_receive_buffer.push_back(hal::byte{ new_byte });
      }
    }
  }
}

inline uint8_t uart::get_line_control(const settings& p_settings)
{
  bit::value<std::uint8_t> line_control_object(0);

  // Set stop bit length
  switch (p_settings.stop) {
    case settings::stop_bits::one:
      line_control_object.clear(line_control::stop);
      break;
    case settings::stop_bits::two:
      line_control_object.set(line_control::stop);
      break;
  }

  // Set frame size to 8 = 0x3
  line_control_object.insert<line_control::word_length>(0x3U);

  // Preset the parity enable and disable it if the parity is set to none
  line_control_object.set(line_control::parity_enable);

  // Set frame parity
  switch (p_settings.parity) {
    case settings::parity::odd:
      line_control_object.insert<line_control::parity>(0x0U);
      break;
    case settings::parity::even:
      line_control_object.insert<line_control::parity>(0x1U);
      break;
    case settings::parity::forced1:
      line_control_object.insert<line_control::parity>(0x2U);
      break;
    case settings::parity::forced0:
      line_control_object.insert<line_control::parity>(0x3U);
      break;
    case settings::parity::none:
      // Turn off parity if the parity is set to none
      line_control_object.clear(line_control::parity_enable);
      break;
  }

  return line_control_object.get();
}

inline void uart::setup_receive_interrupt()
{
  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt(); };

  // A pointer to save the static_callable isr address to.
  cortex_m::interrupt_pointer handler;

  switch (m_port->irq_number) {
    case irq::uart0:
      handler = static_callable<uart, 0, void(void)>(isr).get_handler();
      break;
    case irq::uart1:
      handler = static_callable<uart, 1, void(void)>(isr).get_handler();
      break;
    case irq::uart2:
      handler = static_callable<uart, 2, void(void)>(isr).get_handler();
      break;
    case irq::uart3:
      handler = static_callable<uart, 3, void(void)>(isr).get_handler();
      break;
    case irq::uart4:
    default:
      handler = static_callable<uart, 4, void(void)>(isr).get_handler();
      break;
  }

  // Enable interrupt service routine.
  cortex_m::interrupt(value(m_port->irq_number)).enable(handler);

  // Enable uart interrupt signal
  bit::modify(m_port->reg->group2.interrupt_enable)
    .set<interrupt_enable::receive_interrupt>();
  // 0x3 = 14 bytes in fifo before triggering a receive interrupt.
  // 0x2 = 8
  // 0x1 = 4
  // 0x0 = 1
  bit::modify(m_port->reg->group3.fifo_control)
    .insert<fifo_control::rx_trigger_level>(0x3U);
}
}  // namespace hal::lpc40xx