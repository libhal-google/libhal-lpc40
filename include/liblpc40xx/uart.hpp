#pragma once

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <span>

#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/serial.hpp>
#include <libembeddedhal/utility/interrupt.hpp>
#include <nonstd/ring_span.hpp>

#include "internal/constants.hpp"
#include "internal/pin.hpp"
#include "internal/system_controller.hpp"
#include "internal/uart.hpp"

namespace embed::lpc40xx {
/// Implementation of the UART peripheral for the LPC40xx family of
/// microcontrollers.
class uart : public embed::serial
{
public:
  struct lpc_uart_t
  {
    union
    {
      const volatile uint8_t RBR;
      volatile uint8_t THR;
      volatile uint8_t DLL;
      uint32_t RESERVED0;
    };
    union
    {
      volatile uint8_t DLM;
      volatile uint32_t IER;
    };
    union
    {
      const volatile uint32_t IIR;
      volatile uint8_t FCR;
    };
    volatile uint8_t LCR;
    uint8_t RESERVED1[7]; // Reserved
    const volatile uint8_t LSR;
    uint8_t RESERVED2[7]; // Reserved
    volatile uint8_t SCR;
    uint8_t RESERVED3[3]; // Reserved
    volatile uint32_t ACR;
    volatile uint8_t ICR;
    uint8_t RESERVED4[3]; // Reserved
    volatile uint8_t FDR;
    uint8_t RESERVED5[7]; // Reserved
    volatile uint8_t TER;
    uint8_t RESERVED8[27]; // Reserved
    volatile uint8_t RS485CTRL;
    uint8_t RESERVED9[3]; // Reserved
    volatile uint8_t ADRMATCH;
    uint8_t RESERVED10[3]; // Reserved
    volatile uint8_t RS485DLY;
    uint8_t RESERVED11[3]; // Reserved
    const volatile uint8_t FIFOLVL;
  };

  /// Port contains all of the information that the lpc40xx uart port needs to
  /// operate.
  struct port
  {
    /// Address of the LPC_UART peripheral in memory
    lpc_uart_t* reg;

    /// ResourceID of the UART peripheral to power on at initialization.
    peripheral id;

    irq irq_number;

    /// Refernce to a uart transmitter pin
    internal::pin tx;

    /// Refernce to a uart receiver pin
    internal::pin rx;

    /// Function code to set the transmit pin to uart transmitter
    uint8_t tx_function;

    /// Function code to set the receive pin to uart receiver
    uint8_t rx_function;
  };

  struct lcr
  {
    static constexpr auto word_length = xstd::bitrange::from<0, 1>();
    static constexpr auto stop = xstd::bitrange::from<2>();
    static constexpr auto parity_enable = xstd::bitrange::from<3>();
    static constexpr auto parity = xstd::bitrange::from<4, 5>();
  };

  struct ier
  {
    static constexpr auto receive_interrupt = xstd::bitrange::from<0>();
  };

  struct iir
  {
    static constexpr auto interrupt_id = xstd::bitrange::from<1, 3>();
  };

  struct fcr
  {
    static constexpr auto fifo_enable = xstd::bitrange::from<0>();
    static constexpr auto rx_fifo_clear = xstd::bitrange::from<1>();
    static constexpr auto tx_fifo_clear = xstd::bitrange::from<2>();
    static constexpr auto rx_trigger_level = xstd::bitrange::from<6, 7>();
  };

  /// @param port - a reference to a constant port
  /// @param receive_buffer - pointer to a buffer of bytes that will be used as
  /// a circular buffer to contain received bytes from this uart port.
  explicit uart(const port& p_port, std::span<std::byte> receive_buffer)
    : m_port(p_port)
    , m_busy_writing(false)
    , m_receive_buffer(receive_buffer.begin(), receive_buffer.end())
  {
    std::ranges::fill(receive_buffer, std::byte{ 0 });
  }

  /// @note that the baud rates less than or equal to the peripheral clock
  /// frequency / 48. Otherwise this peripheral cannot guarantee proper
  /// transmission or receive of bytes.
  [[nodiscard]] bool driver_initialize() override;
  [[nodiscard]] bool busy() override;
  void write(std::span<const std::byte> p_data) override;
  std::span<const std::byte> read(std::span<std::byte> p_data) override;
  [[nodiscard]] size_t bytes_available() override;
  void flush() override;

  /// Must not be called when sending data.
  ///
  /// This is typically only used internally by driver_initialize. But if there
  /// is an exact divider and fractional value that the user wants to use, then
  /// they can call this function directly.
  void configure_baud_rate(internal::uart_baud_t calibration);
  /// Reset TX and RX queues
  void reset_uart_queue();
  /// Interrupt service routine for pulling bytes out of the receive buffer.
  void interrupt();

private:
  bool configure_format();
  void setup_receive_interrupt();
  bool has_data() { return xstd::bitmanip(m_port.reg->LSR).test(0); }
  bool finished_sending() { return xstd::bitmanip(m_port.reg->LSR).test(5); }

  const port& m_port;
  std::atomic<bool> m_busy_writing;
  nonstd::ring_span<std::byte> m_receive_buffer;
};

template<int PortNumber, size_t BufferSize = 512>
inline uart& get_uart()
{
  static uart::port port;
  if constexpr (PortNumber == 0) {
    port = uart::port{
      // NOTE: required since LPC_UART0 is of type LPC_UART0_TypeDef in lpc17xx
      // and LPC_UART_TypeDef in lpc40xx causing a "useless cast" warning when
      // compiled for, some odd reason, for either one being compiled, which
      // would make more sense if it only warned us with lpc40xx.
      .reg = reinterpret_cast<uart::lpc_uart_t*>(0x4000'C000),
      .id = peripheral::uart0,
      .irq_number = irq::uart0,
      .tx = internal::pin(0, 2),
      .rx = internal::pin(0, 3),
      .tx_function = 0b001,
      .rx_function = 0b001,
    };
  } else if constexpr (PortNumber == 1) {
    port = uart::port{
      .reg = reinterpret_cast<uart::lpc_uart_t*>(0x4001'0000),
      .id = peripheral::uart1,
      .irq_number = irq::uart1,
      .tx = internal::pin(2, 8),
      .rx = internal::pin(2, 9),
      .tx_function = 0b010,
      .rx_function = 0b010,
    };
  } else if constexpr (PortNumber == 2) {
    port = uart::port{
      .reg = reinterpret_cast<uart::lpc_uart_t*>(0x4008'8000),
      .id = peripheral::uart2,
      .irq_number = irq::uart2,
      .tx = internal::pin(2, 8),
      .rx = internal::pin(2, 9),
      .tx_function = 0b010,
      .rx_function = 0b010,
    };
  } else if constexpr (PortNumber == 3) {
    port = uart::port{
      .reg = reinterpret_cast<uart::lpc_uart_t*>(0x4009'C000),
      .id = peripheral::uart3,
      .irq_number = irq::uart3,
      .tx = internal::pin(4, 28),
      .rx = internal::pin(4, 29),
      .tx_function = 0b010,
      .rx_function = 0b010,
    };
  } else if constexpr (PortNumber == 4) {
    port = uart::port{
      .reg = reinterpret_cast<uart::lpc_uart_t*>(0x400A'4000),
      .id = peripheral::uart4,
      .irq_number = irq::uart4,
      .tx = internal::pin(1, 28),
      .rx = internal::pin(2, 9),
      .tx_function = 0b101,
      .rx_function = 0b011,
    };
  } else {
    static_assert(
      embed::invalid_option<port>,
      "Support UART ports for LPC40xx are UART0, UART2, UART3, and UART4.");
  }

  static std::array<std::byte, BufferSize> receive_buffer;
  static uart uart_object(port, receive_buffer);
  return uart_object;
}
}

namespace embed::lpc40xx {
[[nodiscard]] inline bool uart::driver_initialize()
{
  // Power on UART peripheral
  internal::power(m_port.id).on();

  // Enable fifo for receiving bytes and to enable full access of the FCR
  // register.
  xstd::bitmanip(m_port.reg->FCR).set(fcr::fifo_enable);

  auto baud_rate = settings().baud_rate;
  auto frequency = internal::clock(m_port.id).frequency();
  auto baud_settings = internal::calculate_baud(baud_rate, frequency);
  bool is_valid_format = configure_format();

  // For proper operation of the UART port, the divider must be greater than 2
  // If it is not the cause that means that the baud rate is too high for this
  // device.
  if (baud_settings.divider <= 2 || !is_valid_format) {
    return false;
  }

  configure_baud_rate(baud_settings);

  internal::pin(m_port.tx).function(m_port.tx_function);
  internal::pin(m_port.rx)
    .function(m_port.rx_function)
    .resistor(embed::pin_resistor::pull_up);

  setup_receive_interrupt();

  // Clear the buffer
  flush();

  // Reset the UART queues
  reset_uart_queue();

  return true;
}

[[nodiscard]] inline bool uart::busy()
{
  return m_busy_writing;
}

inline void uart::write(std::span<const std::byte> p_data)
{
  m_busy_writing.store(true);

  for (const auto& byte : p_data) {
    m_port.reg->THR = std::to_integer<uint8_t>(byte);
    while (!finished_sending()) {
      continue;
    }
  }

  m_busy_writing.store(false);
}

inline std::span<const std::byte> uart::read(std::span<std::byte> p_data)
{
  int count = 0;
  for (auto& byte : p_data) {
    if (m_receive_buffer.empty()) {
      break;
    }

    byte = m_receive_buffer.pop_front();
    count++;
  }

  return p_data.subspan(0, count);
}

[[nodiscard]] inline size_t uart::bytes_available()
{
  return m_receive_buffer.size();
}

inline void uart::flush()
{
  while (!m_receive_buffer.empty()) {
    m_receive_buffer.pop_back();
  }
}

inline void uart::configure_baud_rate(internal::uart_baud_t calibration)
{
  static constexpr auto divisor_access = xstd::bitrange::from<7>();

  uint8_t dlm = static_cast<uint8_t>((calibration.divider >> 8) & 0xFF);
  uint8_t dll = static_cast<uint8_t>(calibration.divider & 0xFF);
  uint8_t fdr = static_cast<uint8_t>((calibration.numerator & 0xF) |
                                     (calibration.denominator & 0xF) << 4);

  xstd::bitmanip(m_port.reg->LCR).set(divisor_access);
  m_port.reg->DLM = dlm;
  m_port.reg->DLL = dll;
  m_port.reg->FDR = fdr;
  xstd::bitmanip(m_port.reg->LCR).reset(divisor_access);
}

inline void uart::reset_uart_queue()
{
  xstd::bitmanip(m_port.reg->FCR)
    .set(fcr::rx_fifo_clear)
    .set(fcr::tx_fifo_clear);
}

inline void uart::interrupt()
{
  [[maybe_unused]] auto lsr_value = m_port.reg->LSR;
  auto interrupt_type =
    xstd::bitmanip(m_port.reg->IIR).extract<iir::interrupt_id>().to_ulong();
  if (interrupt_type == 0x2 || interrupt_type == 0x6) {
    while (has_data()) {
      std::byte new_byte{ m_port.reg->RBR };
      if (!m_receive_buffer.full()) {
        m_receive_buffer.push_back(std::byte{ new_byte });
      }
    }
  }
}

inline bool uart::configure_format()
{
  xstd::bitmanip line_control(m_port.reg->LCR);

  // Set stop bit length
  switch (settings().stop) {
    case serial_settings::stop_bits::one:
      line_control.reset(lcr::stop);
      break;
    case serial_settings::stop_bits::two:
      line_control.set(lcr::stop);
      break;
  }

  // Set frame size
  switch (settings().frame_size) {
    case 5:
      line_control.insert<lcr::word_length>(0x0);
      break;
    case 6:
      line_control.insert<lcr::word_length>(0x1);
      break;
    case 7:
      line_control.insert<lcr::word_length>(0x2);
      break;
    case 8:
      line_control.insert<lcr::word_length>(0x3);
      break;
    default:
      return false;
  }

  // Preset the parity enable and disable it if the parity is set to none
  line_control.set(lcr::parity_enable);

  // Set frame parity
  switch (settings().parity) {
    case serial_settings::parity::odd:
      line_control.insert<lcr::parity>(0x0);
      break;
    case serial_settings::parity::even:
      line_control.insert<lcr::parity>(0x1);
      break;
    case serial_settings::parity::forced1:
      line_control.insert<lcr::parity>(0x2);
      break;
    case serial_settings::parity::forced0:
      line_control.insert<lcr::parity>(0x3);
      break;
    case serial_settings::parity::none:
      // Turn off parity if the parity is set to none
      line_control.reset(lcr::parity_enable);
      break;
  }

  // Destructor of line_control will save the contents to the LCR register
  return true;
}

inline void uart::setup_receive_interrupt()
{
  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt(); };

  // A pointer to save the static_callable isr address to.
  cortex_m::interrupt_pointer handler;

  switch (m_port.irq_number) {
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
  cortex_m::interrupt(value(m_port.irq_number)).enable(handler);

  // Enable uart interrupt signal
  xstd::bitmanip(m_port.reg->IER).set(ier::receive_interrupt);
  // 0x3 = 14 bytes in fifo before triggering a receive interrupt.
  // 0x2 = 8
  // 0x1 = 4
  // 0x0 = 1
  xstd::bitmanip(m_port.reg->FCR).insert<fcr::rx_trigger_level>(0x3);
}
}