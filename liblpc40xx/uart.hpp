#pragma once

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

// =============================================================================
//
// Interface
//
// =============================================================================
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
  struct port_info
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
    static constexpr auto divisor = xstd::bitrange::from<7>();
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

  /// @param port - a reference to a constant port_info
  /// @param receive_buffer - pointer to a buffer of bytes that will be used as
  /// a circular buffer to contain received bytes from this uart port.
  explicit uart(const port_info& port, std::span<std::byte> receive_buffer)
    : m_port(port)
    , m_busy_writing(false)
    , m_receive_buffer(receive_buffer.begin(), receive_buffer.end())
  {}

  [[nodiscard]] bool driver_initialize();
  [[nodiscard]] bool busy();
  void write(std::span<const std::byte> p_data);
  void read(std::span<std::byte> p_data);
  [[nodiscard]] size_t bytes_available();
  void flush() override;

  /// Must not be called when sending data.
  ///
  /// This is typically only used internally by driver_initialize. But if there
  /// is an exact divider and fractional value that the user wants to use, then
  /// they can call this function directly.
  void configure_baud_rate(internal::uart_baud_t calibration);
  /// Disable the UART peripheral function such that it can no longer
  /// communicate.
  void disable();
  /// Enable the UART peripheral function such communication can occur.
  void enable();
  /// Interrupt service routine for pulling bytes out of the receive buffer.
  void interrupt();

private:
  bool configure_format();
  void setup_receive_interrupt();
  bool has_data() { return xstd::bitmanip(m_port.reg->LSR).test(0); }
  bool finished_sending() { return xstd::bitmanip(m_port.reg->LSR).test(5); }

  const port_info& m_port;
  std::atomic<bool> m_busy_writing;
  nonstd::ring_span<std::byte> m_receive_buffer;
};
}

// =============================================================================
//
// Implementation
//
// =============================================================================
namespace embed::lpc40xx {

[[nodiscard]] inline bool uart::driver_initialize()
{
  // Power on UART peripheral
  internal::power(m_port.id).on();

  // Ensure that the UART peripheral is disabled before configuring it.
  disable();

  auto baud_rate = settings().baud_rate;
  auto frequency = internal::clock(m_port.id).frequency();
  auto baud_settings = internal::calculate_baud(baud_rate, frequency);

  if (baud_settings.divider == 0) {
    return false;
  }

  configure_baud_rate(baud_settings);
  if (!configure_format()) {
    return false;
  }

  internal::pin(m_port.tx).function(m_port.tx_function);
  internal::pin(m_port.rx)
    .function(m_port.rx_function)
    .resistor(embed::pin_resistor::pull_up);

  setup_receive_interrupt();

  // Clear the buffer
  flush();

  // Enable UART
  enable();

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

inline void uart::read(std::span<std::byte> p_data)
{
  for (auto& byte : p_data) {
    if (m_receive_buffer.empty()) {
      return;
    }

    byte = m_receive_buffer.pop_front();
  }

  return;
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
  uint8_t fdr = static_cast<uint8_t>((calibration.denominator & 0xF) << 4 |
                                     (calibration.numerator & 0xF));

  xstd::bitmanip(m_port.reg->LCR).set(divisor_access);
  m_port.reg->DLM = dlm;
  m_port.reg->DLL = dll;
  m_port.reg->FDR = fdr;
  xstd::bitmanip(m_port.reg->LCR).reset(divisor_access);
}

inline void uart::disable()
{
  xstd::bitmanip(m_port.reg->FCR)
    .set(fcr::rx_fifo_clear)
    .set(fcr::tx_fifo_clear)
    .reset(fcr::fifo_enable);
}

inline void uart::enable()
{
  xstd::bitmanip(m_port.reg->FCR)
    .set(fcr::rx_fifo_clear)
    .set(fcr::tx_fifo_clear)
    .set(fcr::fifo_enable);
}

inline void uart::interrupt()
{
  auto interrupt_type =
    xstd::bitmanip(m_port.reg->IIR).extract<iir::interrupt_id>().to_ulong();
  if (interrupt_type == 0x2 || interrupt_type == 0x6) {
    while (has_data()) {
      std::byte new_byte{ m_port.reg->RBR };
      if (!m_receive_buffer.full()) {
        m_receive_buffer.push_back(new_byte);
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
      line_control.insert<lcr::stop>(0);
      break;
    case serial_settings::stop_bits::two:
      line_control.insert<lcr::stop>(1);
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

  // Set parity bit
  // Preset the parity enable and disable it if the parity is set to none
  line_control.insert<lcr::parity_enable>(0x1);
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
      line_control.insert<lcr::parity_enable>(0x0);
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
  xstd::bitmanip(m_port.reg->FCR).insert<fcr::rx_trigger_level>(0x3);
  // Enable fifo for receiving bytes
  xstd::bitmanip(m_port.reg->FCR).set(fcr::fifo_enable);
}
}