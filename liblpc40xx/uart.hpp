#pragma once

#include <atomic>
#include <cinttypes>
#include <span>

#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/serial.hpp>
#include <libembeddedhal/utility/interrupt.hpp>

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

  /// Bit code for enabling standard uart mode.
  static constexpr uint8_t standard_uart_mode = 0b011;
  /// Bit code for resetting UART FIFO and enabling peripheral
  static constexpr uint8_t enable_uart_code = 0b111;
  /// Bit code to power down UART driver
  static constexpr uint8_t power_down_code = ~enable_uart_code;

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
    pin tx;

    /// Refernce to a uart receiver pin
    pin rx;

    /// Function code to set the transmit pin to uart transmitter
    uint8_t tx_function;

    /// Function code to set the receive pin to uart receiver
    uint8_t rx_function;
  };

  /// @param port - a reference to a constant port_info
  explicit constexpr uart(const port_info& port)
    : m_port(port)
  {}

  [[nodiscard]] bool driver_initialize() override
  {
    // Power on UART peripheral
    power(m_port.id).on();

    configure_baud_rate();
    configure_format();

    m_port.tx.function(m_port.tx_function);
    m_port.rx.function(m_port.rx_function)
      .resistor(embed::pin_resistor::pull_up);

    port_.reg->FCR = port_.reg->FCR | enable_uart;

    // Enable interrupt service routine.
    cortex_m::interrupt(value(m_port_info.irq))
      .enable(get_reciever_interrupt());

    return true;
  }

  void power_down() { port_.reg->FCR = port_.reg->FCR & power_down_code; }

  [[nodiscard]] bool busy() override { return m_busy_writing; }

  void write(std::span<const std::byte> p_data) override
  {
    m_busy_writing.store(true);

    for (const auto& byte : p_data) {
      port_.reg->THR = std::to_integer<uint8_t>(byte);
      while (!transmission_complete()) {
        continue;
      }
    }

    m_busy_writing.store(false);
  }

  void read(std::span<std::byte> p_data) override
  {
    size_t index = 0;

    for (auto& byte : p_data) {
      if (!has_data()) {
        break;
      }
      byte = port_.reg->RBR;
      index++;
    }

    return;
  }

  ///
  [[nodiscard]] size_t bytes_available() override
  {
    return m_recieve_position = m_read_position;
  }

  /// To quickly "flush" the number of available bytes, simply set the number of
  /// read bytes equal to the number of recieved bytes.
  void flush() override { m_read_position.store(m_recieve_position); }

private:
  void configure_format()
  {
    // To be continued...
  }

  void configure_baud_rate()
  {
    constexpr uint8_t set_dlab_bit = (1 << 7);
    auto& system = sjsu::SystemController::GetPlatformController();
    auto peripheral_frequency = system.GetClockRate(port_.id);

    uart_internal::uart_calibration_t calibration =
      uart_internal::generate_uart_calibration(settings.baud_rate,
                                               peripheral_frequency);

    uint8_t dlm = static_cast<uint8_t>((calibration.divide_latch >> 8) & 0xFF);
    uint8_t dll = static_cast<uint8_t>(calibration.divide_latch & 0xFF);
    uint8_t fdr = static_cast<uint8_t>((calibration.multiply & 0xF) << 4 |
                                       (calibration.divide_add & 0xF));

    port_.reg->LCR = set_dlab_bit;
    port_.reg->DLM = dlm;
    port_.reg->DLL = dll;
    port_.reg->FDR = fdr;
    port_.reg->LCR = standard_uart_mode;
  }

  /// @return true if port is still sending the byte.
  bool transmission_complete() { return bit::Read(port_.reg->LSR, 5); }

  /// const reference to lpc40xx::Uart::port_info definition
  const port_info& m_port;
  std::atomic<bool> m_busy_writing;
  std::span<std::byte> m_recieve_buffer;
  std::atomic<uint32_t> m_recieve_position;
  std::atomic<uint32_t> m_read_position;
};
}