#pragma once

#include <cstdint>

#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/i2c/i2c.hpp>
#include <libembeddedhal/static_callable.hpp>

#include "internal/constants.hpp"
#include "internal/pin.hpp"
#include "system_controller.hpp"

namespace embed::lpc40xx {
class i2c : public embed::i2c
{
public:
  struct lpc_i2c_t
  {
    /// Offset: 0x000 I2C control Set Register (R/W)
    volatile uint32_t CONSET;
    /// Offset: 0x004 I2C Status Register (R/ )
    const volatile uint32_t STAT;
    /// Offset: 0x008 I2C Data Register (R/W)
    volatile uint32_t DAT;
    /// Offset: 0x00C I2C Slave Address Register 0 (R/W)
    volatile uint32_t ADR0;
    /// Offset: 0x010 SCH Duty Cycle Register High Half Word (R/W)
    volatile uint32_t SCLH;
    /// Offset: 0x014 SCL Duty Cycle Register Low Half Word (R/W)
    volatile uint32_t SCLL;
    /// Offset: 0x018 I2C control Clear Register ( /W)
    volatile uint32_t CONCLR;
    /// Offset: 0x01C Monitor mode control register (R/W)
    volatile uint32_t MMCTRL;
    /// Offset: 0x020 I2C Slave Address Register 1 (R/W)
    volatile uint32_t ADR1;
    /// Offset: 0x024 I2C Slave Address Register 2 (R/W)
    volatile uint32_t ADR2;
    /// Offset: 0x028 I2C Slave Address Register 3 (R/W)
    volatile uint32_t ADR3;
    /// Offset: 0x02C Data buffer register ( /W)
    const volatile uint32_t DATA_BUFFER;
    /// Offset: 0x030 I2C Slave address mask register 0 (R/W)
    volatile uint32_t MASK0;
    /// Offset: 0x034 I2C Slave address mask register 1 (R/W)
    volatile uint32_t MASK1;
    /// Offset: 0x038 I2C Slave address mask register 2 (R/W)
    volatile uint32_t MASK2;
    /// Offset: 0x03C I2C Slave address mask register 3 (R/W)
    volatile uint32_t MASK3;
  };

  /// lpc40xx i2c peripheral control register flags
  struct control
  {
    // AA
    static constexpr auto assert_acknowledge = 1 << 2;
    // SI
    static constexpr auto interrupt = 1 << 3;
    // STO
    static constexpr auto stop = 1 << 4;
    // STA
    static constexpr auto start = 1 << 5;
    // I2EN
    static constexpr auto interface_enable = 1 << 6;
  };

  /// lpc40xx I2C peripheral state numbers
  enum class master_state : uint32_t
  {
    bus_error = 0x00,
    start_condition = 0x08,
    repeated_start = 0x10,
    slave_address_write_sent_received_ack = 0x18,
    slave_address_write_sent_received_nack = 0x20,
    transmitted_data_received_ack = 0x28,
    transmitted_data_received_nack = 0x30,
    arbitration_lost = 0x38,
    slave_address_read_sent_received_ack = 0x40,
    slave_address_read_sent_received_nack = 0x48,
    received_data_received_ack = 0x50,
    received_data_received_nack = 0x58,
    own_address_received = 0xA0,
    do_nothing = 0xF8
  };

  /// port holds all of the information for an I2C bus on the LPC40xx
  /// platform.
  struct bus
  {
    /// Holds a pointer to the LPC_I2C peripheral reg
    lpc_i2c_t* reg;
    /// ResourceID of the I2C peripheral to power on at initialization.
    peripheral peripheral_id;
    /// IRQ number for this I2C port.
    irq irq_number;
    /// Refernce to I2C data pin.
    internal::pin sda;
    /// Function code to set each pin to the appropriate I2C function.
    uint8_t sda_function;
    /// Refernce to I2C clock pin.
    internal::pin scl;
    /// Function code to set each pin to the appropriate I2C function.
    uint8_t scl_function;
    /// Clock rate duty cycle
    percent duty_cycle = percent::from_ratio(1, 2);
  };

  i2c(bus p_bus, const settings& p_settings = {})
    : m_bus(p_bus)
  {
    if constexpr (!embed::is_platform("lpc40")) {
      static lpc_i2c_t dummy{};
      m_bus.reg = &dummy;
    }
    cortex_m::interrupt::initialize<value(irq::max)>();
    driver_configure(p_settings);
  }

  boost::leaf::result<void> driver_configure(
    const settings& p_settings) noexcept override;
  boost::leaf::result<void> driver_transaction(
    std::byte p_address,
    std::span<const std::byte> p_data_out,
    std::span<std::byte> p_data_in) noexcept override;
  /**
   * @brief I2C interrupt service routine
   *
   */
  void interrupt();
  void disable();
  ~i2c() { disable(); }

private:
  bus& m_bus;
  std::errc m_status = std::errc(0);
  std::byte m_address = std::byte{ 0x00 };
  bool m_busy = false;
  std::span<const std::byte>::iterator m_write_iterator;
  std::span<const std::byte>::iterator m_write_end;
  std::span<std::byte>::iterator m_read_iterator;
  std::span<std::byte>::iterator m_read_end;
};

template<int BusNumber>
inline auto& get_i2c(const i2c::settings& p_settings = {})
{
  // UM10562: Chapter 7: LPC408x/407x I/O configuration page 13
  if constexpr (BusNumber == 0) {
    /// Definition for I2C bus 0 for LPC40xx.
    static i2c::bus bus0 = {
      .reg = reinterpret_cast<i2c::lpc_i2c_t*>(0x4001'C000),
      .peripheral_id = peripheral::i2c0,
      .irq_number = irq::i2c0,
      .sda = internal::pin(0, 0),
      .sda_function = 0b010,
      .scl = internal::pin(0, 1),
      .scl_function = 0b010,
    };

    static i2c i2c0(bus0, p_settings);
    return i2c0;
  } else if constexpr (BusNumber == 1) {
    /// Definition for I2C bus 1 for LPC40xx.
    static i2c::bus bus1 = {
      .reg = reinterpret_cast<i2c::lpc_i2c_t*>(0x4005'C000),
      .peripheral_id = peripheral::i2c1,
      .irq_number = irq::i2c1,
      .sda = internal::pin(1, 30),
      .sda_function = 0b011,
      .scl = internal::pin(1, 31),
      .scl_function = 0b011,
    };

    static i2c i2c1(bus1, p_settings);
    return i2c1;
  } else if constexpr (BusNumber == 2) {
    /// Definition for I2C bus 2 for LPC40xx.
    static i2c::bus bus2 = {
      .reg = reinterpret_cast<i2c::lpc_i2c_t*>(0x400A'0000),
      .peripheral_id = peripheral::i2c2,
      .irq_number = irq::i2c2,
      .sda = internal::pin(0, 10),
      .sda_function = 0b010,
      .scl = internal::pin(0, 11),
      .scl_function = 0b010,
    };

    static i2c i2c2(bus2, p_settings);
    return i2c2;
  } else {
    static_assert(embed::error::invalid_option<BusNumber>,
                  "Supported I2C busses are I2C0, I2C1, and I2C2.");
    return get_i2c<0>();
  }
}
}

namespace embed::lpc40xx {
inline boost::leaf::result<void> i2c::driver_configure(
  const settings& p_settings)
{
  // Power on peripheral
  internal::power(m_bus.peripheral_id).on();

  // Setup pins for SDA and SCL
  internal::pin(m_bus.sda)
    .function(m_bus.sda_function)
    .resistor(pin_resistor::pull_up)
    .open_drain(true);

  internal::pin(m_bus.scl)
    .function(m_bus.scl_function)
    .resistor(pin_resistor::pull_up)
    .open_drain(true);

  // Setup I2C operating freqency
  const auto duty_cycle =
    internal::clock()
      .get_frequency(m_bus.peripheral_id)
      .calculate_duty_cycle(p_settings.clock_rate, m_bus.duty_cycle);
  m_bus.reg->SCLL = duty_cycle.low;
  m_bus.reg->SCLH = duty_cycle.high;

  // Clear all transmission flags
  m_bus.reg->CONCLR = control::assert_acknowledge | control::start |
                      control::stop | control::interrupt;
  // Enable I2C interface
  m_bus.reg->CONSET = control::interface_enable;

  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt(); };

  // A pointer to save the static_callable isr address to.
  cortex_m::interrupt_pointer handler;

  switch (m_bus.irq_number) {
    case irq::i2c0:
      handler = static_callable<i2c, 0, void(void)>(isr).get_handler();
      break;
    case irq::i2c1:
      handler = static_callable<i2c, 1, void(void)>(isr).get_handler();
      break;
    case irq::i2c2:
    default:
      handler = static_callable<i2c, 2, void(void)>(isr).get_handler();
      break;
  }

  // Enable interrupt service routine.
  cortex_m::interrupt(value(m_bus.irq_number)).enable(handler);

  return {};
}

inline void i2c::disable()
{
  // Disable I2C interface
  m_bus.reg->CONCLR = control::interface_enable;

  // Enable interrupt service routine.
  cortex_m::interrupt(static_cast<int>(m_bus.irq_number)).disable();
}

inline boost::leaf::result<void> i2c::driver_transaction(
  std::byte p_address,
  std::span<const std::byte> p_data_out,
  std::span<std::byte> p_data_in)
{
  m_address = p_address;
  m_write_iterator = p_data_out.begin();
  m_write_end = p_data_out.end();
  m_read_iterator = p_data_in.begin();
  m_read_end = p_data_in.end();
  m_busy = true;

  // Start the transaction
  m_bus.reg->CONSET = control::start;

  // i2c::interrupt() will set this to false when the transaction has finished.
  while (m_busy) {
    continue;
  }

  return {};
}

inline void i2c::interrupt()
{
  master_state state = master_state(m_bus.reg->STAT);
  auto& data = m_bus.reg->DAT;
  uint32_t clear_mask = 0;
  uint32_t set_mask = 0;
  bool transaction_finished = false;

  switch (state) {
    case master_state::bus_error: {
      m_status = std::errc::io_error;
      set_mask = control::assert_acknowledge | control::stop;
      break;
    }
    case master_state::start_condition:
    case master_state::repeated_start: {
      data = std::to_integer<uint32_t>(m_address);
      break;
    }
    case master_state::slave_address_write_sent_received_ack: {
      clear_mask = control::start;
      if (m_write_iterator == m_write_end) {
        transaction_finished = true;
        set_mask = control::stop;
      } else {
        data = std::to_integer<uint32_t>(*m_write_iterator++);
      }
      break;
    }
    case master_state::slave_address_write_sent_received_nack: {
      clear_mask = control::start;
      transaction_finished = true;
      m_status = std::errc::no_such_device_or_address;
      set_mask = control::stop;
      break;
    }
    case master_state::transmitted_data_received_ack: {
      if (m_write_iterator == m_write_end) {
        if (m_read_iterator != m_read_end) {
          // Set bit 7 to indicate that this is a READ transaction
          m_address |= std::byte{ 1 << 7 };
          set_mask = control::start;
        } else {
          transaction_finished = true;
          set_mask = control::stop;
        }
      } else {
        data = std::to_integer<uint32_t>(*m_write_iterator++);
      }
      break;
    }
    case master_state::transmitted_data_received_nack: {
      transaction_finished = true;
      set_mask = control::stop;
      break;
    }
    case master_state::arbitration_lost: {
      set_mask = control::start;
      break;
    }
    case master_state::slave_address_read_sent_received_ack: {
      clear_mask = control::start;
      if (m_read_iterator == m_read_end) {
        set_mask = control::stop;
      }
      // If we only want 1 byte, make sure to nack that byte
      else if (m_read_iterator + 1 == m_read_end) {
        clear_mask |= control::assert_acknowledge;
      }
      // If we want more then 1 byte, make sure to ack the first byte
      else {
        set_mask = control::assert_acknowledge;
      }
      break;
    }
    case master_state::slave_address_read_sent_received_nack: {
      clear_mask = control::start;
      m_status = std::errc::no_such_device_or_address;
      transaction_finished = true;
      set_mask = control::stop;
      break;
    }
    case master_state::received_data_received_ack: {
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<std::byte>(data);
      }
      // Check if the position has been pushed past the buffer length
      if (m_read_iterator + 1 == m_read_end) {
        clear_mask = control::assert_acknowledge;
        transaction_finished = true;
      } else {
        set_mask = control::assert_acknowledge;
      }
      break;
    }
    case master_state::received_data_received_nack: {
      transaction_finished = true;
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<std::byte>(data);
      }
      set_mask = control::stop;
      break;
    }
    case master_state::do_nothing: {
      break;
    }
    default: {
      clear_mask = control::stop;
      break;
    }
  }
  // Clear I2C Interrupt flag
  clear_mask |= control::interrupt;
  // Set register controls
  m_bus.reg->CONSET = set_mask;
  m_bus.reg->CONCLR = clear_mask;

  if (transaction_finished) {
    m_busy = false;
  }
}
}
