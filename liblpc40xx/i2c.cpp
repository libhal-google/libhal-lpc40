#include "i2c.hpp"
#include "internal/pin.hpp"
#include "internal/system_controller.hpp"

#include <cinttypes>
#include <libarmcortex/interrupt.hpp>
#include <libembeddedhal/i2c.hpp>
#include <libembeddedhal/utility/interrupt.hpp>

namespace embed::lpc40xx {
i2c::i2c(port& p_port)
  : m_port_info(p_port)
{
  if constexpr (!embed::is_platform("lpc40")) {
    static lpc_i2c_t dummy{};
    m_port_info.reg = &dummy;
  }
}

bool i2c::driver_initialize()
{
  // Power on peripheral
  power(m_port_info.peripheral_id).on();

  // Setup pins for SDA and SCL
  pin(m_port_info.sda_port, m_port_info.sda_pin)
    .function(m_port_info.sda_scl_function)
    .resistor(pin_resistor::none) // This probably shouldn't be none
    .open_drain(true);

  pin(m_port_info.scl_port, m_port_info.scl_pin)
    .function(m_port_info.sda_scl_function)
    .resistor(pin_resistor::none) // This probably shouldn't be none
    .open_drain(true);

  // Setup I2C operating freqency
  configure_clock_rate();

  // Clear all transmission flags
  m_port_info.reg->CONCLR = control::assert_acknowledge | control::start |
                            control::stop | control::interrupt;
  // Enable I2C interface
  m_port_info.reg->CONSET = control::interface_enable;

  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt(); };

  // A pointer to save the static_callable isr address to.
  cortex_m::interrupt_pointer handler;

  switch (m_port_info.irq_number) {
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
  cortex_m::interrupt(value(m_port_info.irq_number)).enable(handler);

  return true;
}

void i2c::driver_power_down()
{
  // Disable I2C interface
  m_port_info.reg->CONCLR = control::interface_enable;

  // Enable interrupt service routine.
  cortex_m::interrupt(static_cast<int>(m_port_info.irq_number)).disable();
}

void i2c::transaction(uint8_t p_address,
                      std::span<const std::byte> p_data_out,
                      std::span<std::byte> p_data_in)
{
  m_address = p_address;
  m_write_iterator = p_data_out.begin();
  m_write_end = p_data_out.end();
  m_read_iterator = p_data_in.begin();
  m_read_end = p_data_in.end();

  // Start the transaction
  m_port_info.reg->CONSET = control::start;
}

bool i2c::busy()
{
  return m_busy;
}

/// I2C interrupt service routine
///
/// @param i2c - this function cannot normally be used as an ISR, so it
/// needs
///        help from a template function, or some other static function to
///        pass it the appropriate port object.
void i2c::interrupt()
{
  master_state state = master_state(m_port_info.reg->STAT);
  auto& data = m_port_info.reg->DAT;
  uint32_t clear_mask = 0;
  uint32_t set_mask = 0;

  switch (state) {
    case master_state::bus_error: {
      m_status = std::errc::io_error;
      set_mask = control::assert_acknowledge | control::stop;
      break;
    }
    case master_state::start_condition:
    case master_state::repeated_start: {
      data = m_address;
      break;
    }
    case master_state::slave_address_write_sent_received_ack: {
      clear_mask = control::start;
      if (m_write_iterator == m_write_end) {
        m_busy = false;
        set_mask = control::stop;
      } else {
        data = std::to_integer<uint32_t>(*m_write_iterator++);
      }
      break;
    }
    case master_state::slave_address_write_sent_received_nack: {
      clear_mask = control::start;
      m_busy = false;
      m_status = std::errc::no_such_device_or_address;
      set_mask = control::stop;
      break;
    }
    case master_state::transmitted_data_received_ack: {
      if (m_write_iterator == m_write_end) {
        if (m_read_iterator != m_read_end) {
          // OR with 1 to set address as READ for the next transaction
          m_address |= 1 << 7;
          set_mask = control::start;
        } else {
          m_busy = false;
          set_mask = control::stop;
        }
      } else {
        data = std::to_integer<uint32_t>(*m_write_iterator++);
      }
      break;
    }
    case master_state::transmitted_data_received_nack: {
      m_busy = false;
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
      m_busy = false;
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
        m_busy = false;
      } else {
        set_mask = control::assert_acknowledge;
      }
      break;
    }
    case master_state::received_data_received_nack: {
      m_busy = false;
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
  m_port_info.reg->CONSET = set_mask;
  m_port_info.reg->CONCLR = clear_mask;
}

i2c::~i2c()
{
  driver_power_down();
}

void i2c::configure_clock_rate()
{
  // Calculating and setting the I2C Clock rate
  // Weight the high side duty cycle more than the lower side by 30% in
  // order to give more time for the bus to charge up.
  const auto frequency = clock(m_port_info.peripheral_id).frequency();
  const uint32_t clock_divider = frequency / settings().clock_rate_hz;
  const uint32_t kScll = clock_divider * settings().duty_cycle;
  const uint32_t kSclh = clock_divider - kScll;

  m_port_info.reg->SCLL = static_cast<uint32_t>(kScll);
  m_port_info.reg->SCLH = static_cast<uint32_t>(kSclh);
}
}
