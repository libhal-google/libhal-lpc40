// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdint>

#include <libhal-lpc40/i2c.hpp>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/static_callable.hpp>

#include "i2c_reg.hpp"

namespace hal::lpc40 {
i2c_reg_t* get_i2c_reg(peripheral p_id)
{
  switch (p_id) {
    case peripheral::i2c0:
      return i2c_reg0;
    case peripheral::i2c1:
      return i2c_reg1;
    case peripheral::i2c2:
    default:
      return i2c_reg2;
  }
}

void disable(i2c::bus_info& p_info)
{
  auto* reg = get_i2c_reg(p_info.peripheral_id);

  // Disable i2c interface
  reg->control_clear = i2c_control::interface_enable;

  // Enable interrupt service routine.
  cortex_m::interrupt(static_cast<int>(p_info.irq_number)).disable();
}

void i2c::interrupt()
{
  auto* reg = get_i2c_reg(m_bus.peripheral_id);

  i2c_host_state state = i2c_host_state(reg->stat);
  auto& data = reg->dat;
  uint32_t clear_mask = 0;
  uint32_t set_mask = 0;
  bool transaction_finished = false;

  switch (state) {
    case i2c_host_state::bus_error: {
      m_status = std::errc::io_error;
      set_mask = i2c_control::assert_acknowledge | i2c_control::stop;
      break;
    }
    case i2c_host_state::start_condition:
    case i2c_host_state::repeated_start: {
      if (m_write_iterator != m_write_end) {
        data = to_8_bit_address(m_address, i2c_operation::write);
      } else {
        data = to_8_bit_address(m_address, i2c_operation::read);
      }
      break;
    }
    case i2c_host_state::peripheral_address_write_sent_received_ack: {
      clear_mask = i2c_control::start;
      if (m_write_iterator == m_write_end) {
        transaction_finished = true;
        set_mask = i2c_control::stop;
      } else {
        data = *m_write_iterator++;
      }
      break;
    }
    case i2c_host_state::peripheral_address_write_sent_received_nack: {
      clear_mask = i2c_control::start;
      transaction_finished = true;
      m_status = std::errc::no_such_device_or_address;
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::transmitted_data_received_ack: {
      if (m_write_iterator == m_write_end) {
        if (m_read_iterator != m_read_end) {
          set_mask = i2c_control::start;
        } else {
          transaction_finished = true;
          set_mask = i2c_control::stop;
        }
      } else {
        data = *(m_write_iterator++);
      }
      break;
    }
    case i2c_host_state::transmitted_data_received_nack: {
      transaction_finished = true;
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::arbitration_lost: {
      set_mask = i2c_control::start;
      break;
    }
    case i2c_host_state::peripheral_address_read_sent_received_ack: {
      clear_mask = i2c_control::start;
      if (m_read_iterator == m_read_end) {
        set_mask = i2c_control::stop;
      }
      // If we only want 1 byte, make sure to nack that byte
      else if (m_read_iterator + 1 == m_read_end) {
        clear_mask |= i2c_control::assert_acknowledge;
      }
      // If we want more then 1 byte, make sure to ack the first byte
      else {
        set_mask = i2c_control::assert_acknowledge;
      }
      break;
    }
    case i2c_host_state::peripheral_address_read_sent_received_nack: {
      clear_mask = i2c_control::start;
      m_status = std::errc::no_such_device_or_address;
      transaction_finished = true;
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::received_data_received_ack: {
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<hal::byte>(data);
      }
      // Check if the buffer has been exhausted
      if (m_read_iterator == m_read_end) {
        clear_mask = i2c_control::assert_acknowledge;
        transaction_finished = true;
      } else {
        set_mask = i2c_control::assert_acknowledge;
      }
      break;
    }
    case i2c_host_state::received_data_received_nack: {
      transaction_finished = true;
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<hal::byte>(data);
      }
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::do_nothing: {
      break;
    }
    default: {
      clear_mask = i2c_control::stop;
      break;
    }
  }

  clear_mask |= i2c_control::interrupt;

  reg->control_set = set_mask;
  reg->control_clear = clear_mask;

  if (transaction_finished) {
    m_busy = false;
  }
}

result<i2c> i2c::get(std::uint8_t p_bus_number, const i2c::settings& p_settings)
{
  i2c::bus_info bus_info{};

  if (p_bus_number <= 2) {
    // "Supported i2c busses are 0, 1, and 2!";
  }

  // UM10562: Chapter 7: LPC408x/407x I/O configuration page 13
  if (p_bus_number == 0) {
    /// Definition for i2c bus 0 for LPC40xx.
    bus_info = {
      .peripheral_id = peripheral::i2c0,
      .irq_number = irq::i2c0,
      .sda = pin(1, 30),
      .sda_function = 0b100,
      .scl = pin(1, 31),
      .scl_function = 0b100,
    };
  } else if (p_bus_number == 1) {
    /// Definition for i2c bus 1 for LPC40xx.
    bus_info = {
      .peripheral_id = peripheral::i2c1,
      .irq_number = irq::i2c1,
      .sda = pin(0, 0),
      .sda_function = 0b011,
      .scl = pin(0, 1),
      .scl_function = 0b011,
    };
  } else if (p_bus_number == 2) {
    /// Definition for i2c bus 2 for LPC40xx.
    bus_info = {
      .peripheral_id = peripheral::i2c2,
      .irq_number = irq::i2c2,
      .sda = pin(0, 10),
      .sda_function = 0b010,
      .scl = pin(0, 11),
      .scl_function = 0b010,
    };
  }

  cortex_m::interrupt::initialize<value(irq::max)>();

  i2c i2c_bus(bus_info);
  HAL_CHECK(i2c_bus.driver_configure(p_settings));

  return i2c_bus;
}

i2c::i2c(i2c&& p_other)
{
  p_other.m_moved = true;
  setup_interrupt();
}

i2c& i2c::operator=(i2c&& p_other)
{
  p_other.m_moved = true;
  setup_interrupt();
  return *this;
}

i2c::~i2c()
{
  if (!m_moved) {
    disable(m_bus);
  }
}

i2c::i2c(bus_info p_bus)
  : m_bus(p_bus)
{
}

status i2c::driver_configure(const settings& p_settings)
{
  auto* reg = get_i2c_reg(m_bus.peripheral_id);

  // Setup i2c operating frequency
  const auto input_clock = clock::get().get_frequency(m_bus.peripheral_id);
  const auto clock_divider = input_clock / p_settings.clock_rate;
  const auto high_side_clocks = clock_divider * m_bus.duty_cycle;
  const auto low_side_clocks = clock_divider - high_side_clocks;

  if (low_side_clocks < 1.0f || high_side_clocks < 1.0f) {
    return hal::new_error(std::errc::result_out_of_range);
  }

  // Power on peripheral
  power(m_bus.peripheral_id).on();

  // Setup pins for SDA and SCL
  pin(m_bus.sda)
    .function(m_bus.sda_function)
    .resistor(pin_resistor::none)
    .open_drain(true);

  pin(m_bus.scl)
    .function(m_bus.scl_function)
    .resistor(pin_resistor::none)
    .open_drain(true);

  using high_t = std::remove_volatile<decltype(reg->duty_cycle_high)>::type;
  using low_t = std::remove_volatile<decltype(reg->duty_cycle_low)>::type;

  reg->duty_cycle_high = static_cast<high_t>(high_side_clocks);
  reg->duty_cycle_low = static_cast<low_t>(low_side_clocks);

  // Clear all transmission flags
  reg->control_clear = i2c_control::assert_acknowledge | i2c_control::start |
                       i2c_control::stop | i2c_control::interrupt;
  // Enable i2c interface
  reg->control_set = i2c_control::interface_enable;

  setup_interrupt();

  return hal::success();
}

void i2c::setup_interrupt()
{
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
  cortex_m::interrupt(hal::value(m_bus.irq_number)).enable(handler);
}

result<i2c::transaction_t> i2c::driver_transaction(
  hal::byte p_address,
  std::span<const hal::byte> p_data_out,
  std::span<hal::byte> p_data_in,
  hal::function_ref<hal::timeout_function> p_timeout)
{
  auto* reg = get_i2c_reg(m_bus.peripheral_id);

  m_status = std::errc{};
  m_address = p_address;
  m_write_iterator = p_data_out.begin();
  m_write_end = p_data_out.end();
  m_read_iterator = p_data_in.begin();
  m_read_end = p_data_in.end();
  m_busy = true;

  // Start the transaction
  reg->control_set = i2c_control::start;

  // i2c::interrupt() will set this to false when the transaction has finished.
  while (m_busy) {
    HAL_CHECK(p_timeout());
  }

  if (m_status != std::errc{}) {
    return hal::new_error(m_status);
  }

  return transaction_t{};
}
}  // namespace hal::lpc40
