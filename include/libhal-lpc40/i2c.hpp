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

#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal/i2c.hpp>

#include "constants.hpp"
#include "internal/pin.hpp"
#include "system_controller.hpp"

namespace hal::lpc40xx {
class i2c : public hal::i2c
{
public:
  using write_iterator = std::span<const hal::byte>::iterator;
  using read_iterator = std::span<hal::byte>::iterator;

  struct reg_t
  {
    /// Offset: 0x000 i2c control set register (r/w)
    volatile uint32_t control_set;
    /// offset: 0x004 i2c status register (r/ )
    const volatile uint32_t stat;
    /// offset: 0x008 i2c data register (r/w)
    volatile uint32_t dat;
    /// offset: 0x00c i2c peripheral address register 0 (r/w)
    volatile uint32_t address0;
    /// offset: 0x010 sch duty cycle register high half word (r/w)
    volatile uint32_t duty_cycle_high;
    /// offset: 0x014 scl duty cycle register low half word (r/w)
    volatile uint32_t duty_cycle_low;
    /// offset: 0x018 i2c control clear register ( /w)
    volatile uint32_t control_clear;
    /// offset: 0x01c monitor mode control register (r/w)
    volatile uint32_t monitor_mode_control;
    /// offset: 0x020 i2c peripheral address register 1 (r/w)
    volatile uint32_t address1;
    /// offset: 0x024 i2c peripheral address register 2 (r/w)
    volatile uint32_t address2;
    /// offset: 0x028 i2c peripheral address register 3 (r/w)
    volatile uint32_t address3;
    /// offset: 0x02c data buffer register ( /w)
    const volatile uint32_t data_buffer;
    /// offset: 0x030 i2c peripheral address mask register 0 (r/w)
    volatile uint32_t mask0;
    /// offset: 0x034 i2c peripheral address mask register 1 (r/w)
    volatile uint32_t mask1;
    /// offset: 0x038 i2c peripheral address mask register 2 (r/w)
    volatile uint32_t mask2;
    /// offset: 0x03c i2c peripheral address mask register 3 (r/w)
    volatile uint32_t mask3;
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

  /// lpc40xx i2c peripheral state numbers
  enum class host_state : uint32_t
  {
    bus_error = 0x00,
    start_condition = 0x08,
    repeated_start = 0x10,
    peripheral_address_write_sent_received_ack = 0x18,
    peripheral_address_write_sent_received_nack = 0x20,
    transmitted_data_received_ack = 0x28,
    transmitted_data_received_nack = 0x30,
    arbitration_lost = 0x38,
    peripheral_address_read_sent_received_ack = 0x40,
    peripheral_address_read_sent_received_nack = 0x48,
    received_data_received_ack = 0x50,
    received_data_received_nack = 0x58,
    own_address_received = 0xA0,
    do_nothing = 0xF8
  };

  /// port holds all of the information for an i2c bus on the LPC40xx
  /// platform.
  struct bus_info
  {
    /// Holds a pointer to the LPC_i2c peripheral reg
    reg_t* reg;
    /// peripheral id used to power on the i2c peripheral at creation
    peripheral peripheral_id;
    /// IRQ number for this i2c port.
    irq irq_number;
    /// i2c data pin
    internal::pin sda;
    /// sda pin function code
    uint8_t sda_function;
    /// i2c clock pin
    internal::pin scl;
    /// scl pin function code
    uint8_t scl_function;
    /// Clock rate duty cycle
    float duty_cycle = 0.5f;
  };

  static constexpr std::errc no_error{ 0 };

  template<int BusNumber>
  static result<i2c&> get(const i2c::settings& p_settings = {})
  {
    i2c::bus_info bus_info{};

    static_assert(hal::is_a_test() || hal::is_platform("lpc40"),
                  "This driver can only be used with the lpc40 series "
                  "microcontrollers or unit tests!");

    static_assert(0 <= BusNumber && BusNumber <= 2,
                  "Supported i2c busses are 0, 1, and 2!");

    // UM10562: Chapter 7: LPC408x/407x I/O configuration page 13
    if constexpr (BusNumber == 0) {
      /// Definition for i2c bus 0 for LPC40xx.
      bus_info = {
        .reg = reinterpret_cast<i2c::reg_t*>(0x4001'C000),
        .peripheral_id = peripheral::i2c0,
        .irq_number = irq::i2c0,
        .sda = internal::pin(1, 30),
        .sda_function = 0b100,
        .scl = internal::pin(1, 31),
        .scl_function = 0b100,
      };
    } else if constexpr (BusNumber == 1) {
      /// Definition for i2c bus 1 for LPC40xx.
      bus_info = {
        .reg = reinterpret_cast<i2c::reg_t*>(0x4005'C000),
        .peripheral_id = peripheral::i2c1,
        .irq_number = irq::i2c1,
        .sda = internal::pin(0, 0),
        .sda_function = 0b011,
        .scl = internal::pin(0, 1),
        .scl_function = 0b011,
      };
    } else if constexpr (BusNumber == 2) {
      /// Definition for i2c bus 2 for LPC40xx.
      bus_info = {
        .reg = reinterpret_cast<i2c::reg_t*>(0x400A'0000),
        .peripheral_id = peripheral::i2c2,
        .irq_number = irq::i2c2,
        .sda = internal::pin(0, 10),
        .sda_function = 0b010,
        .scl = internal::pin(0, 11),
        .scl_function = 0b010,
      };
    }

    if constexpr (hal::is_a_test()) {
      static reg_t dummy{};
      bus_info.reg = &dummy;
    }

    cortex_m::interrupt::initialize<value(irq::max)>();

    static i2c i2c_bus(bus_info);
    HAL_CHECK(i2c_bus.driver_configure(p_settings));

    return i2c_bus;
  }

  status driver_configure(const settings& p_settings) override;
  result<transaction_t> driver_transaction(
    hal::byte p_address,
    std::span<const hal::byte> p_data_out,
    std::span<hal::byte> p_data_in,
    hal::function_ref<hal::timeout_function> p_timeout) override;
  /**
   * @brief interrupt service routine for the bus
   *
   * Should only be called in unit tests!
   * Should not be called in application code.
   *
   */
  void interrupt();

  /**
   * @brief Disable the i2c peripheral
   *
   */
  void disable();

  ~i2c()
  {
    disable();
  }

private:
  i2c(bus_info p_bus)
    : m_bus(p_bus)
  {
  }

  bus_info m_bus;
  std::errc m_status = no_error;
  hal::byte m_address = hal::byte{ 0x00 };
  std::atomic<bool> m_busy = false;
  write_iterator m_write_iterator;
  write_iterator m_write_end;
  read_iterator m_read_iterator;
  read_iterator m_read_end;
};
}  // namespace hal::lpc40xx

namespace hal::lpc40xx {
inline status i2c::driver_configure(const settings& p_settings)
{
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
  internal::pin(m_bus.sda)
    .function(m_bus.sda_function)
    .resistor(pin_resistor::none)
    .open_drain(true);

  internal::pin(m_bus.scl)
    .function(m_bus.scl_function)
    .resistor(pin_resistor::none)
    .open_drain(true);

  using high_t =
    std::remove_volatile<decltype(m_bus.reg->duty_cycle_high)>::type;
  using low_t = std::remove_volatile<decltype(m_bus.reg->duty_cycle_low)>::type;

  m_bus.reg->duty_cycle_high = static_cast<high_t>(high_side_clocks);
  m_bus.reg->duty_cycle_low = static_cast<low_t>(low_side_clocks);

  // Clear all transmission flags
  m_bus.reg->control_clear = control::assert_acknowledge | control::start |
                             control::stop | control::interrupt;
  // Enable i2c interface
  m_bus.reg->control_set = control::interface_enable;

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

  return hal::success();
}

inline void i2c::disable()
{
  // Disable i2c interface
  m_bus.reg->control_clear = control::interface_enable;

  // Enable interrupt service routine.
  cortex_m::interrupt(static_cast<int>(m_bus.irq_number)).disable();
}

inline result<i2c::transaction_t> i2c::driver_transaction(
  hal::byte p_address,
  std::span<const hal::byte> p_data_out,
  std::span<hal::byte> p_data_in,
  hal::function_ref<hal::timeout_function> p_timeout)
{
  m_status = no_error;
  m_address = p_address;
  m_write_iterator = p_data_out.begin();
  m_write_end = p_data_out.end();
  m_read_iterator = p_data_in.begin();
  m_read_end = p_data_in.end();
  m_busy = true;

  // Start the transaction
  m_bus.reg->control_set = control::start;

  // i2c::interrupt() will set this to false when the transaction has finished.
  while (m_busy) {
    HAL_CHECK(p_timeout());
  }

  if (m_status != no_error) {
    return hal::new_error(m_status);
  }

  return transaction_t{};
}

inline void i2c::interrupt()
{
  host_state state = host_state(m_bus.reg->stat);
  auto& data = m_bus.reg->dat;
  uint32_t clear_mask = 0;
  uint32_t set_mask = 0;
  bool transaction_finished = false;

  switch (state) {
    case host_state::bus_error: {
      m_status = std::errc::io_error;
      set_mask = control::assert_acknowledge | control::stop;
      break;
    }
    case host_state::start_condition:
    case host_state::repeated_start: {
      if (m_write_iterator != m_write_end) {
        data = to_8_bit_address(m_address, i2c_operation::write);
      } else {
        data = to_8_bit_address(m_address, i2c_operation::read);
      }
      break;
    }
    case host_state::peripheral_address_write_sent_received_ack: {
      clear_mask = control::start;
      if (m_write_iterator == m_write_end) {
        transaction_finished = true;
        set_mask = control::stop;
      } else {
        data = *m_write_iterator++;
      }
      break;
    }
    case host_state::peripheral_address_write_sent_received_nack: {
      clear_mask = control::start;
      transaction_finished = true;
      m_status = std::errc::no_such_device_or_address;
      set_mask = control::stop;
      break;
    }
    case host_state::transmitted_data_received_ack: {
      if (m_write_iterator == m_write_end) {
        if (m_read_iterator != m_read_end) {
          set_mask = control::start;
        } else {
          transaction_finished = true;
          set_mask = control::stop;
        }
      } else {
        data = *(m_write_iterator++);
      }
      break;
    }
    case host_state::transmitted_data_received_nack: {
      transaction_finished = true;
      set_mask = control::stop;
      break;
    }
    case host_state::arbitration_lost: {
      set_mask = control::start;
      break;
    }
    case host_state::peripheral_address_read_sent_received_ack: {
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
    case host_state::peripheral_address_read_sent_received_nack: {
      clear_mask = control::start;
      m_status = std::errc::no_such_device_or_address;
      transaction_finished = true;
      set_mask = control::stop;
      break;
    }
    case host_state::received_data_received_ack: {
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<hal::byte>(data);
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
    case host_state::received_data_received_nack: {
      transaction_finished = true;
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<hal::byte>(data);
      }
      set_mask = control::stop;
      break;
    }
    case host_state::do_nothing: {
      break;
    }
    default: {
      clear_mask = control::stop;
      break;
    }
  }

  clear_mask |= control::interrupt;

  m_bus.reg->control_set = set_mask;
  m_bus.reg->control_clear = clear_mask;

  if (transaction_finished) {
    m_busy = false;
  }
}
}  // namespace hal::lpc40xx
