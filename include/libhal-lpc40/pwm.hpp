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

#include <libhal/pwm.hpp>

#include "constants.hpp"
#include "internal/pin.hpp"
#include "internal/platform_check.hpp"
#include "system_controller.hpp"

namespace hal::lpc40xx {
/**
 * @brief pwm driver for the lpc40xx series of micro controllers
 *
 * This driver uses the dedicated PWM peripherals, PWM0, and PWM1 for generating
 * pwm signals. Other methods would include using timers and the MotorPWM
 * peripherals.
 *
 * NOTE: Channels within a PWM peripheral device are NOT independent. Meaning
 * that changing the frequency of one channel changes the frequency for all
 * channels within the peripheral block.
 *
 */
class pwm : public hal::pwm
{
public:
  /**
   * @brief Register map for the lpc40xx PWM peripheral
   *
   */
  struct reg_t
  {
    /// Offset: 0x000 Interrupt Register (R/W)
    volatile std::uint32_t interrupt_register;
    /// Offset: 0x004 Timer Control Register (R/W)
    volatile std::uint32_t timer_control_register;
    /// Offset: 0x008 Timer Counter Register (R/W)
    volatile std::uint32_t timer_counter_register;
    /// Offset: 0x00C Prescale Register (R/W)
    volatile std::uint32_t prescale_register;
    /// Offset: 0x010 Prescale Counter Register (R/W)
    volatile std::uint32_t prescale_counter_register;
    /// Offset: 0x014 Match Control Register (R/W)
    volatile std::uint32_t match_control_register;
    /// Offset: 0x018 Match Register 0 (R/W)
    volatile std::uint32_t match_register_0;
    /// Offset: 0x01C Match Register 1 (R/W)
    volatile std::uint32_t match_register_1;
    /// Offset: 0x020 Match Register 2 (R/W)
    volatile std::uint32_t match_register_2;
    /// Offset: 0x024 Match Register 3 (R/W)
    volatile std::uint32_t match_register_3;
    /// Offset: 0x028 Capture Control Register (R/W)
    volatile std::uint32_t capture_control_register;
    /// Offset: 0x02C Capture Register 0 (R/ )
    const volatile std::uint32_t capture_register_0;
    /// Offset: 0x030 Capture Register 1 (R/ )
    const volatile std::uint32_t capture_register_1;
    /// Offset: 0x034 Capture Register 2 (R/ )
    const volatile std::uint32_t capture_register_2;
    /// Offset: 0x038 Capture Register 3 (R/ )
    const volatile std::uint32_t capture_register_3;
    uint32_t reserved0;
    /// Offset: 0x040 Match Register 4 (R/W)
    volatile std::uint32_t match_register_4;
    /// Offset: 0x044 Match Register 5 (R/W)
    volatile std::uint32_t match_register_5;
    /// Offset: 0x048 Match Register 6 (R/W)
    volatile std::uint32_t match_register_6;
    /// Offset: 0x04C PWM Control Register (R/W)
    volatile std::uint32_t pwm_control_register;
    /// Offset: 0x050 Load Enable Register (R/W)
    volatile std::uint32_t load_enable_register;
    std::uint32_t reserved1[7];
    /// Offset: 0x070 Counter Control Register (R/W)
    volatile std::uint32_t counter_control_register;
  };

  /// Channel specific information
  struct channel
  {
    /// Address of the register map
    reg_t* reg;
    /// peripheral id used to power on the i2c peripheral at creation
    peripheral peripheral_id;
    /// Pin to output pwm from
    internal::pin pin;
    /// Channel index
    uint8_t index;
    /// Pin function code
    uint8_t pin_function;
  };

  /**
   * @brief Get a pwm driver.
   *
   * @tparam Peripheral - Peripheral block, either 0 or 1
   * @tparam Channel - PWM output channel within the peripheral block, from 1
   * to 6.
   * @return result<pwm&> - reference to the pwm driver
   */
  template<size_t Peripheral, size_t Channel>
  [[nodiscard]] static result<pwm&> get()
  {
    static_assert(hal::is_a_test() || hal::is_platform("lpc40"),
                  "This driver can only be used with the lpc40 series "
                  "microcontrollers or unit tests!");
    static_assert(Peripheral <= 1,
                  "LPC40 series microcontrollers only have PWM0 and PWM1.");
    static_assert(1 <= Channel && Channel <= 6,
                  "LPC40 series microcontrollers only have channels 1 to 6.");

    channel new_channel;

    new_channel.index = Channel;

    if constexpr (hal::is_a_test()) {
      static reg_t dummy{};
      new_channel.reg = &dummy;
    }

    if constexpr (Peripheral == 0) {
      new_channel.reg = reinterpret_cast<reg_t*>(0x4001'4000);
      new_channel.peripheral_id = peripheral::pwm0;
      new_channel.pin = internal::pin(3, 16 + (Channel - 1));
      new_channel.pin_function = 0b010;
    } else if (Peripheral == 1) {
      new_channel.reg = reinterpret_cast<reg_t*>(0x4001'8000);
      new_channel.peripheral_id = peripheral::pwm1;
      new_channel.pin = internal::pin(2, 0 + (Channel - 1));
      new_channel.pin_function = 0b001;
    }

    static pwm pwm_object(new_channel);
    pwm_object.setup();

    return pwm_object;
  }

private:
  pwm(channel p_channel)
    : m_channel(p_channel)
  {
  }

  void enable(bool p_enable = true);
  void setup();
  [[nodiscard]] uint32_t calculate_duty_cycle(float p_percent) const;
  [[nodiscard]] float get_duty_cycle() const;
  [[nodiscard]] volatile uint32_t& get_match_registers(uint8_t p_match) const;
  result<frequency_t> driver_frequency(hertz p_frequency) override;
  result<duty_cycle_t> driver_duty_cycle(float p_duty_cycle) override;

  channel m_channel;
};

inline void pwm::setup()
{
  /// Controls the counting mode of the PWM peripheral. When set to 0, counts
  /// using the internal prescale counter which is driven by the peripheral
  /// clock. Other modes involve use of an external clock source.
  static constexpr auto mode = bit::mask::from<0, 1>();

  /// If set to a 1, tells the PWM hardware to reset the PWM total count
  /// register to be reset to 0 when it is equal to the match register 0.
  static constexpr auto pwm0_reset = bit::mask::from<1>();

  power(m_channel.peripheral_id).on();

  // Set pre-scalar to 1 so the input frequency to the PWM peripheral is
  // equal to the peripheral clock frequency.
  m_channel.reg->prescale_register = 0;

  // Set to 0 to cause the timer counter to increment after each peripheral
  // clock tick.
  m_channel.reg->prescale_counter_register = 0;

  bit::modify(m_channel.reg->counter_control_register).insert<mode>(0x0U);
  bit::modify(m_channel.reg->match_control_register).set<pwm0_reset>();

  // Enable this pwm channel
  bit::modify(m_channel.reg->pwm_control_register)
    .set(bit::mask::from(8U + m_channel.index));

  m_channel.pin.function(m_channel.pin_function);

  // Set duty cycle to zero
  get_match_registers(m_channel.index) = 0;

  // Enable the PWM output channel
  enable();
}

inline void pwm::enable(bool p_enable)
{
  // When set to a 1, enables the TC (total count) register and begins
  // counting.
  static constexpr auto counter_enable = bit::mask::from<0>();

  // When set to a 1, will reset the total count register.
  static constexpr auto counter_reset = bit::mask::from<1>();

  // Enables PWM mode. Without setting the match registers cannot operate
  // with the timer.
  static constexpr auto pwm_enable = bit::mask::from<3>();

  if (p_enable) {
    // Reset the Timer Counter
    bit::modify(m_channel.reg->timer_control_register).set(counter_reset);
    // Clear reset and allow timer to count
    bit::modify(m_channel.reg->timer_control_register).clear(counter_reset);
    // Enable PWM output
    bit::modify(m_channel.reg->timer_control_register).set(pwm_enable);
    // Enable counting
    bit::modify(m_channel.reg->timer_control_register).set(counter_enable);
  } else {
    // Disable PWM output
    bit::modify(m_channel.reg->timer_control_register).clear(pwm_enable);
  }
}

inline result<pwm::frequency_t> pwm::driver_frequency(hertz p_frequency)
{
  const auto input_clock = clock::get().get_frequency(m_channel.peripheral_id);

  if (p_frequency >= input_clock) {
    return new_error(std::errc::invalid_argument);
  }

  // Get the current duty cycle so we can match it to the updated frequency.
  float previous_duty_cycle = get_duty_cycle();

  // In order to avoid PWM glitches, the PWM must be disabled while updating
  // the MR0 register. Doing this will reset all counters to 0 and allow us to
  // update MR0.
  enable(false);

  // Set frequency by setting match register 0 (the reset register) to the
  // counts required to reach the desired frequency.
  const auto new_frequency = input_clock / p_frequency;

  get_match_registers(0) = static_cast<std::uint32_t>(new_frequency);

  // Re-enable PWM which will also reset all of the counters which allow
  // setting the duty cycle of other PWM channels.
  enable(true);

  // Update the duty cycle based on the previous percentage
  HAL_CHECK(duty_cycle(previous_duty_cycle));

  return frequency_t{};
}

inline float pwm::get_duty_cycle() const
{
  auto period = static_cast<float>(get_match_registers(m_channel.index));
  auto max_period = static_cast<float>(get_match_registers(0));
  return period / max_period;
}

/// Helper method to make getting a pointer to the Match Register 0 more
/// readable in the code.
///
/// @return a pointer to the match 0 register.
inline volatile uint32_t& pwm::get_match_registers(uint8_t p_match) const
{
  switch (p_match) {
    case 1:
      return m_channel.reg->match_register_1;
    case 2:
      return m_channel.reg->match_register_2;
    case 3:
      return m_channel.reg->match_register_3;
    case 4:
      return m_channel.reg->match_register_4;
    case 5:
      return m_channel.reg->match_register_5;
    case 6:
      return m_channel.reg->match_register_6;
    case 0:
    default:
      return m_channel.reg->match_register_0;
  }
}

[[nodiscard]] inline uint32_t pwm::calculate_duty_cycle(float p_percent) const
{
  float pwm_period = static_cast<float>(get_match_registers(0));
  return static_cast<uint32_t>(p_percent * pwm_period);
}

inline result<pwm::duty_cycle_t> pwm::driver_duty_cycle(float p_duty_cycle)
{
  // Set match register for this channel
  get_match_registers(m_channel.index) = calculate_duty_cycle(p_duty_cycle);

  // Setup pwm peripheral to update the duty cycle on the next reset cycle after
  // a match_register_0 match occurs. This way the PWM duty cycle does ont
  // change instantaneously.
  bit::modify(m_channel.reg->load_enable_register)
    .set(bit::mask::from(m_channel.index));

  return duty_cycle_t{};
}
}  // namespace hal::lpc40xx
