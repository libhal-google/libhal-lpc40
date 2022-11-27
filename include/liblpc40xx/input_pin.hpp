#pragma once

#include <array>
#include <cstdint>

#include <libhal/config.hpp>
#include <libhal/input_pin/interface.hpp>

#include "internal/gpio.hpp"
#include "internal/pin.hpp"

namespace hal::lpc40xx {
/**
 * @brief Input pin implementation for the lpc40xx
 *
 */
class input_pin : public hal::input_pin
{
public:
  /**
   * @brief Get the input pin object
   *
   * @tparam Port - selects pin port to use
   * @tparam Pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   * @return input_pin& - reference to a statically allocated input pin
   */
  template<uint8_t Port, uint8_t Pin>
  static input_pin& get(input_pin::settings p_settings = {})
  {
    compile_time_platform_check();
    internal::check_gpio_bounds_at_compile<Port, Pin>();
    static input_pin gpio(Port, Pin, p_settings);
    return gpio;
  }

private:
  /**
   * @brief Construct a new input pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   */
  input_pin(uint8_t p_port,
            uint8_t p_pin,
            const settings& p_settings = {}) noexcept
    : m_port(p_port)
    , m_pin(p_pin)
  {
    driver_configure(p_settings);
  }

  status driver_configure(const settings& p_settings) noexcept override;
  result<bool> driver_level() noexcept override;

  uint8_t m_port{};
  uint8_t m_pin{};
};

inline status input_pin::driver_configure(const settings& p_settings) noexcept
{
  // Set direction to input
  xstd::bitmanip(internal::gpio_reg(m_port)->direction).reset(m_pin);

  internal::pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(false)
    .resistor(p_settings.resistor);

  return {};
}

inline result<bool> input_pin::driver_level() noexcept
{
  return xstd::bitmanip(internal::gpio_reg(m_port)->pin).test(m_pin);
}
}  // namespace hal::lpc40xx
