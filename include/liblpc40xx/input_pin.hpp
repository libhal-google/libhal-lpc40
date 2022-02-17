#pragma once

#include <array>
#include <cstdint>

#include <libembeddedhal/config.hpp>
#include <libembeddedhal/input_pin/interface.hpp>

#include "internal/gpio.hpp"
#include "internal/pin.hpp"

namespace embed::lpc40xx {
/**
 * @brief Input pin implementation for the lpc40xx
 *
 */
class input_pin : public embed::input_pin
{
public:
  /**
   * @brief Construct a new input pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   */
  input_pin(uint32_t p_port,
            uint32_t p_pin,
            const settings& p_settings = {}) noexcept
    : m_port(p_port)
    , m_pin(p_pin)
  {
    driver_configure(p_settings);
  }

private:
  boost::leaf::result<void> driver_configure(
    const settings& p_settings) noexcept override;
  boost::leaf::result<bool> driver_level() noexcept override;

  int m_port{};
  int m_pin{};
};

/**
 * @brief Get the input pin object
 *
 * @tparam Port - selects pin port to use
 * @tparam Pin - selects pin within the port to use
 * @param p_settings - initial pin settings
 * @return input_pin& - reference to a statically allocated input pin
 */
template<int Port, int Pin>
inline input_pin& get_input_pin(input_pin::settings p_settings = {})
{
  internal::check_gpio_bounds_at_compile<Port, Pin>();
  static input_pin gpio(Port, Pin, p_settings);
  return gpio;
}

inline boost::leaf::result<void> input_pin::driver_configure(
  const settings& p_settings) noexcept
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

inline boost::leaf::result<bool> input_pin::driver_level() noexcept
{
  return xstd::bitmanip(internal::gpio_reg(m_port)->pin).test(m_pin);
}
}  // namespace embed::lpc40xx
