#pragma once

#include "internal/gpio.hpp"
#include "internal/pin.hpp"

#include <array>
#include <cstdint>

#include <libembeddedhal/config.hpp>
#include <libembeddedhal/gpio/output_pin.hpp>

namespace embed::lpc40xx {
/**
 * @brief Output pin implementation for the lpc40xx
 *
 */
class output_pin : public embed::output_pin
{
public:
  /**
   * @brief Construct a new output pin object
   *
   * @param p_port -selects pin port to use
   * @param p_pin -selects pin within the port to use
   * @param p_settings - initial output pin settings
   */
  output_pin(int p_port, int p_pin, const settings& p_settings = {})
    : m_port(p_port)
    , m_pin(p_pin)
  {
    driver_configure(p_settings);
  }

private:
  boost::leaf::result<void> driver_configure(
    const settings& p_settings) noexcept override;
  boost::leaf::result<void> driver_level(bool p_high) noexcept override;
  boost::leaf::result<bool> driver_level() noexcept override;

  int m_port{};
  int m_pin{};
};

/**
 * @brief Get the output pin object
 *
 * @tparam Port - selects pin port to use
 * @tparam Pin - selects which pin within the port to use
 * @param p_settings - initial pin settings to use
 * @return output_pin& - reference to a statically allocated output pin
 */
template<int Port, int Pin>
inline output_pin& get_output_pin(output_pin::settings p_settings = {})
{
  internal::check_gpio_bounds_at_compile<Port, Pin>();
  static output_pin gpio(Port, Pin, p_settings);
  return gpio;
}

inline boost::leaf::result<void> output_pin::driver_configure(
  const settings& p_settings) noexcept
{
  driver_level(p_settings.starting_level);
  xstd::bitmanip(internal::gpio_reg(m_port)->direction).set(m_pin);

  internal::pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(p_settings.open_drain)
    .resistor(p_settings.resistor);

  return {};
}

inline boost::leaf::result<void> output_pin::driver_level(bool p_high) noexcept
{
  if (p_high) {
    xstd::bitmanip(internal::gpio_reg(m_port)->pin).set(m_pin);
  } else {
    xstd::bitmanip(internal::gpio_reg(m_port)->pin).reset(m_pin);
  }

  return {};
}

inline boost::leaf::result<bool> output_pin::driver_level() noexcept
{
  return xstd::bitmanip(internal::gpio_reg(m_port)->pin).test(m_pin);
}
}  // namespace embed::lpc40xx
