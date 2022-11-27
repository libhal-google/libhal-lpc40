#pragma once

#include <array>
#include <cstdint>

#include <libhal/config.hpp>
#include <libhal/output_pin/interface.hpp>

#include "internal/gpio.hpp"
#include "internal/pin.hpp"
#include "internal/platform_check.hpp"

namespace hal::lpc40xx {
/**
 * @brief Output pin implementation for the lpc40xx
 *
 */
class output_pin : public hal::output_pin
{
public:
  /**
   * @brief Get the output pin object
   *
   * @tparam Port - selects pin port to use
   * @tparam Pin - selects which pin within the port to use
   * @param p_settings - initial pin settings
   * @return output_pin& - reference to a statically allocated output pin
   */
  template<int Port, int Pin>
  static output_pin& get(output_pin::settings p_settings = {})
  {
    compile_time_platform_check();
    internal::check_gpio_bounds_at_compile<Port, Pin>();
    static output_pin gpio(Port, Pin, p_settings);
    return gpio;
  }

private:
  /**
   * @brief Construct a new output pin object
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   */
  output_pin(std::uint8_t p_port,
             std::uint8_t p_pin,
             const settings& p_settings = {})
    : m_port(p_port)
    , m_pin(p_pin)
  {
    driver_configure(p_settings);
  }

  status driver_configure(const settings& p_settings) noexcept override;
  status driver_level(bool p_high) noexcept override;
  result<bool> driver_level() noexcept override;

  std::uint8_t m_port{};
  std::uint8_t m_pin{};
};

inline status output_pin::driver_configure(const settings& p_settings) noexcept
{
  xstd::bitmanip(internal::gpio_reg(m_port)->direction).set(m_pin);

  internal::pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(p_settings.open_drain)
    .resistor(p_settings.resistor);

  return {};
}

inline status output_pin::driver_level(bool p_high) noexcept
{
  if (p_high) {
    xstd::bitmanip(internal::gpio_reg(m_port)->pin).set(m_pin);
  } else {
    xstd::bitmanip(internal::gpio_reg(m_port)->pin).reset(m_pin);
  }

  return {};
}

inline result<bool> output_pin::driver_level() noexcept
{
  return xstd::bitmanip(internal::gpio_reg(m_port)->pin).test(m_pin);
}
}  // namespace hal::lpc40xx
