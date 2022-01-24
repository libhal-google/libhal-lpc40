#pragma once

#include "internal/gpio.hpp"
#include "internal/pin.hpp"

#include <array>
#include <cstdint>

#include <libembeddedhal/config.hpp>
#include <libembeddedhal/gpio/input_pin.hpp>

namespace embed::lpc40xx {
class input_pin : public embed::input_pin
{
public:
  constexpr input_pin(uint32_t p_port,
                      uint32_t p_pin,
                      const settings& p_settings = {}) noexcept
    : m_port(p_port)
    , m_pin(p_pin)
  {
    driver_configure(p_settings);
  }

  boost::leaf::result<void> driver_configure(
    const settings& p_settings) noexcept override
  {
    // Set direction to input
    xstd::bitmanip(internal::get_gpio_reg(m_port)->DIR).reset(m_pin);

    internal::pin(m_port, m_pin)
      .function(0)
      .dac(false)
      .analog(false)
      .open_drain(false)
      .resistor(p_settings.resistor);

    return {};
  }

  boost::leaf::result<bool> driver_level() noexcept override
  {
    return xstd::bitmanip(internal::get_gpio_reg(m_port)->PIN).test(m_pin);
  }

private:
  int m_port{};
  int m_pin{};
};

template<int Port, int Pin>
inline input_pin& get_input_pin(input_pin::settings p_settings = {})
{
  internal::check_gpio_bounds_at_compile<Port, Pin>();
  static input_pin gpio(Port, Pin, p_settings);
  return gpio;
}
}
