#pragma once

#include "internal/gpio.hpp"
#include "internal/pin.hpp"

#include <array>
#include <cinttypes>

#include <libembeddedhal/context.hpp>
#include <libembeddedhal/gpio/gpio.hpp>

namespace embed::lpc40xx {
class input_pin : public embed::input_pin
{
public:
  constexpr input_pin(uint32_t p_port, uint32_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
    if constexpr (!is_platform("lpc40")) {
      internal::unittest_gpio();
    }
  }

  bool driver_initialize() override
  {
    // Set direction to input
    xstd::bitmanip(internal::gpio_port[m_port]->DIR).reset(m_pin);

    internal::pin(m_port, m_pin)
      .function(0)
      .dac(false)
      .analog(false)
      .open_drain(false)
      .resistor(settings().resistor);

    return true;
  }

  bool level() const override
  {
    return xstd::bitmanip(internal::gpio_port[m_port]->PIN).test(m_pin);
  }

private:
  uint32_t m_port;
  uint32_t m_pin;
};

template<unsigned port, unsigned pin>
inline input_pin& get_input_pin()
{
  static_assert(
    (port <= 4 && pin <= 31) || (port == 5 && pin < 4),
    "For ports between 0 and 4, the pin number must be between 0 and 31. For "
    "port 5, the pin number must be equal to or below 4");

  static input_pin gpio(port, pin);
  return gpio;
}
}
