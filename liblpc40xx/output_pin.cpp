
#include "internal/gpio.hpp"
#include "internal/pin.hpp"
#include "output_pin.hpp"

#include <array>
#include <cinttypes>
#include <libembeddedhal/context.hpp>

namespace embed::lpc40xx {
output_pin::output_pin(uint32_t p_port, uint32_t p_pin)
  : m_port(p_port)
  , m_pin(p_pin)
{
  if constexpr (!is_platform("lpc40")) {
    unittest_gpio();
  }
}

bool output_pin::driver_initialize()
{
  level(settings().starting_level);
  xstd::bitmanip(gpio_port[m_port]->DIR).set(m_pin);

  pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(settings().open_drain)
    .resistor(settings().resistor);

  return true;
}

void output_pin::level(bool p_high)
{
  if (p_high) {
    xstd::bitmanip(gpio_port[m_port]->PIN).set(m_pin);
  } else {
    xstd::bitmanip(gpio_port[m_port]->PIN).reset(m_pin);
  }
}

bool output_pin::level() const
{
  return xstd::bitmanip(gpio_port[m_port]->PIN).test(m_pin);
}
}
