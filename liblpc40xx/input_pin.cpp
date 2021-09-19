
#include "input_pin.hpp"
#include "internal/gpio.hpp"
#include "internal/pin.hpp"

#include <array>
#include <cinttypes>
#include <libembeddedhal/context.hpp>
#include <libembeddedhal/gpio.hpp>

namespace embed::lpc40xx {
input_pin::input_pin(uint32_t p_port, uint32_t p_pin)
  : m_port(p_port)
  , m_pin(p_pin)
{
  if constexpr (!is_platform("lpc40")) {
    unittest_gpio();
  }
}

bool input_pin::driver_initialize()
{
  // Set direction to input
  xstd::bitmanip(gpio_port[m_port]->DIR).reset(m_pin);

  pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(false)
    .resistor(settings().resistor);

  return true;
}

bool input_pin::level() const
{
  return xstd::bitmanip(gpio_port[m_port]->PIN).test(m_pin);
}
}