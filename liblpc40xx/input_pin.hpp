#pragma once

#include "internal/gpio.hpp"
#include "internal/pin.hpp"

#include <cinttypes>
#include <libembeddedhal/context.hpp>

namespace embed::lpc40xx {
class input_pin : public embed::input_pin
{
public:
  input_pin(uint32_t p_port, uint32_t p_pin);
  bool driver_initialize() override;
  bool level() const override;

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
