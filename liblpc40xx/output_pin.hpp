#pragma once

#include <cinttypes>
#include <libembeddedhal/gpio.hpp>

namespace embed::lpc40xx {
class output_pin : public embed::output_pin
{
public:
  output_pin(uint32_t p_port, uint32_t p_pin);
  bool driver_initialize() override;
  void level(bool p_high) override;
  bool level() const override;

private:
  uint32_t m_port;
  uint32_t m_pin;
};

template<unsigned port, unsigned pin>
inline output_pin& get_output_pin()
{
  static_assert(
    (port <= 4 && pin <= 31) || (port == 5 && pin < 4),
    "For ports between 0 and 4, the pin number must be between 0 and 31. For "
    "port 5, the pin number must be equal to or below 4");

  static output_pin gpio(port, pin);
  return gpio;
}
}
