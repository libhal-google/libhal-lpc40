#pragma once

#include <cstdint>

#include <libhal/input_pin.hpp>

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
   * @tparam port - selects pin port to use
   * @tparam pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   * @return input_pin& - reference to a statically allocated input pin
   */
  template<std::uint8_t port, std::uint8_t pin>
  static result<input_pin&> get(input_pin::settings p_settings = {})
  {
    compile_time_platform_check();
    internal::check_gpio_bounds_at_compile<port, pin>();
    static input_pin gpio(port, pin);
    HAL_CHECK(gpio.configure(p_settings));
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
  input_pin(uint8_t p_port, uint8_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
  }

  status driver_configure(const settings& p_settings) override;
  result<bool> driver_level() override;

  uint8_t m_port{};
  uint8_t m_pin{};
};

inline status input_pin::driver_configure(const settings& p_settings)
{
  // Set direction to input
  bit::modify(internal::gpio_reg(m_port)->direction)
    .clear(internal::pin_mask(m_pin));

  internal::pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(false)
    .resistor(p_settings.resistor);

  return hal::success();
}

inline result<bool> input_pin::driver_level()
{
  return bit::extract(internal::pin_mask(m_pin),
                      internal::gpio_reg(m_port)->pin);
}
}  // namespace hal::lpc40xx
