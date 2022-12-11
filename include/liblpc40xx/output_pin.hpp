#pragma once

#include <cstdint>

#include <libhal/output_pin.hpp>

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
   * @tparam port - selects pin port to use
   * @tparam pin - selects which pin within the port to use
   * @param p_settings - initial pin settings
   * @return result<output_pin&> - reference to the statically allocated output
   * pin
   */
  template<std::uint8_t port, std::uint8_t pin>
  static result<output_pin&> get(output_pin::settings p_settings = {})
  {
    compile_time_platform_check();
    internal::check_gpio_bounds_at_compile<port, pin>();
    static output_pin gpio(port, pin);
    HAL_CHECK(gpio.driver_configure(p_settings));
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
  output_pin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
  }

  status driver_configure(const settings& p_settings) noexcept override;
  status driver_level(bool p_high) noexcept override;
  result<bool> driver_level() noexcept override;

  std::uint8_t m_port{};
  std::uint8_t m_pin{};
};

inline status output_pin::driver_configure(const settings& p_settings) noexcept
{
  bit::modify(internal::gpio_reg(m_port)->direction)
    .set(internal::pin_mask(m_pin));

  internal::pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(p_settings.open_drain)
    .resistor(p_settings.resistor);

  return hal::success();
}

inline status output_pin::driver_level(bool p_high) noexcept
{
  if (p_high) {
    bit::modify(internal::gpio_reg(m_port)->pin).set(internal::pin_mask(m_pin));
  } else {
    bit::modify(internal::gpio_reg(m_port)->pin)
      .clear(internal::pin_mask(m_pin));
  }

  return hal::success();
}

inline result<bool> output_pin::driver_level() noexcept
{
  return bit::extract(internal::pin_mask(m_pin),
                      internal::gpio_reg(m_port)->pin);
}
}  // namespace hal::lpc40xx
