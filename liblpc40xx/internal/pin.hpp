#pragma once

#include <cinttypes>
#include <cstring>
#include <libembeddedhal/context.hpp>
#include <libxbitset/bitset.hpp>
#include <libembeddedhal/gpio.hpp>

namespace embed::lpc40xx {
class pin
{
public:
  /// Pin map table for maping pins and ports to registers.
  struct pin_map_t
  {
    /// Register matrix that maps against the 6 ports and the 32 pins per port
    volatile uint32_t matrix[6][32];
  };

  static constexpr intptr_t iocon_address = 0x40000000UL + 0x2C000;
  inline static auto* map = reinterpret_cast<pin_map_t*>(iocon_address);

  // Source: "UM10562 LPC408x/407x User manual" table 83 page 132
  /// Bitmask for setting pin mux function code.
  static constexpr auto range_function = xstd::bitrange::from<0, 2>();

  /// Bitmask for setting resistor mode of pin.
  static constexpr auto range_resistor = xstd::bitrange::from<3, 4>();

  /// Bitmask for setting pin hysteresis mode.
  static constexpr auto range_hysteresis = xstd::bitrange::from<5>();

  /// Bitmask for setting inputs as active low or active high. This will behave
  /// badly if the pin is set to a mode that is an output or set to analog. See
  /// usermanual for more details.
  static constexpr auto range_input_invert = xstd::bitrange::from<6>();

  /// Bitmask for setting a pin to analog mode.
  static constexpr auto range_analog_digital_mode = xstd::bitrange::from<7>();

  /// Bitmask for enabling/disabling digital filter. This can be used to
  /// ignore/reject noise, reflections, or signal bounce (from something like a
  /// switch).
  static constexpr auto range_digital_filter = xstd::bitrange::from<8>();

  /// Bitmask to enable/disable high speed I2C mode
  static constexpr auto range_i2c_highspeed = xstd::bitrange::from<8>();

  /// Bitmask to change the slew rate of signal transitions for outputs for a
  /// pin.
  static constexpr auto range_slew = xstd::bitrange::from<9>();

  /// Bitmask to enable I2C high current drain. This can allow for even faster
  /// I2C communications, as well as allow for more devices on the bus.
  static constexpr auto range_i2c_high_current = xstd::bitrange::from<9>();

  /// Bitmask to enable/disable open drain mode.
  static constexpr auto range_open_drain = xstd::bitrange::from<10>();

  /// Bitmask for enabling/disabling digital to analog pin mode.
  static constexpr auto range_dac_enable = xstd::bitrange::from<16>();

  static void setup_for_unittesting()
  {
    static pin_map_t dummy{};
    map = &dummy;
  }

  constexpr pin(uint32_t p_port, uint32_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
    if constexpr (!is_platform("lpc40")) {
      setup_for_unittesting();
    }
  }

  pin& function(uint8_t p_function_code)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_function>(p_function_code);
    return *this;
  }

  pin& resistor(embed::pin_resistor p_resistor)
  {
    // The pin resistor enumeration matches the values for the LPC40xx so simply
    // cast the enum to an int and this will work.
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_resistor>(static_cast<uint32_t>(p_resistor));
    return *this;
  }

  pin& hysteresis(bool p_enable)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_hysteresis>(p_enable);
    return *this;
  }

  pin& input_invert(bool p_enable)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_input_invert>(p_enable);
    return *this;
  }

  pin& analog(bool p_enable)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_analog_digital_mode>(p_enable);
    return *this;
  }

  pin& digital_filter(bool p_enable)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_digital_filter>(p_enable);
    return *this;
  }

  pin& highspeed_i2c(bool p_enable = true)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_i2c_highspeed>(p_enable);
    return *this;
  }

  pin& high_slew_rate(bool p_high_slew_rate = true)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_slew>(p_high_slew_rate);
    return *this;
  }

  pin& i2c_high_current(bool p_enable = true)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_i2c_high_current>(p_enable);
    return *this;
  }

  pin& open_drain(bool p_enable = true)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_open_drain>(p_enable);
    return *this;
  }

  pin& dac(bool p_enable = true)
  {
    xstd::bitmanip(map->matrix[m_port][m_pin])
      .insert<range_dac_enable>(p_enable);
    return *this;
  }

private:
  uint32_t m_port;
  uint32_t m_pin;
};
} // namespace embed::lpc40xx
