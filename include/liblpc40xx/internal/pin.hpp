#pragma once

#include <cstdint>
#include <cstring>
#include <libembeddedhal/config.hpp>
#include <libembeddedhal/gpio/pin_resistors.hpp>
#include <libxbitset/bitset.hpp>

#include "gpio.hpp"

namespace embed::lpc40xx::internal {
/**
 * @brief lpc40xx pin muxing and control driver used drivers and apps seaking to
 * tune the pins further after initialization.
 *
 */
class pin
{
public:
  /// Pin map table for maping pins and ports to registers.
  struct pin_map_t
  {
    /// Register matrix that maps against the 6 ports and the 32 pins per port
    volatile uint32_t matrix[6][32];
  };

  /// The address of the IO connect peripheral
  static constexpr intptr_t io_connect_address = 0x40000000UL + 0x2C000;

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

  /// @return pin_map_t* -  Return the address of the pin map peripheral
  static pin_map_t* map()
  {
    if constexpr (!embed::is_platform("lpc40")) {
      static pin_map_t dummy{};
      return &dummy;
    } else {
      return reinterpret_cast<pin_map_t*>(io_connect_address);
    }
  }

  /**
   * @brief Construct a new pin object
   *
   * See UM10562 page 99 for more details on which pins can be what function.
   *
   * @param p_port - selects pin port to use
   * @param p_pin - selects pin within the port to use
   */
  constexpr pin(int p_port, int p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {}

  /// Default constructor
  constexpr pin() = default;

  /**
   * @brief Change the function of the pin (mux the pins function)
   *
   * @param p_function_code - the pin function code
   * @return pin& - reference to this pin for chaining
   */
  pin& function(uint8_t p_function_code)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_function>(p_function_code);
    return *this;
  }

  /**
   * @brief Set the internal resistor connection for this pin
   *
   * @param p_resistor - resistor type
   * @return pin& - reference to this pin for chaining
   */
  pin& resistor(embed::pin_resistor p_resistor)
  {
    // The pin resistor enumeration matches the values for the LPC40xx so simply
    // cast the enum to an int and this will work.
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_resistor>(static_cast<uint32_t>(p_resistor));
    return *this;
  }

  /**
   * @brief Disable or enable hysteresis mode for this pin
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& hysteresis(bool p_enable)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_hysteresis>(p_enable);
    return *this;
  }

  /**
   * @brief invert the logic for this pin in input mode
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& input_invert(bool p_enable)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_input_invert>(p_enable);
    return *this;
  }

  /**
   * @brief enable analog mode for this pin (required for dac and adc drivers)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& analog(bool p_enable)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_analog_digital_mode>(p_enable);
    return *this;
  }

  /**
   * @brief enable digital filtering (filter out noise on input lines)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& digital_filter(bool p_enable)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_digital_filter>(p_enable);
    return *this;
  }

  /**
   * @brief Enable high speed mode for i2c pins
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& highspeed_i2c(bool p_enable = true)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_i2c_highspeed>(p_enable);
    return *this;
  }

  /**
   * @brief enable high slew rate for pin
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& high_slew_rate(bool p_enable = true)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin]).insert<range_slew>(p_enable);
    return *this;
  }

  /**
   * @brief enable high current drain for i2c lines
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& i2c_high_current(bool p_enable = true)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_i2c_high_current>(p_enable);
    return *this;
  }

  /**
   * @brief Make the pin open drain (required for the i2c driver)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& open_drain(bool p_enable = true)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_open_drain>(p_enable);
    return *this;
  }

  /**
   * @brief Enable dac mode (required for the dac driver)
   *
   * @param p_enable - enable this mode, set to false to disable this mode
   * @return pin& - reference to this pin for chaining
   */
  pin& dac(bool p_enable = true)
  {
    xstd::bitmanip(map()->matrix[m_port][m_pin])
      .insert<range_dac_enable>(p_enable);
    return *this;
  }

private:
  int m_port{};
  int m_pin{};
};
}  // namespace embed::lpc40xx::internal
