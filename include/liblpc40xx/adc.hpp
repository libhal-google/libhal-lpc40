#pragma once

#include <libembeddedhal/adc/adc.hpp>
#include <libxbitset/bitset.hpp>

#include "internal/pin.hpp"
#include "system_controller.hpp"

namespace embed::lpc40xx {
/**
 * @brief Analog to digital converter
 *
 */
class adc : public embed::adc
{
public:
  /// Channel specific information
  struct channel
  {
    /// Default and highest sampling rate is 1 MHz. Careful as changing this for
    /// one channel changes this for all channels on the lpc40xx mcu.
    frequency clock_rate = frequency(1'000'000);
    /// Port of the adc pin
    uint8_t port;
    /// Pin number of the adc pin
    uint8_t pin;
    /// Channel data index
    uint8_t index;
    /// Pin mux function code
    uint8_t pin_function;
  };

  /// Namespace containing the bitmask objects that are used to manipulate the
  /// lpc40xx ADC Control register.
  struct control_register
  {
    /// In burst mode, sets the ADC channels to be automatically converted.
    /// It bit position represents 1 channel with this 8 channel ADC.
    /// In software mode, this should hold only a single 1 for the single
    /// channel to be converted.
    static constexpr auto channel_select = xstd::bitrange::from<0, 7>();

    /// Sets the channel's clock divider. Potentially saving power if clock is
    /// reduced further.
    static constexpr auto clock_divider = xstd::bitrange::from<8, 15>();

    /// Enable Burst Mode for the ADC. See BurstMode() method of this class to
    /// learn more about what it is and how it works.
    static constexpr auto burst_enable = xstd::bitrange::from<16>();

    /// Power on the ADC
    static constexpr auto power_enable = xstd::bitrange::from<21>();

    /// In order to start a conversion a start code must be inserted into this
    /// bit location.
    static constexpr auto start_code = xstd::bitrange::from<24, 26>();

    /// Not used in this driver, but allows the use of an external pins to
    /// trigger a conversion. This flag indicates if rising or falling edges
    /// trigger the conversion.
    /// 1 = falling, 0 = rising.
    static constexpr auto start_edge = xstd::bitrange::from<27>();
  };

  /// Namespace containing the bitmask objects that are used to manipulate the
  /// lpc40xx ADC Global Data register.
  struct data_register
  {
    /// Result mask holds the latest result from the last ADC that was converted
    static constexpr auto result = xstd::bitrange::from<4, 15>();

    /// Converted channel mask indicates which channel was converted in the
    /// latest conversion.
    static constexpr auto converted_channel = xstd::bitrange::from<24, 26>();

    /// Holds whether or not the ADC overran its conversion.
    static constexpr auto overrun = xstd::bitrange::from<30>();

    /// Indicates when the ADC conversion is complete.
    static constexpr auto done = xstd::bitrange::from<31>();
  };

  /// adc register map
  struct reg_t
  {
    /// Offset: 0x000 A/D Control Register (R/W)
    volatile uint32_t control;
    /// Offset: 0x004 A/D Global Data Register (R/W)
    volatile uint32_t global_data;
    /// Reserved 0
    std::array<uint32_t, 1> reserved0;
    /// Offset: 0x00C A/D Interrupt Enable Register (R/W)
    volatile uint32_t interrupt_enable;
    /// Offset: 0x010-0x02C A/D Channel 0..7 Data Register (R/W)
    volatile std::array<uint32_t, 8> data;
    /// Offset: 0x030 A/D Status Register (R/ )
    const volatile uint32_t stat;
    /// Offset: 0x034 A/D Trim Calibration (R/W)
    volatile uint32_t trim;
  };

  /// @return reg_t* - address of the adc peripheral
  static reg_t* reg()
  {
    if constexpr (!embed::is_platform("lpc40")) {
      static reg_t dummy{};
      return &dummy;
    } else {
      /// A pointer holding the address to the LPC40xx ADC peripheral.
      /// This variable is a dependency injection point for unit testing thus it
      /// is public and mutable. This is needed to perform the "test by side
      /// effect" technique for this class.
      static constexpr intptr_t lpc_apb0_base = 0x40000000UL;
      static constexpr intptr_t lpc_adc_addr = lpc_apb0_base + 0x34000;
      return reinterpret_cast<reg_t*>(lpc_adc_addr);
    }
  }

  /**
   * @brief Construct a new adc object
   *
   * @param p_channel - channel information
   */
  adc(channel p_channel) noexcept
    : m_channel(p_channel)
  {
    internal::power(peripheral::adc).on();

    // For proper operation, analog pins must be set to floating.
    internal::pin(m_channel.port, m_channel.pin)
      .function(m_channel.pin_function)
      .resistor(embed::pin_resistor::none)
      .open_drain(false)
      .analog(true);

    const auto clock_divider = internal::clock()
                                 .get_frequency(peripheral::adc)
                                 .divider(m_channel.clock_rate);

    // Activate burst mode (continuous sampling), power on ADC and set clock
    // divider.
    xstd::bitmanip(reg()->control)
      .set(control_register::burst_enable)
      .set(control_register::power_enable)
      .insert<control_register::clock_divider>(clock_divider);

    // Enable channel. Must be done in a separate write to memory than power on
    // and burst enable.
    xstd::bitmanip(reg()->control).set(m_channel.index);
  }
  /**
   * @brief Get a mutable reference to the channel info object
   *
   * @return auto& - get a reference to the channel information
   */
  auto& get_channel_info() { return m_channel; }

private:
  boost::leaf::result<percent> driver_read() noexcept override;
  channel m_channel;
};

/**
 * @brief Get the adc channel
 *
 * @tparam Channel - Which adc channel to return
 * @return adc& - statically allocated adc object
 */
template<int Channel>
inline adc& get_adc()
{
  enum adc_function : uint8_t
  {
    pin_0123 = 0b001,
    pin_4567 = 0b011
  };

  if constexpr (Channel == 0) {
    constexpr adc::channel channel0 = {
      .port = 0,
      .pin = 23,
      .index = 0,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel0(channel0);
    return adc_channel0;
  } else if constexpr (Channel == 1) {
    constexpr adc::channel channel1 = {
      .port = 0,
      .pin = 24,
      .index = 1,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel1(channel1);
    return adc_channel1;
  } else if constexpr (Channel == 2) {
    constexpr adc::channel channel2 = {
      .port = 0,
      .pin = 25,
      .index = 2,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel2(channel2);
    return adc_channel2;
  } else if constexpr (Channel == 3) {
    constexpr adc::channel channel3 = {
      .port = 0,
      .pin = 26,
      .index = 3,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel3(channel3);
    return adc_channel3;
  } else if constexpr (Channel == 4) {
    constexpr adc::channel channel4 = {
      .port = 1,
      .pin = 30,
      .index = 4,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel4(channel4);
    return adc_channel4;
  } else if constexpr (Channel == 5) {
    constexpr adc::channel channel5 = {
      .port = 1,
      .pin = 31,
      .index = 5,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel5(channel5);
    return adc_channel5;
  } else if constexpr (Channel == 6) {
    constexpr adc::channel channel6 = {
      .port = 0,
      .pin = 12,
      .index = 6,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel6(channel6);
    return adc_channel6;
  } else if constexpr (Channel == 7) {
    constexpr adc::channel channel7 = {
      .port = 0,
      .pin = 13,
      .index = 7,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel7(channel7);
    return adc_channel7;
  } else {
    static_assert(error::invalid_option<Channel>,
                  "\n\n"
                  "LPC40xx Compile Time Error:\n"
                  "    LPC40xx only supports ADC channels from 0 to 7. \n"
                  "\n");
    return get_adc<0>();
  }
}

inline boost::leaf::result<percent> adc::driver_read()
{
  auto sample = xstd::bitmanip(reg()->data[m_channel.index]);
  uint32_t bit_value = sample.extract<data_register::result>().to_ulong();
  return percent::convert<12>(bit_value);
}
}  // namespace embed::lpc40xx