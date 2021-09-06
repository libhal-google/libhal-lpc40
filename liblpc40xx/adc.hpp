#pragma

#include <libembeddedhal/adc.hpp>
#include <libxbitset/bitset.hpp>

namespace embed::lpc40xx {
class adc : public embed::adc
{
  struct channel
  {
    std::pair<unsigned int, unsigned int> adc_pin;
    uint8_t channel;
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
    static constexpr auto kChannelSelect = xstd::bitrange::from<0, 7>();

    /// Sets the channel's clock divider. Potentially saving power if clock is
    /// reduced further.
    static constexpr auto kClockDivider = xstd::bitrange::from<8, 15>();

    /// Enable Burst Mode for the ADC. See BurstMode() method of this class to
    /// learn more about what it is and how it works.
    static constexpr auto kBurstEnable = xstd::bitrange::from<16>();

    /// Power on the ADC
    static constexpr auto kPowerEnable = xstd::bitrange::from<21>();

    /// In order to start a conversion a start code must be inserted into this
    /// bit location.
    static constexpr auto kStartCode = xstd::bitrange::from<24, 26>();

    /// Not used in this driver, but allows the use of an external pins to
    /// trigger a conversion. This flag indicates if rising or falling edges
    /// trigger the conversion.
    /// 1 = falling, 0 = rising.
    static constexpr auto kStartEdge = xstd::bitrange::from<27>();
  };

  /// Namespace containing the bitmask objects that are used to manipulate the
  /// lpc40xx ADC Global Data register.
  struct data_register
  {
    /// Result mask holds the latest result from the last ADC that was converted
    static constexpr auto kResult = xstd::bitrange::from<4, 15>();

    /// Converted channel mask indicates which channel was converted in the
    /// latest conversion.
    static constexpr auto kConvertedChannel = xstd::bitrange::from<24, 26>();

    /// Holds whether or not the ADC overran its conversion.
    static constexpr auto kOverrun = xstd::bitrange::from<30>();

    /// Indicates when the ADC conversion is complete.
    static constexpr auto kDone = xstd::bitrange::from<31>();
  };

  bool driver_initialize() override { return false; }
  full_scale<uint32_t> read() override
  {
    // adc_base->DR[channel_.channel], data_register::kResult
  }
};
}