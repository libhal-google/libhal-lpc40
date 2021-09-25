#pragma once

#include "internal/pin.hpp"
#include "internal/system_controller.hpp"

#include <libembeddedhal/adc.hpp>
#include <libxbitset/bitset.hpp>

namespace embed::lpc40xx {
class adc : public embed::adc
{
  struct channel
  {
    uint8_t port;
    uint8_t pin;
    uint8_t index;
    uint8_t pin_function;
    /// 1 MHz is the fastest sampling rate for ADC. The default is set to this
    /// value.
    uint32_t clock_rate_hz = 1'000'000;
  };

  /// Namespace containing the bitmask objects that are used to manipulate the
  /// lpc40xx ADC Control register.
  struct control_register
  {
    /// In burst mode, sets the ADC channels to be automatically converted.
    /// It bit position represents 1 channel with this 8 channel ADC.
    /// In software mode, this should hold only a single 1 for the single
    /// channel to be converted.
    static constexpr auto channelSelect = xstd::bitrange::from<0, 7>();

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

  struct lpc_adc_t
  {
    /// Offset: 0x000       A/D Control Register (R/W) */
    volatile uint32_t CR;
    /// Offset: 0x004       A/D Global Data Register (R/W) */
    volatile uint32_t GDR;
    uint32_t RESERVED0;
    /// Offset: 0x00C       A/D Interrupt Enable Register (R/W) */
    volatile uint32_t INTEN;
    /// Offset: 0x010-0x02C A/D Channel 0..7 Data Register (R/W) */
    volatile uint32_t DR[8];
    /// Offset: 0x030       A/D Status Register (R/ ) */
    const volatile uint32_t STAT;
    volatile uint32_t ADTRM;
  };

  /// A pointer holding the address to the LPC40xx ADC peripheral.
  /// This variable is a dependency injection point for unit testing thus it is
  /// public and mutable. This is needed to perform the "test by side effect"
  /// technique for this class.
  static constexpr intptr_t lpc_apb0_base = 0x40000000UL;
  static constexpr intptr_t lpc_adc_addr = lpc_apb0_base + 0x34000;
  inline static auto* reg = reinterpret_cast<lpc_adc_t*>(lpc_adc_addr);

  adc(channel& p_channel)
    : m_channel(p_channel)
  {}

  bool driver_initialize() override
  {
    internal::power(peripheral::adc).on();

    // For proper operation, analog pins must be set to floating.
    internal::pin(m_channel.port, m_channel.pin)
      .function(m_channel.pin_function)
      .resistor(embed::pin_resistor::none)
      .open_drain(false)
      .analog(true);

    const auto frequency = internal::clock(peripheral::adc).frequency();
    const auto clock_divider = frequency / m_channel.clock_rate_hz;

    // Activate burst mode (continous sampling), power on ADC and set clock
    // divider.
    xstd::bitmanip(reg->CR)
      .set(control_register::burst_enable)
      .set(control_register::power_enable)
      .insert<control_register::clock_divider>(clock_divider);

    // Enable channel. Must be done in a seperate write to memory than power on
    // and burst enable.
    xstd::bitmanip(reg->CR).set(m_channel.index);

    return true;
  }

  full_scale<uint32_t> read() override
  {
    auto sample = xstd::bitmanip(reg->DR[m_channel.index]);
    auto bit_value = sample.extract<data_register::result>();
    return bit_depth<uint32_t, 12>(bit_value.to_ulong());
  }

  auto& get_channel_info() { return m_channel; }

protected:
  channel& m_channel;
};

template<int channel>
static adc& get_adc()
{
  enum adc_function : uint8_t
  {
    pin_0123 = 0b001,
    pin_4567 = 0b011
  };

  if constexpr (channel == 0) {
    static const adc::channel channel0 = {
      .port = 0,
      .pin = 23,
      .index = 0,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel0(channel0);
    return adc_channel0;
  } else if constexpr (channel == 1) {
    static const adc::channel channel1 = {
      .port = 0,
      .pin = 24,
      .index = 1,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel1(channel1);
    return adc_channel1;
  } else if constexpr (channel == 2) {
    static const adc::channel channel2 = {
      .port = 0,
      .pin = 25,
      .index = 2,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel2(channel2);
    return adc_channel2;
  } else if constexpr (channel == 3) {
    static const adc::channel channel3 = {
      .port = 0,
      .pin = 26,
      .index = 3,
      .pin_function = adc_function::pin_0123,
    };
    static adc adc_channel3(channel3);
    return adc_channel3;
  } else if constexpr (channel == 4) {
    static const adc::channel channel4 = {
      .port = 1,
      .pin = 30,
      .index = 4,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel4(channel4);
    return adc_channel4;
  } else if constexpr (channel == 5) {
    static const adc::channel channel5 = {
      .port = 1,
      .pin = 31,
      .index = 5,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel5(channel5);
    return adc_channel5;
  } else if constexpr (channel == 6) {
    static const adc::channel channel6 = {
      .port = 0,
      .pin = 12,
      .index = 6,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel6(channel6);
    return adc_channel6;
  } else if constexpr (channel == 7) {
    static const adc::channel channel7 = {
      .port = 0,
      .pin = 13,
      .index = 7,
      .pin_function = adc_function::pin_4567,
    };
    static adc adc_channel7(channel7);
    return adc_channel7;
  } else {
    static_assert(channel <= 7,
                  "\n\n"
                  "LPC40xx Compile Time Error:\n"
                  "    LPC40xx only supports ADC channels from 0 to 7. \n"
                  "\n");
    return get_adc<0>();
  }
}
}