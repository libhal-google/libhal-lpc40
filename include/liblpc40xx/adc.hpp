#pragma once

#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal/adc.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "internal/pin.hpp"
#include "internal/platform_check.hpp"
#include "system_controller.hpp"

namespace hal::lpc40xx {
/**
 * @brief Analog to digital converter
 *
 */
class adc : public hal::adc
{
public:
  /// Channel specific information
  struct channel
  {
    /// Default and highest sampling rate is 1 MHz. Careful as changing this for
    /// one channel changes this for all channels on the lpc40xx mcu.
    hertz clock_rate = 1'000'000.0f;
    /// ADC pin
    internal::pin pin;
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
    static constexpr auto channel_select = hal::bit::mask::from<0, 7>();

    /// Sets the channel's clock divider. Potentially saving power if clock is
    /// reduced further.
    static constexpr auto clock_divider = hal::bit::mask::from<8, 15>();

    /// Enable Burst Mode for the ADC. See BurstMode() method of this class to
    /// learn more about what it is and how it works.
    static constexpr auto burst_enable = hal::bit::mask::from<16>();

    /// Power on the ADC
    static constexpr auto power_enable = hal::bit::mask::from<21>();

    /// In order to start a conversion a start code must be inserted into this
    /// bit location.
    static constexpr auto start_code = hal::bit::mask::from<24, 26>();

    /// Not used in this driver, but allows the use of an external pins to
    /// trigger a conversion. This flag indicates if rising or falling edges
    /// trigger the conversion.
    /// 1 = falling, 0 = rising.
    static constexpr auto start_edge = hal::bit::mask::from<27>();
  };

  /// Namespace containing the bitmask objects that are used to manipulate the
  /// lpc40xx ADC Global Data register.
  struct data_register
  {
    /// Result mask holds the latest result from the last ADC that was converted
    static constexpr auto result = hal::bit::mask::from<4, 15>();

    /// Converted channel mask indicates which channel was converted in the
    /// latest conversion.
    static constexpr auto converted_channel = hal::bit::mask::from<24, 26>();

    /// Holds whether or not the ADC overran its conversion.
    static constexpr auto overrun = hal::bit::mask::from<30>();

    /// Indicates when the ADC conversion is complete.
    static constexpr auto done = hal::bit::mask::from<31>();
  };

  /// adc register map
  struct reg_t
  {
    /// Number of channels
    static constexpr size_t channel_length = 8;
    /// Offset: 0x000 A/D Control Register (R/W)
    volatile uint32_t control;
    /// Offset: 0x004 A/D Global Data Register (R/W)
    volatile uint32_t global_data;
    /// Offset: 0x008 Reserved 0
    std::array<uint32_t, 1> reserved0;
    /// Offset: 0x00C A/D Interrupt Enable Register (R/W)
    volatile uint32_t interrupt_enable;
    /// Offset: 0x010-0x02C A/D Channel 0..7 Data Register (R/W)
    std::array<volatile uint32_t, channel_length> data;
    /// Offset: 0x030 A/D Status Register (R/ )
    const volatile uint32_t stat;
    /// Offset: 0x034 A/D Trim Calibration (R/W)
    volatile uint32_t trim;
  };

  /**
   * @brief Returns a reference of the adc peripheral's register map
   *
   * If this function is called on a platform that starts with anything other
   * than "lpc40", then this function return a dummy register map object which
   * can be used for testing.
   *
   * A pointer holding the address to the LPC40xx ADC peripheral. This variable
   * is a dependency injection point for unit testing thus it is public and
   * mutable. This is needed to perform the "test by side effect" technique for
   * this class.
   *
   * @return reg_t& - reference the to adc peripheral register map
   */
  static reg_t& reg()
  {
    if constexpr (hal::is_platform("lpc40")) {
      static constexpr intptr_t lpc_apb0_base = 0x40000000UL;
      static constexpr intptr_t lpc_adc_addr = lpc_apb0_base + 0x34000;
      return *reinterpret_cast<reg_t*>(lpc_adc_addr);
    } else if constexpr (hal::is_a_test()) {
      static reg_t dummy{};
      return dummy;
    }
  }

  /**
   * @brief Get a predefined adc channel
   *
   * - ADC channel 0 is pin(0, 23)
   * - ADC channel 1 is pin(0, 24)
   * - ADC channel 2 is pin(0, 25)
   * - ADC channel 3 is pin(0, 26)
   * - ADC channel 4 is pin(1, 30)
   * - ADC channel 5 is pin(1, 31)
   * - ADC channel 6 is pin(0, 12)
   * - ADC channel 7 is pin(0, 13)
   *
   * @tparam Channel - Which adc channel to return
   * @return adc& - statically allocated adc object
   */
  template<size_t Channel>
  static result<adc&> get()
  {
    compile_time_platform_check();
    static_assert(Channel < reg_t::channel_length,
                  "\n\n"
                  "LPC40xx Compile Time Error:\n"
                  "    LPC40xx only supports ADC channels from 0 to 7. \n"
                  "\n");

    auto channel_info = get_channel_info<Channel>();
    HAL_CHECK(setup(channel_info));
    static adc adc_channel(channel_info);
    return adc_channel;
  }

  /**
   * @brief Construct a custom adc object based on the passed in channel
   * information.
   *
   * Care should be taken to ensure that the adc's operating frequency does not
   * go above 1MHz and that the the channel index is within the bounds of 0
   * to 7. Exceeding these bounds will result in a call to std::abort.
   *
   * Care should also be taken to ensure that two adc's constructed via this
   * method do not overlap in index.
   *
   * The operating frequency is shared across all adc channels, which means that
   * the last adc to be constructed will set sampling frequency for all
   * channels.
   *
   * @param p_channel - Which adc channel to return
   * @return adc& - statically allocated adc object
   */
  static result<adc> construct_custom_channel(const channel& p_channel)
  {
    HAL_CHECK(setup(p_channel));
    adc adc_channel(p_channel);
    return adc_channel;
  }

  virtual ~adc() = default;

private:
  /**
   * @brief Get the channel info object
   *
   * @tparam Channel - which predefined channel to return
   * @return constexpr channel - the channel info
   */
  template<int Channel>
  static constexpr channel get_channel_info()
  {
    enum adc_function : uint8_t
    {
      pin_0123 = 0b001,
      pin_4567 = 0b011
    };
    constexpr std::array channels{
      channel{
        .pin = internal::pin(0, 23),
        .index = 0,
        .pin_function = adc_function::pin_0123,
      },
      channel{
        .pin = internal::pin(0, 24),
        .index = 1,
        .pin_function = adc_function::pin_0123,
      },
      channel{
        .pin = internal::pin(0, 25),
        .index = 2,
        .pin_function = adc_function::pin_0123,
      },
      channel{
        .pin = internal::pin(0, 26),
        .index = 3,
        .pin_function = adc_function::pin_0123,
      },
      channel{
        .pin = internal::pin(1, 30),
        .index = 4,
        .pin_function = adc_function::pin_4567,
      },
      channel{
        .pin = internal::pin(1, 31),
        .index = 5,
        .pin_function = adc_function::pin_4567,
      },
      channel{
        .pin = internal::pin(0, 12),
        .index = 6,
        .pin_function = adc_function::pin_4567,
      },
      channel{
        .pin = internal::pin(0, 13),
        .index = 7,
        .pin_function = adc_function::pin_4567,
      },
    };
    return channels[Channel];
  }

  static status setup(const channel& p_channel)
  {
    using namespace hal::literals;

    if (p_channel.clock_rate > 1.0_MHz) {
      return hal::new_error();
    }

    if (p_channel.index >= reg_t::channel_length) {
      return hal::new_error();
    }

    power(peripheral::adc).on();

    // For proper operation, analog pins must be set to floating.
    p_channel.pin.function(p_channel.pin_function)
      .resistor(hal::pin_resistor::none)
      .open_drain(false)
      .analog(true);

    const auto clock_frequency = clock::get().get_frequency(peripheral::adc);
    const auto clock_divider = clock_frequency / p_channel.clock_rate;
    const auto clock_divider_int = static_cast<std::uint32_t>(clock_divider);

    // Activate burst mode (continuous sampling), power on ADC and set clock
    // divider.
    hal::bit::modify(reg().control)
      .set<control_register::burst_enable>()
      .set<control_register::power_enable>()
      .insert<control_register::clock_divider>(clock_divider_int);

    // Enable channel. Must be done in a separate write to memory than power on
    // and burst enable.
    hal::bit::modify(reg().control)
      .set(bit::mask{ .position = p_channel.index, .width = 1 });

    return hal::success();
  }

  adc(const channel& p_channel) noexcept
    : m_sample(&reg().data[p_channel.index])
  {
  }

  result<float> driver_read() noexcept override
  {
    constexpr auto max = bit_limits<12, size_t>::max();
    constexpr auto max_float = static_cast<float>(max);
    // Read sample from peripheral memory
    auto sample_integer = hal::bit::extract<data_register::result>(*m_sample);
    auto sample = static_cast<float>(sample_integer);
    return sample / max_float;
  }

  volatile uint32_t* m_sample = nullptr;
};
}  // namespace hal::lpc40xx