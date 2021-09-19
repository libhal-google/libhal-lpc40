#include "adc.hpp"
#include "internal/pin.hpp"
#include "internal/system_controller.hpp"

namespace embed::lpc40xx {
adc::adc(channel& p_channel)
  : m_channel(p_channel)
{
  if constexpr (!embed::is_platform("lpc40")) {
    static lpc_adc_t dummy{};
    reg = &dummy;
  }
}

bool adc::driver_initialize()
{
  power(peripheral::adc).on();

  // For proper operation, analog pins must be set to floating.
  pin(m_channel.port, m_channel.pin)
    .function(m_channel.pin_function)
    .resistor(embed::pin_resistor::none)
    .open_drain(false)
    .analog(true);

  const auto frequency = clock(peripheral::adc).frequency();
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

full_scale<uint32_t> adc::read()
{
  auto sample = xstd::bitmanip(reg->DR[m_channel.index]);
  auto bit_value = sample.extract<data_register::result>();
  return bit_depth<uint32_t, 12>(bit_value.to_ulong());
}
}