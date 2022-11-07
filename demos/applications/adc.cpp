#include <cinttypes>
#include <libarmcortex/dwt_counter.hpp>
#include <libhal/serial/util.hpp>
#include <libhal/steady_clock/util.hpp>
#include <liblpc40xx/adc.hpp>
#include <liblpc40xx/constants.hpp>
#include <liblpc40xx/uart.hpp>

hal::status application()
{
  static hal::cortex_m::dwt_counter counter(
    hal::lpc40xx::clock::get().get_frequency(hal::lpc40xx::peripheral::cpu));

  constexpr hal::serial::settings new_settings{ .baud_rate = 38400 };
  auto& uart0 = hal::lpc40xx::uart::get<0>(new_settings);
  HAL_CHECK(hal::write(uart0, "ADC Application Starting...\n"));
  auto& adc4 = hal::lpc40xx::adc::get<4>().value();
  auto& adc2 = hal::lpc40xx::adc::get<2>().value();
  std::array<char, 128> buffer{};

  while (true) {
    using namespace std::chrono_literals;

    // Read ADC values from both ADC channel 2 & ADC 4
    auto percent2 = adc2.read().value();
    auto percent4 = adc4.read().value();
    // Get current uptime
    auto uptime = counter.uptime().value();
    // Compute string from uptime count
    auto count = snprintf(buffer.data(),
                          buffer.size(),
                          "(%f, %f): %" PRIu64 " ns\n",
                          percent2,
                          percent4,
                          uptime);

    if (count > 0) {
      HAL_CHECK(hal::write(
        uart0,
        hal::as_bytes(std::span(buffer.data(), static_cast<size_t>(count)))));
    } else {
      HAL_CHECK(hal::write(uart0, "Could not fit ADC results into buffer!\n"));
    }

    HAL_CHECK(hal::delay(counter, 100ms));
  }

  return hal::success();
}
