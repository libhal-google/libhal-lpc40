
#include <array>
#include <string_view>

#include <libarmcortex/dwt_counter.hpp>
#include <libhal/serial/util.hpp>
#include <libhal/steady_clock/util.hpp>
#include <liblpc40xx/uart.hpp>

hal::status application()
{
  using namespace hal::literals;
  // If CAN baudrate is above 100.0_kHz, then an external crystal must be used
  // for clock rate accuracy.
  // Change the input frequency
  hal::lpc40xx::clock::maximum(12.0_MHz);

  auto& clock = hal::lpc40xx::clock::get();
  const auto cpu_frequency = clock.get_frequency(hal::lpc40xx::peripheral::cpu);
  hal::cortex_m::dwt_counter counter(cpu_frequency);

  auto& uart0 = hal::lpc40xx::uart::get<0>({ .baud_rate = 38400.0f });

  while (true) {
    using namespace std::chrono_literals;

    std::string_view message = "Hello, World!\n";
    HAL_CHECK(hal::write(uart0, message));
    HAL_CHECK(hal::delay(counter, 1s));
    // Echo back anything received
    std::array<hal::byte, 64> read_buffer;
    HAL_CHECK(uart0.write(HAL_CHECK(uart0.read(read_buffer)).data));
  }

  return hal::success();
}
