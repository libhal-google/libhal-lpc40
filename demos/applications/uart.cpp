
#include <array>
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40xx/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application()
{
  using namespace hal::literals;
  // Change the input frequency to match the frequency of the crystal attached
  // to the external OSC pins.
  hal::lpc40xx::clock::maximum(12.0_MHz);

  auto& clock = hal::lpc40xx::clock::get();
  const auto cpu_frequency = clock.get_frequency(hal::lpc40xx::peripheral::cpu);
  hal::cortex_m::dwt_counter counter(cpu_frequency);

  auto& uart0 = HAL_CHECK(hal::lpc40xx::uart::get<0>({
    .baud_rate = 115200.0f,
  }));

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
