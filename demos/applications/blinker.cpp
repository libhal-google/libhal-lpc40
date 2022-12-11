
#include <libarmcortex/dwt_counter.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <liblpc40xx/output_pin.hpp>
#include <liblpc40xx/system_controller.hpp>
#include <liblpc40xx/uart.hpp>

hal::status application()
{
  auto& clock = hal::lpc40xx::clock::get();
  hal::cortex_m::dwt_counter steady_clock(
    clock.get_frequency(hal::lpc40xx::peripheral::cpu));

  auto& uart0 = HAL_CHECK((hal::lpc40xx::uart::get<0>({
    .baud_rate = 38400.0f,
  })));
  HAL_CHECK(hal::write(uart0, "Starting blinker...\n"));

  auto& led = HAL_CHECK((hal::lpc40xx::output_pin::get<1, 18>()));

  while (true) {
    using namespace std::chrono_literals;
    HAL_CHECK(led.level(false));
    HAL_CHECK(hal::delay(steady_clock, 500ms));
    HAL_CHECK(led.level(true));
    HAL_CHECK(hal::delay(steady_clock, 500ms));
  }
}
