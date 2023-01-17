
#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40xx/output_pin.hpp>
#include <libhal-lpc40xx/system_controller.hpp>
#include <libhal-lpc40xx/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application()
{
  auto& clock = hal::lpc40xx::clock::get();
  hal::cortex_m::dwt_counter steady_clock(
    clock.get_frequency(hal::lpc40xx::peripheral::cpu));

  auto& led = HAL_CHECK((hal::lpc40xx::output_pin::get<1, 10>()));

  while (true) {
    using namespace std::chrono_literals;
    HAL_CHECK(led.level(false));
    HAL_CHECK(hal::delay(steady_clock, 500ms));
    HAL_CHECK(led.level(true));
    HAL_CHECK(hal::delay(steady_clock, 500ms));
  }
}
