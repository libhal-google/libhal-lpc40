
#include <libhal-lpc40xx/interrupt_pin.hpp>
#include <libhal-lpc40xx/output_pin.hpp>

hal::status application()
{
  auto& button = HAL_CHECK((hal::lpc40xx::interrupt_pin::get<0, 29>()));
  auto& led = HAL_CHECK((hal::lpc40xx::output_pin::get<1, 18>()));

  HAL_CHECK(led.level(false));

  HAL_CHECK(button.configure({}));
  button.on_trigger([&led]([[maybe_unused]] bool p_level) {
    bool current_voltage_level = led.level().value().state;
    (void)led.level(!current_voltage_level);
  });

  while (true) {
    continue;
  }

  return hal::success();
}
