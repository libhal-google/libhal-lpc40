#include <cstdio>
#include <libembeddedhal/clock.hpp>
#include <liblpc40xx/adc.hpp>
#include <liblpc40xx/input_pin.hpp>
#include <liblpc40xx/interrupt_pin.hpp>
#include <liblpc40xx/output_pin.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  embed::lpc40xx::initialize_platform();

  auto& pin = embed::lpc40xx::get_output_pin<1, 18>();
  auto& state_led = embed::lpc40xx::get_output_pin<1, 10>();

  bool success = pin.initialize();
  success &= state_led.initialize();

  if (!success) {
    return -1;
  }

  while (true) {
    using namespace std::chrono_literals;
    pin.level(true);
    state_led.level(true);
    embed::this_thread::sleep_for(500ms);
    pin.level(false);
    state_led.level(false);
    embed::this_thread::sleep_for(500ms);
  }

  return 0;
}
