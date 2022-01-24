#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libembeddedhal/time.hpp>
#include <liblpc40xx/interrupt_pin.hpp>
#include <liblpc40xx/output_pin.hpp>
#include <liblpc40xx/startup.hpp>

int main()
{
  embed::lpc40xx::initialize_platform();

  auto& button = embed::lpc40xx::get_interrupt_pin<0, 29>();
  auto& led = embed::lpc40xx::get_output_pin<1, 18>();

  (void)button.attach_interrupt(
    [&led]() {
      bool current_voltage_level = led.level().value();
      (void)led.level(!current_voltage_level);
    },
    embed::interrupt_pin::trigger_edge::falling);

  while (true) {
    continue;
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
} // namespace boost