#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <liblpc40xx/interrupt_pin.hpp>
#include <liblpc40xx/output_pin.hpp>

int main()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::system_control().initialize_floating_point_unit();

  auto& button = hal::lpc40xx::interrupt_pin::get<0, 29>();
  auto& led = hal::lpc40xx::output_pin::get<1, 18>();
  (void)led.level(false);

  (void)button.configure({});
  (void)button.on_trigger([&led](bool p_level) {
    (void)p_level;
    bool current_voltage_level = led.level().value();
    (void)led.level(!current_voltage_level);
  });

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
}  // namespace boost