#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libhal/serial/util.hpp>
#include <libhal/steady_clock/util.hpp>
#include <liblpc40xx/input_pin.hpp>
#include <liblpc40xx/output_pin.hpp>
#include <liblpc40xx/system_controller.hpp>

int main()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::system_control().initialize_floating_point_unit();

  hal::cortex_m::dwt_counter clock(
    hal::lpc40xx::internal::clock::get().get_frequency(
      hal::lpc40xx::peripheral::cpu));

  auto& button = hal::lpc40xx::input_pin::get<0, 29>();
  auto& led = hal::lpc40xx::output_pin::get<1, 18>();

  while (true) {
    if (button.level().value()) {
      using namespace std::chrono_literals;
      (void)led.level(false);
      (void)hal::delay(clock, 200ms);
      (void)led.level(true);
      (void)hal::delay(clock, 200ms);
    }
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost