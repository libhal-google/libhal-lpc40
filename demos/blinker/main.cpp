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
#include <liblpc40xx/uart.hpp>

void print(hal::serial& p_serial, std::string_view message)
{
  (void)p_serial.write(std::span{
    reinterpret_cast<const hal::byte*>(message.data()), message.size() });
}

int main()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::system_control().initialize_floating_point_unit();

  auto& clock = hal::lpc40xx::clock::get();
  hal::cortex_m::dwt_counter steady_clock(
    clock.get_frequency(hal::lpc40xx::peripheral::cpu));

  auto& uart0 = hal::lpc40xx::uart::get<0>({ .baud_rate = 38400.0f });
  print(uart0, "Starting blinker...\n");

  auto& led = hal::lpc40xx::output_pin::get<1, 18>();

  while (true) {
    using namespace std::chrono_literals;
    (void)led.level(false);
    (void)hal::delay(steady_clock, 500ms);
    (void)led.level(true);
    (void)hal::delay(steady_clock, 500ms);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
