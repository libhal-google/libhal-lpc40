#include <libembeddedhal/clock.hpp>
#include <liblpc40xx/startup.hpp>
#include <liblpc40xx/uart.hpp>

int main()
{
  embed::lpc40xx::initialize_platform();

  // Required in order to have a high enough input clock rate to generate a
  // 115200 baud rate.
  embed::lpc40xx::internal::clock::get_clock_config().pll->multiply = 2;
  embed::lpc40xx::internal::clock::get_clock_config().pll->enabled = true;
  embed::lpc40xx::internal::clock::get_clock_config().cpu.use_pll0 = true;
  embed::lpc40xx::internal::clock::reconfigure_clocks();

  auto& uart0 = embed::lpc40xx::get_uart<0>();
  bool success = uart0.initialize();

  if (!success) {
    return -1;
  }

  auto test = "Hello, World!\n";
  std::span<const char> payload(test, sizeof(test));

  while (1)
  {
    using namespace std::chrono_literals;
    uart0.write(std::as_bytes(payload));
    embed::this_thread::sleep_for(200ms);
  }

  return 0;
}