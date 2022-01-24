#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libembeddedhal/serial/serial_util.hpp>
#include <libembeddedhal/time.hpp>
#include <liblpc40xx/adc.hpp>
#include <liblpc40xx/startup.hpp>
#include <liblpc40xx/uart.hpp>

int main()
{
  embed::lpc40xx::initialize_platform();

  constexpr embed::serial::settings new_settings{ .baud_rate = 38400 };
  auto& adc = embed::lpc40xx::get_adc<5>();
  auto& uart0 = embed::lpc40xx::get_uart<0>(new_settings);

  while (true) {
    using namespace std::chrono_literals;
    auto percent_string = adc.read().value().to_string();
    embed::write(
      uart0, std::string_view(percent_string.data(), percent_string.size()));
    embed::write(uart0, "\n");
    embed::this_thread::sleep_for(200ms);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
} // namespace boost
