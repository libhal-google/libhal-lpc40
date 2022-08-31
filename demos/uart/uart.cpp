#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <array>
#include <string_view>

#include <libarmcortex/startup.hpp>
#include <libhal/serial/serial_util.hpp>
#include <libhal/time.hpp>
#include <liblpc40xx/startup.hpp>
#include <liblpc40xx/uart.hpp>

int main()
{
  hal::lpc40xx::initialize_platform();
  constexpr hal::serial::settings new_settings{ .baud_rate = 38400 };
  auto& uart0 = hal::lpc40xx::get_uart<0>(new_settings);

  while (true) {
    using namespace std::chrono_literals;

    std::string_view message = "Hello, World!\n";
    (void)hal::write(
      uart0, std::as_bytes(std::span{ message.begin(), message.end() }));
    hal::this_thread::sleep_for(1000ms);

    // Echo back anything received
    std::array<hal::byte, 64> read_buffer;
    (void)uart0.write(uart0.read(read_buffer).value());
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
