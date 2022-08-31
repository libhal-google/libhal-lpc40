#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <array>
#include <string_view>

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libhal/serial/util.hpp>
#include <libhal/steady_clock/util.hpp>
#include <liblpc40xx/uart.hpp>

int main()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::system_control().initialize_floating_point_unit();

  hal::cortex_m::dwt_counter counter(
    hal::lpc40xx::internal::get_clock().get_frequency(
      hal::lpc40xx::peripheral::cpu));
  constexpr hal::serial::settings new_settings{ .baud_rate = 38400.0f };
  auto& uart0 = hal::lpc40xx::get_uart<0>(new_settings);

  while (true) {
    using namespace std::chrono_literals;

    std::string_view message = "Hello, World!\n";
    (void)hal::write_partial(
      uart0,
      std::span{ reinterpret_cast<const hal::byte*>(message.data()),
                 message.size() });

    (void)hal::delay(counter, 1s);

    // Echo back anything received
    std::array<hal::byte, 64> read_buffer;
    (void)uart0.write(uart0.read(read_buffer).value().received);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
