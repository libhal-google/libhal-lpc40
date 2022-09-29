#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libhal/counter/util.hpp>
#include <libhal/serial/util.hpp>
#include <liblpc40xx/adc.hpp>
#include <liblpc40xx/constants.hpp>
#include <liblpc40xx/uart.hpp>

std::span<const hal::byte> to_bytes(std::string_view p_view)
{
  return std::as_bytes(std::span<const char>(p_view.data(), p_view.size()));
}

std::span<const hal::byte> to_bytes(std::span<char> p_char)
{
  return std::as_bytes(std::span<const char>(p_char.begin(), p_char.end()));
}

auto write(hal::serial& p_serial, std::string_view p_string)
{
  return p_serial.write(to_bytes(p_string));
}

int main()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::system_control().initialize_floating_point_unit();

  static hal::cortex_m::dwt_counter counter(
    hal::lpc40xx::get_clock().get_frequency(hal::lpc40xx::peripheral::cpu));

  constexpr hal::serial::settings new_settings{ .baud_rate = 38400 };
  auto& uart0 = hal::lpc40xx::get_uart<0>(new_settings);
  write(uart0, "ADC Application Starting...\n");
  auto& adc4 = hal::lpc40xx::adc::get<4>().value();
  auto& adc2 = hal::lpc40xx::adc::get<2>().value();
  std::array<char, 32> buffer{};

  while (true) {
    using namespace std::chrono_literals;

    // Read ADC values from both ADC channel 2 & ADC 4
    auto percent_string2 = adc2.read().value().to_string();
    auto percent_string4 = adc4.read().value().to_string();
    // Get current uptime
    auto uptime = counter.uptime().value().count;
    // Compute string from uptime count
    auto result = std::to_chars(buffer.begin(), buffer.end(), uptime, 10);
    auto uptime_bytes = to_bytes(std::span(buffer.begin(), result.ptr));

    (void)hal::write(uart0, to_bytes("("));
    (void)hal::write(uart0, to_bytes(percent_string2));
    (void)hal::write(uart0, to_bytes(", "));
    (void)hal::write(uart0, to_bytes(percent_string4));
    (void)hal::write(uart0, to_bytes("): "));
    (void)hal::write(uart0, uptime_bytes);
    (void)hal::write(uart0, to_bytes("ns\n"));

    (void)hal::delay(counter, 100ms);
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
