#define BOOST_LEAF_EMBEDDED
#define BOOST_LEAF_NO_THREADS

#include <cstdint>
#include <libarmcortex/dwt_counter.hpp>
#include <libarmcortex/startup.hpp>
#include <libarmcortex/system_control.hpp>
#include <libembeddedhal/frequency.hpp>
#include <libembeddedhal/percent.hpp>
#include <liblpc40xx/constants.hpp>
#include <liblpc40xx/uart.hpp>

std::span<const std::byte> to_bytes(std::string_view p_view)
{
  return std::as_bytes(std::span<const char>(p_view.data(), p_view.size()));
}

std::span<const std::byte> to_bytes(std::span<char> p_char)
{
  return std::as_bytes(std::span<const char>(p_char.begin(), p_char.end()));
}

auto write(embed::serial& p_serial, std::string_view p_string)
{
  return p_serial.write(to_bytes(p_string));
}

template<typename F>
auto benchmark(embed::counter& p_counter, F p_function)
{
  auto start = p_counter.uptime().value().count;
  p_function();
  auto finish = p_counter.uptime().value().count;
  return finish - start;
}

int main()
{
  embed::cortex_m::initialize_data_section();
  embed::cortex_m::system_control().initialize_floating_point_unit();

  static embed::cortex_m::dwt_counter counter(
    embed::lpc40xx::internal::get_clock().get_frequency(
      embed::lpc40xx::peripheral::cpu));

  constexpr embed::serial::settings new_settings{ .baud_rate = 38400 };
  auto& uart0 = embed::lpc40xx::get_uart<0>(new_settings);

  uart0.write(to_bytes("Starting Benchmark!\n"));
  std::array<char, 128> str{};
  str.fill(0);

  auto fixed_time = benchmark(counter, []() {
    int initial = 1'000'000'000;
    volatile int final_value = 0;
    for (std::int32_t i = 0; i < 10'000; i++) {
      embed::percent percent =
        embed::percent::from_ratio(i, std::numeric_limits<std::int32_t>::max());
      final_value = initial * percent;
    }
  });

  auto float_time = benchmark(counter, []() {
    int initial = 1'000'000'000;
    volatile float final_value = 0;
    for (std::int32_t i = 0; i < 10'000; i++) {
      float percent = float{ i } / std::numeric_limits<std::int32_t>::max();
      final_value = initial * percent;
    }
  });

  auto double_time = benchmark(counter, []() {
    int initial = 1'000'000'000;
    volatile double final_value = 0;
    for (std::int32_t i = 0; i < 10'000; i++) {
      double percent = double{ i } / std::numeric_limits<std::int32_t>::max();
      final_value = initial * percent;
    }
  });

  auto noop_time = benchmark(counter, []() {});

  int amount = 0;
  amount =
    std::snprintf(str.data(), str.size(), "fixed_time  = %u\n", fixed_time);
  uart0.write(to_bytes(str).subspan(0, amount));
  amount =
    std::snprintf(str.data(), str.size(), "float_time  = %u\n", float_time);
  uart0.write(to_bytes(str).subspan(0, amount));
  amount =
    std::snprintf(str.data(), str.size(), "double_time = %u\n", double_time);
  uart0.write(to_bytes(str).subspan(0, amount));
  amount =
    std::snprintf(str.data(), str.size(), " noop_time  = %u\n", noop_time);
  uart0.write(to_bytes(str).subspan(0, amount));

  uart0.write(to_bytes("DONE!!\n"));

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost
